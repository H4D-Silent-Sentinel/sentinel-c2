"""UDP link to every Visual-Tracker (Mac or Jetson). Spec: sentinel docs/jetson-interface.md v1.

  tracker --visual--> C2   udp/47010   what the camera sees; an empty `tracks` list is its heartbeat
  tracker <--cue----- C2   udp/47011   confirmed fused tracks, <= 1/s per track
  tracker <--status-- C2   udp/47011   C2 heartbeat, every step, even with no tracks

Trackers are learned from the address their visual reports come from, plus any --tracker IPs.
Until one is heard, status is also broadcast so a tracker started with no base address finds the C2.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass

from cuas.visual import CUE_PORT, SCHEMA_VERSION, VISUAL_PORT, make_cue, parse_visual


@dataclass
class TrackerPeer:
    ip: str
    src: str = "?"
    first_heard: float = 0.0
    last_heard: float = 0.0
    msgs: int = 0
    tracks_seen: int = 0  # tracks in its latest report
    up: bool = False
    # where it says it is and points (moving cameras like a chaser report this with every message)
    e: float | None = None
    n: float | None = None
    yaw: float | None = None
    hfov: float | None = None


class TrackerLink:
    def __init__(self, base, visual_port: int = VISUAL_PORT, cue_port: int = CUE_PORT,
                 static: list[str] | None = None, broadcast: str | None = "255.255.255.255",
                 timeout_s: float = 3.0, cue_interval_s: float = 1.0, log=print):
        self.base, self.cue_port, self.log = base, cue_port, log
        self.static, self.broadcast, self.timeout_s = list(static or []), broadcast, timeout_s
        self.cue_interval_s = cue_interval_s
        self.gate = None  # optional: () -> bool; False drops camera reports (operator mute)
        self.peers: dict[tuple[str, str], TrackerPeer] = {}  # (ip, camera src) -> peer
        self._lock = threading.Lock()
        self._last_cue: dict[int, float] = {}
        self._stop = threading.Event()
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", visual_port))
        self.rx.settimeout(0.5)
        self.visual_port = self.rx.getsockname()[1]
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._thread = threading.Thread(target=self._listen, daemon=True, name="trackers")

    def start(self) -> "TrackerLink":
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        self.rx.close()
        self.tx.close()

    # -- inbound ----------------------------------------------------------------
    def _listen(self):
        self.log(f"trackers: listening on udp/{self.visual_port}")
        while not self._stop.is_set():
            try:
                data, addr = self.rx.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.on_datagram(data, addr[0])

    def on_datagram(self, data: bytes, ip: str, now: float | None = None):
        now = time.time() if now is None else now
        try:
            m = json.loads(data)
        except ValueError:
            self.log(f"trackers: non-JSON datagram from {ip}")
            return
        if not isinstance(m, dict) or m.get("v") != SCHEMA_VERSION or m.get("type") != "visual":
            return
        src = str(m.get("src", "?"))
        with self._lock:  # any valid visual report counts as a heartbeat, even if we can't use it
            key = (ip, src)  # one machine can run several cameras (e.g. post-1 and a phone as chaser-1)
            p = self.peers.get(key)
            if p is None:
                p = self.peers[key] = TrackerPeer(ip, first_heard=now)
            p.src, p.last_heard, p.msgs = src, now, p.msgs + 1
            p.tracks_seen = len(m.get("tracks") or [])
            sen = m.get("sensor") or {}
            try:
                if sen.get("lat") is not None and sen.get("lon") is not None:
                    p.e, p.n = self.base.site.frame.to_en(float(sen["lat"]), float(sen["lon"]))
                else:
                    p.e = p.n = None
                p.yaw = None if sen.get("yaw_deg") is None else float(sen["yaw_deg"]) % 360
                p.hfov = None if sen.get("hfov_deg") is None else float(sen["hfov_deg"])
            except (TypeError, ValueError):
                pass
        try:
            obs = parse_visual(data, self.base.site.frame, self.base.site.cameras)
        except (ValueError, KeyError, TypeError) as e:
            self.log(f"trackers: unusable report from {ip} ({e})")
            return
        if obs and (self.gate is None or self.gate()):
            self.base.on_visual(obs)

    def check_health(self, now: float | None = None) -> list[TrackerPeer]:
        """Mark peers up/down and log changes. Returns a snapshot of all peers."""
        now = time.time() if now is None else now
        with self._lock:
            for p in self.peers.values():
                up = now - p.last_heard < self.timeout_s
                if up != p.up:
                    p.up = up
                    self.log(f"trackers: {p.src} @ {p.ip} " + ("UP" if up else f"DOWN (silent {now - p.last_heard:.0f} s)"))
            return [TrackerPeer(**vars(p)) for p in self.peers.values()]

    # -- outbound ---------------------------------------------------------------
    def destinations(self) -> set[str]:
        with self._lock:
            return set(self.static) | {p.ip for p in self.peers.values()}

    def _send(self, msg: bytes, ips) -> int:
        n = 0
        for ip in ips:
            try:
                self.tx.sendto(msg, (ip, self.cue_port))
                n += 1
            except OSError:
                pass
        return n

    def push_cues(self, snapshot: list[dict], now: float | None = None) -> int:
        now = time.time() if now is None else now
        ips, n = self.destinations(), 0
        if not ips:
            return 0
        for tr in snapshot:
            if not tr["confirmed"] or now - self._last_cue.get(tr["id"], 0.0) < self.cue_interval_s:
                continue
            self._last_cue[tr["id"]] = now
            n += self._send(make_cue(tr, self.base.site.frame, now), ips)
        return n

    def push_status(self, status: dict) -> int:
        msg = json.dumps({"v": SCHEMA_VERSION, "type": "status", **status}, separators=(",", ":")).encode()
        ips = self.destinations()
        with self._lock:
            anyone_up = any(p.up for p in self.peers.values())
        if self.broadcast and not anyone_up:
            ips = ips | {self.broadcast}
        return self._send(msg, ips)
