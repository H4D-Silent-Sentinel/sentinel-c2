"""Operator hand controls (the tango program: a Tango 2 radio over USB).

  tango --control/controls--> ROAREE   udp/47013

What each control does here:
  mute_acoustic / mute_radar / mute_camera  reports from that sensor group are not fused while on
  node_select (+1/-1) + node_toggle         pick a node; toggle ignores / restores its reports
  tak_out                                   CoT output to TAK on (default) or off
  track_select (+1/-1)                      pick a track to focus on the dashboard

Every change is logged to the dashboard event list.
"""
from __future__ import annotations

import json
import socket
import threading
import time

OPERATOR_PORT = 47013
TIMEOUT_S = 2.5  # no message for this long -> controller lost
GROUPS = ("acoustic", "radar", "camera")


def _cycle(ids: list, cur, step: int):
    if not ids:
        return None
    if cur not in ids:
        return ids[0] if step > 0 else ids[-1]
    return ids[(ids.index(cur) + step) % len(ids)]


class Operator:
    def __init__(self, hub, port: int | None = OPERATOR_PORT, timeout_s: float = TIMEOUT_S, log=print):
        self.hub, self.log, self.timeout_s = hub, log, timeout_s
        self.muted: set[str] = set()
        self.ignored: set[int] = set()
        self.tak_out = True
        self.track: int | None = None
        self.node: int | None = None
        self.connected = False
        self.last_heard = 0.0
        self.src: str | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.sock = None
        if port is not None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", port))
            self.sock.settimeout(0.5)

    # -- gates the sensor links check before fusing a report ----------------------------
    def accepts_packet(self, pkt) -> bool:
        return "acoustic" not in self.muted and pkt.node_id not in self.ignored

    def accepts(self, group: str) -> bool:
        return group not in self.muted

    # -- receive ------------------------------------------------------------------------
    def start(self) -> "Operator":
        if self.sock:
            threading.Thread(target=self._run, daemon=True, name="operator").start()
        return self

    def close(self):
        self._stop.set()
        if self.sock:
            self.sock.close()

    def _run(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                self.check(time.time())
                continue
            except OSError:
                break
            self.on_datagram(data, addr[0])

    def on_datagram(self, data: bytes, ip: str, now: float | None = None) -> None:
        now = time.time() if now is None else now
        try:
            m = json.loads(data)
            kind = m["type"]
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            self.last_heard, self.src = now, f"{m.get('src', 'tango')}@{ip}"
            if kind == "controls":
                self._set_connected(bool(m.get("connected")), now)
                if self.connected:  # heartbeat carries switch positions; apply any we haven't seen
                    for name, on in (m.get("state") or {}).items():
                        if name == "tak_out" and bool(on) != self.tak_out or \
                           name.startswith("mute_") and name[5:] in GROUPS and bool(on) != (name[5:] in self.muted):
                            self._apply(name, bool(on), now)
            elif kind == "control":
                self._set_connected(True, now)
                self._apply(str(m.get("name")), m.get("value"), now)

    def check(self, now: float) -> None:
        with self._lock:
            if self.connected and now - self.last_heard > self.timeout_s:
                self.connected = False
                self._note(now, "warn", f"Controller silent for {self.timeout_s:.0f} s.")

    def _set_connected(self, on: bool, now: float) -> None:
        if on != self.connected:
            self.connected = on
            self._note(now, "ok" if on else "warn", "Controller connected." if on else "Controller unplugged.")

    # -- controls -----------------------------------------------------------------------
    def _apply(self, name: str, value, now: float) -> None:
        if name.startswith("mute_") and name[5:] in GROUPS:
            g = name[5:]
            (self.muted.add if value else self.muted.discard)(g)
            self._note(now, "warn" if value else "info", f"{g.capitalize()} {'muted' if value else 'unmuted'} by operator.")
            self.hub.base.mark_updated()
        elif name == "tak_out":
            self.tak_out = bool(value)
            self._note(now, "info", f"TAK output {'on' if self.tak_out else 'off'}.")
        elif name == "node_select":
            self.node = _cycle(sorted(self.hub.base.site.nodes), self.node, int(value))
            self._note(now, "info", f"Node {self.node} selected.")
        elif name == "node_toggle":
            if self.node is None:
                self._note(now, "info", "Node toggle ignored: no node selected.")
            elif self.node in self.ignored:
                self.ignored.discard(self.node)
                self._note(now, "info", f"Node {self.node} restored by operator.")
            else:
                self.ignored.add(self.node)
                self._note(now, "warn", f"Node {self.node} ignored by operator.")
        elif name == "track_select":
            ids = [t["id"] for t in self.hub.snap]
            self.track = _cycle(ids, self.track, int(value))
            self._note(now, "info", f"Drone {self.track} selected." if self.track is not None else "No tracks to select.")

    def _note(self, now: float, level: str, text: str) -> None:
        self.log(f"operator: {text}")
        ev = getattr(self.hub, "events", None)
        if ev is not None:
            ev.add(now, level, text)

    # -- dashboard ----------------------------------------------------------------------
    def view(self, now: float) -> dict:
        with self._lock:
            if not self.connected:
                st = "down" if self.src else "off"
                text = "Not connected. Run `tango` with the Tango 2 plugged in." if st == "off" else "Controller lost."
            else:
                parts = [f"Muted: {', '.join(sorted(self.muted))}." if self.muted else "No sensors muted.",
                         f"Ignored nodes: {', '.join(map(str, sorted(self.ignored)))}." if self.ignored else "",
                         f"Node {self.node} selected." if self.node is not None else "",
                         f"Drone {self.track} selected." if self.track is not None else "",
                         "TAK output on." if self.tak_out else "TAK output off."]
                st, text = "ok", " ".join(p for p in parts if p)
            return dict(kind="operator", id="operator", name="Tango 2", label=self.src or "", status=st, text=text,
                        muted=sorted(self.muted), ignored=sorted(self.ignored), track=self.track,
                        tak_out=self.tak_out)
