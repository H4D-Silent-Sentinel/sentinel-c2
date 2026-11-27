"""The C2 hub: collects from the ESP32 node network and the Visual-Trackers, fuses
(using sentinel's Base), and keeps both sides updated continuously.

  ESP32 nodes ──LoRa──▶ base_rx ESP32 ──USB──▶ ┌──────────────┐ ──cue/status──▶ Visual-Tracker(s)
              ◀──LoRa── (HINT downlink) ◀──USB─ │    ROAREE    │ ◀──visual──────
  radar (real or fake_radar) ──udp/47012──────▶ └──────────────┘ ──▶ map / TAK
"""
from __future__ import annotations

import threading
import time

from cuas.base import NODE_STALE_S, Base
from cuas.config import Site

from .esp32 import Esp32Link
from .hints import HintConfig, HintPolicy
from .operator import OPERATOR_PORT, Operator
from .radar import RADAR_PORT, RadarLink
from .ui import EventLog, summary
from .tracker_link import TrackerLink


FUSE_MIN_S = 0.1  # fusion runs as soon as data arrives, at most ~10 times a second


class Hub:
    def __init__(self, site: Site, serial_port: str | None = None, trackers: list[str] | None = None,
                 visual_port: int = 47010, cue_port: int = 47011, broadcast: str | None = "255.255.255.255",
                 hint_cfg: HintConfig | None = None, hints_enabled: bool = True,
                 radar_port: int | None = RADAR_PORT, operator_port: int | None = OPERATOR_PORT, log=print):
        self.log = log
        self.base = Base(site)
        self.esp = Esp32Link(serial_port, self.base, log=log) if serial_port else None
        self.trackers = TrackerLink(self.base, visual_port, cue_port, static=trackers,
                                    broadcast=broadcast, log=log)
        self.radar = RadarLink(self.base, radar_port, log=log) if radar_port is not None else None
        self.hints = HintPolicy(self.base, self._send_hint, hint_cfg, log=log)
        self.hints_enabled = hints_enabled
        self.events = EventLog()
        self.operator = Operator(self, operator_port, log=log)
        if self.esp:
            self.esp.gate = self.operator.accepts_packet
        self.trackers.gate = lambda: self.operator.accepts("camera")
        if self.radar:
            self.radar.gate = lambda: self.operator.accepts("radar")
        self.seq = 0
        self.snap: list[dict] = []
        self.version = 0  # bumps on every fusion cycle; the dashboard streams on change
        self._cv = threading.Condition()

    def _send_hint(self, pkt) -> bool:
        return self.esp.send_packet(pkt) if self.esp else False

    def start(self) -> "Hub":
        if self.esp:
            self.esp.start()
        self.trackers.start()
        if self.radar:
            self.radar.start()
        self.operator.start()
        return self

    def close(self):
        if self.esp:
            self.esp.close()
        self.trackers.close()
        if self.radar:
            self.radar.close()
        self.operator.close()

    def status(self, snap: list[dict], now: float) -> dict:
        with self.base.lock:
            online = sum(1 for ns in self.base.nodes.values() if ns.last_seen and now - ns.last_seen < NODE_STALE_S)
            detecting = sorted(nid for nid, ns in self.base.nodes.items() if ns.detecting)
        return {"src": "sentinel-c2", "t": now, "seq": self.seq,
                "esp32": self.esp.state if self.esp else "off",
                "nodes_online": online, "nodes_total": len(self.base.site.nodes), "nodes_detecting": detecting,
                "tracks": len(snap), "confirmed": sum(1 for t in snap if t["confirmed"]),
                "hints_sent": len(self.hints.sent),
                "radars": {k: v["up"] for k, v in (self.radar.check_health(now) if self.radar else {}).items()}}

    # -- fast path: runs whenever sensor data arrives --------------------------
    def fuse(self, now: float | None = None) -> list[dict]:
        """Fuse new reports into tracks and cue the cameras. Called on data arrival."""
        now = time.time() if now is None else now
        snap = self.base.step(now)
        self.trackers.check_health(now)
        self.trackers.push_cues(snap, now)
        self.snap = snap
        with self._cv:
            self.version += 1
            self._cv.notify_all()
        return snap

    def wait_version(self, seen: int, timeout: float) -> int:
        """Block until a fusion cycle newer than `seen` has run (or timeout). Returns the version."""
        with self._cv:
            self._cv.wait_for(lambda: self.version != seen, timeout)
            return self.version

    # -- slow path: once a second ----------------------------------------------
    def housekeep(self, snap: list[dict], now: float | None = None) -> dict:
        """Node hints, status heartbeat to trackers, event log."""
        now = time.time() if now is None else now
        if self.hints_enabled:
            self.hints.step(snap, now)
        st = self.status(snap, now)
        self.trackers.push_status(st)
        self.events.observe(summary(self, now), self.hints.sent)
        self.seq += 1
        return st

    def step(self, now: float | None = None) -> dict:
        """One full cycle (fuse + housekeep). Used by tests and scripts."""
        now = time.time() if now is None else now
        snap = self.fuse(now)
        return {"snap": snap, "status": self.housekeep(snap, now)}

    def run_forever(self, period_s: float = 1.0, on_step=None):
        """Fuse as soon as any sensor reports (capped at 1/FUSE_MIN_S Hz); housekeep every period_s."""
        last_house = float("-inf")
        while True:
            data = self.base.wait_update(FUSE_MIN_S)
            t0 = time.time()
            due = t0 - last_house >= period_s
            if data or self.base.pending or due:
                snap = self.fuse(t0)
                if due:
                    last_house = t0
                    st = self.housekeep(snap, t0)
                    if on_step:
                        on_step({"snap": snap, "status": st})
            time.sleep(max(0.0, FUSE_MIN_S - (time.time() - t0)))
