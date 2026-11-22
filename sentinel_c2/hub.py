"""The C2 hub: collects from the ESP32 node network and the Visual-Trackers, fuses
(using silent-sentinel's Base), and keeps both sides updated continuously.

  ESP32 nodes ──LoRa──▶ base_rx ESP32 ──USB──▶ ┌──────────────┐ ──cue/status──▶ Visual-Tracker(s)
              ◀──LoRa── (HINT downlink) ◀──USB─ │  sentinel-c2 │ ◀──visual──────
                                                └──────────────┘ ──▶ map / TAK
"""
from __future__ import annotations

import time

from cuas.base import NODE_STALE_S, Base
from cuas.config import Site

from .esp32 import Esp32Link
from .hints import HintConfig, HintPolicy
from .tracker_link import TrackerLink


class Hub:
    def __init__(self, site: Site, serial_port: str | None = None, trackers: list[str] | None = None,
                 visual_port: int = 47010, cue_port: int = 47011, broadcast: str | None = "255.255.255.255",
                 hint_cfg: HintConfig | None = None, hints_enabled: bool = True, log=print):
        self.log = log
        self.base = Base(site)
        self.esp = Esp32Link(serial_port, self.base, log=log) if serial_port else None
        self.trackers = TrackerLink(self.base, visual_port, cue_port, static=trackers,
                                    broadcast=broadcast, log=log)
        self.hints = HintPolicy(self.base, self._send_hint, hint_cfg, log=log)
        self.hints_enabled = hints_enabled
        self.seq = 0

    def _send_hint(self, pkt) -> bool:
        return self.esp.send_packet(pkt) if self.esp else False

    def start(self) -> "Hub":
        if self.esp:
            self.esp.start()
        self.trackers.start()
        return self

    def close(self):
        if self.esp:
            self.esp.close()
        self.trackers.close()

    def status(self, snap: list[dict], now: float) -> dict:
        with self.base.lock:
            online = sum(1 for ns in self.base.nodes.values() if ns.last_seen and now - ns.last_seen < NODE_STALE_S)
            detecting = sorted(nid for nid, ns in self.base.nodes.items() if ns.detecting)
        return {"src": "sentinel-c2", "t": now, "seq": self.seq,
                "esp32": self.esp.state if self.esp else "off",
                "nodes_online": online, "nodes_total": len(self.base.site.nodes), "nodes_detecting": detecting,
                "tracks": len(snap), "confirmed": sum(1 for t in snap if t["confirmed"]),
                "hints_sent": len(self.hints.sent)}

    def step(self, now: float | None = None) -> dict:
        """One cycle: fuse, push cues, hint nodes, heartbeat everyone. Call ~1 Hz."""
        now = time.time() if now is None else now
        snap = self.base.step(now)
        self.trackers.check_health(now)
        self.trackers.push_cues(snap, now)
        if self.hints_enabled:
            self.hints.step(snap, now)
        st = self.status(snap, now)
        self.trackers.push_status(st)
        self.seq += 1
        return {"snap": snap, "status": st}

    def run_forever(self, period_s: float = 1.0, on_step=None):
        while True:
            t0 = time.time()
            out = self.step(t0)
            if on_step:
                on_step(out)
            time.sleep(max(0.0, period_s - (time.time() - t0)))
