"""Re-acquire hints: keep the ESP32 nodes on a drone the camera still has.

When a confirmed track is still being updated by a Visual-Tracker or a radar, and a node that
heard that drone (or sits close to where it is now) has dropped it, the C2 sends that
node a HINT over LoRa: "look for a rotor comb at f0 for the next hold_s seconds".
The node's detector then re-confirms on fewer hits near that f0 (see silent-sentinel
cuas/detector.py set_hint). Hints repeat every resend_s while the condition holds and
expire on the node by themselves, so a lost downlink packet is harmless.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from cuas.base import NODE_STALE_S
from cuas.packet import make_hint


@dataclass
class HintConfig:
    visual_fresh_s: float = 3.0  # a camera/radar must have updated the track this recently
    vouch_sources: tuple = ("visual:", "radar:")  # sensors that count as "still has it"
    hold_s: float = 10.0  # how long each hint stays active on the node
    resend_s: float = 4.0  # per-node repeat interval while the node stays lost
    memory_s: float = 120.0  # a node "heard this drone" if it reported this f0 within this window
    radius_m: float = 300.0  # nodes this close to the track are hinted even if they never heard it
    f0_tol: float = 0.10  # relative f0 match for "same drone"


@dataclass
class HintSent:
    t: float
    node_id: int
    track_id: int
    f0_hz: float
    reason: str  # "lost" (node had it) or "near" (track is close to the node)
    delivered: bool


class HintPolicy:
    def __init__(self, base, send, cfg: HintConfig | None = None, log=print):
        self.base, self.send, self.cfg, self.log = base, send, cfg or HintConfig(), log
        self.seq = 0
        self.sent: list[HintSent] = []
        self._last: dict[int, float] = {}

    def step(self, snapshot: list[dict], now: float | None = None) -> list[HintSent]:
        now = time.time() if now is None else now
        c, base = self.cfg, self.base
        best: dict[int, tuple[float, dict, str]] = {}  # node -> (distance, track, reason)
        with base.lock:
            for tr in snapshot:
                live = base.tracker.tracks.get(tr["id"])
                if not tr["confirmed"] or not tr["f0"] or live is None:
                    continue
                vis_t = max((t for s, t in live.sources.items() if s.startswith(c.vouch_sources)), default=None)
                if vis_t is None or now - vis_t > c.visual_fresh_s:
                    continue  # no camera/radar is (still) seeing it: nothing to vouch for
                f0 = tr["f0"]
                for nid, node in base.site.nodes.items():
                    ns = base.nodes.get(nid)
                    if ns is None or ns.detecting or not (ns.last_seen and now - ns.last_seen < NODE_STALE_S):
                        continue
                    d = math.hypot(node.e - tr["e"], node.n - tr["n"])
                    det = ns.last_det
                    heard = det is not None and now - det.t < c.memory_s and abs(det.f0 - f0) <= c.f0_tol * f0
                    if not heard and d > c.radius_m:
                        continue
                    if nid not in best or d < best[nid][0]:
                        best[nid] = (d, tr, "lost" if heard else "near")
        out = []
        for nid, (d, tr, reason) in sorted(best.items()):
            if now - self._last.get(nid, -1e9) < c.resend_s:
                continue
            self._last[nid] = now
            conf = max(0, min(100, int(100 - tr["sigma_m"])))
            pkt = make_hint(nid, tr["f0"], c.hold_s, self.seq, conf)
            self.seq = (self.seq + 1) & 0xFFFF
            ok = bool(self.send(pkt))
            h = HintSent(now, nid, tr["id"], tr["f0"], reason, ok)
            out.append(h)
            self.log(f"hint: node {nid} <- T{tr['id']} f0 {tr['f0']:.1f} Hz ({reason}, {d:.0f} m)"
                     + ("" if ok else " NOT SENT (ESP32 link down)"))
        self.sent.extend(out)
        del self.sent[:-200]
        return out
