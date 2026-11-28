"""Radar station over LoRa (sentinel firmware/radar_node) into the radar input.

  RD-03D -> radar_node ESP32 --LoRa, RadarPacket--> base_rx --USB serial--> ROAREE

The radar station's position, boresight and range come from the site file's "radars" list,
keyed by radar_id. The RD-03D reports up to 3 targets without IDs; targets are associated
packet to packet by nearest neighbour within GATE_M so the radar input sees a stable track id.
Each packet becomes one `radar` message (sentinel_c2/radar.py) with src "radar-<id>".
"""
from __future__ import annotations

import itertools
import math
import time

from cuas.packet import RadarPacket, RadarPlot

from .radar import make_radar

GATE_M = 2.5  # max movement between packets (5 per second) for a target to keep its id
CONF = 0.8
RANGE_SIGMA_M = 0.15
AZ_SIGMA_DEG = 3.0


class Associator:
    """Assigns persistent ids to unlabelled targets by nearest neighbour."""

    def __init__(self, gate_m: float = GATE_M):
        self.gate_m = gate_m
        self.prev: dict[int, RadarPlot] = {}
        self._ids = itertools.count(1)

    def update(self, targets: list[RadarPlot]) -> list[tuple[int, RadarPlot]]:
        pairs = sorted((math.hypot(t.x_m - p.x_m, t.y_m - p.y_m), i, tid)
                       for i, t in enumerate(targets) for tid, p in self.prev.items())
        taken_t, taken_id, ids = set(), set(), {}
        for d, i, tid in pairs:
            if d <= self.gate_m and i not in taken_t and tid not in taken_id:
                ids[i] = tid
                taken_t.add(i)
                taken_id.add(tid)
        out = [(ids[i] if i in ids else next(self._ids), t) for i, t in enumerate(targets)]
        self.prev = {tid: t for tid, t in out}
        return out


def to_message(radar, pkt: RadarPacket, ids: list[tuple[int, RadarPlot]], t: float) -> bytes:
    tracks = []
    for tid, p in ids:
        off = math.degrees(math.atan2(-p.x_m if radar.flip_x else p.x_m, p.y_m))
        tracks.append({"id": tid, "range_m": round(math.hypot(p.x_m, p.y_m), 3),
                       "az_deg": round((radar.yaw_deg + off) % 360, 2), "el_deg": 0.0,
                       "vr_mps": p.speed_mps, "conf": CONF, "range_sigma_m": RANGE_SIGMA_M,
                       "az_sigma_deg": AZ_SIGMA_DEG})
    sensor = {"lat": radar.lat, "lon": radar.lon, "alt_m": radar.height_m, "max_range_m": radar.max_range_m}
    return make_radar(f"radar-{pkt.radar_id}", t, sensor, tracks)


class LoRaRadar:
    def __init__(self, radar_link, site, log=print):
        self.link, self.site, self.log = radar_link, site, log
        self.assoc: dict[int, Associator] = {}
        self._unknown: set[int] = set()

    def on_packet(self, pkt: RadarPacket, now: float | None = None) -> None:
        now = time.time() if now is None else now
        radar = self.site.radars.get(pkt.radar_id)
        if radar is None:
            if pkt.radar_id not in self._unknown:
                self._unknown.add(pkt.radar_id)
                self.log(f"radar: LoRa radar {pkt.radar_id} is not in the site file's \"radars\"; ignored")
            return
        ids = self.assoc.setdefault(pkt.radar_id, Associator()).update(pkt.targets)
        self.link.on_datagram(to_message(radar, pkt, ids, now), "lora", now)
