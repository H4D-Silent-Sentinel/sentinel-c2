"""Synthetic drone over the example intersection: what the nodes and the camera would report."""
from __future__ import annotations

import math
from pathlib import Path

from cuas.config import Site
from cuas.packet import FLAG_CONFIRMED, Packet, PktType

_UP = Path(__file__).resolve().parents[2]
SS = next((p for p in (_UP / "sentinel", _UP / "silent-sentinel", _UP / "ss") if p.exists()), _UP / "silent-sentinel")
# sentinel checkout next to this repo; override with SILENT_SENTINEL
DRONE_E, DRONE_N, DRONE_ALT, F0 = 60.0, 40.0, 40.0, 200.0


def site_path() -> Path:
    import os
    root = Path(os.environ.get("SILENT_SENTINEL", SS))
    return root / "sites" / "example_intersection.json"


def load_site() -> Site:
    return Site.load(site_path())


def detection(site: Site, nid: int, seq: int, lost: bool = False) -> Packet:
    node = site.nodes[nid]
    r = math.sqrt((node.e - DRONE_E) ** 2 + (node.n - DRONE_N) ** 2 + (DRONE_ALT - node.height_m) ** 2)
    level = site.source_level_db - 20 * math.log10(max(r, 1.0)) - site.atten_db_per_m * r
    if lost:
        return Packet(PktType.LOST, nid, seq=seq, floor_db=40.0)
    return Packet(PktType.DETECTION, nid, seq=seq, flags=FLAG_CONFIRMED, f0_hz=F0, level_db=level,
                  score_db=12, n_harm=6, confidence=80, floor_db=40.0)


def rx_line(pkt: Packet) -> str:
    return f"PKT -70 9.5 {pkt.encode().hex()}\n"


def camera_obs(site: Site) -> dict:
    """Bearing/elevation/range from post-1 to the drone, as a v1 visual track."""
    cam = site.cameras["post-1"]
    ce, cn = site.frame.to_en(cam.lat, cam.lon)
    de, dn = DRONE_E - ce, DRONE_N - cn
    rng = math.hypot(de, dn)
    return {"id": 7, "conf": 0.9, "class": "drone", "az_deg": math.degrees(math.atan2(de, dn)) % 360,
            "el_deg": math.degrees(math.atan2(DRONE_ALT - cam.height_m, rng)), "az_sigma_deg": 0.5,
            "range_m": rng, "range_sigma_m": 15.0}


def visual_msg(t: float, tracks: list[dict]) -> dict:
    return {"v": 1, "type": "visual", "src": "post-1", "t": t,
            "sensor": {"yaw_deg": 0, "pitch_deg": 0, "hfov_deg": 60, "vfov_deg": 45}, "tracks": tracks}
