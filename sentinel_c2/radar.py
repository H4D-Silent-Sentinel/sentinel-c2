"""Radar input for the C2: one JSON object per UDP datagram, same style as the camera link.

  radar --radar--> C2   udp/47012

  {"v": 1, "type": "radar", "src": "radar-1", "t": 1763400000.5,
   "sensor": {"lat": 40.8080, "lon": -73.9640, "alt_m": 10.0},
   "tracks": [{"id": 3, "range_m": 182.4, "az_deg": 71.2, "el_deg": 11.9, "vr_mps": -6.1,
               "snr_db": 17.5, "conf": 0.8, "range_sigma_m": 3.0, "az_sigma_deg": 1.0}]}

range_m is slant range; az true north clockwise; el up +; vr_mps radial velocity (- = closing).
Optional sensor fields for the radar scope: "max_range_m", "scan_s", and "shadows":
[[az_from, az_to, beyond_m], ...] (sectors where line of sight is blocked past a range).
An empty `tracks` list is the radar's heartbeat. A plot reaches the C2 tracker only once the radar
has reported the same track `id` UPDATE_SCANS times in a row, and may start a new C2 track only after
INIT_SCANS in a row with conf >= 0.6. One-off plots (birds, clutter) never reach the tracker: a
coasting track has a wide gate and would otherwise be kept alive by them. Every plot still shows
on the radar scope.
"""
from __future__ import annotations

import json
import math
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

SCHEMA_VERSION = 1
RADAR_PORT = 47012
INIT_CONF = 0.6
INIT_SCANS = 3  # consecutive reports of the same radar track id before it may start a C2 track
UPDATE_SCANS = 2  # ... before it may update one


@dataclass
class RadarObs:
    src: str
    t: float
    sensor_e: float
    sensor_n: float
    sensor_alt: float
    track_id: int | None
    range_m: float
    az_deg: float
    el_deg: float
    vr_mps: float | None
    conf: float
    range_sigma_m: float
    az_sigma_deg: float

    def fix(self):
        """(e, n), 2x2 covariance, altitude: the plot as a position measurement."""
        el = math.radians(self.el_deg)
        gr = self.range_m * math.cos(el)
        a = math.radians(self.az_deg)
        e, n = self.sensor_e + gr * math.sin(a), self.sensor_n + gr * math.cos(a)
        u_r = np.array([math.sin(a), math.cos(a)])
        u_t = np.array([math.cos(a), -math.sin(a)])
        ts = gr * math.radians(self.az_sigma_deg)
        R = self.range_sigma_m ** 2 * np.outer(u_r, u_r) + ts ** 2 * np.outer(u_t, u_t)
        return (e, n), R, self.sensor_alt + self.range_m * math.sin(el)


def make_radar(src: str, t: float, sensor: dict, tracks: list[dict]) -> bytes:
    return json.dumps({"v": SCHEMA_VERSION, "type": "radar", "src": src, "t": t, "sensor": sensor,
                       "tracks": tracks}, separators=(",", ":")).encode()


def parse_radar(raw: bytes | str, frame) -> list[RadarObs]:
    m = json.loads(raw)
    if not isinstance(m, dict) or m.get("v") != SCHEMA_VERSION or m.get("type") != "radar":
        raise ValueError("not a v1 radar message")
    s = m.get("sensor") or {}
    if s.get("lat") is None or s.get("lon") is None:
        raise ValueError(f"no position for radar {m.get('src')}")
    se, sn = frame.to_en(float(s["lat"]), float(s["lon"]))
    out = []
    for tr in m.get("tracks") or []:
        out.append(RadarObs(
            src=str(m.get("src", "radar")), t=float(m.get("t", time.time())), sensor_e=se, sensor_n=sn,
            sensor_alt=float(s.get("alt_m", 0.0)), track_id=tr.get("id"), range_m=float(tr["range_m"]),
            az_deg=float(tr["az_deg"]) % 360, el_deg=float(tr.get("el_deg", 0.0)),
            vr_mps=tr.get("vr_mps"), conf=float(tr.get("conf", 0.5)),
            range_sigma_m=float(tr.get("range_sigma_m", 5.0)), az_sigma_deg=float(tr.get("az_sigma_deg", 1.5))))
    return out


class RadarLink:
    """Listens for radar reports and feeds them into the Base tracker as position fixes."""

    def __init__(self, base, port: int = RADAR_PORT, timeout_s: float = 3.0, log=print):
        self.base, self.timeout_s, self.log = base, timeout_s, log
        self.radars: dict[str, dict] = {}  # src -> {"ip", "last_heard", "msgs", "plots", "up"}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.gate = None  # optional: () -> bool; False keeps radar out of fusion (operator mute)
        self._streak: dict[str, dict] = {}  # src -> {radar track id: consecutive reports}
        self.plots: dict[str, deque] = {}  # src -> recent plots, for the radar scope
        self.coverage: dict[str, dict] = {}  # src -> max_range_m / scan_s / shadows, if the radar sends them
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", port))
        self.rx.settimeout(0.5)
        self.port = self.rx.getsockname()[1]
        self._thread = threading.Thread(target=self._listen, daemon=True, name="radar")

    def start(self) -> "RadarLink":
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        self.rx.close()

    def _listen(self):
        self.log(f"radar: listening on udp/{self.port}")
        while not self._stop.is_set():
            try:
                data, addr = self.rx.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.on_datagram(data, addr[0])

    def on_datagram(self, data: bytes, ip: str, now: float | None = None) -> int:
        now = time.time() if now is None else now
        try:
            obs = parse_radar(data, self.base.site.frame)
            m = json.loads(data)
            src = m.get("src", "radar")
            e, n = self.base.site.frame.to_en(float(m["sensor"]["lat"]), float(m["sensor"]["lon"]))
        except (ValueError, KeyError, TypeError) as e:
            self.log(f"radar: bad message from {ip}: {e}")
            return 0
        with self._lock:
            r = self.radars.setdefault(src, {"ip": ip, "last_heard": 0.0, "msgs": 0, "plots": 0, "up": False})
            r.update(ip=ip, last_heard=now, msgs=r["msgs"] + 1, plots=len(obs), e=e, n=n,
                     alt=float(m["sensor"].get("alt_m", 0.0)))
            sen = m["sensor"]
            self.coverage[src] = {k: sen[k] for k in ("max_range_m", "scan_s", "shadows") if k in sen}
            q = self.plots.setdefault(src, deque(maxlen=400))
            for o in obs:
                q.append(dict(t=now, id=o.track_id, az=o.az_deg, range_m=o.range_m, el=o.el_deg,
                              gr=o.range_m * math.cos(math.radians(o.el_deg)), conf=o.conf, vr=o.vr_mps))
        prev = self._streak.get(src, {})
        streak = {o.track_id: prev.get(o.track_id, 0) + 1 for o in obs if o.track_id is not None}
        self._streak[src] = streak  # ids missing from this report start over
        with self.base.lock:
            for o in (obs if self.gate is None or self.gate() else []):
                n_seen = streak.get(o.track_id, 0)
                if n_seen < UPDATE_SCANS:
                    continue
                z, R, alt = o.fix()
                firm = o.conf >= INIT_CONF and n_seen >= INIT_SCANS
                self.base.tracker.update_position(z, R, o.t, source=f"radar:{o.src}", alt_m=alt, can_init=firm)
        self.base.mark_updated()  # fuse now, not on the next timer tick
        return len(obs)

    def check_health(self, now: float | None = None) -> dict[str, dict]:
        now = time.time() if now is None else now
        with self._lock:
            for src, r in self.radars.items():
                up = now - r["last_heard"] < self.timeout_s
                if up != r["up"]:
                    r["up"] = up
                    self.log(f"radar: {src} @ {r['ip']} " + ("UP" if up else f"DOWN (silent {now - r['last_heard']:.0f} s)"))
            return {k: dict(v) for k, v in self.radars.items()}
