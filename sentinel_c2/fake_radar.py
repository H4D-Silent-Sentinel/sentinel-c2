"""Simulated radar: software only, transmits nothing. Sends v1 `radar` messages to the C2.

  python -m sentinel_c2.fake_radar --site sites/example_intersection.json --c2 127.0.0.1
  python -m sentinel_c2.fake_radar --site ... --at 0,-120 --shadow 270:340:120 --loop

Runs in real time: reports carry the wall clock, as a real radar's would, so the C2's tracker
sees true drone speeds. (Speeding the sim up would make the drone look 8x faster to the C2.)

The drone flies silent-sentinel's simulated path (cuas.sim.scenario.DEFAULT_PATH, or --path).
What it models, roughly, so the C2 sees realistic gaps and junk:
  - detection probability from a radar-equation SNR: falls off as 1/R^4, small drone RCS
  - measurement noise in range, azimuth, elevation; radial (Doppler) velocity
  - building shadows: azimuth sectors where anything beyond a range is hidden (--shadow)
  - false alarms (birds, clutter) at a Poisson rate per scan, mostly low SNR
  - its own short-lived track IDs, like a real radar's plot output
Treat the numbers as a stand-in to exercise the pipeline, not a prediction of any real radar.
"""
from __future__ import annotations

import argparse
import math
import socket
import time
from dataclasses import dataclass, field

import numpy as np

from .radar import RADAR_PORT, make_radar


@dataclass
class Shadow:
    az_from: float
    az_to: float
    beyond_m: float

    def hides(self, az: float, ground_range: float) -> bool:
        a, b = self.az_from % 360, self.az_to % 360
        inside = a <= az <= b if a <= b else (az >= a or az <= b)
        return inside and ground_range > self.beyond_m


@dataclass
class RadarModel:
    e: float = 0.0  # radar position in the site frame, m
    n: float = 0.0
    alt_m: float = 10.0
    snr_ref_db: float = 20.0  # SNR of a -20 dBsm target at ref_m
    ref_m: float = 300.0
    rcs_dbsm: float = -20.0  # small quadcopter
    snr_thresh_db: float = 12.0
    max_range_m: float = 1500.0
    range_sigma_m: float = 3.0
    az_sigma_deg: float = 1.0
    el_sigma_deg: float = 1.5
    false_alarms_per_scan: float = 0.3
    shadows: list[Shadow] = field(default_factory=list)

    def snr_db(self, slant_m: float) -> float:
        return self.snr_ref_db + 40 * math.log10(self.ref_m / max(slant_m, 1.0)) + (self.rcs_dbsm + 20.0)

    def p_detect(self, snr: float) -> float:
        return 1 / (1 + math.exp(-(snr - self.snr_thresh_db) / 1.5))


class FakeRadar:
    def __init__(self, model: RadarModel, drones, rng=None):
        self.m, self.drones = model, drones
        self.rng = rng or np.random.default_rng(0)
        self._ids: dict[int, int] = {}  # drone index -> current radar track id
        self._miss: dict[int, int] = {}
        self._next_id = 1

    def _tid(self, k: int, seen: bool) -> int | None:
        if not seen:
            self._miss[k] = self._miss.get(k, 0) + 1
            if self._miss[k] >= 3:  # radar drops its track after 3 missed scans
                self._ids.pop(k, None)
            return None
        self._miss[k] = 0
        if k not in self._ids:
            self._ids[k], self._next_id = self._next_id, self._next_id + 1
        return self._ids[k]

    def scan(self, t: float, dt: float) -> tuple[list[dict], list[dict]]:
        """One scan at sim time t. Returns (plots, truth)."""
        m, rng = self.m, self.rng
        plots, truth = [], []
        for k, d in enumerate(self.drones):
            p0, p1 = d.pos(t), d.pos(t + dt)
            if p0 is None:
                self._tid(k, False)
                continue
            de, dn, du = p0[0] - m.e, p0[1] - m.n, d.alt - m.alt_m
            gr = math.hypot(de, dn)
            slant = math.sqrt(gr * gr + du * du)
            az = math.degrees(math.atan2(de, dn)) % 360
            hidden = any(s.hides(az, gr) for s in m.shadows)
            snr = m.snr_db(slant)
            seen = (not hidden and slant <= m.max_range_m and rng.random() < m.p_detect(snr))
            truth.append({"drone": k, "e": p0[0], "n": p0[1], "range_m": slant, "az_deg": az,
                          "hidden": hidden, "snr_db": snr, "seen": seen})
            tid = self._tid(k, seen)
            if not seen:
                continue
            vr = 0.0
            if p1 is not None:
                vr = (math.dist((m.e, m.n), p1) - gr) / dt
            s = snr + rng.normal(0, 1.5)
            plots.append({"id": tid, "range_m": round(slant + rng.normal(0, m.range_sigma_m), 1),
                          "az_deg": round((az + rng.normal(0, m.az_sigma_deg)) % 360, 2),
                          "el_deg": round(math.degrees(math.atan2(du, gr)) + rng.normal(0, m.el_sigma_deg), 2),
                          "vr_mps": round(vr + rng.normal(0, 0.3), 2), "snr_db": round(s, 1),
                          "conf": round(min(1.0, max(0.0, (s - m.snr_thresh_db + 6) / 12)), 2),
                          "range_sigma_m": m.range_sigma_m, "az_sigma_deg": m.az_sigma_deg})
        for _ in range(rng.poisson(m.false_alarms_per_scan)):  # birds / clutter / multipath
            r = rng.uniform(50, min(m.max_range_m, 800))
            s = m.snr_thresh_db + rng.exponential(2.0)
            plots.append({"id": 10000 + int(rng.integers(0, 9999)), "range_m": round(r, 1),
                          "az_deg": round(rng.uniform(0, 360), 2), "el_deg": round(rng.uniform(0, 15), 2),
                          "vr_mps": round(rng.normal(0, 4), 2), "snr_db": round(s, 1),
                          "conf": round(min(1.0, (s - m.snr_thresh_db + 6) / 12), 2),
                          "range_sigma_m": m.range_sigma_m, "az_sigma_deg": m.az_sigma_deg})
        return plots, truth


def _parse_shadow(s: str) -> Shadow:
    a, b, r = (float(x) for x in s.split(":"))
    return Shadow(a, b, r)


def main(argv=None):
    from cuas.config import Site
    from cuas.sim.scenario import DEFAULT_PATH, DroneSim

    ap = argparse.ArgumentParser(prog="sentinel_c2.fake_radar", description=__doc__.splitlines()[0])
    ap.add_argument("--site", required=True, help="silent-sentinel site file (for the local frame)")
    ap.add_argument("--c2", default="127.0.0.1", help="C2 address")
    ap.add_argument("--port", type=int, default=RADAR_PORT)
    ap.add_argument("--src", default="radar-1")
    ap.add_argument("--at", default="0,-120", help="radar position e,n in metres from the site origin")
    ap.add_argument("--alt", type=float, default=10.0, help="radar height, m")
    ap.add_argument("--path", help="drone waypoints e,n;e,n;... (default: silent-sentinel's DEFAULT_PATH)")
    ap.add_argument("--drone-speed", type=float, default=8.0)
    ap.add_argument("--drone-alt", type=float, default=40.0)
    ap.add_argument("--scan", type=float, default=0.5, help="seconds per scan")
    ap.add_argument("--shadow", action="append", default=[], metavar="AZ_FROM:AZ_TO:BEYOND_M",
                    help="building shadow, repeatable, e.g. 20:110:150")
    ap.add_argument("--false-alarms", type=float, default=0.3, help="mean false plots per scan")
    ap.add_argument("--loop", action="store_true", help="fly the path again when it ends")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)

    site = Site.load(a.site)
    e, n = (float(x) for x in a.at.split(","))
    lat, lon = site.frame.to_ll(e, n)
    path = DEFAULT_PATH if not a.path else [tuple(float(v) for v in p.split(",")) for p in a.path.split(";")]
    drone = DroneSim(path, speed=a.drone_speed, alt=a.drone_alt)
    model = RadarModel(e=e, n=n, alt_m=a.alt, shadows=[_parse_shadow(s) for s in a.shadow],
                       false_alarms_per_scan=a.false_alarms)
    radar = FakeRadar(model, [drone], np.random.default_rng(a.seed))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor = {"lat": round(lat, 7), "lon": round(lon, 7), "alt_m": a.alt}
    print(f"fake radar {a.src} at ({e:.0f}, {n:.0f}) m -> {a.c2}:{a.port}; drone path {drone.duration:.0f} s. "
          "Simulation only, nothing is transmitted over the air. Ctrl-C to stop.", flush=True)
    t_sim, t0 = 0.0, time.time()
    try:
        while True:
            if t_sim > drone.duration + 5:
                if not a.loop:
                    break
                t_sim, t0 = 0.0, time.time()
            plots, truth = radar.scan(t_sim, a.scan)
            sock.sendto(make_radar(a.src, time.time(), sensor, plots), (a.c2, a.port))
            tr = truth[0] if truth else None
            state = ("no drone" if tr is None else "HIDDEN (shadow)" if tr["hidden"] else
                     f"seen r={tr['range_m']:.0f} m az={tr['az_deg']:.0f}" if tr["seen"] else
                     f"missed (snr {tr['snr_db']:.0f} dB)")
            print(f"t={t_sim:6.1f}s  {len(plots)} plot(s)  drone: {state}", flush=True)
            t_sim += a.scan
            time.sleep(max(0.0, t0 + t_sim - time.time()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
