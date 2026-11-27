"""One-command demo of the whole system with no hardware: the real C2 and dashboard, fed by
simulated sensors that all watch the same simulated drone.

  python -m sentinel_c2.demo                # opens the dashboard; Ctrl-C to stop

  - acoustic nodes  -> a fake ESP32 base receiver on a virtual serial port (real PKT lines,
                       answers the C2's TX/HINT lines like the firmware does)
  - camera post-1   -> real v1 `visual` reports over UDP, including heartbeats
  - radar-1         -> the fake radar over UDP (with a building shadow)
  - chaser-1        -> a moving camera (like the phone in chaser mode): circles the site, reports its
                       own GPS position and heading every second, and points at the drone
The node and camera models are sentinel's network simulator (cuas.sim.scenario).
Everything is software; nothing is transmitted over the air.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pty
import socket
import threading
import time
from pathlib import Path

import numpy as np


def _default_site() -> str | None:
    here = Path(__file__).resolve().parents[2]
    for p in (here / "sentinel", here / "silent-sentinel", here / "ss"):
        f = p / "sites" / "example_intersection.json"
        if f.exists():
            return str(f)
    return None


class FakeReceiver:
    """The base receiver ESP32, on a pty: writes PKT lines, answers TX lines with TXOK."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.hints = 0
        threading.Thread(target=self._rx, daemon=True, name="fake-esp32").start()

    def _rx(self):
        buf = b""
        while True:
            try:
                buf += os.read(self.master, 1024)
            except OSError:
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.startswith(b"TX "):
                    self.hints += 1
                    self.write(f"TXOK {len(line.strip()) // 2 - 1}\n")

    def write(self, s: str):
        try:
            os.write(self.master, s.encode())
        except OSError:
            pass


def run_sensors(site, rx: FakeReceiver, visual_port: int, radar_port: int, shadows, radar_at, loop: bool,
                stop: threading.Event, seed: int = 0):
    from cuas.sim.scenario import DEFAULT_PATH, DroneSim, NetworkSim

    from .fake_radar import FakeRadar, RadarModel, _parse_shadow
    from .radar import make_radar

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cams = [cid for cid, c in site.cameras.items() if c.lat is not None]
    e, n = radar_at
    lat, lon = site.frame.to_ll(e, n)
    model = RadarModel(e=e, n=n, shadows=[_parse_shadow(s) for s in shadows])
    rsensor = {"lat": lat, "lon": lon, "alt_m": model.alt_m, "max_range_m": model.max_range_m, "scan_s": 0.5,
               "shadows": [[s.az_from, s.az_to, s.beyond_m] for s in model.shadows]}
    run = 0
    while not stop.is_set():
        drone = DroneSim(DEFAULT_PATH, speed=8.0)
        sim = NetworkSim(site, [drone], seed=seed + run)
        radar = FakeRadar(model, [DroneSim(DEFAULT_PATH, speed=8.0)], np.random.default_rng(seed + run))
        t0 = time.time()
        next_scan = next_cam_hb = 0.0
        cam_seen: dict[str, float] = {}

        def pace(t_sim):
            while not stop.is_set() and time.time() < t0 + t_sim:
                nonlocal next_scan, next_cam_hb
                now_sim = time.time() - t0
                if now_sim >= next_scan:  # radar scans every 0.5 s
                    plots, _ = radar.scan(next_scan, 0.5)
                    sock.sendto(make_radar("radar-1", time.time(), rsensor, plots), ("127.0.0.1", radar_port))
                    next_scan += 0.5
                if now_sim >= next_cam_hb:  # chaser-1: a moving camera that reports its own pose
                    ang = now_sim / 40 * 2 * math.pi
                    ce, cn = 90 * math.cos(ang), 90 * math.sin(ang)
                    dp = drone.pos(now_sim)
                    yaw = (math.degrees(math.atan2(dp[0] - ce, dp[1] - cn)) % 360) if dp else math.degrees(ang) % 360
                    clat, clon = site.frame.to_ll(ce, cn)
                    tracks = []
                    if dp and math.hypot(dp[0] - ce, dp[1] - cn) < 150:
                        rng_h = math.hypot(dp[0] - ce, dp[1] - cn)
                        tracks.append({"id": 1, "az_deg": round(yaw + float(np.random.normal(0, 1.0)), 2),
                                       "el_deg": round(math.degrees(math.atan2(drone.alt - 2, rng_h)), 2),
                                       "az_sigma_deg": 1.0, "conf": 0.7, "class": "drone"})
                    msg = {"v": 1, "type": "visual", "src": "chaser-1", "t": time.time(),
                           "sensor": {"lat": clat, "lon": clon, "alt_m": 2.0, "yaw_deg": round(yaw, 1),
                                      "pitch_deg": 15.0, "hfov_deg": 65.0, "vfov_deg": 50.0}, "tracks": tracks}
                    sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", visual_port))
                if now_sim >= next_cam_hb:  # camera heartbeat when it has nothing to report
                    for cid in cams:
                        if time.time() - cam_seen.get(cid, 0) > 0.9:
                            hb = {"v": 1, "type": "visual", "src": cid, "t": time.time(),
                                  "sensor": {"yaw_deg": 0, "pitch_deg": 0, "hfov_deg": 60, "vfov_deg": 45}, "tracks": []}
                            sock.sendto(json.dumps(hb).encode(), ("127.0.0.1", visual_port))
                    next_cam_hb += 1.0
                time.sleep(0.02)

        for t, kind, payload, _ in sim.events(dt=0.1):
            pace(t)
            if stop.is_set():
                return
            if kind == "lora":
                rx.write(f"PKT {payload.rssi:.0f} 9.0 {payload.encode().hex()}\n")
            else:
                m = json.loads(payload)
                m["t"] = time.time()  # the sim stamps its own clock; a real camera stamps the wall clock
                cam_seen[m["src"]] = time.time()
                sock.sendto(json.dumps(m).encode(), ("127.0.0.1", visual_port))
        run += 1
        if not loop:
            return


def main(argv=None):
    from cuas.config import Site

    from .__main__ import open_window
    from .hub import Hub
    from .ui import serve

    ap = argparse.ArgumentParser(prog="sentinel_c2.demo", description=__doc__.splitlines()[0])
    ap.add_argument("--site", default=_default_site(), help="sentinel site file")
    ap.add_argument("--ui-port", type=int, default=8090)
    ap.add_argument("--no-open", action="store_true", help="don't open a window")
    ap.add_argument("--shadow", action="append", default=None, metavar="AZ_FROM:AZ_TO:BEYOND_M",
                    help="radar building shadow (default 270:340:120)")
    ap.add_argument("--radar-at", default="0,-120", help="radar position e,n in metres")
    ap.add_argument("--once", action="store_true", help="fly the path once instead of looping")
    a = ap.parse_args(argv)
    if not a.site:
        ap.error("--site is required (couldn't find ../sentinel/sites/example_intersection.json)")

    site = Site.load(a.site)
    rx = FakeReceiver()
    hub = Hub(site, serial_port=rx.path, broadcast=None, log=lambda m: print(time.strftime("%H:%M:%S"), m, flush=True))
    hub.start()
    serve(hub, port=a.ui_port)
    stop = threading.Event()
    threading.Thread(target=run_sensors, daemon=True, name="sim", args=(
        site, rx, hub.trackers.visual_port, hub.radar.port, a.shadow or ["270:340:120"],
        tuple(float(v) for v in a.radar_at.split(",")), not a.once, stop)).start()
    url = f"http://127.0.0.1:{a.ui_port}"
    print(f"demo running: dashboard {url}  ·  radar scope {url}/radar  ·  Ctrl-C to stop", flush=True)
    if not a.no_open:
        open_window(url)
    try:
        hub.run_forever(1.0)
    except KeyboardInterrupt:
        stop.set()
        hub.close()


if __name__ == "__main__":
    main()
