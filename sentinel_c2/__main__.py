"""sentinel-c2: collection point between the ESP32 acoustic nodes and the Visual-Trackers.

  python -m sentinel_c2 --site sites/x.json --serial /dev/cu.usbserial-XXXX [--tracker 192.168.1.30] [--map] [--cot]
  python -m sentinel_c2 ports           # list serial ports (find the base receiver ESP32)
"""
from __future__ import annotations

import argparse
import sys
import time
from types import SimpleNamespace


def cmd_run(a):
    from cuas.config import Site

    from .hub import Hub

    site = Site.load(a.site)
    hub = Hub(site, a.serial, a.tracker, a.visual_port, a.cue_port,
              broadcast=None if a.no_broadcast else a.broadcast, hints_enabled=not a.no_hints)
    if a.map or a.cot:
        from cuas.__main__ import _outputs
        _outputs(SimpleNamespace(map=a.map, map_port=a.map_port, cot=a.cot), hub.base)
    hub.start()
    print(f"sentinel-c2 up: {len(site.nodes)} nodes, ESP32 {'on ' + a.serial if a.serial else 'off'}, "
          f"trackers {a.tracker or 'auto'}. Ctrl-C to stop.", flush=True)

    def show(out):
        st, snap = out["status"], out["snap"]
        peers = hub.trackers.check_health()
        trk = ", ".join(f"{p.src}{'' if p.up else '(DOWN)'}" for p in peers) or "none yet"
        tracks = " | ".join(f"T{t['id']} ±{t['sigma_m']:.0f}m {'/'.join(t['sources'])}"
                            for t in snap if t["confirmed"]) or "no tracks"
        print(f"{time.strftime('%H:%M:%S')} esp32 {st['esp32']} nodes {st['nodes_online']}/{st['nodes_total']} "
              f"trackers [{trk}]  {tracks}", flush=True)

    try:
        hub.run_forever(1.0, show)
    except KeyboardInterrupt:
        hub.close()


def cmd_ports(_):
    from serial.tools import list_ports
    for p in list_ports.comports():
        print(p.device, "-", p.description)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["ports"]:
        return cmd_ports(None)
    ap = argparse.ArgumentParser(prog="sentinel_c2", description=__doc__.splitlines()[0],
                                 epilog="`python -m sentinel_c2 ports` lists serial ports.")
    ap.add_argument("--site", required=True, help="silent-sentinel site file (node + camera positions)")
    ap.add_argument("--serial", help="base receiver ESP32 port; omit to run trackers-only")
    ap.add_argument("--tracker", action="append", help="Visual-Tracker IP (repeatable; also auto-learned)")
    ap.add_argument("--visual-port", type=int, default=47010)
    ap.add_argument("--cue-port", type=int, default=47011)
    ap.add_argument("--broadcast", default="255.255.255.255", help="where to announce before any tracker is heard")
    ap.add_argument("--no-broadcast", action="store_true")
    ap.add_argument("--no-hints", action="store_true", help="don't send re-acquire hints to the nodes")
    ap.add_argument("--map", action="store_true", help="serve silent-sentinel's live map")
    ap.add_argument("--map-port", type=int, default=8080)
    ap.add_argument("--cot", nargs="?", const="udp://239.2.3.1:6969", help="send CoT to TAK")
    cmd_run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
