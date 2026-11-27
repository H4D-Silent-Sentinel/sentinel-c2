"""ROAREE (Silent Sentinel C2): collection point between the ESP32 acoustic nodes and the Visual-Trackers.

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
              broadcast=None if a.no_broadcast else a.broadcast, hints_enabled=not a.no_hints,
              radar_port=None if a.no_radar else a.radar_port,
              operator_port=None if a.no_operator else a.operator_port)
    if a.map:
        from cuas.__main__ import _outputs
        _outputs(SimpleNamespace(map=a.map, map_port=a.map_port, cot=None), hub.base)
    if a.cot:  # CoT to TAK; the hand controller's TAK switch turns it off and on
        from cuas.cot import CotSender
        cot, op, frame = CotSender(a.cot), hub.operator, site.frame
        hub.base.on_track.append(lambda tr: op.tak_out and tr["confirmed"] and cot.track(tr, frame))
        hub.base.on_node.append(lambda node, s: op.tak_out and node and cot.node(node, s))
        print(f"cot: sending to {a.cot}")
    hub.start()
    if a.ui:
        from .ui import TrackerControl, _find_visual_tracker, serve
        serve(hub, host=a.ui_host, port=a.ui_port,
              tracker=TrackerControl(a.visual_tracker or _find_visual_tracker(), url=a.tracker_url))
        if a.open:
            open_window(f"http://127.0.0.1:{a.ui_port}")
    print(f"ROAREE up: {len(site.nodes)} nodes, ESP32 {'on ' + a.serial if a.serial else 'off'}, "
          f"trackers {a.tracker or 'auto'}. Ctrl-C to stop.", flush=True)

    def show(out):
        st, snap = out["status"], out["snap"]
        peers = hub.trackers.check_health()
        trk = ", ".join(f"{p.src}{'' if p.up else '(DOWN)'}" for p in peers) or "none yet"
        tracks = " | ".join(f"T{t['id']} ±{t['sigma_m']:.0f}m {'/'.join(t['sources'])}"
                            for t in snap if t["confirmed"]) or "no tracks"
        rad = ", ".join(f"{k}{'' if up else '(DOWN)'}" for k, up in st["radars"].items()) or "none"
        print(f"{time.strftime('%H:%M:%S')} esp32 {st['esp32']} nodes {st['nodes_online']}/{st['nodes_total']} "
              f"trackers [{trk}] radars [{rad}]  {tracks}", flush=True)

    try:
        hub.run_forever(1.0, show)
    except KeyboardInterrupt:
        hub.close()


def open_window(url: str):
    """Open the dashboard in its own app-style window (Chrome/Edge --app), else the default browser."""
    import shutil
    import subprocess
    import webbrowser
    mac = ["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
           "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge"]
    for exe in mac + [shutil.which(n) for n in ("google-chrome", "chromium", "chromium-browser", "msedge")]:
        if exe and shutil.os.path.exists(exe):
            subprocess.Popen([exe, f"--app={url}", "--window-size=1440,900"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
    webbrowser.open(url)


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
    ap.add_argument("--site", required=True, help="sentinel site file (node + camera positions)")
    ap.add_argument("--serial", help="base receiver ESP32 port; omit to run trackers-only")
    ap.add_argument("--tracker", action="append", help="Visual-Tracker IP (repeatable; also auto-learned)")
    ap.add_argument("--visual-port", type=int, default=47010)
    ap.add_argument("--cue-port", type=int, default=47011)
    ap.add_argument("--broadcast", default="255.255.255.255", help="where to announce before any tracker is heard")
    ap.add_argument("--no-broadcast", action="store_true")
    ap.add_argument("--no-hints", action="store_true", help="don't send re-acquire hints to the nodes")
    ap.add_argument("--radar-port", type=int, default=47012, help="where radar reports arrive")
    ap.add_argument("--no-radar", action="store_true")
    ap.add_argument("--operator-port", type=int, default=47013, help="where the tango hand controller sends")
    ap.add_argument("--no-operator", action="store_true")
    ap.add_argument("--ui", action="store_true", help="serve the operator dashboard")
    ap.add_argument("--ui-port", type=int, default=8090)
    ap.add_argument("--ui-host", default="127.0.0.1", help="0.0.0.0 to view the dashboard from other machines")
    ap.add_argument("--open", action="store_true", help="with --ui: open the dashboard in its own window")
    ap.add_argument("--visual-tracker", help="Visual-Tracker folder, for the Camera tab's Start button "
                                             "(default: ../Visual-Tracker next to this repo)")
    ap.add_argument("--tracker-url", default="http://127.0.0.1:8001", help="Visual-Tracker dashboard address")
    ap.add_argument("--map", action="store_true", help="serve sentinel's live map")
    ap.add_argument("--map-port", type=int, default=8080)
    ap.add_argument("--cot", nargs="?", const="udp://239.2.3.1:6969", help="send CoT to TAK")
    cmd_run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
