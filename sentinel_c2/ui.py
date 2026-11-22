"""Operator dashboard: every sensor, its link to the C2, and what the C2 makes of it, in plain words.

  python -m sentinel_c2 --site ... --ui            # then open http://127.0.0.1:8090

Standard library only; the page needs no internet (the plan view is drawn from the site file).
GET /            the dashboard: every sensor and the C2's picture
GET /radar       the radar scope: what each radar sees, as a classic radar display
GET /camera      the camera tracker (Visual-Tracker's Live view), with a button to start it
GET /state.json, /radar.json, /api/tracker   the data behind each page (also handy for scripts)
POST /api/tracker/start, /api/tracker/link   start Visual-Tracker / switch on its link to the C2
                 (only accepted from this machine)
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import urllib.request
from pathlib import Path
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files

from cuas.base import NODE_STALE_S

FRESH_S = 5.0  # a sensor counts as "seeing it now" if it updated the track this recently
COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def _ago(s: float | None) -> str:
    if s is None:
        return "never"
    if s < 2:
        return "just now"
    if s < 90:
        return f"{s:.0f} s ago"
    if s < 5400:
        return f"{s / 60:.0f} min ago"
    return f"{s / 3600:.0f} h ago"


def source_name(src: str) -> str:
    kind, _, name = src.partition(":")
    return {"acoustic": "microphones", "visual": f"camera {name}", "radar": f"radar {name}"}.get(kind, src)


def summary(hub, now: float | None = None) -> dict:
    """Plain-language picture of the whole system. Everything the dashboard shows comes from here."""
    now = time.time() if now is None else now
    base, site = hub.base, hub.base.site
    sensors = []

    # -- acoustic network: the ESP32 receiver, then each node behind it -------------------
    esp = hub.esp
    if esp is None:
        sensors.append(dict(kind="receiver", id="esp32", name="ESP32 receiver", status="off",
                            text="Not in use (C2 started without --serial)."))
    else:
        up = esp.state == "up"
        sensors.append(dict(kind="receiver", id="esp32", name="ESP32 receiver", status="ok" if up else "down",
                            text=(f"Connected on {esp.port}. Last data {_ago(now - esp.last_rx if esp.last_rx else None)}."
                                  if up else f"Unplugged or not responding on {esp.port}. Reconnecting…")))
    receiver_ok = esp is not None and esp.state == "up"
    with base.lock:
        for nid, node in site.nodes.items():
            ns = base.nodes[nid]
            label = node.label or f"Node {nid}"
            online = ns.last_seen > 0 and now - ns.last_seen < NODE_STALE_S
            if esp is None:
                st, text = "off", "Not connected (no ESP32 receiver)."
            elif not ns.last_seen:
                st, text = ("down" if receiver_ok else "unknown"), (
                    "Never heard from. Check power and antenna." if receiver_ok else "Unknown: receiver unplugged.")
            elif not online:
                st, text = "down", f"Silent since {_ago(now - ns.last_seen)}. Check power and antenna."
            elif ns.detecting and ns.last_det:
                st = "alert"
                text = f"Hearing a drone ({ns.last_det.level_db:.0f} dB, rotor {ns.last_det.f0:.0f} Hz)."
            else:
                st = "ok"
                text = "Listening, nothing heard." + (f" Background {ns.floor_db:.0f} dB." if ns.floor_db else "")
            sensors.append(dict(kind="node", id=f"node-{nid}", name=f"Node {nid}", label=label, status=st, text=text,
                                e=node.e, n=node.n, detecting=bool(ns.detecting)))

    # -- cameras ------------------------------------------------------------------------
    peers = {p.src: p for p in hub.trackers.check_health(now)}
    cam_ids = list(site.cameras) + [s for s in peers if s not in site.cameras]
    for cid in cam_ids:
        cam, p = site.cameras.get(cid), peers.get(cid)
        e = n = None
        if cam and cam.lat is not None:
            e, n = site.frame.to_en(cam.lat, cam.lon)
        if p is None and cam is not None and cam.lat is None:
            st, text = "off", "Not launched (mobile camera)."
        elif p is None:
            st, text = "down", "Not connected. Start the Visual-Tracker and point it at this C2."
        elif not p.up:
            st, text = "down", f"Connection lost {_ago(now - p.last_heard)}."
        elif p.tracks_seen:
            st, text = "alert", f"Sees {p.tracks_seen} drone{'s' if p.tracks_seen > 1 else ''}."
        else:
            st, text = "ok", "Connected, nothing in view."
        sensors.append(dict(kind="camera", id=f"cam-{cid}", name=f"Camera {cid}",
                            label=(cam.label if cam and cam.label else ""), status=st, text=text, e=e, n=n))

    # -- radars -------------------------------------------------------------------------
    if hub.radar:
        for rid, r in hub.radar.check_health(now).items():
            if not r["up"]:
                st, text = "down", f"Connection lost {_ago(now - r['last_heard'])}."
            elif r["plots"]:
                st, text = "alert", f"{r['plots']} contact{'s' if r['plots'] > 1 else ''} this scan."
            else:
                st, text = "ok", "Connected, nothing detected."
            sensors.append(dict(kind="radar", id=f"radar-{rid}", name=f"Radar {rid}", label="", status=st,
                                text=text, e=r.get("e"), n=r.get("n")))

    # -- tracks -------------------------------------------------------------------------
    tracks = []
    with base.lock:
        live = dict(base.tracker.tracks)
        snap = base.tracker.snapshot(now)
    for tr in snap:
        srcs = live[tr["id"]].sources if tr["id"] in live else {}
        fresh = sorted(source_name(s) for s, t in srcs.items() if now - t <= FRESH_S)
        speed = math.hypot(tr["ve"], tr["vn"])
        heading = COMPASS[int(((math.degrees(math.atan2(tr["ve"], tr["vn"])) % 360) + 22.5) // 45) % 8]
        tracks.append(dict(id=tr["id"], confirmed=tr["confirmed"], fresh=bool(fresh), e=tr["e"], n=tr["n"], sigma_m=tr["sigma_m"],
                           speed_mps=speed, heading=heading if speed >= 0.5 else None, seen_by=fresh,
                           ever_by=sorted(source_name(s) for s in srcs), last_s=tr["coast_s"], label=tr["label"],
                           f0=tr["f0"], history=[(round(e, 1), round(n, 1)) for e, n in tr["history"]]))
    tracks.sort(key=lambda t: (not (t["confirmed"] and t["fresh"]), not t["confirmed"], t["id"]))

    # -- verdict ------------------------------------------------------------------------
    confirmed = [t for t in tracks if t["confirmed"] and t["fresh"]]
    lost = [t for t in tracks if t["confirmed"] and not t["fresh"]]
    problems = [s for s in sensors if s["status"] == "down"]
    if confirmed:
        verdict = dict(level="alert", title="DRONE TRACKED" if len(confirmed) == 1 else f"{len(confirmed)} DRONES TRACKED",
                       text="; ".join(_track_sentence(t) for t in confirmed))
    elif lost:
        t = min(lost, key=lambda t: t["last_s"])
        verdict = dict(level="warn", title="DRONE LOST",
                       text=f"No sensor has seen drone {t['id']} for {t['last_s']:.0f} s. Last estimate shown on the plan.")
    elif tracks:
        verdict = dict(level="warn", title="POSSIBLE DRONE",
                       text="A sensor picked something up. Waiting for a second report to confirm.")
    else:
        verdict = dict(level="clear", title="ALL CLEAR", text="No drones detected.")
    health = ("All sensors connected." if not problems else
              f"{len(problems)} sensor{'s' if len(problems) > 1 else ''} not reporting: "
              + ", ".join(s["name"] for s in problems) + ".")
    return dict(t=now, site=site.name, verdict=verdict, health=health, health_ok=not problems,
                sensors=sensors, tracks=tracks, events=list(hub.events.items) if hasattr(hub, "events") else [],
                hints_sent=len(hub.hints.sent))


def _track_sentence(t: dict) -> str:
    who = " + ".join(t["seen_by"]) if t["seen_by"] else "no sensor right now (estimating)"
    move = f", moving {t['heading']} at {t['speed_mps']:.0f} m/s" if t["heading"] else ", hovering"
    return f"Drone {t['id']} seen by {who}, position ±{t['sigma_m']:.0f} m{move}"


class EventLog:
    """Turns successive summaries into a short plain-English log of what changed."""

    def __init__(self, maxlen: int = 60):
        self.items: deque = deque(maxlen=maxlen)
        self._prev: dict | None = None
        self._hints = 0

    def add(self, t: float, level: str, text: str):
        self.items.appendleft(dict(t=t, time=time.strftime("%H:%M:%S", time.localtime(t)), level=level, text=text))

    def observe(self, s: dict, hint_list: list):
        t = s["t"]
        prev = self._prev
        if prev is None:
            self.add(t, "info", f"C2 started. {s['health']}")
        else:
            old = {x["id"]: x for x in prev["sensors"]}
            for x in s["sensors"]:
                o = old.get(x["id"])
                if o is None or o["status"] == x["status"]:
                    continue
                if x["status"] == "down":
                    self.add(t, "warn", f"{x['name']}: {x['text']}")
                elif o["status"] in ("down", "unknown", "off") and x["status"] in ("ok", "alert"):
                    self.add(t, "ok", f"{x['name']} connected.")
                elif x["kind"] == "node" and x["status"] == "alert":
                    self.add(t, "alert", f"{x['name']} hears a drone.")
                elif x["kind"] == "node" and o["status"] == "alert" and x["status"] == "ok":
                    self.add(t, "info", f"{x['name']} lost the sound.")
            was = {x["id"]: x for x in prev["tracks"] if x["confirmed"] and x["fresh"]}
            now_ = {x["id"]: x for x in s["tracks"] if x["confirmed"] and x["fresh"]}
            gone = {x["id"] for x in s["tracks"]}
            for i in now_.keys() - was.keys():
                self.add(t, "alert", f"Drone {i} tracked, seen by {' + '.join(now_[i]['seen_by']) or 'sensors'}.")
            for i in was.keys() - now_.keys():
                self.add(t, "warn" if i in gone else "info",
                         f"Drone {i}: lost contact, estimating." if i in gone else f"Drone {i} dropped.")
        for h in hint_list[self._hints:]:
            self.add(h.t, "info", f"Re-cued Node {h.node_id} to listen for drone {h.track_id} "
                                  f"({'it lost the sound' if h.reason == 'lost' else 'drone is nearby'})"
                                  + ("" if h.delivered else " — NOT SENT, receiver unplugged") + ".")
        self._hints = len(hint_list)
        self._prev = s


def radar_view(hub, now: float | None = None, window_s: float = 12.0) -> dict:
    """Everything the radar scope draws: per radar, its recent plots and the C2's tracks around it."""
    now = time.time() if now is None else now
    out = []
    if not hub.radar:
        return dict(t=now, radars=out)
    health = hub.radar.check_health(now)
    with hub.base.lock:
        snap = [t for t in hub.base.tracker.snapshot(now)]
    for src, r in health.items():
        plots = [dict(p, age=now - p["t"]) for p in list(hub.radar.plots.get(src, [])) if now - p["t"] <= window_s]
        cov = hub.radar.coverage.get(src, {})
        tracks = []
        for t in snap:
            de, dn = t["e"] - r.get("e", 0.0), t["n"] - r.get("n", 0.0)
            tracks.append(dict(id=t["id"], confirmed=t["confirmed"], az=math.degrees(math.atan2(de, dn)) % 360,
                               gr=math.hypot(de, dn), sigma_m=t["sigma_m"],
                               by=sorted(source_name(s) for s in t["sources"])))
        out.append(dict(src=src, up=r["up"], last_heard_s=now - r["last_heard"], plots=plots, tracks=tracks,
                        max_range_m=cov.get("max_range_m"), scan_s=cov.get("scan_s", 1.0),
                        shadows=cov.get("shadows", [])))
    return dict(t=now, site=hub.base.site.name, radars=out)


class TrackerControl:
    """Starts the Visual-Tracker (its own launcher, so macOS camera permission works) and, once its
    dashboard answers, switches on its link to this C2 and picks the model."""

    def __init__(self, path: str | None, url: str = "http://127.0.0.1:8001", model: str = "v4", launcher=None,
                 log=print):
        self.path = Path(path).expanduser() if path else None
        self.url, self.model, self.log = url.rstrip("/"), model, log
        self.launcher = launcher or self._launch
        self.state = "idle"  # idle | starting | running | failed
        self.message = ""
        self._configured = False

    @property
    def script(self) -> Path | None:
        return self.path / "Start Tracker.command" if self.path else None

    def _get(self, path: str, timeout: float = 0.6):
        with urllib.request.urlopen(self.url + path, timeout=timeout) as r:
            return json.loads(r.read() or b"null")

    def status(self) -> dict:
        try:
            stats = self._get("/live/stats")
            running = True
        except Exception:
            stats, running = None, False
        if running:
            self.state = "running"
            if not self._configured:
                self.configure()
        elif self.state == "running":
            self.state, self._configured = "idle", False
        link = (stats or {}).get("link") or {}
        problem = None
        if not self.path or not self.path.exists():
            problem = "Visual-Tracker folder not found. Start the C2 with --visual-tracker /path/to/Visual-Tracker."
        elif not (self.path / ".venv" / "bin" / "python").exists():
            problem = ("Visual-Tracker isn't set up on this Mac yet. In its folder run: "
                       "python3 -m venv .venv && .venv/bin/pip install -r requirements.txt")
        return dict(state=self.state, running=running, url=self.url, message=self.message, problem=problem,
                    can_start=sys.platform == "darwin" and problem is None,
                    sending=bool(link.get("enabled")), c2_up=bool((link.get("c2") or {}).get("up")),
                    src=link.get("src"), cue=link.get("cue"))

    def configure(self) -> bool:
        try:
            self._get(f"/live/set?link=1&model={self.model}")
            self._configured = True
            self.log(f"camera tracker: link to C2 on, model {self.model}")
            return True
        except Exception as e:
            self.message = f"Couldn't switch on the tracker's link: {e}"
            return False

    def _launch(self, script: Path):
        if sys.platform != "darwin":
            raise RuntimeError("Starting the camera tracker from here only works on a Mac; start it by hand.")
        subprocess.Popen(["open", str(script)])  # opens in its own Terminal window, logs visible, Ctrl-C there stops it

    def start(self) -> dict:
        st = self.status()
        if st["running"]:
            return st
        if st["problem"]:
            self.message = st["problem"]
            return self.status()
        try:
            self.launcher(self.script)
        except Exception as e:
            self.state, self.message = "failed", str(e)
            return self.status()
        self.state, self._configured = "starting", False
        self.message = ("Starting. A Terminal window opens for the tracker; the first time, macOS asks for camera "
                        "access. Allow it. This can take a minute while the model and VLM load.")
        self.log("camera tracker: starting " + str(self.script))
        return self.status()


def _find_visual_tracker() -> str | None:
    here = Path(__file__).resolve().parents[2]
    for p in (here / "Visual-Tracker", here / "vt-repo"):
        if (p / "base_link.py").exists():
            return str(p)
    return None


def serve(hub, host: str = "127.0.0.1", port: int = 8090, log=print, tracker: TrackerControl | None = None,
          ) -> ThreadingHTTPServer:
    tracker = tracker or TrackerControl(_find_visual_tracker(), log=log)
    pages = {p: files("sentinel_c2").joinpath(f"static/{f}").read_bytes()
             for p, f in (("/", "ui.html"), ("/index.html", "ui.html"), ("/radar", "radar.html"),
                          ("/camera", "camera.html"))}
    lock = threading.Lock()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/state.json", "/radar.json"):
                with lock:
                    body = json.dumps(summary(hub) if path == "/state.json" else radar_view(hub)).encode()
                ctype = "application/json"
            elif path == "/api/tracker":
                body, ctype = json.dumps(tracker.status()).encode(), "application/json"
            elif path in pages:
                body, ctype = pages[path], "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = self.path.split("?")[0]
            if self.client_address[0] not in ("127.0.0.1", "::1"):
                self.send_error(403, "Only from the C2 machine itself")
                return
            if path == "/api/tracker/start":
                out = tracker.start()
            elif path == "/api/tracker/link":
                tracker.configure()
                out = tracker.status()
            else:
                self.send_error(404)
                return
            body = json.dumps(out).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer((host, port), H)
    srv.tracker = tracker  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True, name="ui").start()
    log(f"dashboard: http://{host}:{srv.server_address[1]}")
    return srv
