"""Dashboard: plain-language summary, event log, and the HTTP pages."""
import json
import urllib.request

from cuas.packet import PktType

from sentinel_c2.hub import Hub
from sentinel_c2.radar import make_radar
from sentinel_c2.ui import serve, summary

from scenario import camera_obs, detection, load_site, visual_msg


def make_hub():
    return Hub(load_site(), serial_port=None, visual_port=0, cue_port=0, broadcast=None, radar_port=0, operator_port=0, log=lambda *_: None)


def feed(hub, t0, t1, hearing=(1, 2, 3, 4, 5, 6), camera=True):
    for t in range(t0, t1):
        for nid in hub.base.site.nodes:
            hub.base.on_packet(detection(hub.base.site, nid, t * 10 + nid, lost=nid not in hearing), float(t))
        if camera:
            hub.trackers.on_datagram(json.dumps(visual_msg(t, [camera_obs(hub.base.site)])).encode(), "10.0.0.9", now=t)
        hub.step(float(t) + 0.5)


def test_verdicts_follow_what_the_sensors_see():
    hub = make_hub()
    s = summary(hub, 1000.0)
    assert s["verdict"]["title"] == "ALL CLEAR"
    assert any(x["name"] == "ESP32 receiver" and x["status"] == "off" for x in s["sensors"])
    assert any(x["name"] == "Camera chaser-1" and x["status"] == "off" for x in s["sensors"])  # mobile, not a fault
    assert any(x["name"] == "Camera post-1" and x["status"] == "down" for x in s["sensors"])  # fixed post, missing
    feed(hub, 1000, 1010)
    s = summary(hub, 1009.6)
    assert s["verdict"]["title"] == "DRONE TRACKED", s["verdict"]
    assert "camera post-1" in s["verdict"]["text"] and "microphones" in s["verdict"]["text"]
    cam = next(x for x in s["sensors"] if x["name"] == "Camera post-1")
    assert cam["status"] == "alert" and cam["text"].startswith("Sees 1 drone.")
    s = summary(hub, 1030.0)  # everything went quiet: lost, not "tracked"
    assert s["verdict"]["title"] in ("DRONE LOST", "ALL CLEAR")
    hub.close()


def test_stand_in_is_named_wherever_the_drone_is():
    hub = make_hub()
    for t in range(1000, 1010):
        obs = dict(camera_obs(hub.base.site), label="phone stand-in")
        hub.trackers.on_datagram(json.dumps(visual_msg(t, [obs])).encode(), "10.0.0.9", now=t)
        hub.step(float(t) + 0.5)
    s = summary(hub, 1009.6)
    assert s["verdict"]["title"] == "DRONE TRACKED", s["verdict"]
    assert "(phone stand-in)" in s["verdict"]["text"]
    assert s["tracks"][0]["label"] == "phone stand-in"
    hub.close()


def test_event_log_reads_like_english():
    hub = make_hub()
    hub.step(999.0)  # C2 up first, sensors arrive after
    feed(hub, 1000, 1010)
    texts = [e["text"] for e in hub.events.items]
    assert texts[-1].startswith("C2 started.")
    assert any(t.startswith("Camera post-1 connected") for t in texts)
    assert any("tracked, seen by" in t for t in texts)
    hub.close()


def test_pages_and_json_are_served():
    hub = make_hub()
    lat, lon = hub.base.site.frame.to_ll(0, -120)
    hub.radar.on_datagram(make_radar("radar-1", 1000.0, {"lat": lat, "lon": lon, "alt_m": 10, "max_range_m": 1500,
                                                         "shadows": [[270, 340, 120]]},
                                     [{"id": 1, "range_m": 150, "az_deg": 20, "el_deg": 10, "conf": 0.9}]), "x")
    srv = serve(hub, port=0, log=lambda *_: None)
    url = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        for path, needle in (("/", b"Sensors"), ("/radar", b"RADAR")):
            assert needle in urllib.request.urlopen(url + path).read()
        st = json.loads(urllib.request.urlopen(url + "/state.json").read())
        assert {"verdict", "sensors", "tracks", "events", "health"} <= st.keys()
        rv = json.loads(urllib.request.urlopen(url + "/radar.json").read())
        r = rv["radars"][0]
        assert r["src"] == "radar-1" and r["shadows"] == [[270, 340, 120]] and len(r["plots"]) == 1
    finally:
        srv.shutdown()
        hub.close()


def test_camera_tab_starts_tracker_and_turns_on_its_link(tmp_path):
    """Start button -> launcher runs Start Tracker.command; once the tracker's dashboard answers,
    the C2 switches on its link to the C2 with model v4."""
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.parse import parse_qs, urlparse

    from sentinel_c2.ui import TrackerControl

    vt = tmp_path / "Visual-Tracker"
    (vt / ".venv" / "bin").mkdir(parents=True)
    (vt / ".venv" / "bin" / "python").write_text("")
    (vt / "Start Tracker.command").write_text("")
    settings = {"link": False, "model": "v3", "standin": False}

    class FakeVT(BaseHTTPRequestHandler):
        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/live/set":
                for k, v in parse_qs(u.query).items():
                    settings[k] = v[0] in ("1", "true") if k in ("link", "standin") else v[0]
                body = b'{"ok": true}'
            else:
                body = json.dumps({"settings": {"standin": settings["standin"]},
                                   "link": {"enabled": settings["link"], "src": "post-1",
                                            "c2": {"up": settings["link"]}}}).encode()
            self.send_response(200)
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    fake = {}

    def launcher(script):  # stands in for `open "Start Tracker.command"`
        assert script.name == "Start Tracker.command"
        fake["srv"] = srv = ThreadingHTTPServer(("127.0.0.1", 0), FakeVT)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        ctl.url = f"http://127.0.0.1:{srv.server_address[1]}"

    ctl = TrackerControl(str(vt), url="http://127.0.0.1:9", launcher=launcher, log=lambda *_: None)
    hub = make_hub()
    srv = serve(hub, port=0, log=lambda *_: None, tracker=ctl)
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        assert b"Start camera tracker" in urllib.request.urlopen(base + "/camera").read()
        st = json.loads(urllib.request.urlopen(base + "/api/tracker").read())
        assert st["running"] is False and st["problem"] is None
        req = urllib.request.Request(base + "/api/tracker/start", method="POST")
        st = json.loads(urllib.request.urlopen(req).read())
        assert st["running"] is True  # fake tracker came up during start
        st = json.loads(urllib.request.urlopen(base + "/api/tracker").read())
        assert settings == {"link": True, "model": "v4", "standin": False}
        assert st["sending"] is True and st["c2_up"] is True and st["standin"] is False
        req = urllib.request.Request(base + "/api/tracker/standin?on=1", method="POST")
        st = json.loads(urllib.request.urlopen(req).read())
        assert settings["standin"] is True and st["standin"] is True
    finally:
        srv.shutdown()
        if "srv" in fake:
            fake["srv"].shutdown()
        hub.close()


def test_camera_tab_explains_missing_setup(tmp_path):
    from sentinel_c2.ui import TrackerControl
    (tmp_path / "Visual-Tracker").mkdir()
    st = TrackerControl(str(tmp_path / "Visual-Tracker"), url="http://127.0.0.1:9", log=lambda *_: None).status()
    assert st["running"] is False and "pip install -r requirements.txt" in st["problem"] and not st["can_start"]
    st = TrackerControl(None, url="http://127.0.0.1:9", log=lambda *_: None).status()
    assert "--visual-tracker" in st["problem"]


def test_moving_camera_shows_where_it_is_and_points():
    hub = make_hub()
    site = hub.base.site
    lat, lon = site.frame.to_ll(30.0, -50.0)
    msg = {"v": 1, "type": "visual", "src": "chaser-1", "t": 1000.0,
           "sensor": {"lat": lat, "lon": lon, "alt_m": 3, "yaw_deg": 135.0, "pitch_deg": 10, "hfov_deg": 65, "vfov_deg": 50},
           "tracks": []}
    hub.trackers.on_datagram(json.dumps(msg).encode(), "10.0.0.20", now=1000.0)
    s = summary(hub, 1000.5)
    ch = next(x for x in s["sensors"] if x["name"] == "Camera chaser-1")
    assert ch["status"] == "ok" and abs(ch["e"] - 30) < 0.5 and abs(ch["n"] + 50) < 0.5
    assert ch["yaw"] == 135.0 and ch["hfov"] == 65 and "Pointing 135° (moving)" in ch["text"]
    no_gps = dict(msg, sensor={"yaw_deg": 135.0, "hfov_deg": 65})
    hub.trackers.on_datagram(json.dumps(no_gps).encode(), "10.0.0.20", now=1001.0)
    ch = next(x for x in summary(hub, 1001.5)["sensors"] if x["name"] == "Camera chaser-1")
    assert "waiting for its GPS fix" in ch["text"] and ch["e"] is None
    hub.close()


def test_two_cameras_on_one_machine_are_two_sensors():
    hub = make_hub()
    site = hub.base.site
    lat, lon = site.frame.to_ll(30.0, -50.0)
    for t in (1000.0, 1000.5):
        hub.trackers.on_datagram(json.dumps(visual_msg(t, [])).encode(), "10.0.0.9", now=t)  # post-1
        hub.trackers.on_datagram(json.dumps({"v": 1, "type": "visual", "src": "chaser-1", "t": t,
                                             "sensor": {"lat": lat, "lon": lon, "yaw_deg": 90.0}, "tracks": []}).encode(),
                                 "10.0.0.9", now=t)  # same machine
    s = summary(hub, 1001.0)
    cams = {x["name"]: x["status"] for x in s["sensors"] if x["kind"] == "camera"}
    assert cams == {"Camera post-1": "ok", "Camera chaser-1": "ok"}, cams
    assert hub.trackers.destinations() == {"10.0.0.9"}  # cues still go to that machine once
    hub.close()


def test_proximity_reports_closest_live_track_to_a_node():
    hub = make_hub()
    assert summary(hub, 1000.0)["proximity"] is None
    feed(hub, 0, 8)
    s = summary(hub, 8.0)
    p = s["proximity"]
    assert p is not None and p["node"] in hub.base.site.nodes and p["dist_m"] >= 0
    assert p["level"] in ("inside", "close", "approach", "clear")
    assert s["alert_m"] == sorted(s["alert_m"])
