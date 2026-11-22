"""Dashboard: plain-language summary, event log, and the HTTP pages."""
import json
import urllib.request

from cuas.packet import PktType

from sentinel_c2.hub import Hub
from sentinel_c2.radar import make_radar
from sentinel_c2.ui import serve, summary

from scenario import camera_obs, detection, load_site, visual_msg


def make_hub():
    return Hub(load_site(), serial_port=None, visual_port=0, cue_port=0, broadcast=None, radar_port=0, log=lambda *_: None)


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
    assert cam["status"] == "alert" and cam["text"] == "Sees 1 drone."
    s = summary(hub, 1030.0)  # everything went quiet: lost, not "tracked"
    assert s["verdict"]["title"] in ("DRONE LOST", "ALL CLEAR")
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
