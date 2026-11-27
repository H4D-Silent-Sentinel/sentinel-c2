"""Hand controller (tango): mutes, node ignore, TAK switch, track focus, controller loss."""
import json
from types import SimpleNamespace

from test_ui import feed, make_hub

from sentinel_c2.ui import summary


def ctl(hub, name, value, t):
    hub.operator.on_datagram(json.dumps({"v": 1, "type": "control", "src": "tango", "t": t,
                                         "name": name, "value": value}).encode(), "127.0.0.1", now=t)


def test_camera_mute_keeps_camera_out_of_fusion():
    hub = make_hub()
    ctl(hub, "mute_camera", True, 0.0)
    feed(hub, 0, 6, hearing=())
    assert not hub.snap, "camera reports must not open tracks while muted"
    cam = next(s for s in summary(hub, 6.0)["sensors"] if s["kind"] == "camera")
    assert cam["status"] == "off" and cam["text"].startswith("Muted by operator")
    ctl(hub, "mute_camera", False, 6.0)
    feed(hub, 6, 12, hearing=())
    assert hub.snap


def test_node_ignore_drops_that_node_only():
    hub = make_hub()
    ids = sorted(hub.base.site.nodes)
    ctl(hub, "node_select", 1, 0.0)
    ctl(hub, "node_toggle", True, 0.1)
    gate = hub.operator.accepts_packet
    assert not gate(SimpleNamespace(node_id=ids[0])) and gate(SimpleNamespace(node_id=ids[1]))
    ctl(hub, "node_toggle", True, 0.2)
    assert gate(SimpleNamespace(node_id=ids[0]))


def test_track_select_marks_track_and_logs():
    hub = make_hub()
    feed(hub, 0, 8)
    ctl(hub, "track_select", 1, 8.0)
    s = summary(hub, 8.0)
    assert [t["id"] for t in s["tracks"] if t["selected"]] == [hub.operator.track]
    assert any("selected" in e["text"] for e in hub.events.items)


def test_controller_loss_shows_on_dashboard():
    hub = make_hub()
    hub.operator.on_datagram(json.dumps({"v": 1, "type": "controls", "connected": True, "state": {}}).encode(),
                             "127.0.0.1", now=0.0)
    assert next(s for s in summary(hub, 0.5)["sensors"] if s["kind"] == "operator")["status"] == "ok"
    hub.operator.check(10.0)
    assert next(s for s in summary(hub, 10.0)["sensors"] if s["kind"] == "operator")["status"] == "down"


def test_heartbeat_syncs_switch_positions():
    hub = make_hub()
    hub.operator.on_datagram(json.dumps({"v": 1, "type": "controls", "connected": True,
                                         "state": {"mute_radar": True, "tak_out": False}}).encode(), "127.0.0.1", now=0.0)
    assert hub.operator.muted == {"radar"} and hub.operator.tak_out is False
