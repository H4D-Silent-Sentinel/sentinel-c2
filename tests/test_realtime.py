"""Real time: the C2 fuses the moment a sensor reports, and the dashboard is pushed the result."""
import json
import threading
import time
import urllib.request

from sentinel_c2.hub import Hub
from sentinel_c2.ui import serve

from scenario import detection, load_site


def make_hub():
    return Hub(load_site(), serial_port=None, visual_port=0, cue_port=0, broadcast=None, radar_port=0, operator_port=0,
               log=lambda *_: None)


def test_report_reaches_a_track_without_waiting_for_a_timer():
    hub = make_hub()
    threading.Thread(target=hub.run_forever, args=(5.0,), daemon=True).start()  # housekeeping only every 5 s
    time.sleep(0.3)  # let the loop settle into waiting
    v0 = hub.version
    t0 = time.time()
    for nid in hub.base.site.nodes:
        hub.base.on_packet(detection(hub.base.site, nid, nid), t0)
    while not hub.snap and time.time() - t0 < 2.0:
        time.sleep(0.005)
    took = time.time() - t0
    hub.close()
    assert hub.snap, "no track"
    assert hub.version > v0
    assert took < 0.3, f"{took:.2f} s from packets to track"  # was up to ~1 s on the old 1 Hz loop


def test_dashboard_stream_pushes_each_fusion_cycle():
    hub = make_hub()
    srv = serve(hub, port=0, log=lambda *_: None)
    url = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        resp = urllib.request.urlopen(url + "/events", timeout=5)
        assert resp.headers["Content-Type"] == "text/event-stream"
        for nid in hub.base.site.nodes:
            hub.base.on_packet(detection(hub.base.site, nid, nid), 1000.0)
        hub.fuse(1000.0)
        line = b""
        while not line.startswith(b"data: "):
            line = resp.readline()
        st = json.loads(line[len(b"data: "):])
        assert {"verdict", "sensors", "tracks"} <= st.keys()
        resp.close()
    finally:
        srv.shutdown()
        hub.close()
