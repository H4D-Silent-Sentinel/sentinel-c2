"""Whole chain on one machine, real sockets and a real serial port (pty):

  fake base_rx ESP32 (pty)  <->  Hub  <->  Visual-Tracker's BaseLink (UDP)

Checks: the C2 heartbeat reaches the tracker, the tracker finds the C2 on its own,
cues flow, hints go down the serial port as `TX <hex>`, and an unplugged ESP32 is
reported to the tracker and picked up again when it comes back.
Needs a Visual-Tracker checkout with base_link.py (VISUAL_TRACKER env, default ../vt-repo).
"""
import json
import os
import pty
import socket
import sys
import termios
import tempfile
import threading
import time
from pathlib import Path

import pytest

from cuas.packet import Packet, PktType

from sentinel_c2.hub import Hub
from sentinel_c2.hints import HintConfig

from scenario import camera_obs, detection, load_site, rx_line

_UP = Path(__file__).resolve().parents[2]
VT = Path(os.environ.get("VISUAL_TRACKER") or next(
    (p for p in (_UP / "Visual-Tracker", _UP / "vt-repo") if (p / "base_link.py").exists()), _UP / "Visual-Tracker"))
pytestmark = pytest.mark.skipif(not (VT / "base_link.py").exists(), reason="no Visual-Tracker checkout")


class FakeBaseRx:
    """Plays firmware/base_rx on a pty: emits PKT lines, answers TX lines with TXOK."""

    def __init__(self, link_path: str):
        self.master, slave = pty.openpty()
        self.slave_name = os.ttyname(slave)
        self._slave = slave
        if os.path.lexists(link_path):
            os.unlink(link_path)
        os.symlink(self.slave_name, link_path)  # stable path, like /dev/cu.usbserial-XXXX
        self.tx: list[Packet] = []
        self.alive = True
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = b""
        while self.alive:
            try:
                chunk = os.read(self.master, 1024)
            except OSError:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode().strip()
                if line.startswith("TX "):
                    self.tx.append(Packet.decode(bytes.fromhex(line[3:])))
                    try:
                        os.write(self.master, f"TXOK {len(line[3:]) // 2}\n".encode())
                    except OSError:
                        return

    def write(self, s: str):
        os.write(self.master, s.encode())

    def unplug(self):
        self.alive = False
        for fd in (self._slave, self.master):
            try:
                termios.tcflush(fd, termios.TCIOFLUSH)  # never wait on a drain at close (macOS)
            except (OSError, termios.error):
                pass
            try:
                os.close(fd)
            except OSError:
                pass


def wait(cond, timeout=6.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def chain():
    sys.path.insert(0, str(VT))
    import base_link
    site = load_site()
    d = tempfile.mkdtemp()
    port = os.path.join(d, "esp32")
    esp = FakeBaseRx(port)
    logs = []
    hub = Hub(site, serial_port=port, trackers=["127.0.0.1"], visual_port=0, cue_port=0, broadcast=None, radar_port=0, operator_port=0,
              hint_cfg=HintConfig(resend_s=1.0), log=logs.append)
    hub.esp.retry_s = 0.2
    old = os.environ.pop("VT_BASE", None)
    tracker = base_link.BaseLink(None, "post-1", cue_port=0)  # no base address: must find the C2 itself
    if old is not None:
        os.environ["VT_BASE"] = old
    hub.trackers.cue_port = tracker.cue_port
    hub.start()
    stop = threading.Event()

    def hub_loop():
        while not stop.is_set():
            hub.step()
            time.sleep(0.2)

    threading.Thread(target=hub_loop, daemon=True).start()
    ctx = dict(site=site, hub=hub, esp=esp, tracker=tracker, port=port, logs=logs, base_link=base_link)
    yield ctx
    stop.set()
    hub.close()  # joins the serial reader before releasing the port
    tracker.close()
    ctx["esp"].unplug()


def test_full_chain(chain):
    site, hub, esp, tracker, bl = chain["site"], chain["hub"], chain["esp"], chain["tracker"], chain["base_link"]

    # 1. heartbeat: the tracker hears the C2 before anything is detected, and learns its address
    assert wait(lambda: tracker.c2_status()["up"]), "no C2 status reached the tracker"
    assert wait(lambda: tracker.c2_status().get("esp32") == "up")
    assert tracker.addr[0] == "127.0.0.1"
    tracker.addr = ("127.0.0.1", hub.trackers.visual_port)  # test uses an ephemeral port instead of 47010

    # 2. tracker heartbeats keep the C2's view of it up
    tracker.publish([], 640, 480, {"hfov": 60})
    assert wait(lambda: any(p.up for p in hub.trackers.check_health()))

    # 3. all six ESP32 nodes hear the drone; the camera sees it too -> confirmed track, cue to tracker
    seq = [0]

    def feed(hearing, camera=True):
        for nid in site.nodes:
            seq[0] += 1
            esp.write(rx_line(detection(site, nid, seq[0], lost=nid not in hearing)))
        if camera:  # send the camera's observation straight to the C2 the way BaseLink would
            msg = {"v": 1, "type": "visual", "src": "post-1", "t": time.time(),
                   "sensor": {"yaw_deg": 0, "pitch_deg": 0, "hfov_deg": 60, "vfov_deg": 45},
                   "tracks": [camera_obs(site)]}
            tracker.tx.sendto(json.dumps(msg).encode(), tracker.addr)

    for _ in range(15):
        feed({1, 2, 3, 4, 5, 6})
        time.sleep(0.25)
    assert wait(lambda: tracker.cues_received > 0), chain["logs"][-10:]
    assert tracker.c2_status()["confirmed"] >= 1
    assert hub.esp.rx_packets >= 60

    # 4. nodes 1 and 6 lose it while the camera keeps it: hints go down the serial port
    for _ in range(12):
        feed({2, 3, 4, 5})
        time.sleep(0.25)
    assert wait(lambda: {p.node_id for p in esp.tx} >= {1, 6}), chain["logs"][-10:]
    assert all(p.type == PktType.HINT and abs(p.f0_hz - 200) < 5 for p in esp.tx)
    assert {p.node_id for p in esp.tx} <= {1, 6}
    assert wait(lambda: hub.esp.tx_ok >= 2)

    # 5. unplug the ESP32: the tracker is told, the link to the tracker stays up
    esp.unplug()
    assert wait(lambda: tracker.c2_status().get("esp32") == "down")
    assert tracker.c2_status()["up"]

    # 6. plug it back in (same port path): the C2 reconnects by itself
    esp2 = FakeBaseRx(chain["port"])
    chain["esp"] = esp2
    assert wait(lambda: tracker.c2_status().get("esp32") == "up")
    esp2.write(rx_line(detection(site, 2, 999)))
    n = hub.esp.rx_packets
    assert wait(lambda: hub.esp.rx_packets > n)
