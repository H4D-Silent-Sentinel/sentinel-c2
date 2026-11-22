"""Hint policy on simulated time, no sockets."""
import json

from cuas.base import Base
from cuas.packet import PktType

from sentinel_c2.hints import HintConfig, HintPolicy
from sentinel_c2.tracker_link import TrackerLink

from scenario import camera_obs, detection, load_site, visual_msg


class Rig:
    def __init__(self):
        self.site = load_site()
        self.base = Base(self.site)
        self.sent = []
        self.hints = HintPolicy(self.base, lambda p: self.sent.append(p) or True, HintConfig(), log=lambda *_: None)
        self.trk = TrackerLink(self.base, visual_port=0, cue_port=0, broadcast=None, log=lambda *_: None)
        self.seq = 0

    def run(self, t0, t1, nodes_hearing, camera=True):
        snap = []
        for t in range(t0, t1):
            for nid in self.site.nodes:
                self.seq += 1
                self.base.on_packet(detection(self.site, nid, self.seq, lost=nid not in nodes_hearing), float(t))
            if camera:
                self.trk.on_datagram(json.dumps(visual_msg(t, [camera_obs(self.site)])).encode(), "10.0.0.9", now=t)
            snap = self.base.step(float(t) + 0.5)
            self.hints.step(snap, float(t) + 0.5)
        return snap

    def close(self):
        self.trk.close()


def test_hint_goes_to_nodes_that_lost_the_drone_while_camera_holds_it():
    rig = Rig()
    snap = rig.run(1000, 1010, nodes_hearing={1, 2, 3, 4, 5, 6})
    assert any(t["confirmed"] and t["f0"] for t in snap), snap
    assert rig.sent == []  # everyone hears it: nothing to do
    rig.run(1010, 1020, nodes_hearing={2, 3, 4})  # 1, 5, 6 drop it; camera still on it
    hinted = {p.node_id for p in rig.sent}
    assert hinted == {1, 5, 6}, hinted
    p = rig.sent[0]
    assert p.type == PktType.HINT and abs(p.f0_hz - 200.0) < 5 and p.uptime_ms == 10000
    per_node = [p for p in rig.sent if p.node_id == 1]
    assert 2 <= len(per_node) <= 3  # repeated every ~4 s while it stays lost, not every step
    rig.close()


def test_no_hints_without_the_camera():
    rig = Rig()
    rig.run(1000, 1010, nodes_hearing={1, 2, 3, 4, 5, 6})
    rig.run(1010, 1020, nodes_hearing={2, 3, 4}, camera=False)
    late = [p for p in rig.sent]
    # camera reports stop at 1010; hints may go out for visual_fresh_s after that, then stop
    assert all(p.node_id in {1, 5, 6} for p in late)
    n = len(rig.sent)
    rig.run(1020, 1030, nodes_hearing={2, 3, 4}, camera=False)
    assert len(rig.sent) == n
    rig.close()
