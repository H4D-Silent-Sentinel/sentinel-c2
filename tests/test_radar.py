"""Fake radar -> C2, on simulated time (no sockets, no sleeping)."""
import math

import numpy as np
from cuas.base import Base
from cuas.sim.scenario import DEFAULT_PATH, DroneSim

from sentinel_c2.fake_radar import FakeRadar, RadarModel, Shadow
from sentinel_c2.hints import HintConfig, HintPolicy
from sentinel_c2.radar import RadarLink, make_radar, parse_radar

from scenario import detection, load_site

QUIET = dict(log=lambda *_: None)


def fly(site, radar, base, link, t0, t1, dt=0.5, each=None):
    """Run scans from t0 to t1 sim seconds; returns per-step (truth, snapshot)."""
    lat, lon = site.frame.to_ll(radar.m.e, radar.m.n)
    out = []
    t = t0
    while t < t1:
        plots, truth = radar.scan(t, dt)
        link.on_datagram(make_radar("radar-1", 1000 + t, {"lat": lat, "lon": lon, "alt_m": radar.m.alt_m}, plots),
                         "10.0.0.7", now=1000 + t)
        if each:
            each(1000 + t)
        out.append((truth, base.step(1000 + t + 0.01)))
        t += dt
    return out


def rig(shadows=(), false_alarms=0.3, drones=None, seed=1):
    site = load_site()
    base = Base(site)
    link = RadarLink(base, port=0, **QUIET)
    drones = drones or [DroneSim(DEFAULT_PATH, speed=8.0)]
    radar = FakeRadar(RadarModel(e=0, n=-120, shadows=list(shadows), false_alarms_per_scan=false_alarms),
                      drones, np.random.default_rng(seed))
    return site, base, link, radar


def test_message_round_trip():
    site = load_site()
    lat, lon = site.frame.to_ll(0, -120)
    raw = make_radar("radar-1", 5.0, {"lat": lat, "lon": lon, "alt_m": 10},
                     [{"id": 1, "range_m": 100.0, "az_deg": 90.0, "el_deg": 0.0, "conf": 0.9}])
    (o,) = parse_radar(raw, site.frame)
    (e, n), R, alt = o.fix()
    assert abs(e - 100) < 0.5 and abs(n + 120) < 0.5 and abs(alt - 10) < 0.5
    assert abs(R[0, 0] - 5.0 ** 2) < 1e-6  # east = along the beam: range sigma (default 5 m)
    assert abs(R[1, 1] - (100 * math.radians(1.5)) ** 2) < 1e-6  # north = cross-range: 1.5 deg at 100 m


def test_radar_alone_tracks_the_drone():
    site, base, link, radar = rig()
    steps = fly(site, radar, base, link, 0, radar.drones[0].duration)
    errs = []
    for truth, snap in steps:
        conf = [t for t in snap if t["confirmed"] and any(s.startswith("radar:") for s in t["sources"])]
        if truth and conf:
            tr = min(conf, key=lambda t: math.hypot(t["e"] - truth[0]["e"], t["n"] - truth[0]["n"]))
            errs.append(math.hypot(tr["e"] - truth[0]["e"], tr["n"] - truth[0]["n"]))
    assert len(errs) > 0.7 * len(steps), (len(errs), len(steps))
    assert np.median(errs) < 15, np.median(errs)
    link.close()


def test_clutter_alone_never_confirms():
    site, base, link, radar = rig(false_alarms=3.0, drones=[DroneSim([(5000, 5000), (5001, 5000)])])
    steps = fly(site, radar, base, link, 0, 120)
    assert not any(t["confirmed"] for _, snap in steps for t in snap)
    link.close()


def test_building_shadow_hides_the_drone():
    site, base, link, radar = rig(shadows=[Shadow(270, 360, 100)])  # buildings to the NW beyond 100 m
    steps = fly(site, radar, base, link, 0, 30)
    hidden = [tr for truth, _ in steps for tr in truth if tr["hidden"]]
    assert hidden and not any(tr["seen"] for tr in hidden)
    link.close()


def test_radar_vouches_for_node_hints():
    """Nodes 1/5/6 lose the drone while the radar still has it -> they get hints."""
    site, base, link, radar = rig(drones=[DroneSim([(55, 40), (65, 40)], speed=0.1)], false_alarms=0)
    sent = []
    hints = HintPolicy(base, lambda p: sent.append(p) or True, HintConfig(), **QUIET)
    seq = [0]

    def nodes(hearing):
        def each(t):
            for nid in site.nodes:
                seq[0] += 1
                base.on_packet(detection(site, nid, seq[0], lost=nid not in hearing), t)
            hints.step(base.step(t), t)
        return each

    fly(site, radar, base, link, 0, 20, each=nodes({1, 2, 3, 4, 5, 6}))
    assert sent == []
    fly(site, radar, base, link, 20, 30, each=nodes({2, 3, 4}))
    assert {p.node_id for p in sent} == {1, 5, 6}
    link.close()
