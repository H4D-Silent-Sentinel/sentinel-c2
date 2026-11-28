"""Radar station over LoRa: RadarPacket lines from the base receiver become radar plots and a track."""
import json

from cuas.base import Base
from cuas.config import Site
from cuas.packet import RadarPacket, RadarPlot

from sentinel_c2.esp32 import Esp32Link
from sentinel_c2.radar import RadarLink
from sentinel_c2.radar_lora import Associator, LoRaRadar

QUIET = dict(log=lambda *_: None)


def site(tmp_path, **radar):
    f = tmp_path / "site.json"
    f.write_text(json.dumps({"origin": [40.8, -73.96], "drone_alt_m": 1,
                             "nodes": [{"id": 1, "lat": 40.8, "lon": -73.96}],
                             "radars": [{"id": 2, "lat": 40.8, "lon": -73.96, "yaw_deg": 90, **radar}]}))
    return Site.load(f)


def rig(tmp_path, **radar):
    base = Base(site(tmp_path, **radar))
    link = RadarLink(base, port=0, **QUIET)
    esp = Esp32Link.__new__(Esp32Link)  # line handling only, no serial port
    esp.__dict__.update(base=base, log=QUIET["log"], gate=None, last_rx=0.0, rx_packets=0, bad_lines=0,
                        tx_ok=0, tx_err=0)
    esp.on_radar = LoRaRadar(link, base.site, **QUIET).on_packet
    return base, link, esp


def line(pkt):
    return f"PKT -70 9.0 {pkt.encode().hex()}"


def test_target_forward_of_an_east_facing_radar_is_east(tmp_path):
    base, link, esp = rig(tmp_path)
    for k in range(4):
        esp._on_line(line(RadarPacket(2, k, [RadarPlot(0.0, 4.0 - 0.1 * k, -0.5)])))
    tracks = base.step()
    assert len(tracks) == 1
    assert abs(tracks[0]["e"] - 3.7) < 0.5 and abs(tracks[0]["n"]) < 0.5
    assert link.radars["radar-2"]["msgs"] == 4
    assert esp.rx_packets == 4 and all(ns.packets == 0 for ns in base.nodes.values())  # not a listening node


def test_flip_x_mirrors_azimuth(tmp_path):
    base, link, esp = rig(tmp_path, flip_x=True)
    esp._on_line(line(RadarPacket(2, 0, [RadarPlot(1.0, 1.0, 0.0)])))
    assert round(link.plots["radar-2"][-1]["az"]) == 45


def test_unknown_radar_ignored(tmp_path):
    base, link, esp = rig(tmp_path)
    esp._on_line(line(RadarPacket(9, 0, [RadarPlot(0.0, 3.0, 0.0)])))
    assert link.radars == {}


def test_heartbeat_keeps_radar_up_without_plots(tmp_path):
    base, link, esp = rig(tmp_path)
    esp._on_line(line(RadarPacket(2, 0, [])))
    assert link.radars["radar-2"]["plots"] == 0


def test_ids_persist_within_gate():
    a = Associator()
    first = a.update([RadarPlot(0, 4, 0), RadarPlot(2, 6, 0)])
    second = a.update([RadarPlot(2.3, 5.5, 0), RadarPlot(0.1, 3.6, 0)])
    assert {i for i, _ in first} == {i for i, _ in second}
    assert a.update([RadarPlot(-6, 1, 0)])[0][0] not in {i for i, _ in first}
