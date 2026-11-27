# sentinel-c2

The collection point for Silent Sentinel. It reads the ESP32 acoustic node network, takes camera reports from every Visual-Tracker, fuses them, and keeps both sides updated continuously.

```
 ESP32 listening nodes ──LoRa──▶ base_rx ESP32 ──USB serial──▶ ┌─────────────┐ ──cue + status──▶ Visual-Tracker(s)
                       ◀─LoRa── (HINT downlink)  ◀─USB serial── │ sentinel-c2 │ ◀────visual──────  (Mac / Jetson)
 radar (simulated for now) ───────────udp/47012───────────────▶ └─────────────┘ ──▶ live map, TAK
```

The fusion itself (packet ingest, loudness localization, Kalman tracker, map, TAK) is [silent-sentinel](https://github.com/H4D-Silent-Sentinel/silent-sentinel)'s `cuas` package, used as a library. This repo adds the links and the two-way traffic:

| Link | Direction | What | Rate |
|---|---|---|---|
| ESP32 serial | node → C2 | `PKT <rssi> <snr> <hex>` node detections, LOST, heartbeats | as nodes report |
| ESP32 serial | C2 → node | `TX <hex>` HINT packet, base_rx transmits over LoRa | per node every 4 s while needed |
| UDP 47010 | tracker → C2 | `visual` (an empty `tracks` list is the tracker's heartbeat) | 5 Hz with tracks, 1 Hz idle |
| UDP 47011 | C2 → tracker | `cue` for each confirmed track | ≤ 1 Hz per track |
| UDP 47011 | C2 → tracker | `status` heartbeat with link and node health | 1 Hz, always |
| UDP 47012 | radar → C2 | `radar` plots: range, azimuth, elevation, radial velocity (an empty list is its heartbeat) | each scan |

## Operator app (no hardware needed to try it)

```sh
python -m sentinel_c2.demo          # whole system simulated; opens the app window
```

A real run with the dashboard: add `--ui --open` to the normal command. It opens in its own window (Chrome/Edge app mode); otherwise go to http://127.0.0.1:8090. Use `--ui-host 0.0.0.0` to view it from other machines on the network.

- **All sensors** (`/`): a banner (ALL CLEAR / POSSIBLE DRONE / DRONE TRACKED / DRONE LOST), every sensor's link to the C2 in plain words (green connected, orange detecting, red not reporting, grey not in use), an offline site plan, the drones, and a plain-English event log.
- **Radar scope** (`/radar`): a classic radar display with the sweep, range rings, contacts fading over 12 s, building shadows hatched, and the C2's fused drones on top.
- **Camera** (`/camera`): a **Start camera tracker** button. It runs Visual-Tracker's own `Start Tracker.command`, so macOS camera permission works and its logs stay visible in a Terminal window. Once Visual-Tracker answers, the C2 switches on its link to the C2 with the v4 model and shows its Live view in the tab. Visual-Tracker is found next to this repo, or set `--visual-tracker /path`. Starting it only works from the Mac running the C2. No drone to fly? Tick **Phone stands in for the drone** in the tab's top bar and hold up a second phone; it shows up as a drone labelled "phone stand-in" everywhere in the app.
- `/state.json`, `/radar.json` and `/api/tracker` return the same data for scripts or other systems.
- Live updates: the C2 fuses each report the moment it arrives (at most 10 times a second) and pushes the result to the open pages over `/events` and `/events/radar` (server-sent events). Node hints, tracker status and the event log run once a second.

The demo feeds the real C2 from simulated sensors that all watch one simulated drone:
- a fake ESP32 receiver on a virtual serial port (real packets, answers hints)
- camera post-1
- the fake radar

Nothing is transmitted.

## Run

```sh
pip install -e .                       # pulls silent-sentinel from GitHub
python -m sentinel_c2 ports            # find the base receiver ESP32
python -m sentinel_c2 --site path/to/site.json --serial /dev/cu.usbserial-XXXX --map
```

- `--tracker <ip>` (repeatable) sends to a tracker before it has spoken. Without it, the C2 learns trackers from their reports and broadcasts its status until one is heard, so a tracker started without `--base` finds the C2 by itself. Campus Wi-Fi often blocks broadcast; use `--tracker` there.
- No `--serial` runs trackers-only (camera tests without the node network).
- `--no-hints` turns the node downlink off. `--cot` sends tracks to TAK.

The site file is silent-sentinel's (`sites/*.json`): node positions, plus any fixed camera posts by `src` id.

Each second the console prints the ESP32 link state, nodes online, each tracker (up or DOWN), and confirmed tracks.

## Radar (simulated)

There is no real radar yet. **(Radar order complete as of 11/24)**
A real one transmits RF and needs spectrum authorization, so the C2 is tested against a software stand-in that only sends UDP packets:

```sh
python -m sentinel_c2 --site path/to/site.json                    # terminal 1
python -m sentinel_c2.fake_radar --site path/to/site.json \
    --at 0,-120 --shadow 270:340:120 --loop                         # terminal 2
```

The fake radar flies silent-sentinel's simulated drone path and models:
- detection that falls off with range (radar equation, small-drone RCS)
- measurement noise and Doppler
- false plots from birds and clutter
- building shadows: `--shadow AZ_FROM:AZ_TO:BEYOND_M` hides the drone in that sector past that range

It runs in real time on purpose: reports carry the wall clock, so the C2's tracker sees true drone speeds.

Message format and C2 ingest are in `sentinel_c2/radar.py`. A real radar plugs in by sending the same `radar` message to udp/47012, or through a small adapter from its vendor's output. The C2 only starts a track from a radar plot after the radar has held that track ID for 3 reports in a row. Across 20 random runs, heavy clutter (3 false plots per scan) never produced a confirmed track, and the drone was tracked ≥ 97% of the time with ~2–3 m median error.

Radar counts like a camera for node hints: if the radar still has a drone the nodes lost, they get re-cued.

A radar plot only reaches the tracker once the radar has reported the same track ID twice in a row. One-off plots (birds, clutter) are shown on the scope but never fused. A coasting track has a wide gate, and one-off plots used to keep a stale track alive, so one drone showed as two. Across replayed demo flights, that went from 20–34 s of a phantom second drone in half the runs to none (`tests/test_radar.py`).

## Keeping the nodes on target (HINT downlink)

The nodes lose a drone when it goes quiet or behind a building. If a Visual-Tracker or radar is still updating that track (within the last 3 s), the C2 sends a HINT over LoRa to each node that (a) reported that drone's rotor frequency in the last 2 min, or (b) is within 300 m of where the track is now, and isn't hearing it.

The hint says "look for a rotor comb at f0 for the next 10 s". The node's detector then re-confirms on 3 hits near that f0 instead of 6 of 8. Hints repeat every 4 s while the node stays lost, and they expire on the node by themselves, so a dropped radio packet costs nothing. A hint at the wrong frequency changes nothing, and noise or wind with a hint active still raises no alarm (tests in silent-sentinel `tests/test_detector.py`).

Packet: the normal 24-byte layout, type 4, `node_id` = target (0xFF = all), `f0` = frequency, `uptime_ms` = hold time. See `cuas/packet.py: make_hint`.

## Status message (C2 → tracker, udp/47011)

```json
{"v": 1, "type": "status", "src": "sentinel-c2", "t": 1763400001.0, "seq": 42,
 "esp32": "up", "nodes_online": 5, "nodes_total": 6, "nodes_detecting": [2, 3],
 "tracks": 1, "confirmed": 1, "hints_sent": 3}
```

`esp32` is `up`, `down` (port gone, reconnecting every 2 s) or `off` (no `--serial`). A tracker that hears nothing from the C2 for 3 s shows the base link as down.

## Tests

```sh
pip install -e ".[dev]"
python -m pytest            # end-to-end test needs a Visual-Tracker checkout: VISUAL_TRACKER=/path/to/Visual-Tracker
```

`tests/test_e2e.py` runs the whole chain on one machine: a fake base-receiver ESP32 on a virtual serial port, the hub, and Visual-Tracker's real `BaseLink`. It checks heartbeats, discovery, cues, hints arriving on the serial port as `TX` lines, and recovery after the ESP32 is unplugged and plugged back in.

## Bench test with one node and no radio

Plug a listening node straight into USB and pass its port as `--serial`. Nodes print the same `PKT` lines as base_rx and accept the same `TX` lines, so hints reach the node over the cable. On the node's serial console, `hint 200 10` applies a hint by hand.
