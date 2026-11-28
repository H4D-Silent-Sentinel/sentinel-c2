# ROAREE

Remote Observation, Autonomous Recognition, and Event Evaluation. Command and control (C2) software for Silent Sentinel.

ROAREE ingests reports from the acoustic node network, Visual-Tracker cameras, radar and the tango hand controller, fuses them into tracks, and returns cues and status to the sensors. Fusion (packet ingest, acoustic localization, Kalman tracker, map, CoT output) is provided by the `cuas` package from [sentinel](https://github.com/H4D-Silent-Sentinel/sentinel).

```
 ESP32 listening nodes ──LoRa──▶ base_rx ESP32 ──USB serial──▶ ┌─────────────┐ ──cue + status──▶ Visual-Tracker(s)
                       ◀─LoRa── (HINT downlink)  ◀─USB serial── │   ROAREE    │ ◀────visual──────  (Mac / Jetson)
 radar ───────────────────────────────udp/47012───────────────▶ │             │ ──▶ dashboard, map, TAK
 tango hand controller ───────────────udp/47013───────────────▶ └─────────────┘
```

## Requirements

- Python 3.10 or newer (`python3 --version`). Dependencies have prebuilt packages for 3.10–3.14 on macOS, Windows and Linux. The `python3` bundled with the macOS command-line tools is 3.9 and is not supported.
- git. The install fetches `sentinel` from GitHub.

## Installation

```sh
pip install git+https://github.com/H4D-Silent-Sentinel/roaree.git
```

For development:

```sh
git clone https://github.com/H4D-Silent-Sentinel/roaree.git && cd roaree
pip install -e ".[dev]"
```

## Usage

```sh
roaree ports                                                        # list serial ports
roaree --site path/to/site.json --serial /dev/cu.usbserial-XXXX --ui --open
```

`python -m sentinel_c2` is equivalent to `roaree`.

| Option | Description |
|---|---|
| `--site FILE` | Site file from sentinel (`sites/*.json`): node positions and fixed camera posts by `src` id. Required. |
| `--serial PORT` | Base receiver ESP32. Omit to run without the node network. |
| `--tracker IP` | Visual-Tracker address, repeatable. Without it, trackers are learned from their reports and status is broadcast until one is heard. Required on networks that block broadcast. |
| `--no-broadcast` | Do not broadcast status before a tracker is heard. |
| `--no-hints` | Disable the HINT downlink to the nodes. |
| `--radar-port`, `--no-radar` | Radar listener port (default 47012), or disable it. |
| `--operator-port`, `--no-operator` | Hand controller listener port (default 47013), or disable it. |
| `--ui`, `--ui-port`, `--ui-host` | Serve the dashboard (default 127.0.0.1:8090). `--ui-host 0.0.0.0` exposes it on the network. |
| `--open` | With `--ui`, open the dashboard in its own window (Chrome or Edge app mode). |
| `--map`, `--map-port` | Serve the sentinel map (default port 8080). |
| `--cot [URL]` | Send CoT to TAK (default `udp://239.2.3.1:6969`). |
| `--visual-tracker DIR`, `--tracker-url URL` | Visual-Tracker checkout and dashboard address for the Camera page. |

The console prints link state, nodes online, tracker and radar status, and confirmed tracks once per second.

### Simulation

```sh
python -m sentinel_c2.demo
```

Runs ROAREE against simulated sensors observing one simulated UAS: an ESP32 receiver on a virtual serial port (real packet format, responds to HINTs), camera post-1, and the radar simulator. Nothing is transmitted.

## Dashboard

| Page | Content |
|---|---|
| `/` | System state (ALL CLEAR, POSSIBLE DRONE, DRONE TRACKED, DRONE LOST), per-sensor link status, site plan, tracks, event log. |
| `/radar` | PPI scope: sweep, range rings, plots with 12 s persistence, masked sectors, fused tracks. |
| `/camera` | Starts Visual-Tracker (`Start Tracker.command`, local machine only) and shows its live view. **Phone stands in for the drone** enables a phone as a test target. |
| `/state.json`, `/radar.json`, `/api/tracker` | The same data as JSON. |
| `/events`, `/events/radar` | Server-sent events, pushed on every fusion cycle. |

Fusion runs on report arrival, at most 10 Hz. HINTs, tracker status and the event log run at 1 Hz.

### Proximity alert

The dashboard reports the distance from the closest live track to the nearest listening node: APPROACHING within 100 m, CLOSE within 40 m. **Audio alert** in the header enables an audible alert (browsers require a click before playing sound): 1.4 kHz beeps within 100 m, 2.6 kHz within 40 m, with the interval falling from ~1 s at 100 m to ~70 ms at the node. Bands: `ALERT_CLOSE_M`, `ALERT_NEAR_M`, `ALERT_FAR_M` in `sentinel_c2/ui.py`.

## Interfaces

| Link | Direction | Message | Rate |
|---|---|---|---|
| ESP32 serial | node → C2 | `PKT <rssi> <snr> <hex>`: detections, LOST, heartbeats | as reported |
| ESP32 serial | C2 → node | `TX <hex>`: HINT packet, transmitted by base_rx over LoRa | per node every 4 s while needed |
| UDP 47010 | tracker → C2 | `visual`; an empty `tracks` list is the heartbeat | 5 Hz with tracks, 1 Hz idle |
| UDP 47011 | C2 → tracker | `cue` per confirmed track | ≤ 1 Hz per track |
| UDP 47011 | C2 → tracker | `status` | 1 Hz |
| UDP 47012 | radar → C2 | `radar` plots: range, azimuth, elevation, radial velocity; an empty list is the heartbeat | per scan |
| UDP 47013 | controller → C2 | `control` per change; `controls` heartbeat with switch positions | on change, 1 Hz |

### Status (C2 → tracker, UDP 47011)

```json
{"v": 1, "type": "status", "src": "sentinel-c2", "t": 1763400001.0, "seq": 42,
 "esp32": "up", "nodes_online": 5, "nodes_total": 6, "nodes_detecting": [2, 3],
 "tracks": 1, "confirmed": 1, "hints_sent": 3}
```

`esp32`: `up`, `down` (port lost, reconnect every 2 s) or `off` (no `--serial`). Trackers mark the C2 link down after 3 s without a status message.

## Radar

Message format and ingest: `sentinel_c2/radar.py`. A radar connects by sending `radar` messages to UDP 47012, directly or through an adapter.

- Track initiation requires the same radar track ID on 3 consecutive reports with confidence ≥ 0.6.
- Track updates require the same radar track ID on 2 consecutive reports. Single plots (birds, clutter) are displayed on the scope and not fused.
- Radar updates count as sensor coverage for HINT generation.

### Radar simulator

```sh
roaree --site path/to/site.json                                            # terminal 1
python -m sentinel_c2.fake_radar --site path/to/site.json \
    --at 0,-120 --shadow 270:340:120 --loop                                 # terminal 2
```

Flies the sentinel simulated UAS path in real time and sends UDP only. Models:

- range-dependent probability of detection (radar equation, small-UAS RCS)
- measurement noise and Doppler
- false plots from birds and clutter
- masking: `--shadow AZ_FROM:AZ_TO:BEYOND_M` blocks detection in that sector beyond that range

Results (`tests/test_radar.py`, 20 randomized runs, 3 false plots per scan): no false confirmed tracks; target held ≥ 97% of the time; median position error 2–3 m. Replayed demo flights produce no duplicate tracks.

## Hand controller

[tango](https://github.com/H4D-Silent-Sentinel/tango) reads a TBS Tango 2 in USB joystick mode and sends controls to UDP 47013. Handling: `sentinel_c2/operator.py`.

| Control | Effect |
|---|---|
| `mute_acoustic`, `mute_radar`, `mute_camera` | Reports from that sensor group are not fused while on. Shown as "Muted by operator". |
| `node_select`, `node_toggle` | Select a listening node; ignore or restore its reports. |
| `tak_out` | CoT output on or off (with `--cot`). |
| `track_select` | Select a track on the dashboard. |

The controller is reported lost after 2.5 s without a message or when tango reports it unplugged. All changes are written to the event log.

## HINT downlink

When a track has a camera or radar update within the last 3 s, ROAREE sends a HINT over LoRa to each node that is not detecting it and either reported that track's rotor frequency in the last 2 min or is within 300 m of the track.

A HINT sets the node detector to re-confirm on 3 hits near the given f0 instead of 6 of 8, for 10 s. HINTs repeat every 4 s while needed and expire on the node. A HINT at the wrong frequency, or noise and wind during an active HINT, does not produce a detection (sentinel `tests/test_detector.py`).

Packet: standard 24-byte layout, type 4; `node_id` = target (0xFF = all), `f0` = frequency, `uptime_ms` = hold time. See `cuas/packet.py: make_hint`.

## Bench testing

A listening node connected directly over USB can be passed as `--serial`. Nodes emit the same `PKT` lines as base_rx and accept the same `TX` lines. On the node console, `hint <f0> <seconds>` applies a HINT manually.

## Tests

```sh
pip install -e ".[dev]"
python -m pytest
```

`tests/test_e2e.py` runs the full chain on one machine: simulated base receiver on a virtual serial port, the hub, and Visual-Tracker's `BaseLink`. It covers heartbeats, discovery, cues, HINT delivery as `TX` lines, and recovery after the receiver is disconnected. It requires a Visual-Tracker checkout: `VISUAL_TRACKER=/path/to/Visual-Tracker`.
