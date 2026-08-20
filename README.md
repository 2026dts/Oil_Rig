# Oil Rig PLC Monitoring & Motor Control System

Modbus TCP bridge that reads a Delta **DVP-10SX PLC** (via a **USR-W630** serial-to-Ethernet converter) and publishes rig telemetry — environment, wind, and 12V DC motor data — to a **ThingsBoard CE** dashboard. The dashboard can also switch the motor ON/OFF, which is written back to the PLC.

For the full write-up (architecture diagrams, register map, calibration notes, troubleshooting), see **`Oil_Rig_Implementation_Guide.docx`**. This README is the quick-start / working reference.

---

## Contents

- [Architecture](#architecture)
- [Repo Contents](#repo-contents)
- [Requirements](#requirements)
- [Setup](#setup)
- [Running](#running)
- [PLC Register Map](#plc-register-map)
- [ThingsBoard Telemetry Keys](#thingsboard-telemetry-keys)
- [Motor Runtime Persistence](#motor-runtime-persistence)
- [Troubleshooting](#troubleshooting)
- [Known Limitations / TODO](#known-limitations--todo)

---

## Architecture

```
Sensors                       PLC                Gateway/PC              Cloud
--------                  -----------           ------------          ----------
ENV-485 (temp/hum/          Delta                                     ThingsBoard
 pressure/gas)  ─┐        DVP-10SX      RS-232    USR-W630     TCP     dashboard
Anemometer      ─┼─RS485─▶ (mirrors  ──────────▶ (serial→WiFi) ──────▶ (MQTT +
Current amp/shunt┤        into regs                :8899               RPC)
 (motor current) │        4096-4107)
Voltage sensor  ─┘             │
                                ▼
                          12V DC motor
                        (TOPSFLO TM30A
                          diaphragm pump)
```

`oil_rig.py` polls the PLC's 12 holding registers over Modbus TCP every 2s, converts raw values to engineering units, computes derived fields (motor power, flow rate, runtime, IAQ), and publishes everything to ThingsBoard over MQTT. It also listens for RPC commands from the dashboard to turn the motor ON/OFF by writing back to the PLC.

Full wiring diagrams: `reference Diagram.png` (clean block diagram), `overall_circuit.jpeg` / `Plc diagram.jpeg` (hand-marked), `Circuit_Diagram_of_PLC.pdf` (3-page EPLAN schematic — treat as authoritative when the hand sketches are ambiguous).

## Repo Contents

| File | Purpose |
|---|---|
| `oil_rig.py` | Main PLC → ThingsBoard bridge (telemetry + RPC motor control) |
| `wind_speed.py` | Standalone, read-only register diagnostic tool |
| `motor_runtime_state.json` | Persisted cumulative motor runtime (auto-generated) |
| `wind_speed_diagnostic_log.csv` | Raw register log produced by `wind_speed.py` |
| `overall_circuit.jpeg`, `Plc diagram.jpeg` | Hand-marked wiring references |
| `reference Diagram.png` | Clean digital block diagram |
| `Circuit_Diagram_of_PLC.pdf` | 3-page EPLAN-style formal schematic |
| `Oil_Rig_Implementation_Guide.docx` | Full implementation guide (this README's longer sibling) |

## Requirements

- Python 3.8+
- Network reachability from the gateway PC to the USR-W630's IP on TCP port 8899
- A ThingsBoard CE instance + device access token

```bash
pip install pymodbus paho-mqtt --break-system-packages
# (drop --break-system-packages on Windows / inside a venv)
```

## Setup

1. Wire the panel per the diagrams above. Double-check shunt polarity (+ve/−ve) and current-amplifier terminal numbers (5,6,7,8 input / 1,2 output) before powering on.
2. Power up the 12V SMPS and 24V DC SMPS; confirm the PLC's POWER and RUN LEDs are lit.
3. Confirm PLC ladder logic mirrors ENV-485, anemometer, current-amplifier and voltage-sensor readings into holding registers `4096`–`4106`, and that register `4107` accepts a motor ON/OFF write.
4. Configure the USR-W630 as a **TCP Server** on port `8899`, matching the PLC's serial baud rate.
5. Edit the connection block at the top of `oil_rig.py`:

   ```python
   PLC_IP = "10.158.91.46"        # USR-W630 IP, Server mode, TCP
   PLC_PORT = 8899
   PLC_SLAVE_ID = 1

   THINGSBOARD_HOST = "allcad-chennai.selfip.com"
   THINGSBOARD_MQTT_PORT = 1883
   THINGSBOARD_ACCESS_TOKEN = "<device access token>"

   READ_INTERVAL_SEC = 2
   ```

   Generate a fresh device access token per deployment (ThingsBoard → Devices → your device → Manage credentials) rather than reusing one from another unit.

6. Create the device in ThingsBoard and add widgets for the telemetry keys listed below.

## Running

```bash
python oil_rig.py
```

Expect to see `Connected to PLC via USR-W630` and `Connected to ThingsBoard, rc = 0` in the console, followed by a telemetry print-out every 2 seconds. Cross-check a couple of values (temperature, motor_voltage) against a known-good reference before trusting the feed.

**What it does:**
- Loads any previously persisted `motor_runtime_state.json` on startup
- Reads all 12 registers every `READ_INTERVAL_SEC`, scales to engineering units, computes `motor_power` (V×I), `flow_rate`, uptime/downtime/total-runtime, and an approximate IAQ index
- Publishes to ThingsBoard over MQTT
- Listens for RPC (`getValue`, `setMotorState`) and writes motor ON/OFF back to register `4107`
- Auto-reconnects the Modbus TCP link if `wind_speed` repeats an identical value for 10 consecutive reads
- Flushes runtime counters to disk on clean shutdown (Ctrl+C)

### Diagnostic tool

```bash
python wind_speed.py
```

Read-only — never writes to the PLC or publishes to ThingsBoard. Logs all 12 raw registers to `wind_speed_diagnostic_log.csv` every 2s. Run for 15–30 minutes (ideally while conditions are actually changing), then Ctrl+C for a per-register "% of reads changed" summary. Safe to run alongside `oil_rig.py`.

## PLC Register Map

Base address `4096`, 12 contiguous holding registers, read as one block per poll.

| Address (index) | Field | Scaling | Unit | Status |
|---|---|---|---|---|
| 4096 (0) | `temperature` | raw / 100 | °C | Confirmed |
| 4097 (1) | `humidity` | raw / 100 | %RH | Confirmed |
| 4098–4099 (2,3) | `pressure` | (Hi<<16 \| Lo) / 100 | hPa | Confirmed |
| 4100–4101 (4,5) | `gas_resistance` | (Hi<<16 \| Lo) | Ohm | Confirmed |
| 4102 (6) | `wind_speed` | raw / 10 | m/s | **Unconfirmed** |
| 4103 (7) | `wind_direction` | 2=Right, 3=Left | – | Confirmed (rotation sense only) |
| 4104 (8) | `motor_voltage` | raw × 1 (whole volts) | V | **Unconfirmed** |
| 4105 (9) | `motor_current` | raw × 0.08 (`CURRENT_SCALE`) | A | Rescaled estimate |
| 4106 (10) | `motor_power_plc_raw` | raw × 1 | W | Diagnostics only, not used |
| 4107 (11) | `motor_control` | 0 / 1 | – | Also the write address for ON/OFF |

> ⚠️ **Calibration warning:** `wind_speed`, `motor_voltage` and `motor_current` scale factors are not yet confirmed against a manufacturer datasheet or clamp-meter reading. `CURRENT_SCALE` (currently `0.08` in `oil_rig.py`) is a single-point estimate — recalibrate with a real clamp-meter reading taken while the pump is actually running under load if accuracy matters.

## ThingsBoard Telemetry Keys

| Key(s) | Description |
|---|---|
| `temperature`, `humidity`, `pressure`, `gas_resistance` | Environmental readings |
| `iaq_index`, `iaq_label` | Derived air-quality score (0–500) and label — heuristic, not Bosch BSEC |
| `wind_speed`, `wind_direction`, `wind_direction_text` | Anemometer readings |
| `motor_voltage`, `motor_current`, `motor_power` | Motor electrical telemetry (`motor_power` = V × I, calculated) |
| `motor_control`, `flow_rate` | Motor ON/OFF state and derived flow rate (L/min) |
| `motor_uptime_min`, `motor_downtime_min`, `motor_total_running_min` | Runtime counters |

**RPC methods** (dashboard → script):
- `getValue` → returns current motor state (bool)
- `setMotorState` → `{ "success": bool, "state": 0|1 }`, writes register `4107`

## Motor Runtime Persistence

`motor_runtime_state.json` stores a single field, `total_running_sec`, so cumulative pump runtime survives script restarts, crashes, and reboots. Written roughly once a minute (every 30 reads at the default 2s interval) and flushed on clean shutdown.

```json
{"total_running_sec": 2982.0}
```

≈ 49.7 minutes cumulative as of the last save in this repo snapshot.

## Troubleshooting

**Registers appear frozen** — Run `wind_speed.py` for 15–30 min and check the per-register change summary. In prior diagnostic runs, *every register except `motor_current` and `motor_power_raw`* froze for the full session. Since `motor_current` comes in through the current amplifier's own dedicated analog channel and kept changing normally, this points to the **RS-485/PLC refresh path** (ENV-485, anemometer, voltage sensor mirroring) rather than the USR-W630 ↔ Python TCP link. Check RS-485 A/B (yellow/green) wiring and whether the PLC ladder logic is actually re-polling those slaves.

**Modbus read errors / connection drops**
- Confirm the USR-W630 is powered and its TCP Server is listening on the configured port.
- Check the framer (`'ascii'`) matches the PLC's serial configuration.
- A high `error_count` in `wind_speed.py`'s summary points to the USR-W630 ↔ PLC serial link; if it's low but specific registers still don't change, suspect the sensor/ladder-logic side instead.

**Motor current/voltage look wrong** — Scale factors are estimates (see register map above); recalibrate against a multimeter/clamp-meter reading taken under actual running load.

**ThingsBoard shows no telemetry**
- Verify `THINGSBOARD_ACCESS_TOKEN` matches the device exactly.
- Confirm outbound MQTT (port 1883) isn't blocked by a firewall.
- Check the console for `Connected to ThingsBoard, rc = 0` — any non-zero `rc` means an auth or connection problem.

## Known Limitations / TODO

- [ ] Confirm `wind_speed`, `motor_voltage` scale factors against actual sensor datasheets
- [ ] Recalibrate `CURRENT_SCALE` with a clamp-meter reading under load
- [ ] Investigate root cause of the RS-485/PLC register-freeze issue (see Troubleshooting)
- [ ] IAQ index is a simplified heuristic, not Bosch BSEC — treat as a relative trend indicator only
- [ ] `motor_power_plc_raw` (register 4106) is read but currently unused/diagnostic-only

---

*Last updated: August 2026 — Oomnieye Digital Twin Solutions Pvt Ltd*
