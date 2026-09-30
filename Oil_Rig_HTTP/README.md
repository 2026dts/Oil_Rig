# Oil Rig PLC Monitor (HTTP Web Dashboard)

A single-file Flask application (`PLC.py`) that reads a PLC over Modbus through a USR-W630
serial-to-WiFi gateway, shows live values on a web dashboard, stores history in SQLite, runs a
wind-sensor watchdog that power-cycles a relay, and raises a browser alarm when wind speed gets
too high while the pump is running.

```
 PLC (Modbus ASCII)  ──►  USR-W630 gateway  ──►  PLC.py (Flask, port 5000)  ──►  Any browser on the network
   holding regs 4096+       10.10.12.76:8899        │                                http://<host>:5000
                                                    ├── plc_data.db              (SQLite history + alerts)
                                                    ├── motor_runtime_state.json (running / idle totals)
                                                    └── Relay at 10.10.12.187    (wind watchdog power cycle)
```

---

## 1. Folder layout

```
Oil_Rig/
├── PLC.py                      # the whole application
├── audio/
│   └── audio_alert.mp3         # alarm sound played in the browser (you provide this file)
├── plc_data.db                 # created automatically on first run
└── motor_runtime_state.json    # created automatically on first run
```

Run `PLC.py` from inside this folder. The database and runtime files are created in the
current working directory, and the audio file is looked up next to `PLC.py`.

---

## 2. Requirements and install

- Python 3.9 or newer
- Network access to the USR-W630 (`PLC_IP:PLC_PORT`) and to the relay (`10.10.12.187`)

```bash
pip install flask pymodbus
```

The code uses the `device_id=` keyword on Modbus calls, which needs a recent pymodbus (3.x).
SQLite is part of Python, so nothing else is needed. The `sqlite3` command-line tool is only
useful for inspecting the database (`sudo apt install sqlite3`).

The Overview page loads Chart.js 3.9.1 from `cdnjs.cloudflare.com`. The browser viewing the
dashboard needs internet access for the historical graphs. Everything else works offline.

---

## 3. Run

```bash
cd ~/Oil_Rig
python PLC.py
```

Open `http://localhost:5000` on the machine itself, or `http://<machine-ip>:5000` from any
other computer or phone on the same network.

To restart after replacing the file:

```bash
killall python
python PLC.py
```

Then press **Ctrl+Shift+R** in the browser so it loads the new page code.

### Optional: run as a systemd service

`/etc/systemd/system/plc-dashboard.service`

```ini
[Unit]
Description=Oil Rig PLC Dashboard
After=network-online.target
Wants=network-online.target

[Service]
User=minipc62
WorkingDirectory=/home/minipc62/Oil_Rig
ExecStart=/usr/bin/python3 /home/minipc62/Oil_Rig/PLC.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now plc-dashboard
sudo systemctl restart plc-dashboard      # after updating PLC.py
journalctl -u plc-dashboard -f            # live log
```

---

## 4. Configuration (top of `PLC.py`)

### PLC link

| Setting | Value | Meaning |
|---|---|---|
| `PLC_IP` | `10.10.12.76` | USR-W630 address |
| `PLC_PORT` | `8899` | USR-W630 TCP port |
| `PLC_SLAVE_ID` | `1` | Modbus slave / device id |
| `BASE_ADDRESS` | `4096` | First holding register read |
| `REGISTER_COUNT` | `12` | Registers read per poll |
| `READ_INTERVAL_SEC` | `2` | Poll period in seconds |
| `CURRENT_SCALE` | `0.08` | Raw current × this = amps |
| `MOTOR_CONTROL_WRITE_ADDRESS` | `4107` | Register written for pump ON (1) / OFF (0) |

The Modbus client uses **ASCII framing** over TCP (`framer="ascii"`, 3 s timeout).

### Storage

| Setting | Value | Meaning |
|---|---|---|
| `DB_PATH` | `plc_data.db` | SQLite database file |
| `RUNTIME_STATE_FILE` | `motor_runtime_state.json` | Total running / idle seconds |
| `RUNTIME_SAVE_EVERY_N_READS` | `30` | Save runtime totals every 30 polls (about 60 s) |
| `HISTORY_LEN` | `300` | In-memory samples for live widgets (300 × 2 s = 10 min) |

### Look and layout

| Setting | Value | Meaning |
|---|---|---|
| `TRANSPARENT_PAGE` | `True` | See-through page for embedding in another page |
| `TRANSPARENT_TEXT` | `"light"` | Text colour on the host background (`light` or `dark`) |
| `CARD_BG` | `rgba(186, 230, 253, 0.35)` | Card colour, last number is opacity |
| `COMPACT_UI` | `True` | Smaller cards and fonts |
| `WHITE_BG_PAGES` | `{"overview"}` | Pages that get a solid white background |

### Wind alarm and audio

| Setting | Value | Meaning |
|---|---|---|
| `WIND_ALERT_LIMIT` | `2.2` | m/s at or above which a critical alert is raised |
| `AUDIO_DIR` | `<folder of PLC.py>/audio` | Where `audio_alert.mp3` must be |

### Widget limits (`UI_LIMITS`)

These drive gauge colour zones, band bars and the status colours in the health matrix.

| Key | Values |
|---|---|
| `wind` | max 3, calm 1.0, high 2.0, danger 2.5 (m/s) |
| `temperature` | min -10, max 60, low 5, high 40, critical 50 (°C) |
| `humidity` | low 30, high 80 (%RH) |
| `pressure` | min 950, max 1060, low 980, high 1040 (hPa) |
| `voltage` | min 0, max 15, nominal 12, low 10.8, high 13.2, no_supply 1 (V) |
| `current` | max 10, warn 7, trip 9 (A) |
| `power` | max 150 (W, 100 % on the load ring) |

`UI_LIMITS["wind"]` only colours the widgets. The alarm uses `WIND_ALERT_LIMIT`.

---

## 5. PLC register map

Registers are read starting at 4096. Index = offset from 4096.

| Index | Address | Value | Conversion |
|---|---|---|---|
| 0 | 4096 | Temperature | signed 16-bit × 0.01 → °C |
| 1 | 4097 | Humidity | signed 16-bit × 0.01 → %RH |
| 2, 3 | 4098–4099 | Pressure | 32-bit (high word, low word) × 0.01 → hPa |
| 6 | 4102 | Wind speed | signed 16-bit × 0.1 → m/s |
| 8 | 4104 | Pump voltage | signed 16-bit → V |
| 9 | 4105 | Pump current | signed 16-bit × `CURRENT_SCALE` → A |
| 11 | 4107 | Pump state | non-zero = running (also the write register) |

Power is calculated in Python: `voltage × current` (W). Registers 4, 5, 7 and 10 are read but
not used (wind direction is intentionally ignored).

---

## 6. Web pages

All pages share the top bar (logo, alarm bell, clock, PLC link status) and the tab bar.

| Path | Page | Content |
|---|---|---|
| `/` | Overview | Pump switch, wind gauge, environment card, pump load bars, health matrix, pump activity, historical graphs |
| `/motor` | Pump Control | Large ON / OFF switch |
| `/anemometer` | Anemometer | Large wind gauge |
| `/temperature` | Environment | Thermometer, humidity ring, pressure gauge, dew point |
| `/pump` | Pump Monitor | Voltage and current gauges, power / load ring |

### Overview sections

1. **Top cards:** Pump Control (ON/OFF), Wind Speed gauge, Environment (temperature,
   humidity, pressure), Pump Load (power, voltage, current bars).
2. **System Health & Diagnostics Matrix:** pump, Modbus link, wind, temperature, humidity,
   pressure, voltage, current, load, duty cycle, current run, total running, total idle.
3. **Pump Activity:** Current Downtime, and State History (one bar per poll, green =
   running, red = stopped) with duty cycle.
4. **Time-Scale Graphs:** Temperature & Humidity (two Y axes), Voltage & Current (two Y
   axes), Pump Power and Pressure side by side. Each has 1H / 1D / 1W / 1M / 1Y buttons.

### Graph controls

- **Mouse wheel** over a graph: zoom in / out around the cursor
- **Click and drag:** pan left / right while zoomed
- **Double-click:** reset to the full range

Graphs average the data down to about 150 points, and the Y axis fits the visible data, so
small changes are visible. Zooming in brings back finer detail.

---

## 7. HTTP API

All responses are JSON unless noted.

### Live data

| Method | Path | Returns |
|---|---|---|
| GET | `/api/all` | Every current value, runtime counters, `connected`, `last_update` |
| GET | `/api/history` | Last 300 in-memory samples (about 10 min) |
| GET | `/api/temperature` | temperature, humidity, pressure |
| GET | `/api/anemometer` | wind_speed |
| GET | `/api/pump` | motor_voltage, motor_current, motor_power, motor_control |
| GET | `/api/motor` | motor_control, uptime / downtime, total running / idle minutes |

### Pump control

| Method | Path | Effect |
|---|---|---|
| POST | `/api/motor/on` | Writes 1 to register 4107. `200` on success, `503` if the PLC rejected it |
| POST | `/api/motor/off` | Writes 0 to register 4107. Same responses |

```bash
curl -X POST http://localhost:5000/api/motor/on
```

### Historical graphs

| Method | Path | Source |
|---|---|---|
| GET | `/api/graph/1h` | `readings` table (raw) |
| GET | `/api/graph/1d` | `hourly_agg` table |
| GET | `/api/graph/1w` | `daily_agg`, last 7 days |
| GET | `/api/graph/1m` | `daily_agg`, last 30 days |
| GET | `/api/graph/1y` | `daily_agg`, grouped by week |

The `?metric=` query parameter is accepted but ignored: every response contains all metrics.
See **Known issues** for the current state of these endpoints.

### Alarms

| Method | Path | Effect |
|---|---|---|
| GET | `/api/alerts` | Last 200 alerts (newest first), `unread` count, `limit` |
| POST | `/api/alerts/<id>/read` | Mark one alert as read |
| POST | `/api/alerts/read_all` | Mark every alert as read |
| GET | `/audio/audio_alert.mp3` | The alarm MP3 file |

---

## 8. Database (`plc_data.db`)

| Table | Written | Kept | Content |
|---|---|---|---|
| `readings` | every poll (2 s) | 7 days | raw temperature, humidity, pressure, wind, voltage, current, power, pump state |
| `hourly_agg` | when the hour changes | forever | hourly averages and running-sample count |
| `daily_agg` | when the date changes | forever | daily min / max / avg per metric, running seconds |
| `alerts` | when an alarm fires | forever | ts, level, title, message, value, is_read |
| `app_state` | on alarm arm / disarm | forever | key/value; holds `wind_alarm_armed` so it survives restarts |

Old raw readings are deleted once a day, when the date changes.

Useful commands:

```bash
sqlite3 plc_data.db ".tables"
sqlite3 plc_data.db "SELECT COUNT(*) FROM readings;"
sqlite3 plc_data.db "SELECT datetime(timestamp,'unixepoch','localtime'), temperature, wind_speed, motor_state FROM readings ORDER BY id DESC LIMIT 5;"
sqlite3 plc_data.db "SELECT id, datetime(ts,'unixepoch','localtime'), title, value, is_read FROM alerts ORDER BY id DESC;"
du -h plc_data.db
```

Reset the alarm inbox (stop the app first):

```bash
sqlite3 plc_data.db "DELETE FROM alerts; DELETE FROM app_state;"
```

---

## 9. Wind watchdog (relay power cycle)

Runs after every successful poll. It detects a stuck or unpowered wind sensor.

| Rule | Condition | Setting |
|---|---|---|
| 1 | Pump **running** and wind reads **0 m/s** on every poll for 10 s | `WIND_ZERO_HOLD_SEC = 10` |
| 2 | Pump **stopped** and wind reads **above 0** on every poll for 15 s | `WIND_STUCK_OFF_HOLD_SEC = 15` |

When either rule fires:

1. `RELAY_OFF_URL` is called.
2. Wait `RELAY_OFF_TIME_SEC` (5 s).
3. `RELAY_ON_URL` is called, retried up to `RELAY_ON_RETRIES` (3) times.
4. If `RESTORE_PUMP_AFTER_RELAY` is on, the pump state from before the cycle is re-sent until
   the PLC reports it steadily for `RESTORE_STABLE_SEC` (6 s), for at most
   `RESTORE_TIMEOUT_SEC` (60 s).
5. If someone uses the dashboard switch during this, the restore is cancelled.

The cycle runs in its own thread, so polling and the web page keep working. A failed read
resets both rule timers. If the app is stopped in the middle of a cycle, it switches the relay
back ON before exiting.

| Setting | Default |
|---|---|
| `WIND_RELAY_ENABLED` | `True` (master switch) |
| `WIND_STUCK_OFF_ENABLED` | `True` (rule 2 on/off) |
| `WIND_ZERO_THRESHOLD` | `0.0` m/s |
| `RELAY_ON_URL` | `http://10.10.12.187/relay/0?turn=on` |
| `RELAY_OFF_URL` | `http://10.10.12.187/relay/0?turn=off` |
| `RELAY_HTTP_TIMEOUT` | `3` s |
| `RELAY_COOLDOWN_SEC` | `0` s |
| `RESTORE_SETTLE_SEC` | `2` s |

---

## 10. Wind alarm and notifications

### When an alert is raised (server side)

- Pump is **running** and wind is **≥ `WIND_ALERT_LIMIT` (2.2 m/s)**.
- Only **one alert per pump run**. After it fires, the alarm is disarmed.
- It re-arms only when the pump goes **OFF → ON** again.
- No alert while the pump is off.
- The armed state is saved in `app_state`, so restarting the app mid-run does not create a
  duplicate.

Because this runs in `PLC.py`, alerts are recorded even when no browser is open.

### Bell and inbox (browser side)

- Top-right bell shows a red badge with the unread count.
- Clicking the bell opens **Alarms & Alerts** with **All / Unread / Read** tabs and
  **Mark All Read**.
- Clicking an alert marks it read. It stays in the list, like mail. The count only includes
  unread alerts.
- The page checks for alerts every 3 s, on every tab of the dashboard.

### Alarm sound

- Plays `audio/audio_alert.mp3` **in the browser**, so it is heard on whichever computer or
  phone has the dashboard open, not on the server.
- Plays only for unread **Critical Wind Speed** alerts.
- Plays the whole MP3, waits 5 s (`ALARM_GAP_MS` in `NOTIF_JS`), then repeats.
- Stops as soon as the alert is read. Marking it read on one computer stops the sound on the
  others within about 3 s.
- The bell turns red and pulses while the alarm is active.

**Browser autoplay rule:** browsers do not allow sound until the user has clicked or pressed a
key on the page. If an alert is active before that, a red **"Click to enable alarm sound"**
button appears next to the bell. Any click on the page starts the sound.

For a control-room screen that should sound without any click, allow autoplay for the
dashboard address once:

- **Firefox:** click the icon left of the address bar → **Autoplay** → **Allow Audio and Video**.
- **Chrome:** autoplay is allowed automatically after regular use of the site, but this is not
  guaranteed. Firefox is the reliable choice for an unattended screen.

If the MP3 is missing, `/audio/audio_alert.mp3` returns 404 and there is no sound. Alerts and
the bell still work.

---

## 11. Runtime counters

- **Up / Down:** time since the pump last changed state (resets on every change).
- **Total running / Total idle:** add 2 s per poll, saved to `motor_runtime_state.json` about
  every 60 s and on normal shutdown. Up to about 60 s can be lost on a power cut.
- To reset the totals: stop the app, delete `motor_runtime_state.json`, start the app.

---

## 12. Troubleshooting

| Symptom | Check |
|---|---|
| "PLC offline" | Can the machine reach `PLC_IP:PLC_PORT`? `nc -vz 10.10.12.76 8899`. Check the gateway is powered and in Modbus ASCII mode. |
| Values freeze, then "Reconnecting" in the log | The app reconnects when the wind register stays identical for 10 polls (`STALE_THRESHOLD`). Frequent reconnects point to the gateway or wiring. |
| Pump switch says "PLC did not accept the command" | The write to register 4107 failed. Check the link and that the PLC accepts writes there. |
| Relay does not switch | Open `RELAY_ON_URL` in a browser from the server. Look for `[RELAY]` lines in the log. |
| No alarm sound | Is `audio/audio_alert.mp3` present? Is the "Click to enable alarm sound" button showing? Is the browser tab muted? |
| Alert did not appear | Was the pump running? Has an alert already fired this run? Check `sqlite3 plc_data.db "SELECT * FROM app_state;"` (`0` = disarmed until next pump start). |
| Graphs blank | The browser needs internet for Chart.js. Also see Known issues. |
| Page looks unchanged after an update | Hard refresh with Ctrl+Shift+R. |

Log prefixes: `[PLC]`, `[DATA]`, `[DB]`, `[WIND-WATCH]`, `[RELAY]`, `[RESTORE]`, `[ALARM]`,
`[runtime]`, `[API]`.

---

## 13. Known issues

These were found by testing the current code with sample data. They affect only the
historical graphs. Live values, pump control, the watchdog and the alarm are not affected.

| # | Area | Problem | Effect |
|---|---|---|---|
| 1 | `/api/graph/1h` | The filter `timestamp >= (datetime('now') - 3600)` compares against a text date, so it matches every row. | 1H shows **all stored readings (up to 7 days)**, not the last hour. |
| 2 | `/api/graph/1d` | `hour_bucket` (a number) is compared with `datetime('now','-1 day')` (text). In SQLite a number is never ≥ text. | 1D is **always empty**. |
| 3 | `/api/graph/1w`, `/1m` | The query does not select `motor_voltage_avg` / `motor_current_avg`, but the code reads them. | **HTTP 500** as soon as `daily_agg` has rows. |
| 4 | `/api/graph/1y` | The code reads `temperature_avg`, `power`, `voltage`, `current`, which are not columns of the query. | **HTTP 500** as soon as `daily_agg` has rows. |
| 5 | `aggregate_daily()` | It runs just after midnight but summarises the **new** day, not the day that just ended. | Completed days are not stored correctly, so 1W / 1M / 1Y have little or wrong data. |
| 6 | `aggregate_hourly()` | Labels the previous hour's data with the new hour's start time. | 1D points are shifted by one hour. |
| 7 | Graph widgets | The footer text "Needs 1h data / Hourly avg" is fixed placeholder text. | Cosmetic only. |

---

## 14. Quick reference

```bash
# start
cd ~/Oil_Rig && python PLC.py

# restart
killall python && python PLC.py

# open
http://localhost:5000
http://<machine-ip>:5000

# latest readings
sqlite3 plc_data.db "SELECT datetime(timestamp,'unixepoch','localtime'), wind_speed, motor_state FROM readings ORDER BY id DESC LIMIT 5;"

# alert inbox
curl http://localhost:5000/api/alerts

# pump on / off
curl -X POST http://localhost:5000/api/motor/on
curl -X POST http://localhost:5000/api/motor/off
```
