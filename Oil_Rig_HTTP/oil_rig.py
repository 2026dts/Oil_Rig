"""
PLC -> USR-W630 -> HTTP Web Dashboard  (widget edition)
=======================================================
Local Flask dashboard styled like the DG monitor: tabbed navigation,
widget cards (radial gauges, band bars, thermometer, rings, trends,
segmented motor switch) and a System Health & Diagnostics matrix.

UPDATED: Tracks total_idle_sec alongside total_running_sec for cumulative
idle time accumulation (persisted to disk, survives restart).

Pages:
    /              Overview (all widgets + health matrix)
    /motor         Pump Control: ON/OFF switch only
    /anemometer    Wind gauge, Beaufort scale, stats, trend
    /temperature   Thermometer, humidity ring, barometer, trends
    /pump          Pump Monitor: voltage, current, power, running time, idle time

HTTP APIs:
    GET  /api/motor
    POST /api/motor/on
    POST /api/motor/off
    GET  /api/anemometer
    GET  /api/temperature
    GET  /api/pump
    GET  /api/all
    GET  /api/history

Wind watchdog (backend), two rules:
    Rule 1: pump RUNNING and wind reads 0 m/s continuously for 10 s.
    Rule 2: pump STOPPED and wind still reads above 0 continuously for 15 s.
    Either rule: call RELAY_OFF_URL, wait 5 s, call RELAY_ON_URL. The pump
    state from before the relay cycle is then put back and held until the
    PLC confirms it.

Install:
    pip install flask pymodbus

Run:
    python plc_web_dashboard.py

Then open:
    http://localhost:5000
"""

import json
import os
import sqlite3
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime, timedelta

from flask import Flask, jsonify, render_template_string, request, send_from_directory
from pymodbus.client import ModbusTcpClient

# ----------------------------------------------------------------------
# PLC / USR-W630 SETTINGS
# ----------------------------------------------------------------------

PLC_IP = "10.10.12.76"
PLC_PORT = 8899
PLC_SLAVE_ID = 1

BASE_ADDRESS = 4096
REGISTER_COUNT = 12
READ_INTERVAL_SEC = 2

CURRENT_SCALE = 0.08
MOTOR_CONTROL_WRITE_ADDRESS = BASE_ADDRESS + 11  # 4107

# Runtime persistence
RUNTIME_STATE_FILE = "motor_runtime_state.json"
RUNTIME_SAVE_EVERY_N_READS = 30

# Trend history kept in memory (300 samples x 2 s = 10 minutes)
HISTORY_LEN = 300

# Transparent page for embedding (e.g. inside the Cupola server).
# True  = only the widget cards and tiles have a background; the page,
#         header and tabs are see-through so the host page shows behind.
# False = normal grey page background with white header and tabs.
TRANSPARENT_PAGE = True
# Colour of the text that sits directly on the host background (logo name,
# tabs, PLC status, matrix heading): "light" for a dark/photo background,
# "dark" for a white/light background.
TRANSPARENT_TEXT = "light"
# Widget card background when TRANSPARENT_PAGE is on: light blue, see-through.
# Last number = opacity (0 = fully clear, 1 = solid). 0.35 keeps the host visible.
CARD_BG = "rgba(186, 230, 253, 0.35)"
# Smaller widgets, text and tabs so the dashboard fits in a small embedded panel.
COMPACT_UI = True
# Pages that get a solid white background instead of the see-through one.
# Keys: "overview", "motor", "anemometer", "temperature", "pump".
WHITE_BG_PAGES = {"overview"}

# SQLite Database
DB_PATH = "plc_data.db"

# Alarm audio file. Expected path: <project>/audio/audio_alert.mp3
AUDIO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "audio")

# Wind alarm: one CRITICAL alert per pump run when wind >= this value (m/s).
# The next alert can only fire after the pump is switched OFF and ON again.
WIND_ALERT_LIMIT = 2.2

# ----------------------------------------------------------------------
# SQLITE SETUP
# ----------------------------------------------------------------------

def init_db():
    """Create SQLite database and 3 tables if they don't exist."""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # Table 1: Raw readings (every 2 sec)
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL NOT NULL,
            temperature REAL,
            humidity REAL,
            pressure REAL,
            wind_speed REAL,
            motor_voltage REAL,
            motor_current REAL,
            motor_power REAL,
            motor_state INTEGER
        )
    ''')
    cursor.execute('CREATE INDEX IF NOT EXISTS idx_readings_timestamp ON readings(timestamp)')
    
    # Table 2: Hourly aggregates
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS hourly_agg (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            hour_bucket REAL NOT NULL,
            temp_avg REAL,
            humidity_avg REAL,
            pressure_avg REAL,
            wind_avg REAL,
            motor_voltage_avg REAL,
            motor_current_avg REAL,
            motor_power_avg REAL,
            motor_running_count INTEGER,
            sample_count INTEGER
        )
    ''')
    cursor.execute('CREATE INDEX IF NOT EXISTS idx_hourly_bucket ON hourly_agg(hour_bucket)')
    # One row per hour: remove duplicates left by older versions, then enforce it.
    cursor.execute('''
        DELETE FROM hourly_agg WHERE id NOT IN (
            SELECT MAX(id) FROM hourly_agg GROUP BY hour_bucket)
    ''')
    cursor.execute('CREATE UNIQUE INDEX IF NOT EXISTS idx_hourly_bucket_u ON hourly_agg(hour_bucket)')
    
    # Table 3: Daily aggregates
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS daily_agg (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            day_date TEXT NOT NULL UNIQUE,
            temp_min REAL,
            temp_max REAL,
            temp_avg REAL,
            humidity_min REAL,
            humidity_max REAL,
            humidity_avg REAL,
            pressure_min REAL,
            pressure_max REAL,
            pressure_avg REAL,
            wind_max REAL,
            wind_avg REAL,
            motor_voltage_avg REAL,
            motor_current_avg REAL,
            motor_power_max REAL,
            motor_power_avg REAL,
            motor_running_sec REAL,
            sample_count INTEGER
        )
    ''')
    cursor.execute('CREATE INDEX IF NOT EXISTS idx_daily_date ON daily_agg(day_date)')

    # Table 4: Alarm inbox (kept forever, like mail)
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS alerts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts REAL NOT NULL,
            level TEXT NOT NULL,
            title TEXT NOT NULL,
            message TEXT,
            value REAL,
            is_read INTEGER NOT NULL DEFAULT 0
        )
    ''')
    # Small key/value store (alarm arming state survives restarts)
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS app_state (
            key TEXT PRIMARY KEY,
            value TEXT
        )
    ''')

    conn.commit()
    conn.close()
    print("[DB] Initialized SQLite (readings, hourly_agg, daily_agg, alerts)")


def insert_reading(temp, humidity, pressure, wind, voltage, current, power, motor_state):
    """Insert one raw reading into readings table."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO readings (timestamp, temperature, humidity, pressure, wind_speed,
                                  motor_voltage, motor_current, motor_power, motor_state)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (time.time(), temp, humidity, pressure, wind, voltage, current, power, motor_state))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[DB] Error inserting reading: {e}")


HOURLY_SQL = """
    INSERT OR REPLACE INTO hourly_agg
        (hour_bucket, temp_avg, humidity_avg, pressure_avg, wind_avg,
         motor_voltage_avg, motor_current_avg, motor_power_avg, motor_running_count, sample_count)
    SELECT CAST(timestamp / 3600 AS INTEGER) * 3600,
           AVG(temperature), AVG(humidity), AVG(pressure), AVG(wind_speed),
           AVG(motor_voltage), AVG(motor_current), AVG(motor_power),
           SUM(motor_state = 1), COUNT(*)
    FROM readings
    WHERE timestamp >= ? AND timestamp < ?
    GROUP BY CAST(timestamp / 3600 AS INTEGER)
"""

DAILY_SQL = """
    INSERT OR REPLACE INTO daily_agg
        (day_date, temp_min, temp_max, temp_avg, humidity_min, humidity_max, humidity_avg,
         pressure_min, pressure_max, pressure_avg, wind_max, wind_avg,
         motor_voltage_avg, motor_current_avg, motor_power_max, motor_power_avg,
         motor_running_sec, sample_count)
    SELECT date(timestamp, 'unixepoch', 'localtime') AS d,
           MIN(temperature), MAX(temperature), AVG(temperature),
           MIN(humidity), MAX(humidity), AVG(humidity),
           MIN(pressure), MAX(pressure), AVG(pressure),
           MAX(wind_speed), AVG(wind_speed),
           AVG(motor_voltage), AVG(motor_current), MAX(motor_power), AVG(motor_power),
           SUM(motor_state = 1) * ?, COUNT(*)
    FROM readings
    WHERE timestamp >= ?
    GROUP BY d
"""

_agg_lock = threading.Lock()


def _local_day_start(days_ago=0):
    day = datetime.now().date() - timedelta(days=days_ago)
    return datetime.combine(day, datetime.min.time()).timestamp()


def refresh_aggregates():
    """Rebuild recent hourly and daily rows straight from the raw readings.

    Hourly: every completed hour of the last 6 hours.
    Daily:  yesterday and today (today is a running partial day).
    Safe to run any time; rows are replaced, never duplicated.
    """
    with _agg_lock:
        try:
            now = time.time()
            current_hour = int(now // 3600) * 3600
            conn = sqlite3.connect(DB_PATH, timeout=10)
            conn.execute(HOURLY_SQL, (current_hour - 6 * 3600, current_hour))
            conn.execute(DAILY_SQL, (READ_INTERVAL_SEC, _local_day_start(1)))
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"[DB] Error refreshing aggregates: {e}")


def repair_aggregates_once():
    """One-time repair for databases written by the old aggregation code,
    which stored hourly rows under the wrong hour and daily rows for the
    wrong day. Everything still covered by raw readings is rebuilt."""
    with _agg_lock:
        try:
            conn = sqlite3.connect(DB_PATH, timeout=10)
            done = conn.execute(
                "SELECT value FROM app_state WHERE key = 'agg_repaired_v2'").fetchone()
            first = conn.execute("SELECT MIN(timestamp) FROM readings").fetchone()[0]
            if done or first is None:
                conn.close()
                return
            first_hour = int(first // 3600) * 3600
            first_day = datetime.fromtimestamp(first).strftime('%Y-%m-%d')
            current_hour = int(time.time() // 3600) * 3600

            conn.execute("DELETE FROM hourly_agg WHERE hour_bucket >= ?", (first_hour,))
            conn.execute("DELETE FROM daily_agg WHERE day_date >= ?", (first_day,))
            conn.execute(HOURLY_SQL, (first_hour, current_hour))
            conn.execute(DAILY_SQL, (READ_INTERVAL_SEC, first))
            conn.execute("INSERT OR REPLACE INTO app_state (key, value) VALUES ('agg_repaired_v2', '1')")
            conn.commit()
            h = conn.execute("SELECT COUNT(*) FROM hourly_agg").fetchone()[0]
            d = conn.execute("SELECT COUNT(*) FROM daily_agg").fetchone()[0]
            conn.close()
            print(f"[DB] Aggregates rebuilt from raw readings: {h} hourly rows, {d} daily rows")
        except Exception as e:
            print(f"[DB] Error repairing aggregates: {e}")


def delete_old_readings(days=7):
    """Delete raw readings older than N days (keep aggregates forever)."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cutoff = time.time() - (days * 86400)
        cursor.execute('DELETE FROM readings WHERE timestamp < ?', (cutoff,))
        deleted = cursor.rowcount
        conn.commit()
        conn.close()
        if deleted > 0:
            print(f"[DB] Deleted {deleted} readings older than {days} days")
    except Exception as e:
        print(f"[DB] Error deleting old readings: {e}")

# ----------------------------------------------------------------------
# WIND WATCHDOG -> RELAY CYCLE
# Rule 1: pump RUNNING and wind stays 0 for WIND_ZERO_HOLD_SEC in a row.
# Rule 2: pump STOPPED and wind stays above 0 for WIND_STUCK_OFF_HOLD_SEC.
# Either rule: relay OFF, wait RELAY_OFF_TIME_SEC, relay ON. Afterwards the
# pump is put back to the state it had before the relay cycle.
# ----------------------------------------------------------------------

WIND_RELAY_ENABLED = True
WIND_ZERO_HOLD_SEC = 10            # rule 1: pump ON, wind 0 continuously this long
WIND_STUCK_OFF_ENABLED = True      # rule 2 on/off
WIND_STUCK_OFF_HOLD_SEC = 15       # rule 2: pump OFF, wind above 0 continuously this long
RELAY_OFF_TIME_SEC = 5             # time between relay OFF and relay ON
WIND_ZERO_THRESHOLD = 0.0          # wind speed <= this value counts as "0"
RELAY_ON_URL = "http://10.10.12.187/relay/0?turn=on"
RELAY_OFF_URL = "http://10.10.12.187/relay/0?turn=off"
RELAY_HTTP_TIMEOUT = 3             # seconds per HTTP request
RELAY_ON_RETRIES = 3               # ON is retried so the relay never stays off
RELAY_COOLDOWN_SEC = 0             # extra wait after a cycle before watching again

# After the relay cycle, put the pump back to its state from before the cycle.
RESTORE_PUMP_AFTER_RELAY = True
RESTORE_SETTLE_SEC = 2             # wait after relay ON before checking the pump
RESTORE_STABLE_SEC = 6             # pump must hold the state this long to count as restored
RESTORE_TIMEOUT_SEC = 60           # give up re-sending the command after this long

# While the relay cycle power-cycles the PLC link, keep showing the last known
# pump status instead of "Offline". The hold ends at the first good PLC read
# after relay ON. If the PLC is not back RELAY_OFFLINE_GRACE_SEC after relay ON,
# "Offline" is shown as normal (a real fault is never hidden for long).
RELAY_HIDE_OFFLINE = True
RELAY_OFFLINE_GRACE_SEC = 30

# ----------------------------------------------------------------------
# WIDGET LIMITS  (edit these to match your equipment)
# Used by gauges, band bars and the health matrix status colours.
# ----------------------------------------------------------------------

UI_LIMITS = {
    # m/s. Gauge runs 0 to max; calm/high/danger set the blue/green/amber/red zones.
    "wind": {"max": 3, "calm": 1.0, "high": 2.0, "danger": 2.5},
    # degC
    "temperature": {"min": -10, "max": 60, "low": 5, "high": 40, "critical": 50},
    # %RH
    "humidity": {"low": 30, "high": 80},
    # hPa
    "pressure": {"min": 950, "max": 1060, "low": 980, "high": 1040},
    # V. Below "no_supply" the pump is treated as de-energised, not faulty.
    "voltage": {"min": 0, "max": 15, "nominal": 12, "low": 10.8, "high": 13.2, "no_supply": 1},
    # A
    "current": {"max": 10, "warn": 7, "trip": 9},
    # W, rated pump power (100 % on the load ring) = 15 V x 10 A
    "power": {"max": 150},
}

# ----------------------------------------------------------------------
# SHARED STATE
# ----------------------------------------------------------------------

app = Flask(__name__)

plc_client = None
plc_lock = threading.Lock()
data_lock = threading.Lock()

latest_data = {
    "temperature": None,
    "humidity": None,
    "pressure": None,
    "wind_speed": None,
    "motor_voltage": None,
    "motor_current": None,
    "motor_power": None,
    "motor_control": 0,
    "motor_uptime_min": 0,
    "motor_downtime_min": 0,
    "motor_total_running_min": 0,
    "motor_total_idle_min": 0,  # NEW: total idle time (cumulative)
    "connected": False,
    "last_update": None,
}

HISTORY_KEYS = (
    "temperature",
    "humidity",
    "pressure",
    "wind_speed",
    "motor_voltage",
    "motor_current",
    "motor_power",
    "motor_control",
)
history = deque(maxlen=HISTORY_LEN)

motor_uptime_sec = 0
motor_downtime_sec = 0
total_running_sec = 0.0
total_idle_sec = 0.0  # NEW: cumulative idle time
_runtime_save_counter = 0

_last_wind_raw = None
_stale_count = 0
STALE_THRESHOLD = 10

# SQLite aggregation tracking
_last_hour_agg = int(time.time() / 3600)
_last_day_agg = datetime.now().date()

_prev_motor_state = None           # last pump state read from the PLC
_wind_zero_since = None            # rule 1: time wind first read 0 with the pump running
_wind_on_since = None              # rule 2: time wind first read >0 with the pump stopped
_watch_resume_at = 0.0             # watchdog paused until this time (cooldown)
_wind_check_lock = threading.Lock()
_wind_check_running = False
_relay_off_active = False          # True between relay OFF and a confirmed relay ON
_plc_read_seq = 0                  # increases on every successful PLC read
_last_manual_command_at = 0.0      # time of last ON/OFF click on the dashboard
_offline_hold_until = 0.0          # hide "Offline" until this time (relay cycle)
_relay_on_at = 0.0                 # time the relay was last confirmed ON


# ----------------------------------------------------------------------
# RUNTIME TRACKING
# ----------------------------------------------------------------------

def load_runtime_state():
    global total_running_sec, total_idle_sec
    try:
        with open(RUNTIME_STATE_FILE, "r") as f:
            saved = json.load(f)
            total_running_sec = float(saved.get("total_running_sec", 0.0))
            total_idle_sec = float(saved.get("total_idle_sec", 0.0))
        print(f"[runtime] Loaded total running: {total_running_sec / 3600:.2f} h")
        print(f"[runtime] Loaded total idle: {total_idle_sec / 3600:.2f} h")
    except (FileNotFoundError, json.JSONDecodeError, ValueError, TypeError):
        total_running_sec = 0.0
        total_idle_sec = 0.0
        print("[runtime] Starting runtime and idle counters from 0.")


def save_runtime_state():
    try:
        with open(RUNTIME_STATE_FILE, "w") as f:
            json.dump({
                "total_running_sec": total_running_sec,
                "total_idle_sec": total_idle_sec
            }, f)
    except Exception as exc:
        print("[runtime] Save error:", exc)


def update_motor_runtime(motor_state):
    global motor_uptime_sec, motor_downtime_sec
    global total_running_sec, total_idle_sec, _runtime_save_counter

    if motor_state:
        motor_uptime_sec += READ_INTERVAL_SEC
        motor_downtime_sec = 0
        total_running_sec += READ_INTERVAL_SEC
    else:
        motor_downtime_sec += READ_INTERVAL_SEC
        motor_uptime_sec = 0
        total_idle_sec += READ_INTERVAL_SEC

    _runtime_save_counter += 1
    if _runtime_save_counter >= RUNTIME_SAVE_EVERY_N_READS:
        save_runtime_state()
        _runtime_save_counter = 0

    return {
        "motor_uptime_min": round(motor_uptime_sec / 60, 2),
        "motor_downtime_min": round(motor_downtime_sec / 60, 2),
        "motor_total_running_min": round(total_running_sec / 60, 2),
        "motor_total_idle_min": round(total_idle_sec / 60, 2),
    }


# ----------------------------------------------------------------------
# WIND WATCHDOG -> RELAY CYCLE
# ----------------------------------------------------------------------

def call_relay(url):
    """Send one HTTP GET to the relay. Returns True on a 2xx response."""
    try:
        with urllib.request.urlopen(url, timeout=RELAY_HTTP_TIMEOUT) as resp:
            body = resp.read(200).decode(errors="replace")
            print(f"[RELAY] {url} -> HTTP {resp.status} {body}")
            return 200 <= resp.status < 300
    except Exception as exc:
        print(f"[RELAY] {url} failed: {exc}")
        return False


def relay_on_with_retry():
    global _relay_off_active, _relay_on_at, _offline_hold_until
    for attempt in range(1, RELAY_ON_RETRIES + 1):
        if call_relay(RELAY_ON_URL):
            _relay_off_active = False
            _relay_on_at = time.time()
            if _offline_hold_until:
                _offline_hold_until = _relay_on_at + RELAY_OFFLINE_GRACE_SEC
            return True
        print(f"[RELAY] ON attempt {attempt}/{RELAY_ON_RETRIES} failed")
        time.sleep(1)
    print("[RELAY] WARNING: relay ON was not confirmed. Check the relay manually.")
    _offline_hold_until = 0.0   # relay may still be off: show the real status
    return False


def wait_for_fresh_read(last_seq, timeout):
    """Block until the polling loop has done a new PLC read. Returns new seq or None."""
    end = time.time() + timeout
    while time.time() < end:
        if _plc_read_seq != last_seq:
            return _plc_read_seq
        time.sleep(0.2)
    return None


def restore_pump_state(target, sequence_started):
    """Re-send the pump command until the PLC reports `target` steadily.
    Stops if someone uses the dashboard switch meanwhile."""
    name = "RUNNING" if target else "STOPPED"
    print(f"[RESTORE] Pump was {name} before the relay cycle. Checking in {RESTORE_SETTLE_SEC} s...")
    time.sleep(RESTORE_SETTLE_SEC)

    deadline = time.time() + RESTORE_TIMEOUT_SEC
    seq = _plc_read_seq
    held_since = None
    attempt = 0

    while time.time() < deadline:
        if _last_manual_command_at > sequence_started:
            print("[RESTORE] Cancelled: pump was switched from the dashboard during the relay cycle.")
            return False

        new_seq = wait_for_fresh_read(seq, READ_INTERVAL_SEC * 3)
        if new_seq is None:
            print("[RESTORE] Waiting for the PLC link...")
            continue
        seq = new_seq

        if _prev_motor_state == target:
            held_since = held_since or time.time()
            if time.time() - held_since >= RESTORE_STABLE_SEC:
                print(f"[RESTORE] Pump confirmed {name} by the PLC.")
                return True
        else:
            held_since = None
            attempt += 1
            print(f"[RESTORE] PLC reports pump {'STOPPED' if target else 'RUNNING'}. "
                  f"Sending {'ON' if target else 'OFF'} (attempt {attempt})...")
            write_motor_control(target)

    print(f"[RESTORE] WARNING: pump {name} not confirmed within {RESTORE_TIMEOUT_SEC} s. "
          "Check the pump and PLC.")
    return False


def relay_cycle_sequence(pump_before, reason):
    """Relay OFF -> wait -> relay ON, then restore the pump state.
    Runs in its own thread so polling and the web UI are never blocked."""
    global _wind_check_running, _relay_off_active, _watch_resume_at, _offline_hold_until
    try:
        sequence_started = time.time()
        print(f"[WIND-WATCH] {reason}. Relay OFF for {RELAY_OFF_TIME_SEC} s.")
        _relay_off_active = True
        if RELAY_HIDE_OFFLINE:
            # covers OFF time + ON retries; shortened to the grace period once relay is ON
            _offline_hold_until = time.time() + RELAY_OFF_TIME_SEC + \
                RELAY_ON_RETRIES * (RELAY_HTTP_TIMEOUT + 1) + RELAY_OFFLINE_GRACE_SEC
        call_relay(RELAY_OFF_URL)
        try:
            time.sleep(RELAY_OFF_TIME_SEC)
        finally:
            relay_on_with_retry()
            print("[WIND-WATCH] Relay cycle finished (relay back ON).")

        if RESTORE_PUMP_AFTER_RELAY and pump_before is not None:
            restore_pump_state(pump_before, sequence_started)
    except Exception as exc:
        print("[WIND-WATCH] Error:", exc)
    finally:
        _watch_resume_at = time.time() + RELAY_COOLDOWN_SEC
        with _wind_check_lock:
            _wind_check_running = False


def start_relay_cycle(pump_before, reason):
    global _wind_check_running
    with _wind_check_lock:
        if _wind_check_running:
            return
        _wind_check_running = True
    threading.Thread(target=relay_cycle_sequence, args=(pump_before, reason), daemon=True).start()


def watch_wind(data):
    """Called after every PLC read (data=None when the read failed).
    Rule 1: pump RUNNING and wind 0 on every poll for WIND_ZERO_HOLD_SEC.
    Rule 2: pump STOPPED and wind above 0 on every poll for WIND_STUCK_OFF_HOLD_SEC.
    Either rule starts the relay cycle."""
    global _prev_motor_state, _plc_read_seq, _wind_zero_since, _wind_on_since

    if data is None:
        _wind_zero_since = _wind_on_since = None   # no reading = not continuous
        return

    motor_state = data["motor_control"]
    _prev_motor_state = motor_state
    _plc_read_seq += 1

    if not WIND_RELAY_ENABLED or _wind_check_running or time.time() < _watch_resume_at:
        _wind_zero_since = _wind_on_since = None   # restart the counts after a cycle
        return

    wind = data.get("wind_speed")
    if wind is None:
        _wind_zero_since = _wind_on_since = None
        return

    now = time.time()
    wind_is_zero = wind <= WIND_ZERO_THRESHOLD

    # Rule 1: pump running, no wind
    if motor_state == 1 and wind_is_zero:
        if _wind_zero_since is None:
            _wind_zero_since = now
            print(f"[WIND-WATCH] Pump RUNNING and wind 0 m/s. Counting {WIND_ZERO_HOLD_SEC} s...")
        elif now - _wind_zero_since >= WIND_ZERO_HOLD_SEC:
            _wind_zero_since = None
            start_relay_cycle(motor_state, f"Rule 1: wind 0 m/s for {WIND_ZERO_HOLD_SEC} s with pump RUNNING")
    else:
        _wind_zero_since = None

    # Rule 2: pump stopped, wind still showing
    if WIND_STUCK_OFF_ENABLED and motor_state == 0 and not wind_is_zero:
        if _wind_on_since is None:
            _wind_on_since = now
            print(f"[WIND-WATCH] Pump STOPPED but wind {wind} m/s. Counting {WIND_STUCK_OFF_HOLD_SEC} s...")
        elif now - _wind_on_since >= WIND_STUCK_OFF_HOLD_SEC:
            _wind_on_since = None
            start_relay_cycle(motor_state, f"Rule 2: wind above 0 for {WIND_STUCK_OFF_HOLD_SEC} s with pump STOPPED")
    else:
        _wind_on_since = None


# ----------------------------------------------------------------------
# MODBUS
# ----------------------------------------------------------------------

def create_plc_client():
    return ModbusTcpClient(
        host=PLC_IP,
        port=PLC_PORT,
        framer="ascii",
        timeout=3,
    )


def connect_plc():
    global plc_client

    with plc_lock:
        try:
            if plc_client:
                plc_client.close()
        except Exception:
            pass

        plc_client = create_plc_client()

        try:
            connected = plc_client.connect()
        except Exception as exc:
            print("[PLC] Connection error:", exc)
            connected = False

    with data_lock:
        latest_data["connected"] = bool(connected)

    if connected:
        print(f"[PLC] Connected to USR-W630 {PLC_IP}:{PLC_PORT}")
    else:
        print(f"[PLC] Connection failed: {PLC_IP}:{PLC_PORT}")

    return connected


def read_plc_data():
    """Read and scale only the values required by the web dashboard."""
    global _last_wind_raw, _stale_count

    with plc_lock:
        if plc_client is None:
            return None

        try:
            rr = plc_client.read_holding_registers(
                address=BASE_ADDRESS,
                count=REGISTER_COUNT,
                device_id=PLC_SLAVE_ID,
            )
        except Exception as exc:
            print("[PLC] Read exception:", exc)
            return None

    if rr is None or rr.isError():
        print("[PLC] Modbus read error:", rr)
        return None

    regs = rr.registers
    if len(regs) < REGISTER_COUNT:
        print("[PLC] Incomplete register response")
        return None

    # Temperature and humidity: confirmed /100 scaling from existing code.
    temperature_raw = regs[0]
    humidity_raw = regs[1]

    if temperature_raw > 32767:
        temperature_raw -= 65536

    if humidity_raw > 32767:
        humidity_raw -= 65536

    temperature = round(temperature_raw * 0.01, 2)
    humidity = round(humidity_raw * 0.01, 2)

    # Pressure = 32-bit value made from high + low words, then /100.
    pressure_combined = (regs[2] << 16) | regs[3]
    pressure = round(pressure_combined * 0.01, 2)

    # Wind speed only. Wind direction intentionally excluded.
    wind_raw = regs[6]
    if wind_raw > 32767:
        wind_raw -= 65536
    wind_speed = round(wind_raw * 0.1, 2)

    # Existing motor electrical scaling.
    voltage_raw = regs[8]
    current_raw = regs[9]

    if voltage_raw > 32767:
        voltage_raw -= 65536
    if current_raw > 32767:
        current_raw -= 65536

    motor_voltage = round(voltage_raw, 3)
    motor_current = current_raw * CURRENT_SCALE
    motor_power = round(motor_voltage * motor_current, 2)

    motor_control = 1 if regs[11] else 0

    runtime = update_motor_runtime(motor_control)

    # Existing stale-data strategy watches wind speed.
    if _last_wind_raw is not None and regs[6] == _last_wind_raw:
        _stale_count += 1
    else:
        _stale_count = 0

    _last_wind_raw = regs[6]

    data = {
        "temperature": temperature,
        "humidity": humidity,
        "pressure": pressure,
        "wind_speed": wind_speed,
        "motor_voltage": motor_voltage,
        "motor_current": motor_current,
        "motor_power": motor_power,
        "motor_control": motor_control,
        **runtime,
        "connected": True,
        "last_update": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    return data


def write_motor_control(state):
    """Write motor command. state=1 ON, state=0 OFF."""
    if state not in (0, 1):
        return False

    with plc_lock:
        if plc_client is None:
            return False

        try:
            result = plc_client.write_register(
                address=MOTOR_CONTROL_WRITE_ADDRESS,
                value=state,
                device_id=PLC_SLAVE_ID,
            )
        except Exception as exc:
            print("[PLC] Motor write exception:", exc)
            return False

    if result is None or result.isError():
        print("[PLC] Motor write failed:", result)
        return False

    print(f"[PLC] Motor command: {'ON' if state else 'OFF'}")

    # Do not fake the measured PLC state permanently. This makes the UI
    # responsive until the next poll confirms the actual register state.
    with data_lock:
        latest_data["motor_control"] = state

    return True


# ----------------------------------------------------------------------
# WIND ALARM (one alert per pump run)
# ----------------------------------------------------------------------
# Armed when the pump switches OFF -> ON. While armed and the pump is running,
# the first wind reading >= WIND_ALERT_LIMIT creates ONE critical alert and
# disarms. Nothing more is raised until the next OFF -> ON.

_alarm_lock = threading.Lock()
_alarm_armed = None       # loaded from app_state on first use
_alarm_prev_motor = None


def _state_get(key, default=None):
    try:
        conn = sqlite3.connect(DB_PATH)
        row = conn.execute("SELECT value FROM app_state WHERE key=?", (key,)).fetchone()
        conn.close()
        return row[0] if row else default
    except Exception:
        return default


def _state_set(key, value):
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.execute("INSERT OR REPLACE INTO app_state (key, value) VALUES (?, ?)", (key, str(value)))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[ALARM] state save error: {e}")


def insert_alert(level, title, message, value=None):
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.execute(
            "INSERT INTO alerts (ts, level, title, message, value, is_read) VALUES (?, ?, ?, ?, ?, 0)",
            (time.time(), level, title, message, value),
        )
        conn.commit()
        conn.close()
        print(f"[ALARM] {level.upper()}: {title} - {message}")
    except Exception as e:
        print(f"[ALARM] insert error: {e}")


def check_wind_alarm(data):
    global _alarm_armed, _alarm_prev_motor
    with _alarm_lock:
        if _alarm_armed is None:
            _alarm_armed = _state_get("wind_alarm_armed", "1") == "1"

        motor = 1 if data.get("motor_control") else 0

        # New pump run -> arm again
        if _alarm_prev_motor == 0 and motor == 1 and not _alarm_armed:
            _alarm_armed = True
            _state_set("wind_alarm_armed", "1")
        _alarm_prev_motor = motor

        wind = data.get("wind_speed")
        if not (_alarm_armed and motor == 1 and wind is not None):
            return
        try:
            wind = float(wind)
        except (TypeError, ValueError):
            return

        if wind >= WIND_ALERT_LIMIT:
            _alarm_armed = False
            _state_set("wind_alarm_armed", "0")
            insert_alert(
                "critical",
                "Critical Wind Speed",
                f"Wind reached {wind:.1f} m/s while the pump was running "
                f"(limit {WIND_ALERT_LIMIT} m/s). Strong wind can damage the rig "
                f"equipment. Check site conditions and consider stopping the pump.",
                wind,
            )


def plc_polling_loop():
    global _stale_count, _last_hour_agg, _last_day_agg, _offline_hold_until

    load_runtime_state()
    init_db()
    repair_aggregates_once()
    refresh_aggregates()

    while True:
        try:
            if plc_client is None or not latest_data.get("connected"):
                connect_plc()

            data = read_plc_data()

            if data:
                with data_lock:
                    latest_data.update(data)
                    sample = {key: data.get(key) for key in HISTORY_KEYS}
                    sample["t"] = data["last_update"]
                    history.append(sample)

                # PLC is back after a relay cycle: stop hiding "Offline"
                if _offline_hold_until and not _relay_off_active and \
                        _relay_on_at and time.time() >= _relay_on_at:
                    _offline_hold_until = 0.0
                    print("[RELAY] PLC link back after relay cycle.")

                # Insert into SQLite
                insert_reading(
                    data["temperature"], data["humidity"], data["pressure"],
                    data["wind_speed"], data["motor_voltage"], data["motor_current"],
                    data["motor_power"], data["motor_control"]
                )
                
                # Hourly / daily aggregation (rebuilt from raw readings)
                now_hour = int(time.time() / 3600)
                if now_hour > _last_hour_agg:
                    _last_hour_agg = now_hour
                    threading.Thread(target=refresh_aggregates, daemon=True).start()

                # Purge raw readings older than 7 days, once a day
                now_date = datetime.now().date()
                if now_date > _last_day_agg:
                    _last_day_agg = now_date
                    threading.Thread(target=lambda: delete_old_readings(7), daemon=True).start()

                watch_wind(data)
                check_wind_alarm(data)

                print(
                    f"[DATA] T={data['temperature']}C "
                    f"H={data['humidity']}% "
                    f"P={data['pressure']}hPa "
                    f"Wind={data['wind_speed']}m/s "
                    f"V={data['motor_voltage']}V "
                    f"I={data['motor_current']:.3f}A "
                    f"Power={data['motor_power']}W "
                    f"Motor={'ON' if data['motor_control'] else 'OFF'}"
                )

                if _stale_count >= STALE_THRESHOLD:
                    print("[PLC] Wind data stale. Reconnecting USR-W630...")
                    _stale_count = 0
                    with data_lock:
                        latest_data["connected"] = False
                    connect_plc()

            else:
                watch_wind(None)
                with data_lock:
                    latest_data["connected"] = False
                print("[PLC] No data. Reconnecting...")
                connect_plc()

        except Exception as exc:
            print("[PLC] Polling error:", exc)
            with data_lock:
                latest_data["connected"] = False

        time.sleep(READ_INTERVAL_SEC)


def snapshot(keys=None):
    with data_lock:
        d = dict(latest_data)
    # During a relay cycle the PLC link drops on purpose: keep showing the last
    # known values instead of "Offline". `relay_reset` tells the page why.
    if RELAY_HIDE_OFFLINE and not d.get("connected") and time.time() < _offline_hold_until:
        d["connected"] = True
        d["relay_reset"] = True
    if keys is None:
        return d
    return {key: d.get(key) for key in keys}


# ----------------------------------------------------------------------
# HTTP API
# ----------------------------------------------------------------------

@app.get("/audio/audio_alert.mp3")
def alarm_audio_file():
    """Serve the alarm MP3 stored in the local audio folder."""
    return send_from_directory(AUDIO_DIR, "audio_alert.mp3")


@app.get("/api/all")
def api_all():
    return jsonify(snapshot())


@app.get("/api/history")
def api_history():
    with data_lock:
        return jsonify(list(history))


@app.get("/api/temperature")
def api_temperature():
    return jsonify(snapshot([
        "temperature",
        "humidity",
        "pressure",
        "connected",
        "last_update",
    ]))


@app.get("/api/anemometer")
def api_anemometer():
    return jsonify(snapshot([
        "wind_speed",
        "connected",
        "last_update",
    ]))


@app.get("/api/pump")
def api_pump():
    return jsonify(snapshot([
        "motor_voltage",
        "motor_current",
        "motor_power",
        "motor_control",
        "connected",
        "last_update",
    ]))


@app.get("/api/motor")
def api_motor():
    return jsonify(snapshot([
        "motor_control",
        "motor_uptime_min",
        "motor_downtime_min",
        "motor_total_running_min",
        "motor_total_idle_min",
        "connected",
        "last_update",
    ]))


@app.post("/api/motor/on")
def api_motor_on():
    global _last_manual_command_at
    _last_manual_command_at = time.time()
    success = write_motor_control(1)
    return jsonify({
        "success": success,
        "motor_control": 1 if success else snapshot(["motor_control"])["motor_control"],
    }), (200 if success else 503)


@app.post("/api/motor/off")
def api_motor_off():
    global _last_manual_command_at
    _last_manual_command_at = time.time()
    success = write_motor_control(0)
    return jsonify({
        "success": success,
        "motor_control": 0 if success else snapshot(["motor_control"])["motor_control"],
    }), (200 if success else 503)


@app.get("/api/alerts")
def api_alerts():
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        rows = conn.execute(
            "SELECT id, ts, level, title, message, value, is_read FROM alerts ORDER BY id DESC LIMIT 200"
        ).fetchall()
        unread = conn.execute("SELECT COUNT(*) FROM alerts WHERE is_read = 0").fetchone()[0]
        conn.close()
        return jsonify({"alerts": [dict(r) for r in rows], "unread": unread,
                        "limit": WIND_ALERT_LIMIT})
    except Exception as e:
        return jsonify({"alerts": [], "unread": 0, "error": str(e)}), 500


@app.post("/api/alerts/<int:alert_id>/read")
def api_alert_read(alert_id):
    conn = sqlite3.connect(DB_PATH)
    conn.execute("UPDATE alerts SET is_read = 1 WHERE id = ?", (alert_id,))
    conn.commit()
    conn.close()
    return jsonify({"success": True})


@app.post("/api/alerts/read_all")
def api_alert_read_all():
    conn = sqlite3.connect(DB_PATH)
    conn.execute("UPDATE alerts SET is_read = 1 WHERE is_read = 0")
    conn.commit()
    conn.close()
    return jsonify({"success": True})


GRAPH_FIELDS = ("temperature", "humidity", "pressure", "wind", "power", "voltage", "current")


def _graph_points(rows):
    """rows: (timestamp, temperature, humidity, pressure, wind, power, voltage, current)"""
    out = []
    for r in rows:
        point = {"timestamp": float(r[0])}
        for name, v in zip(GRAPH_FIELDS, r[1:]):
            point[name] = None if v is None else round(float(v), 3)
        out.append(point)
    return out


def _readings_bucketed(conn, since, bucket_sec):
    return conn.execute(f"""
        SELECT CAST(timestamp / {bucket_sec} AS INTEGER) * {bucket_sec} AS t,
               AVG(temperature), AVG(humidity), AVG(pressure), AVG(wind_speed),
               AVG(motor_power), AVG(motor_voltage), AVG(motor_current)
        FROM readings
        WHERE timestamp >= ?
        GROUP BY CAST(timestamp / {bucket_sec} AS INTEGER)
        ORDER BY t
    """, (since,)).fetchall()


@app.get("/api/graph/<scale>")
def api_graph(scale):
    """Time-scale graph data. Every point has the same keys:
    timestamp, temperature, humidity, pressure, wind, power, voltage, current.

        1h  last hour,     raw readings (one point every 2 s)
        1d  last 24 hours, 5-minute averages from readings
        1w  last 7 days,   hourly averages from readings
        1m  last 30 days,  hourly averages from hourly_agg
        1y  last 365 days, daily averages from daily_agg
    """
    now = time.time()
    try:
        conn = sqlite3.connect(DB_PATH, timeout=10)
        if scale == "1h":
            rows = conn.execute("""
                SELECT timestamp, temperature, humidity, pressure, wind_speed,
                       motor_power, motor_voltage, motor_current
                FROM readings WHERE timestamp >= ? ORDER BY timestamp
            """, (now - 3600,)).fetchall()
        elif scale == "1d":
            rows = _readings_bucketed(conn, now - 86400, 300)
        elif scale == "1w":
            rows = _readings_bucketed(conn, now - 7 * 86400, 3600)
        elif scale == "1m":
            rows = conn.execute("""
                SELECT hour_bucket, temp_avg, humidity_avg, pressure_avg, wind_avg,
                       motor_power_avg, motor_voltage_avg, motor_current_avg
                FROM hourly_agg WHERE hour_bucket >= ? ORDER BY hour_bucket
            """, (now - 30 * 86400,)).fetchall()
        elif scale == "1y":
            day_rows = conn.execute("""
                SELECT day_date, temp_avg, humidity_avg, pressure_avg, wind_avg,
                       motor_power_avg, motor_voltage_avg, motor_current_avg
                FROM daily_agg WHERE day_date >= date('now', 'localtime', '-365 days')
                ORDER BY day_date
            """).fetchall()
            rows = [(datetime.strptime(r[0], "%Y-%m-%d").timestamp(),) + tuple(r[1:]) for r in day_rows]
        else:
            conn.close()
            return jsonify({"error": "scale must be 1h, 1d, 1w, 1m or 1y"}), 400
        conn.close()
        return jsonify({"scale": scale, "data": _graph_points(rows)})
    except Exception as e:
        print(f"[API] Graph error: {e}")
        return jsonify({"error": str(e)}), 500


# ----------------------------------------------------------------------
# WEB UI - ICONS
# ----------------------------------------------------------------------

ICONS = {
    "grid": '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/>'
            '<rect x="14" y="14" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/>',
    "power": '<path d="M12 2v10"/><path d="M18.4 6.6a9 9 0 1 1-12.77.04"/>',
    "toggle": '<rect x="2" y="6" width="20" height="12" rx="6"/><circle cx="8" cy="12" r="2.5"/>',
    "wind": '<path d="M17.7 7.7a2.5 2.5 0 1 1 1.8 4.3H2"/><path d="M9.6 4.6A2 2 0 1 1 11 8H2"/>'
            '<path d="M12.6 19.4A2 2 0 1 0 14 16H2"/>',
    "thermometer": '<path d="M14 4v10.54a4 4 0 1 1-4 0V4a2 2 0 0 1 4 0Z"/>',
    "droplet": '<path d="M12 22a7 7 0 0 0 7-7c0-2-1-3.9-3-5.5s-3.5-4-4-6.5c-.5 2.5-2 4.9-4 6.5C6 11.1 5 13 5 15a7 7 0 0 0 7 7z"/>',
    "gauge": '<path d="m12 14 4-4"/><path d="M3.34 19a10 10 0 1 1 17.32 0"/>',
    "zap": '<path d="M13 2 3 14h9l-1 8 10-12h-9l1-8z"/>',
    "activity": '<path d="M22 12h-4l-3 9L9 3l-3 9H2"/>',
    "radio": '<path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/><path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/>'
             '<circle cx="12" cy="12" r="2"/><path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/>'
             '<path d="M19.1 4.9C23 8.8 23 15.1 19.1 19.1"/>',
    "clock": '<circle cx="12" cy="12" r="10"/><path d="M12 6v6l4 2"/>',
    "shield": '<path d="M20 13c0 5-3.5 7.5-7.66 8.95a1 1 0 0 1-.67-.01C7.5 20.5 4 18 4 13V6a1 1 0 0 1 1-1'
              'c2 0 4.5-1.2 6.24-2.72a1.17 1.17 0 0 1 1.52 0C14.51 3.81 17 5 19 5a1 1 0 0 1 1 1z"/>',
    "fan": '<path d="M10.83 16.38a6.08 6.08 0 0 1-8.62-7l5.41 1.45a6.08 6.08 0 0 1 7-8.62l-1.45 5.41'
           'a6.08 6.08 0 0 1 8.62 7l-5.41-1.45a6.08 6.08 0 0 1-7 8.62l1.45-5.41Z"/><path d="M12 12v.01"/>',
    "cpu": '<rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/>'
           '<path d="M15 2v2M15 20v2M2 15h2M2 9h2M20 15h2M20 9h2M9 2v2M9 20v2"/>',
    "send": '<path d="m22 2-7 20-4-9-9-4Z"/><path d="M22 2 11 13"/>',
    "trend": '<path d="m22 7-8.5 8.5-5-5L2 17"/><path d="M16 7h6v6"/>',
    "cloud": '<path d="M22 16.92v.08a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2v-.5a2.5 2.5 0 0 1 2.29-2.5h1.4a2.5 2.5 0 0 0 2.36-3.75A3 3 0 0 1 20 10h2a2 2 0 0 1 2 2z"/>',
}


def icon(name):
    return (
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" '
        'stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">'
        + ICONS[name] + "</svg>"
    )


# ----------------------------------------------------------------------
# WEB UI - STYLES
# ----------------------------------------------------------------------

APP_CSS = r"""
:root{
  --bg:#f5f7fa;--card:#fff;--ink:#0f172a;--ink2:#1e293b;--sub:#64748b;--muted:#94a3b8;--line:#e6ebf1;
  --blue:#2563eb;--green:#16a34a;--green2:#22c55e;--amber:#d97706;--red:#dc2626;--cyan:#0891b2;
}
*{box-sizing:border-box}
html,body{margin:0}
body{font-family:Inter,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--ink);
     -webkit-font-smoothing:antialiased;min-height:100vh}
button{font-family:inherit}
:focus-visible{outline:2px solid var(--blue);outline-offset:2px}

/* top bar + tabs */
.topbar{height:56px;background:#fff;display:flex;align-items:center;justify-content:space-between;gap:12px;
        padding:0 22px;border-bottom:1px solid var(--line)}
.brand{display:flex;align-items:center;gap:10px;text-decoration:none;color:var(--ink)}
.brand-mark{width:30px;height:30px;border-radius:9px;background:var(--ink);color:#fff;display:grid;place-items:center}
.brand-mark svg{width:16px;height:16px}
.brand-name{display:block;font-weight:800;font-size:14px}
.brand-sub{display:block;font-size:10.5px;color:var(--muted)}
.conn{display:flex;align-items:center;gap:8px;font-size:11.5px;color:var(--sub);border:1px solid var(--line);
      border-radius:999px;padding:6px 12px;background:#fff;white-space:nowrap}
.conn .dot{width:8px;height:8px;border-radius:50%;background:#ef4444}
.conn.online .dot{background:var(--green2);animation:ping 1.8s infinite}
.conn b{color:var(--ink)}
@keyframes ping{0%{box-shadow:0 0 0 0 rgba(34,197,94,.5)}70%{box-shadow:0 0 0 7px rgba(34,197,94,0)}100%{box-shadow:0 0 0 0 rgba(34,197,94,0)}}
.tabs{display:flex;gap:2px;background:#fff;border-bottom:1px solid var(--line);padding:0 12px;overflow-x:auto;
      position:sticky;top:0;z-index:10}
.tabs a{display:flex;align-items:center;gap:7px;padding:12px 14px;font-size:12.5px;font-weight:600;color:#334155;
        text-decoration:none;border-bottom:2px solid transparent;white-space:nowrap}
.tabs a svg{width:14px;height:14px}
.tabs a:hover{color:var(--blue)}
.tabs a.active{color:var(--blue);border-bottom-color:var(--blue);background:#f5f8ff}
main{max-width:1640px;margin:0 auto;padding:14px}

/* widget grid + card */
.widgets{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}
.widgets + .widgets{margin-top:14px}
.widgets.cols-3{grid-template-columns:repeat(3,minmax(0,1fr))}
.span-2{grid-column:span 2}
.span-3{grid-column:span 3}
.span-all{grid-column:1/-1}
.w{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px 18px 12px;display:flex;
   flex-direction:column;min-height:218px;min-width:0;box-shadow:0 1px 2px rgba(15,23,42,.03)}
.w-head{display:flex;align-items:flex-start;gap:10px}
.w-icon{width:28px;height:28px;border-radius:8px;display:grid;place-items:center;flex:none}
.w-icon svg{width:15px;height:15px}
.tone-blue{background:#eff6ff;color:#2563eb}.tone-green{background:#ecfdf5;color:#059669}
.tone-cyan{background:#ecfeff;color:#0891b2}.tone-amber{background:#fffbeb;color:#d97706}
.tone-violet{background:#f5f3ff;color:#7c3aed}.tone-red{background:#fef2f2;color:#dc2626}
.tone-dark{background:#0f172a;color:#fff}.tone-slate{background:#f1f5f9;color:#475569}
.w-title{font-size:13px;font-weight:700;line-height:1.3}
.w-sub{font-size:10.5px;color:var(--muted);margin-top:2px}
.badge{margin-left:auto;font-size:9.5px;font-weight:800;letter-spacing:.05em;padding:3px 8px;border-radius:6px;
       text-transform:uppercase;white-space:nowrap}
.badge.ok{background:#dcfce7;color:#15803d}.badge.warn{background:#fef3c7;color:#b45309}
.badge.bad{background:#fee2e2;color:#b91c1c}.badge.idle{background:#f1f5f9;color:#64748b}
.w-body{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;padding:10px 0;gap:8px}
.w-body.left{align-items:stretch;justify-content:flex-start;padding-top:20px;gap:6px}
.w-foot{border-top:1px solid var(--line);padding-top:10px;margin-top:6px;display:flex;justify-content:space-between;
        align-items:center;gap:10px;font-size:10.5px;color:var(--sub)}
.w-foot b{color:var(--ink);font-weight:700}
.w-foot a{color:var(--blue);font-weight:700;text-decoration:none}
.fv{font-weight:700}.fv.ok{color:var(--green)}.fv.warn{color:var(--amber)}.fv.bad{color:var(--red)}.fv.idle{color:var(--muted)}
.fv.ok:before,.fv.warn:before,.fv.bad:before{content:"";display:inline-block;width:7px;height:7px;border-radius:50%;
       background:currentColor;margin-right:5px;vertical-align:0}

/* radial gauge */
.gauge{width:100%;max-width:220px}
.gauge.lg{max-width:330px}
.gauge svg{display:block;width:100%;height:auto}
.g-track{fill:none;stroke:#eef2f7;stroke-width:8}
.g-zone{fill:none;stroke-width:8}
.g-tick{stroke:#cbd5e1;stroke-width:1.2}
.g-tick.major{stroke:#94a3b8;stroke-width:2}
.g-lbl{font-size:8px;fill:#94a3b8;text-anchor:middle;dominant-baseline:middle;font-weight:600}
.g-needle{transform-origin:100px 100px;transition:transform .9s cubic-bezier(.22,1,.36,1);fill:var(--ink2)}
.g-hub{fill:var(--ink2)}
.readout{display:flex;align-items:baseline;gap:5px;margin-top:-30px}
.readout b{font-size:22px;font-weight:800;letter-spacing:-.02em}
.readout small{font-size:10px;color:var(--muted);font-weight:700;text-transform:uppercase}
.readout.lg{margin-top:-54px}
.readout.lg b{font-size:38px}
.pill{display:inline-flex;align-items:center;gap:5px;font-size:9.5px;font-weight:800;letter-spacing:.05em;
      padding:3px 10px;border-radius:999px;border:1px solid;text-transform:uppercase}
.pill:before{content:"";width:5px;height:5px;border-radius:50%;background:currentColor}
.pill.ok{color:#15803d;background:#f0fdf4;border-color:#bbf7d0}
.pill.warn{color:#b45309;background:#fffbeb;border-color:#fde68a}
.pill.bad{color:#be123c;background:#fff1f2;border-color:#fecdd3}
.pill.idle{color:#64748b;background:#f8fafc;border-color:#e2e8f0}

/* key/value + band bar */
.kv{display:flex;justify-content:space-between;align-items:baseline;gap:10px}
.kv-l{font-size:9.5px;font-weight:800;letter-spacing:.06em;color:var(--sub);text-transform:uppercase}
.kv-v{font-size:17px;font-weight:800;letter-spacing:-.01em}
.kv-v small{font-size:11px;margin-left:2px}
.kv-v.blue{color:var(--blue)}.kv-v.ok{color:var(--green)}.kv-v.warn{color:var(--amber)}.kv-v.bad{color:var(--red)}.kv-v.idle{color:var(--muted)}
.band{position:relative;width:100%;margin:4px 0 6px}
.band-track{position:relative;height:5px;border-radius:3px;background:#eef2f7;overflow:hidden}
.band-seg{position:absolute;top:0;bottom:0}
.band-marker{position:absolute;top:-4px;left:0;width:3px;height:13px;border-radius:2px;background:var(--ink);
             transform:translateX(-50%);transition:left .8s cubic-bezier(.22,1,.36,1)}
.band-ticks{position:relative;height:15px}
.band-ticks span{position:absolute;top:4px;transform:translateX(-50%);font-size:8.5px;color:var(--muted);
                 font-weight:600;white-space:nowrap}
.band-ticks span:first-child{transform:none}
.band-ticks span:last-child{transform:translateX(-100%)}
.band-ticks span.mid{color:var(--green)}

/* motor switch */
.state-big{font-size:34px;font-weight:900;letter-spacing:-.03em;line-height:1}
.state-big.lg{font-size:46px}
.state-big.ok{color:var(--green)}.state-big.stop{color:var(--red)}.state-big.idle{color:var(--muted)}
.seg{display:inline-flex;background:#f1f5f9;border:1px solid var(--line);border-radius:999px;padding:3px;gap:2px}
.seg button{border:0;background:transparent;padding:6px 20px;border-radius:999px;font-size:11px;font-weight:700;
            color:#94a3b8;cursor:pointer;transition:background .2s,color .2s,box-shadow .2s}
.seg button:hover:not(:disabled){color:var(--ink)}
.seg button.active{background:#fff;box-shadow:0 1px 3px rgba(15,23,42,.14)}
.seg button.active[data-v="1"]{color:var(--green)}
.seg button.active[data-v="0"]{color:var(--red)}
.seg button:disabled{cursor:wait;opacity:.6}
.seg.lg button{padding:6px 22px;font-size:11.5px}
.cmd-result{font-size:11px;color:var(--sub);min-height:15px;text-align:center}

/* thermometer + ring */
.thermo{width:46px;flex:none}
.thermo.lg{width:74px}
.thermo svg{width:100%;height:auto;display:block}
.th-fill{transform-origin:30px 128px;transition:transform .9s cubic-bezier(.22,1,.36,1),fill .5s}
.th-bulb{transition:fill .5s}
.ring{position:relative;width:86px;height:86px;flex:none}
.ring.lg{width:170px;height:170px}
.ring svg{width:100%;height:100%;transform:rotate(-90deg)}
.ring-bg{fill:none;stroke:#eef2f7}
.ring-fg{fill:none;stroke-linecap:round;transition:stroke-dashoffset .9s cubic-bezier(.22,1,.36,1),stroke .5s}
.ring-c{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}
.ring-c b{font-size:18px;font-weight:800}
.ring.lg .ring-c b{font-size:34px}
.ring-c small{font-size:8.5px;color:var(--muted);font-weight:700;text-transform:uppercase;letter-spacing:.06em}

.env-row{display:flex;align-items:center;justify-content:center;gap:14px;width:100%}
.env-col{display:flex;flex-direction:column;align-items:flex-start;gap:4px}
.vsep{width:1px;align-self:stretch;background:var(--line)}
.big{font-size:28px;font-weight:800;letter-spacing:-.02em;line-height:1.1}
.big small{font-size:12px;color:var(--muted);margin-left:3px;font-weight:700}
.big.xl{font-size:46px}
.muted{color:var(--muted);font-size:10.5px}

/* kpi + stats */
.kpi{font-size:40px;font-weight:850;letter-spacing:-.03em;line-height:1}
.kpi small{font-size:13px;color:var(--muted);margin-left:4px;font-weight:700}
.stats{display:grid;grid-template-columns:repeat(3,1fr);width:100%;gap:8px}
.stat{background:#f8fafc;border:1px solid var(--line);border-radius:10px;padding:10px 6px;text-align:center}
.stat span{display:block;font-size:9px;font-weight:800;letter-spacing:.06em;color:var(--sub);text-transform:uppercase}
.stat b{display:block;font-size:19px;font-weight:800;margin-top:4px}

/* trend + state strip */
.trend{position:relative;width:100%;height:170px}
.trend svg{width:100%;height:100%;display:block}
.trend-grid{stroke:#eef2f7;stroke-width:1}
.trend-axis{position:absolute;left:0;top:0;bottom:0;display:flex;flex-direction:column;justify-content:space-between;
            font-size:9px;color:var(--muted);font-weight:600;pointer-events:none}
.trend-axis span{background:rgba(255,255,255,.85);padding:0 3px;border-radius:3px}
.trend-empty{height:100%;width:100%;display:grid;place-items:center;color:var(--muted);font-size:12px}
.strip{display:flex;gap:1px;height:44px;width:100%}
.strip i{flex:1;border-radius:1.5px;background:#e2e8f0;min-width:1px}
.strip i.on{background:#22c55e}.strip i.off{background:#fca5a5}
.legend{display:flex;gap:14px;font-size:10.5px;color:var(--sub)}
.legend i{display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:5px;vertical-align:-1px}

/* pump fan */
.pump-fan{width:68px;height:68px;border-radius:18px;display:grid;place-items:center;background:#f1f5f9;color:#94a3b8;
          transition:background .4s,color .4s}
.pump-fan svg{width:38px;height:38px}
.pump-fan.on{background:#ecfdf5;color:#16a34a}
.pump-fan.on svg{animation:spin 1.4s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}

/* health matrix */
.section-h{display:flex;align-items:center;gap:8px;margin:22px 2px 10px;font-size:10px;font-weight:800;
           letter-spacing:.14em;color:var(--sub);text-transform:uppercase}
.section-h .ic{width:22px;height:22px;border-radius:7px;background:#eef2ff;color:#4f46e5;display:grid;place-items:center}
.section-h .ic svg{width:12px;height:12px}
.matrix{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}
.tile{display:flex;align-items:center;gap:12px;padding:13px 14px;border-radius:10px;border:1px solid var(--line);
      background:#fff;transition:background .4s,border-color .4s;min-width:0}
.tile.ok{border-color:#bbf7d0;background:linear-gradient(90deg,#fff 0%,#f0fdf4 100%)}
.tile.warn{border-color:#fcd34d;background:linear-gradient(90deg,#fff 0%,#fffbeb 100%)}
.tile.bad{border-color:#fca5a5;background:linear-gradient(90deg,#fff 0%,#fef2f2 100%)}
.tile-ic{width:30px;height:30px;border-radius:8px;display:grid;place-items:center;flex:none}
.tile-ic svg{width:15px;height:15px}
.tile-l{font-size:9px;font-weight:800;letter-spacing:.1em;color:var(--sub);text-transform:uppercase}
.tile-s{display:flex;align-items:center;gap:6px;font-size:11.5px;font-weight:800;margin-top:3px;text-transform:uppercase;
        white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.tile-s:before{content:"";width:7px;height:7px;border-radius:50%;background:#cbd5e1;flex:none}
.tile.ok .tile-s:before{background:#22c55e}.tile.warn .tile-s:before{background:#f59e0b}.tile.bad .tile-s:before{background:#ef4444}

@media (max-width:1280px){
  .widgets,.widgets.cols-3,.matrix{grid-template-columns:repeat(2,minmax(0,1fr))}
  .span-3{grid-column:1/-1}
}
@media (max-width:640px){
  .widgets,.widgets.cols-3,.matrix{grid-template-columns:1fr}
  .span-2,.span-3{grid-column:auto}
  .topbar{padding:0 14px}
  .conn-addr,.brand-sub,#conn-time{display:none}
  .brand-name{white-space:nowrap}
}
@media (prefers-reduced-motion:reduce){
  *,*:before,*:after{animation:none!important;transition:none!important}
}
"""

# Overview: content sits in a centred column and every row of cards stretches
# to the same width, so the page is balanced on wide screens.
OVERVIEW_LAYOUT_CSS = r"""
.page-overview main{max-width:1100px;margin:0 auto}
@media (min-width:641px){
  .page-overview .widgets,
  .page-overview .widgets.cols-3,
  .page-overview .matrix{grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}
}
"""

WHITE_PAGE_CSS = r"""
html,body{background:#fff!important;overflow-x:hidden}
.topbar,.tabs{background:#fff!important}
html{scrollbar-width:thin;scrollbar-color:rgba(100,116,139,.35) transparent}
::-webkit-scrollbar{width:6px;height:6px;background:transparent}
::-webkit-scrollbar-track,::-webkit-scrollbar-corner{background:transparent}
::-webkit-scrollbar-thumb{background:rgba(100,116,139,.35);border-radius:3px}
::-webkit-scrollbar-button{display:none}
"""

TRANSPARENT_CSS = r"""
html,body{background:transparent!important;overflow-x:hidden}
/* see-through scrollbars */
html{scrollbar-width:thin;scrollbar-color:rgba(255,255,255,.35) transparent}
::-webkit-scrollbar{width:6px;height:6px;background:transparent}
::-webkit-scrollbar-track,::-webkit-scrollbar-corner{background:transparent}
::-webkit-scrollbar-thumb{background:rgba(255,255,255,.35);border-radius:3px}
::-webkit-scrollbar-thumb:hover{background:rgba(255,255,255,.55)}
::-webkit-scrollbar-button{display:none}
.topbar,.tabs{background:transparent!important;border-bottom:0!important;box-shadow:none!important}
.tabs a.active{background:transparent!important}
.conn{background:transparent!important}
main{background:transparent!important}
"""

GLASS_CARD_CSS = r"""
.w,.tile,.tile.ok,.tile.warn,.tile.bad{background:__CARD_BG__!important;
  border-color:rgba(255,255,255,.6)!important;box-shadow:0 2px 10px rgba(15,23,42,.10)!important}
.tile.ok{border-color:rgba(74,222,128,.85)!important}
.tile.warn{border-color:rgba(251,191,36,.95)!important}
.tile.bad{border-color:rgba(248,113,113,.95)!important}
.w-foot{border-top-color:rgba(15,23,42,.12)!important}
.vsep{background:rgba(15,23,42,.14)!important}
.stat{background:rgba(255,255,255,.25)!important;border-color:rgba(255,255,255,.55)!important}
.seg{background:rgba(255,255,255,.35)!important;border-color:rgba(255,255,255,.65)!important}
.pump-fan{background:rgba(255,255,255,.3)!important}
.pump-fan.on{background:rgba(220,252,231,.6)!important}
.trend-axis span{background:rgba(255,255,255,.5)!important}
.g-track,.ring-bg{stroke:rgba(255,255,255,.45)!important}
.band-track{background:rgba(255,255,255,.45)!important}
/* keep text readable over any background: darker greys + soft white halo */
.w,.tile{--sub:#1e293b;--muted:#334155;
  text-shadow:0 0 2px rgba(255,255,255,.95),0 0 6px rgba(255,255,255,.7)}
.w svg text{fill:#1e293b!important;paint-order:stroke;stroke:rgba(255,255,255,.9);stroke-width:2.5px;stroke-linejoin:round}
.seg button{color:#334155}
"""

COMPACT_CSS = r"""
main{padding:10px}
.widgets{gap:10px}
.widgets + .widgets{margin-top:10px}
.w{padding:10px 12px 8px;min-height:0;border-radius:12px}
.w-head{gap:8px}
.w-icon{width:22px;height:22px;border-radius:6px}
.w-icon svg{width:12px;height:12px}
.w-title{font-size:11.5px}
.w-sub{font-size:9px;margin-top:1px}
.badge{font-size:8px;padding:2px 6px}
.w-body{padding:6px 0;gap:5px}
.w-body.left{padding-top:10px;gap:4px}
.w-foot{font-size:9px;padding-top:6px;margin-top:4px}
.gauge{max-width:160px}
.gauge.lg{max-width:210px}
.readout{margin-top:-22px}
.readout b{font-size:17px}
.readout.lg{margin-top:-30px}
.readout.lg b{font-size:24px}
.pill{font-size:8px;padding:2px 8px}
.state-big{font-size:24px}
.state-big.lg{font-size:30px}
.seg button{padding:4px 14px;font-size:10px}
.seg.lg button{padding:5px 18px;font-size:10.5px}
.cmd-result{font-size:9.5px;min-height:12px}
.kpi{font-size:28px}
.kpi small{font-size:11px}
.big{font-size:22px}
.big.xl{font-size:32px}
.big small{font-size:10px}
.kv-l{font-size:8.5px}
.kv-v{font-size:14px}
.band-ticks span{font-size:7.5px}
.ring{width:68px;height:68px}
.ring.lg{width:120px;height:120px}
.ring-c b{font-size:14px}
.ring.lg .ring-c b{font-size:24px}
.ring-c small{font-size:7.5px}
.thermo{width:34px}
.thermo.lg{width:52px}
.env-row{gap:10px}
.stat{padding:7px 4px}
.stat b{font-size:15px}
.trend{height:90px}
.strip{height:30px}
.pump-fan{width:48px;height:48px;border-radius:12px}
.pump-fan svg{width:26px;height:26px}
.section-h{margin:14px 2px 8px;font-size:9px}
.section-h .ic{width:18px;height:18px}
.matrix{gap:8px}
.tile{padding:8px 10px;gap:8px}
.tile-ic{width:24px;height:24px}
.tile-ic svg{width:12px;height:12px}
.tile-l{font-size:8px}
.tile-s{font-size:10px}
.topbar{height:44px;padding:0 12px}
.brand-mark{width:24px;height:24px;border-radius:7px}
.brand-mark svg{width:13px;height:13px}
.brand-name{font-size:12.5px}
.conn{font-size:10px;padding:4px 10px}
.tabs{padding:0 6px;flex-wrap:wrap;overflow:visible}
.tabs a{padding:8px 9px;font-size:11px;gap:5px}
.tabs a svg{width:12px;height:12px}
/* small fixed-width cards: as many per row as fit, no forced wrapping */
@media (min-width:641px){
  .widgets,.widgets.cols-3,.matrix{grid-template-columns:repeat(auto-fill,180px)}
  .span-2{grid-column:span 2}
  .span-3{grid-column:span 2}
}
.ring{width:60px;height:60px}
.ring-c b{font-size:13px}
.env-row{gap:8px;flex-wrap:wrap}
.w-head>div:nth-child(2){min-width:0;flex:1}
.w-title,.w-sub{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.badge{max-width:48%;overflow:hidden;text-overflow:ellipsis;font-size:7.5px;padding:2px 5px}
.w-head{gap:6px}
.w-icon{width:20px;height:20px}
.w-title{font-size:11px}
.w-foot>span{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;min-width:0}
/* a little shorter so a row of cards fits a small panel without scrolling */
.ring.lg{width:100px;height:100px}
.ring.lg .ring-c b{font-size:21px}
.thermo.lg{width:42px}
.big.xl{font-size:28px}
.gauge{max-width:150px}
.gauge.lg{max-width:156px}
.readout{margin-top:-20px}
.readout.lg{margin-top:-22px}
.readout.lg b{font-size:21px}
.topbar{height:38px}
.tabs a{padding:6px 9px}
main{padding:8px 10px}
.w-body{padding:4px 0}
.kpi{font-size:26px}
"""

TRANSPARENT_LIGHT_TEXT_CSS = r"""
.brand-name,.tabs a,.conn,.conn b,.section-h{color:#fff!important;text-shadow:0 1px 3px rgba(0,0,0,.55)}
.tabs a{opacity:.85}
.tabs a:hover,.tabs a.active{opacity:1}
.tabs a.active{border-bottom-color:#fff!important}
.conn{border-color:rgba(255,255,255,.55)!important}
"""

# ----------------------------------------------------------------------
# WEB UI - SHARED WIDGET LIBRARY (vanilla JS, no CDN needed)
# ----------------------------------------------------------------------

WIDGETS_JS = r"""
const $ = (id) => document.getElementById(id);
const TONE_COLOR = {ok: "#16a34a", warn: "#d97706", bad: "#dc2626", idle: "#94a3b8"};
const SEVERITY = {idle: 0, ok: 1, warn: 2, bad: 3};

/* ---------- formatting ---------- */
function num(v) {
    if (v === null || v === undefined || v === "") return null;
    const n = Number(v);
    return isNaN(n) ? null : n;
}
function fmt(v, d) {
    if (d === undefined) d = 1;
    const n = num(v);
    return n === null ? "--" : n.toFixed(d);
}
function clamp(v, a, b) { return Math.min(b, Math.max(a, v)); }
function fmtPower(w) {
    const n = num(w);
    if (n === null) return {v: "--", u: "W"};
    return Math.abs(n) >= 1000 ? {v: (n / 1000).toFixed(2), u: "kW"} : {v: n.toFixed(1), u: "W"};
}
function fmtDuration(min) {
    const n = num(min);
    if (n === null) return "--";
    const h = Math.floor(n / 60), m = Math.floor(n % 60), s = Math.round((n * 60) % 60);
    return h ? h + " h " + m + " m" : m + " m " + s + " s";
}
function windowLabel(samples) {
    const sec = samples * LIMITS.poll_sec;
    return sec >= 60 ? (sec / 60).toFixed(1) + " min" : Math.round(sec) + " s";
}

/* ---------- DOM setters ---------- */
function setText(id, v, fb) {
    const e = $(id);
    if (!e) return;
    if (fb === undefined) fb = "--";
    e.textContent = (v === null || v === undefined || v === "") ? fb : v;
}
function setHTML(id, h) { const e = $(id); if (e) e.innerHTML = h; }
function setClass(id, base, tone) { const e = $(id); if (e) e.className = base + " " + tone; }
function setBadge(id, s) { const e = $(id); if (!e) return; e.className = "badge " + s[0]; e.textContent = s[1]; }
function setPill(id, s) { const e = $(id); if (!e) return; e.className = "pill " + s[0]; e.textContent = s[1]; }
function setFv(id, s) { const e = $(id); if (!e) return; e.className = "fv " + s[0]; e.textContent = s[1]; }
function setTile(key, s) {
    const e = $("tile-" + key);
    if (!e) return;
    e.classList.remove("ok", "warn", "bad", "idle");
    e.classList.add(s[0]);
    setText("tile-" + key + "-s", s[1]);
}
function worst(list) {
    return list.reduce((a, b) => SEVERITY[b[0]] > SEVERITY[a[0]] ? b : a, ["idle", "No data"]);
}
function summary(list) {
    const w = worst(list);
    return [w[0], w[0] === "ok" ? "Normal" : w[1]];
}

/* ---------- status rules (driven by UI_LIMITS) ---------- */
const BEAUFORT = [[0.5, "Calm"], [1.6, "Light air"], [3.4, "Light breeze"], [5.5, "Gentle breeze"],
    [8.0, "Moderate breeze"], [10.8, "Fresh breeze"], [13.9, "Strong breeze"], [17.2, "Near gale"],
    [20.8, "Gale"], [24.5, "Strong gale"], [28.5, "Storm"], [32.7, "Violent storm"], [Infinity, "Hurricane"]];
function beaufort(v) {
    const n = num(v);
    if (n === null) return {n: null, name: "--"};
    for (let i = 0; i < BEAUFORT.length; i++) {
        if (Math.abs(n) < BEAUFORT[i][0]) return {n: i, name: BEAUFORT[i][1]};
    }
    return {n: 12, name: "Hurricane"};
}
function dewPoint(t, rh) {
    t = num(t); rh = num(rh);
    if (t === null || rh === null || rh <= 0) return null;
    const a = 17.62, b = 243.12;
    const g = Math.log(rh / 100) + (a * t) / (b + t);
    return (b * g) / (a - g);
}

const Status = {
    link(d) { return d && d.connected ? ["ok", "Online"] : ["bad", "Processing"]; },
    motor(d) {
        if (!d || !d.connected) return ["idle", "Unknown"];
        return d.motor_control ? ["ok", "Running"] : ["warn", "Stopped"];
    },
    wind(v) {
        const n = num(v), w = LIMITS.wind;
        if (n === null) return ["idle", "No data"];
        if (n < w.calm) return ["ok", "Calm"];
        if (n < w.high) return ["ok", "Moderate"];
        if (n < w.danger) return ["warn", "Strong"];
        return ["bad", "Severe"];
    },
    temperature(v) {
        const n = num(v), t = LIMITS.temperature;
        if (n === null) return ["idle", "No data"];
        if (n < t.low) return ["warn", "Low"];
        if (n >= t.critical) return ["bad", "Critical"];
        if (n >= t.high) return ["warn", "High"];
        return ["ok", "Normal"];
    },
    humidity(v) {
        const n = num(v), h = LIMITS.humidity;
        if (n === null) return ["idle", "No data"];
        if (n < h.low) return ["warn", "Dry"];
        if (n > h.high) return ["warn", "Humid"];
        return ["ok", "Normal"];
    },
    pressure(v) {
        const n = num(v), p = LIMITS.pressure;
        if (n === null) return ["idle", "No data"];
        if (n < p.low) return ["warn", "Low"];
        if (n > p.high) return ["warn", "High"];
        return ["ok", "Normal"];
    },
    voltage(v) {
        const n = num(v), x = LIMITS.voltage;
        if (n === null) return ["idle", "No data"];
        if (n < x.no_supply) return ["idle", "No supply"];
        if (n < x.low) return ["warn", "Under-voltage"];
        if (n > x.high) return ["bad", "Over-voltage"];
        return ["ok", "In band"];
    },
    current(v) {
        const n = num(v), c = LIMITS.current;
        if (n === null) return ["idle", "No data"];
        if (n >= c.trip) return ["bad", "Overload"];
        if (n >= c.warn) return ["warn", "High"];
        return ["ok", "Normal"];
    },
    load(w) {
        const n = num(w);
        if (n === null) return ["idle", "No data"];
        const pct = n / LIMITS.power.max * 100;
        if (pct >= 90) return ["bad", "Overload"];
        if (pct >= 70) return ["warn", "High load"];
        return ["ok", pct.toFixed(0) + "% load"];
    }
};

/* ---------- colour zones ---------- */
const Z = {
    wind() { const w = LIMITS.wind; return [{to: w.calm, color: "#38bdf8"}, {to: w.high, color: "#22c55e"}, {to: w.danger, color: "#f59e0b"}, {to: w.max, color: "#ef4444"}]; },
    voltage() { const v = LIMITS.voltage; return [{to: v.low, color: "#ef4444"}, {to: v.high, color: "#22c55e"}, {to: v.max, color: "#ef4444"}]; },
    current() { const c = LIMITS.current; return [{to: c.warn, color: "#22c55e"}, {to: c.trip, color: "#f59e0b"}, {to: c.max, color: "#ef4444"}]; },
    pressure() { const p = LIMITS.pressure; return [{to: p.low, color: "#f59e0b"}, {to: p.high, color: "#22c55e"}, {to: p.max, color: "#f59e0b"}]; }
};
/* gauge label positions: sit on the zone boundaries so the scale reads cleanly */
const T = {
    wind() { const w = LIMITS.wind, step = w.max / 6; return [0, 1, 2, 3, 4, 5, 6].map((i) => +(i * step).toFixed(1)); },
    pressure() { const p = LIMITS.pressure; return [p.min, p.low, (p.low + p.high) / 2, p.high, p.max]; },
    voltage() { const v = LIMITS.voltage, step = (v.max - v.min) / 5; return [0, 1, 2, 3, 4, 5].map((i) => +(v.min + i * step).toFixed(1)); },
    current() { const c = LIMITS.current; return [0, c.max / 4, c.max / 2, c.warn, c.trip, c.max]; }
};
const B = {
    voltage() {
        const v = LIMITS.voltage, q = (v.high - v.low) * 0.2;
        return {min: v.min, max: v.max, unit: "V", mid: v.nominal, ticks: T.voltage(),
            segments: [{from: v.min, to: v.low, color: "#ef4444"}, {from: v.low, to: v.low + q, color: "#f59e0b"},
                {from: v.low + q, to: v.high - q, color: "#22c55e"}, {from: v.high - q, to: v.high, color: "#f59e0b"},
                {from: v.high, to: v.max, color: "#ef4444"}]};
    },
    current() {
        const c = LIMITS.current;
        return {min: 0, max: c.max, unit: "A", ticks: [0, c.warn, c.trip, c.max],
            segments: [{from: 0, to: c.warn, color: "#22c55e"}, {from: c.warn, to: c.trip, color: "#f59e0b"},
                {from: c.trip, to: c.max, color: "#ef4444"}]};
    },
    power() {
        const p = LIMITS.power.max;
        return {min: 0, max: p, unit: "W", ticks: [0, p / 2, p],
            segments: [{from: 0, to: p * 0.7, color: "#22c55e"}, {from: p * 0.7, to: p * 0.9, color: "#f59e0b"},
                {from: p * 0.9, to: p, color: "#ef4444"}]};
    },
    temperature() {
        const t = LIMITS.temperature;
        return {min: t.min, max: t.max, unit: "°", ticks: [t.min, t.low, t.high, t.critical, t.max],
            segments: [{from: t.min, to: t.low, color: "#38bdf8"}, {from: t.low, to: t.high, color: "#22c55e"},
                {from: t.high, to: t.critical, color: "#f59e0b"}, {from: t.critical, to: t.max, color: "#ef4444"}]};
    },
    humidity() {
        const h = LIMITS.humidity;
        return {min: 0, max: 100, unit: "%", ticks: [0, h.low, h.high, 100],
            segments: [{from: 0, to: h.low, color: "#f59e0b"}, {from: h.low, to: h.high, color: "#22c55e"},
                {from: h.high, to: 100, color: "#f59e0b"}]};
    },
    beaufort() {
        return {min: 0, max: 12, unit: "", ticks: [0, 3, 6, 9, 12],
            segments: [{from: 0, to: 3, color: "#38bdf8"}, {from: 3, to: 5, color: "#22c55e"},
                {from: 5, to: 7, color: "#f59e0b"}, {from: 7, to: 12, color: "#ef4444"}]};
    }
};

/* ---------- SVG helpers ---------- */
function polar(cx, cy, r, deg) {
    const a = (deg - 90) * Math.PI / 180;
    return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
}
function arcPath(cx, cy, r, a1, a2) {
    const p1 = polar(cx, cy, r, a1), p2 = polar(cx, cy, r, a2);
    const large = (a2 - a1) > 180 ? 1 : 0;
    return "M" + p1[0].toFixed(2) + " " + p1[1].toFixed(2) + " A" + r + " " + r + " 0 " + large + " 1 " +
        p2[0].toFixed(2) + " " + p2[1].toFixed(2);
}

/* ---------- Radial gauge (like the Engine Speed tachometer) ---------- */
const Gauge = {
    mount(id, cfg) {
        const el = $(id);
        if (!el) return;
        cfg = Object.assign({min: 0, max: 100, zones: [], ticks: null, decimals: 0}, cfg);
        const cx = 100, cy = 100, r = 74, A0 = -120, A1 = 120, span = (cfg.max - cfg.min) || 1;
        const ang = (v) => A0 + (clamp(v, cfg.min, cfg.max) - cfg.min) / span * (A1 - A0);
        let s = '<svg viewBox="0 0 200 150" role="img" aria-label="gauge">';
        s += '<path class="g-track" d="' + arcPath(cx, cy, r, A0, A1) + '"/>';
        let from = cfg.min;
        cfg.zones.forEach((z) => {
            const to = Math.min(z.to, cfg.max);
            if (to > from) s += '<path class="g-zone" stroke="' + z.color + '" d="' + arcPath(cx, cy, r, ang(from), ang(to)) + '"/>';
            from = to;
        });
        const tickLine = (a, len, cls) => {
            const p1 = polar(cx, cy, r - 8, a), p2 = polar(cx, cy, r - 8 - len, a);
            return '<line class="' + cls + '" x1="' + p1[0].toFixed(1) + '" y1="' + p1[1].toFixed(1) +
                '" x2="' + p2[0].toFixed(1) + '" y2="' + p2[1].toFixed(1) + '"/>';
        };
        for (let i = 0; i <= 40; i++) s += tickLine(A0 + i * (A1 - A0) / 40, 4, "g-tick");
        const ticks = cfg.ticks || [cfg.min, cfg.min + span / 4, cfg.min + span / 2, cfg.min + 3 * span / 4, cfg.max];
        ticks.forEach((t) => {
            s += tickLine(ang(t), 8, "g-tick major");
            const p = polar(cx, cy, r + 13, ang(t));
            s += '<text class="g-lbl" x="' + p[0].toFixed(1) + '" y="' + p[1].toFixed(1) + '">' + (+t.toFixed(cfg.decimals)) + '</text>';
        });
        s += '<g class="g-needle" style="transform:rotate(' + A0 + 'deg)"><path d="M' + (cx - 3.5) + ' ' + cy + ' L' + cx + ' ' +
            (cy - r + 20) + ' L' + (cx + 3.5) + ' ' + cy + ' Z"/></g>';
        s += '<circle class="g-hub" cx="' + cx + '" cy="' + cy + '" r="7"/><circle cx="' + cx + '" cy="' + cy + '" r="2.5" fill="#fff"/></svg>';
        el.innerHTML = s;
        el._ang = ang;
        el._a0 = A0;
    },
    set(id, v) {
        const el = $(id);
        if (!el || !el._ang) return;
        const n = num(v), g = el.querySelector(".g-needle");
        if (g) g.style.transform = "rotate(" + (n === null ? el._a0 : el._ang(n)) + "deg)";
    }
};

/* ---------- Band bar (like the Frequency bar) ---------- */
const Band = {
    mount(id, cfg) {
        const el = $(id);
        if (!el) return;
        cfg = Object.assign({min: 0, max: 100, segments: [], ticks: [], unit: "", mid: null}, cfg);
        const span = (cfg.max - cfg.min) || 1;
        const pct = (v) => (clamp(v, cfg.min, cfg.max) - cfg.min) / span * 100;
        const segs = cfg.segments.map((s) =>
            '<div class="band-seg" style="left:' + pct(s.from) + '%;width:' + (pct(s.to) - pct(s.from)) + '%;background:' + s.color + '"></div>').join("");
        const ticks = cfg.ticks.map((t) =>
            '<span class="' + (t === cfg.mid ? "mid" : "") + '" style="left:' + pct(t) + '%">' + (+Number(t).toFixed(1)) + cfg.unit + '</span>').join("");
        el.innerHTML = '<div class="band-track">' + segs + '</div><div class="band-marker"></div><div class="band-ticks">' + ticks + '</div>';
        el._pct = pct;
    },
    set(id, v) {
        const el = $(id);
        if (!el || !el._pct) return;
        const n = num(v), m = el.querySelector(".band-marker");
        m.style.left = (n === null ? 0 : el._pct(n)) + "%";
        m.style.opacity = n === null ? 0.25 : 1;
    }
};

/* ---------- Ring (donut) ---------- */
const Ring = {
    C: 2 * Math.PI * 42,
    mount(id, cfg) {
        const el = $(id);
        if (!el) return;
        cfg = Object.assign({label: "", stroke: 9}, cfg);
        el.innerHTML = '<svg viewBox="0 0 100 100"><circle class="ring-bg" cx="50" cy="50" r="42" stroke-width="' + cfg.stroke + '"/>' +
            '<circle class="ring-fg" cx="50" cy="50" r="42" stroke="#94a3b8" stroke-width="' + cfg.stroke + '" stroke-dasharray="' + Ring.C +
            '" stroke-dashoffset="' + Ring.C + '"/></svg><div class="ring-c"><b id="' + id + '-v">--</b><small>' + cfg.label + '</small></div>';
    },
    set(id, v, max, color, txt) {
        const el = $(id);
        if (!el) return;
        const n = num(v), f = n === null ? 0 : clamp(n / max, 0, 1), fg = el.querySelector(".ring-fg");
        fg.style.strokeDashoffset = Ring.C * (1 - f);
        fg.style.stroke = color;
        setText(id + "-v", txt !== undefined ? txt : fmt(n, 0));
    }
};

/* ---------- Thermometer ---------- */
const Thermo = {
    mount(id) {
        const el = $(id);
        if (!el) return;
        let ticks = "";
        for (let i = 0; i <= 5; i++) {
            const y = 128 - i * 114 / 5;
            ticks += '<line x1="44" x2="' + (i % 5 === 0 ? 53 : 49) + '" y1="' + y + '" y2="' + y + '" stroke="#cbd5e1" stroke-width="1.5"/>';
        }
        el.innerHTML = '<svg viewBox="0 0 60 160" role="img" aria-label="thermometer">' +
            '<rect x="19" y="4" width="22" height="134" rx="11" fill="#f1f5f9"/><circle cx="30" cy="138" r="19" fill="#f1f5f9"/>' +
            '<rect x="25" y="12" width="10" height="116" rx="5" fill="#e2e8f0"/>' +
            '<rect class="th-fill" x="25" y="12" width="10" height="116" rx="5" fill="#cbd5e1" style="transform:scaleY(0)"/>' +
            '<circle class="th-bulb" cx="30" cy="138" r="12.5" fill="#cbd5e1"/>' + ticks + '</svg>';
    },
    set(id, v) {
        const el = $(id);
        if (!el) return;
        const n = num(v), t = LIMITS.temperature;
        const f = n === null ? 0 : clamp((n - t.min) / (t.max - t.min), 0.03, 1);
        const c = n === null ? "#cbd5e1" : n < t.low ? "#0ea5e9" : n >= t.critical ? "#dc2626" : n >= t.high ? "#f97316" : "#10b981";
        const fill = el.querySelector(".th-fill"), bulb = el.querySelector(".th-bulb");
        fill.style.transform = "scaleY(" + f + ")";
        fill.style.fill = c;
        bulb.style.fill = c;
    }
};

/* ---------- Trend (area sparkline) ---------- */
const Trend = {
    draw(id, values, cfg) {
        const el = $(id);
        if (!el) return;
        cfg = Object.assign({color: "#2563eb", decimals: 1, unit: ""}, cfg || {});
        const n = values.length, pts = [];
        values.forEach((v, i) => { const x = num(v); if (x !== null) pts.push([i, x]); });
        if (pts.length < 2) { el.innerHTML = '<div class="trend-empty">Collecting data, the trend appears after a few polls.</div>'; return; }
        let lo = Math.min(...pts.map((p) => p[1])), hi = Math.max(...pts.map((p) => p[1]));
        const rlo = lo, rhi = hi;
        if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
        const pad = (hi - lo) * 0.15, y0 = lo - pad, y1 = hi + pad, W = 600, H = 160;
        const X = (i) => (n <= 1 ? 0 : i / (n - 1) * W), Y = (v) => H - (v - y0) / (y1 - y0) * H;
        const line = pts.map((p, k) => (k ? "L" : "M") + X(p[0]).toFixed(1) + " " + Y(p[1]).toFixed(1)).join(" ");
        const area = line + " L" + X(pts[pts.length - 1][0]).toFixed(1) + " " + H + " L" + X(pts[0][0]).toFixed(1) + " " + H + " Z";
        let grid = "";
        for (let i = 1; i < 4; i++) grid += '<line class="trend-grid" vector-effect="non-scaling-stroke" x1="0" x2="' + W + '" y1="' + (H * i / 4) + '" y2="' + (H * i / 4) + '"/>';
        const gid = id + "-grad";
        el.innerHTML = '<svg viewBox="0 0 ' + W + ' ' + H + '" preserveAspectRatio="none"><defs><linearGradient id="' + gid +
            '" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="' + cfg.color + '" stop-opacity=".25"/><stop offset="1" stop-color="' +
            cfg.color + '" stop-opacity="0"/></linearGradient></defs>' + grid + '<path d="' + area + '" fill="url(#' + gid + ')"/>' +
            '<path d="' + line + '" fill="none" stroke="' + cfg.color + '" stroke-width="2" stroke-linejoin="round" vector-effect="non-scaling-stroke"/></svg>' +
            '<div class="trend-axis"><span>' + rhi.toFixed(cfg.decimals) + cfg.unit + '</span><span>' + rlo.toFixed(cfg.decimals) + cfg.unit + '</span></div>';
    }
};

/* ---------- ON/OFF state strip ---------- */
function drawStrip(id, states) {
    const el = $(id);
    if (!el) return;
    if (!states.length) { el.innerHTML = '<div class="trend-empty">Collecting data, history appears after a few polls.</div>'; return; }
    el.innerHTML = states.map((s) => '<i class="' + (s === null || s === undefined ? "" : (s ? "on" : "off")) + '"></i>').join("");
}
function series(h, key) { return h.map((x) => num(x[key])); }
function statsOf(arr) {
    const v = arr.filter((x) => x !== null);
    if (!v.length) return null;
    return {min: Math.min(...v), max: Math.max(...v), avg: v.reduce((a, b) => a + b, 0) / v.length, n: v.length};
}

/* ---------- connection + polling ---------- */
function status(d, serverDown) {
    const c = $("conn");
    if (c) c.classList.toggle("online", !!(d && d.connected));
    setText("conn-text", serverDown ? "Dashboard offline" : (d && d.connected ? "PLC online" : "Processing"));
    setText("conn-time", d && d.last_update ? d.last_update : "--");
    document.querySelectorAll(".js-poll").forEach((e) => {
        e.textContent = d && d.last_update ? d.last_update.split(" ")[1] : "--";
    });
}
async function getJSON(url) {
    const r = await fetch(url, {cache: "no-store"});
    if (!r.ok) throw new Error("HTTP " + r.status);
    return await r.json();
}
function startLoop(fn, ms) {
    let busy = false;
    const run = async () => {
        if (busy) return;
        busy = true;
        try { await fn(); } catch (e) { status({connected: false}, true); } finally { busy = false; }
    };
    run();
    setInterval(run, ms || LIMITS.poll_sec * 1000);
}

/* ---------- motor control (segmented Off / On switch) ---------- */
let lastCommand = null;
function cmdStatus() {
    if (!lastCommand) return ["idle", "No command sent"];
    const t = lastCommand.at.toLocaleTimeString();
    return lastCommand.ok ? ["ok", (lastCommand.on ? "ON" : "OFF") + " accepted " + t] : ["bad", "Failed " + t];
}
function renderMotor(p, d) {
    const on = !!d.motor_control, conn = !!d.connected;
    const st = $(p + "-state");
    if (st) {
        st.textContent = conn ? (on ? "Running" : "Stopped") : "Processing";
        st.className = st.className.replace(/\b(ok|stop|idle)\b/g, "").trim() + " " + (conn ? (on ? "ok" : "stop") : "idle");
    }
    document.querySelectorAll("#" + p + "-seg button").forEach((b) => {
        b.classList.toggle("active", conn && Number(b.dataset.v) === (on ? 1 : 0));
        b.setAttribute("aria-pressed", conn && Number(b.dataset.v) === (on ? 1 : 0) ? "true" : "false");
    });
    setBadge(p + "-badge", Status.motor(d));
}
async function sendMotor(on, resultId) {
    const btns = document.querySelectorAll(".seg button");
    btns.forEach((b) => { b.disabled = true; });
    setText(resultId, on ? "Turning pump on..." : "Turning pump off...", "");
    try {
        const r = await fetch(on ? "/api/motor/on" : "/api/motor/off", {method: "POST"});
        const d = await r.json();
        const ok = r.ok && d.success;
        lastCommand = {ok: ok, on: on, at: new Date()};
        setText(resultId, ok ? "Pump turned " + (on ? "on" : "off") + "."
            : "PLC did not accept the command. Check the PLC link and try again.", "");
    } catch (e) {
        lastCommand = {ok: false, on: on, at: new Date()};
        setText(resultId, "Dashboard server is not responding. Check that the Python app is running.", "");
    } finally {
        btns.forEach((b) => { b.disabled = false; });
        if (window.refresh) window.refresh();
    }
}
"""

# ----------------------------------------------------------------------
# WEB UI - PAGE SHELL
# ----------------------------------------------------------------------

NOTIF_CSS = r"""
/* ===== Top-right: bell + clock ===== */
.tb-right{display:flex;align-items:center;gap:12px}
.tb-btn{position:relative;width:34px;height:34px;border-radius:10px;border:1px solid #e2e8f0;background:#fff;
  color:#475569;display:inline-flex;align-items:center;justify-content:center;cursor:pointer;transition:.15s}
.tb-btn:hover{background:#f8fafc;border-color:#cbd5e1;color:#0f172a}
.tb-btn.open{background:#eff6ff;border-color:#93c5fd;color:#2563eb}
.tb-badge{position:absolute;top:-6px;right:-6px;min-width:18px;height:18px;padding:0 5px;border-radius:9px;
  background:#ef4444;color:#fff;font-size:10.5px;font-weight:700;line-height:18px;text-align:center;
  box-shadow:0 0 0 2px #fff}
.tb-badge[hidden]{display:none}
.tb-clock{font-size:12px;color:#475569;white-space:nowrap;font-variant-numeric:tabular-nums}
.tb-bell.ringing svg{animation:bellRing 1s ease-in-out 3;transform-origin:50% 2px}
@keyframes bellRing{0%,100%{transform:rotate(0)}10%{transform:rotate(16deg)}20%{transform:rotate(-14deg)}
  30%{transform:rotate(12deg)}40%{transform:rotate(-10deg)}50%{transform:rotate(6deg)}60%{transform:rotate(-4deg)}70%{transform:rotate(0)}}
.tb-bell.has-unread{color:#dc2626;border-color:#fecaca;background:#fef2f2}

/* ===== Alarm panel ===== */
.np{position:fixed;top:60px;right:16px;width:400px;max-width:calc(100vw - 24px);max-height:70vh;z-index:2000;
  background:#fff;border:1px solid #e2e8f0;border-radius:14px;box-shadow:0 18px 40px rgba(15,23,42,.18);
  display:flex;flex-direction:column;overflow:hidden;animation:npIn .15s ease-out;color:#0f172a}
.np[hidden]{display:none}
@keyframes npIn{from{opacity:0;transform:translateY(-6px)}to{opacity:1;transform:none}}
.np-head{display:flex;align-items:center;gap:10px;padding:14px 16px 10px}
.np-ic{width:28px;height:28px;border-radius:8px;background:#fee2e2;color:#dc2626;display:inline-flex;
  align-items:center;justify-content:center;flex-shrink:0}
.np-title{font-size:15px;flex:1}
.np-markall{border:1px solid #86efac;background:#f0fdf4;color:#15803d;font-size:11.5px;font-weight:600;
  padding:4px 10px;border-radius:7px;cursor:pointer}
.np-markall:hover{background:#dcfce7}
.np-markall:disabled{opacity:.45;cursor:default}
.np-close{border:0;background:none;font-size:22px;line-height:1;color:#94a3b8;cursor:pointer;padding:0 2px}
.np-close:hover{color:#0f172a}
.np-tabs{display:flex;gap:4px;padding:0 12px;border-bottom:1px solid #e2e8f0}
.np-tab{border:0;background:none;padding:8px 10px;font-size:12.5px;color:#64748b;cursor:pointer;
  border-bottom:2px solid transparent;margin-bottom:-1px}
.np-tab.active{color:#2563eb;border-bottom-color:#2563eb;font-weight:600}
.np-tab .n{display:inline-block;margin-left:4px;padding:0 6px;border-radius:8px;background:#fee2e2;
  color:#dc2626;font-size:10px;font-weight:700}
.np-list{overflow-y:auto;flex:1}
.np-item{display:flex;gap:10px;padding:12px 16px;border-bottom:1px solid #f1f5f9;cursor:pointer;position:relative}
.np-item:hover{background:#f8fafc}
.np-item.unread{background:#fff7f7}
.np-item.unread:hover{background:#fef2f2}
.np-item-ic{width:30px;height:30px;border-radius:50%;background:#fee2e2;color:#dc2626;display:inline-flex;
  align-items:center;justify-content:center;font-weight:800;flex-shrink:0;font-size:15px}
.np-item.read .np-item-ic{background:#f1f5f9;color:#94a3b8}
.np-item-body{flex:1;min-width:0}
.np-item-top{display:flex;align-items:center;gap:8px}
.np-item-top b{font-size:13px}
.np-item.read .np-item-top b{font-weight:600;color:#475569}
.np-tag{font-size:9.5px;font-weight:700;letter-spacing:.04em;padding:1px 6px;border-radius:5px;
  background:#dc2626;color:#fff}
.np-item.read .np-tag{background:#e2e8f0;color:#64748b}
.np-val{margin-left:auto;font-size:12px;font-weight:700;color:#dc2626;white-space:nowrap}
.np-item.read .np-val{color:#64748b}
.np-msg{margin:4px 0 4px;font-size:12px;line-height:1.45;color:#334155}
.np-item.read .np-msg{color:#64748b}
.np-time{font-size:11px;color:#94a3b8}
.np-dot{position:absolute;left:6px;top:50%;width:6px;height:6px;margin-top:-3px;border-radius:50%;background:#ef4444}
.np-empty{display:flex;flex-direction:column;align-items:center;gap:4px;padding:34px 16px 38px;text-align:center}
.np-empty[hidden]{display:none}
.np-check{font-size:34px;color:#16a34a;line-height:1;margin-bottom:6px}
.np-empty b{font-size:14px}
.np-empty span{font-size:12px;color:#64748b}
.tb-bell.alarming{color:#fff;background:#dc2626;border-color:#dc2626;animation:bellPulse 1.2s ease-in-out infinite}
.tb-bell.alarming svg{animation:bellRing 1s ease-in-out infinite;transform-origin:50% 2px}
@keyframes bellPulse{0%,100%{box-shadow:0 0 0 0 rgba(220,38,38,.55)}50%{box-shadow:0 0 0 7px rgba(220,38,38,0)}}
@media (max-width:640px){.tb-clock{display:none}.np{top:52px;right:8px}}
"""

NOTIF_JS = r"""
(function () {
    const $id = (id) => document.getElementById(id);
    const bell = $id("notif-bell"), panel = $id("notif-panel");
    if (!bell || !panel) return;

    const store = {
        get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (e) { return d; } },
        set(k, v) { try { localStorage.setItem(k, v); } catch (e) {} }
    };
    let alerts = [], filter = "all";
    let lastMax = parseInt(store.get("plc_alert_last_id", "-1"), 10);

    const esc = (t) => String(t == null ? "" : t).replace(/[&<>"']/g,
        (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"}[c]));
    const fmtDate = (ts) => new Date(ts * 1000).toLocaleString("en-GB", {
        day: "numeric", month: "short", year: "numeric",
        hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: true});
    function ago(ts) {
        const s = Math.max(0, Date.now() / 1000 - ts);
        if (s < 60) return "just now";
        if (s < 3600) return Math.floor(s / 60) + " min ago";
        if (s < 86400) return Math.floor(s / 3600) + " h ago";
        return Math.floor(s / 86400) + " d ago";
    }

    /* ---- Anemometer alarm MP3 only; no generated beeper/tone sound ---- */
    const ALARM_GAP_MS = 5000;  // 5-second silent break AFTER the MP3 finishes
    const alarmAudio = new Audio("/audio/audio_alert.mp3");
    alarmAudio.preload = "auto";
    let alarmTimer = null, unreadAnemometerNow = 0;
    let audioBlocked = false;   // browser refused autoplay; retried quietly

    function isAnemometerAlarm(a) {
        return a && !a.is_read && a.title === "Critical Wind Speed";
    }

    function stopAlarmAudio() {
        alarmAudio.pause();
        try { alarmAudio.currentTime = 0; } catch (e) {}
    }

    function scheduleNextAlarm() {
        if (alarmTimer) {
            clearTimeout(alarmTimer);
            alarmTimer = null;
        }

        if (unreadAnemometerNow <= 0) return;

        alarmTimer = setTimeout(() => {
            alarmTimer = null;
            playAlarm();
        }, ALARM_GAP_MS);
    }

    function playAlarm() {
        if (unreadAnemometerNow <= 0) return;

        // Always let the MP3 play from the beginning to the end.
        stopAlarmAudio();

        const p = alarmAudio.play();
        if (p && typeof p.then === "function") {
            p.then(() => { audioBlocked = false; }).catch(() => {
                audioBlocked = true;
                // Browser blocked autoplay. Try again after the normal gap; it will
                // succeed once autoplay is allowed for this site or after any click.
                scheduleNextAlarm();
            });
        }
    }

    // When the 6-second MP3 ends, wait 5 seconds before playing it again.
    alarmAudio.addEventListener("ended", () => {
        if (unreadAnemometerNow > 0) scheduleNextAlarm();
    });

    function syncAlarm(unreadAnemometer) {
        const wasActive = unreadAnemometerNow > 0;
        unreadAnemometerNow = unreadAnemometer;
        bell.classList.toggle("alarming", unreadAnemometer > 0);

        if (unreadAnemometer > 0 && !wasActive && alarmAudio.paused && !alarmTimer) {
            // First unread anemometer alarm: play immediately.
            playAlarm();
        } else if (unreadAnemometer === 0) {
            if (alarmTimer) {
                clearTimeout(alarmTimer);
                alarmTimer = null;
            }
            stopAlarmAudio();
            audioBlocked = false;
        }
    }

    function unlockAudio() {
        // A click / key press lets the browser play sound. Only start the MP3 here
        // if it was blocked, so normal clicks don't restart a playing alarm.
        if (unreadAnemometerNow > 0 && audioBlocked) {
            if (alarmTimer) { clearTimeout(alarmTimer); alarmTimer = null; }
            playAlarm();
        }
    }

    ["pointerdown", "keydown", "touchstart"].forEach((ev) =>
        document.addEventListener(ev, unlockAudio, {capture: true, passive: true}));

    function ring() {
        bell.classList.remove("ringing");
        void bell.offsetWidth;
        bell.classList.add("ringing");
    }

    function render() {
        const unread = alerts.filter((a) => !a.is_read).length;
        const unreadAnemometer = alerts.filter(isAnemometerAlarm).length;
        const badge = $id("notif-count");
        badge.textContent = unread > 99 ? "99+" : unread;
        badge.hidden = unread === 0;
        bell.classList.toggle("has-unread", unread > 0);
        // Audio is ONLY for unread anemometer/wind alarms.
        syncAlarm(unreadAnemometer);
        $id("np-total").textContent = unread;
        $id("mark-all-read").disabled = unread === 0;
        panel.querySelectorAll(".np-tab").forEach((t) => {
            const f = t.dataset.filter;
            const label = f.charAt(0).toUpperCase() + f.slice(1);
            t.innerHTML = label + (f === "unread" && unread ? '<span class="n">' + unread + "</span>" : "");
        });

        const list = alerts.filter((a) => filter === "all" || (filter === "unread" ? !a.is_read : a.is_read));
        const empty = $id("notif-empty");
        empty.hidden = list.length > 0;
        if (!list.length) {
            empty.querySelector("b").textContent = filter === "read" ? "Nothing here yet" : "All Clear";
            empty.querySelector("span").textContent = filter === "read"
                ? "Alerts you have read will stay listed here."
                : "No active alarms or critical events detected.";
        }
        $id("notif-list").innerHTML = list.map((a) =>
            '<div class="np-item ' + (a.is_read ? "read" : "unread") + '" data-id="' + a.id + '">' +
            (a.is_read ? "" : '<span class="np-dot"></span>') +
            '<span class="np-item-ic">!</span>' +
            '<div class="np-item-body">' +
            '<div class="np-item-top"><b>' + esc(a.title) + '</b><span class="np-tag">' + esc(String(a.level).toUpperCase()) + "</span>" +
            (a.value != null ? '<span class="np-val">' + Number(a.value).toFixed(1) + " m/s</span>" : "") + "</div>" +
            '<p class="np-msg">' + esc(a.message) + "</p>" +
            '<span class="np-time">' + fmtDate(a.ts) + " &middot; " + ago(a.ts) + "</span>" +
            "</div></div>").join("");
    }

    async function refresh() {
        try {
            const r = await fetch("/api/alerts", {cache: "no-store"});
            if (!r.ok) return;
            const d = await r.json();
            alerts = d.alerts || [];
            const maxId = alerts.length ? alerts[0].id : 0;
            const wasAlreadyAlarming = unreadAnemometerNow > 0;
            let newAlertArrived = false;
            let newAnemometerAlertArrived = false;

            if (lastMax < 0) {
                lastMax = maxId;                    // first visit: don't ring for old alerts
                store.set("plc_alert_last_id", lastMax);
            } else if (maxId > lastMax) {
                newAnemometerAlertArrived = alerts.some(
                    (a) => a.id > lastMax && isAnemometerAlarm(a)
                );
                lastMax = maxId;                    // brand-new alert arrived
                store.set("plc_alert_last_id", lastMax);
                newAlertArrived = true;
                ring();
            }

            render();

            // If another anemometer alert arrives while one is already unread,
            // play the MP3 immediately and restart the normal 5-second-gap cycle.
            if (newAnemometerAlertArrived && wasAlreadyAlarming) {
                if (alarmTimer) {
                    clearTimeout(alarmTimer);
                    alarmTimer = null;
                }
                playAlarm();
            }
        } catch (e) { /* server down: keep last list */ }
    }

    async function markRead(id) {
        const a = alerts.find((x) => x.id === id);
        if (!a || a.is_read) return;
        a.is_read = 1;
        render();
        try { await fetch("/api/alerts/" + id + "/read", {method: "POST"}); } catch (e) {}
    }

    function openPanel(open) {
        panel.hidden = !open;
        bell.classList.toggle("open", open);
        if (open) render();
    }

    bell.addEventListener("click", (e) => {
        e.stopPropagation();
        ring();
        openPanel(panel.hidden);
    });
    $id("notif-close").addEventListener("click", () => openPanel(false));
    $id("mark-all-read").addEventListener("click", async () => {
        alerts.forEach((a) => { a.is_read = 1; });
        render();
        try { await fetch("/api/alerts/read_all", {method: "POST"}); } catch (e) {}
    });
    panel.querySelectorAll(".np-tab").forEach((t) => t.addEventListener("click", () => {
        filter = t.dataset.filter;
        panel.querySelectorAll(".np-tab").forEach((x) => x.classList.toggle("active", x === t));
        render();
    }));
    $id("notif-list").addEventListener("click", (e) => {
        const item = e.target.closest(".np-item");
        if (item) markRead(parseInt(item.dataset.id, 10));
    });
    panel.addEventListener("click", (e) => e.stopPropagation());
    document.addEventListener("click", () => { if (!panel.hidden) openPanel(false); });
    document.addEventListener("keydown", (e) => { if (e.key === "Escape") openPanel(false); });

    const clock = $id("tb-clock");
    const tick = () => { if (clock) clock.textContent = new Date().toLocaleString("en-GB", {
        day: "numeric", month: "short", year: "numeric", hour: "2-digit", minute: "2-digit",
        second: "2-digit", hour12: true}); };
    tick(); setInterval(tick, 1000);

    refresh();
    setInterval(refresh, 3000);
})();
"""

BASE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<meta name="color-scheme" content="light">
<title>{{ title }} | PLC Monitor</title>
<style>{{ css|safe }}</style>
</head>
<body class="page-{{ active }}">
<header class="topbar">
    <a class="brand" href="/">
        <span class="brand-mark">{{ logo|safe }}</span>
        <span><span class="brand-name">PLC Monitor</span></span>
    </a>
    <div class="tb-right">
        <button type="button" id="notif-bell" class="tb-btn tb-bell" title="Alarms &amp; Alerts" aria-label="Alarms and alerts">
            <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M6 8a6 6 0 0 1 12 0c0 7 3 9 3 9H3s3-2 3-9"/><path d="M10.3 21a1.94 1.94 0 0 0 3.4 0"/></svg>
            <span class="tb-badge" id="notif-count" hidden>0</span>
        </button>
        <span class="tb-clock" id="tb-clock">--</span>
        <div class="conn" id="conn" role="status">
            <span class="dot"></span><b id="conn-text">Connecting...</b>
            <span class="conn-addr">{{ plc }}</span>
            <span id="conn-time">--</span>
        </div>
    </div>
</header>
<div class="np" id="notif-panel" hidden>
    <div class="np-head">
        <span class="np-ic"><svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.3 3.9 1.8 18a2 2 0 0 0 1.7 3h17a2 2 0 0 0 1.7-3L13.7 3.9a2 2 0 0 0-3.4 0z"/><path d="M12 9v4"/><path d="M12 17h.01"/></svg></span>
        <b class="np-title">Alarms &amp; Alerts ( <span id="np-total">0</span> )</b>
        <button type="button" class="np-markall" id="mark-all-read">&#10003; Mark All Read</button>
        <button type="button" class="np-close" id="notif-close" aria-label="Close">&times;</button>
    </div>
    <div class="np-tabs">
        <button type="button" class="np-tab active" data-filter="all">All</button>
        <button type="button" class="np-tab" data-filter="unread">Unread</button>
        <button type="button" class="np-tab" data-filter="read">Read</button>
    </div>
    <div class="np-list" id="notif-list"></div>
    <div class="np-empty" id="notif-empty">
        <div class="np-check">&#10003;</div>
        <b>All Clear</b>
        <span>No active alarms or critical events detected.</span>
    </div>
</div>
<nav class="tabs">{{ nav|safe }}</nav>
<main>{{ body|safe }}</main>
<script>const LIMITS = {{ limits|safe }};</script>
<script>{{ widgets_js|safe }}</script>
<script>{{ notif_js|safe }}</script>
{{ script|safe }}
</body>
</html>
"""

NAV_ITEMS = [
    ("/", "overview", "Overview", "grid"),
    ("/motor", "motor", "Pump Control", "power"),
    ("/anemometer", "anemometer", "Anemometer", "wind"),
    ("/temperature", "temperature", "Environment", "thermometer"),
    ("/pump", "pump", "Pump Monitor", "zap"),
]


def render_page(title, active, body, script=""):
    nav = "".join(
        f'<a href="{href}" class="{"active" if key == active else ""}">{icon(ic)}{label}</a>'
        for href, key, label, ic in NAV_ITEMS
    )
    limits = dict(UI_LIMITS, poll_sec=READ_INTERVAL_SEC)

    css = APP_CSS + (COMPACT_CSS if COMPACT_UI else "") + OVERVIEW_LAYOUT_CSS
    if TRANSPARENT_PAGE:
        glass = GLASS_CARD_CSS.replace("__CARD_BG__", CARD_BG)
        if active in WHITE_BG_PAGES:
            # white page, same light blue cards, with a visible card edge
            css += WHITE_PAGE_CSS + glass + ".w{border-color:rgba(147,197,253,.7)!important}"
        else:
            css += TRANSPARENT_CSS + glass + (
                TRANSPARENT_LIGHT_TEXT_CSS if TRANSPARENT_TEXT == "light" else "")
    elif active in WHITE_BG_PAGES:
        css += WHITE_PAGE_CSS
    css += NOTIF_CSS

    return render_template_string(
        BASE_HTML,
        title=title,
        active=active,
        css=css,
        logo=icon("cpu"),
        plc=f"{PLC_IP}:{PLC_PORT}",
        nav=nav,
        body=body,
        limits=json.dumps(limits),
        widgets_js=WIDGETS_JS,
        notif_js=NOTIF_JS,
        script=script,
    )


# ----------------------------------------------------------------------
# WEB UI - HTML BUILDING BLOCKS
# ----------------------------------------------------------------------

def widget(title, sub, icon_name, body, foot_left="", foot_right="",
           badge_id=None, icon_tone="blue", cls="", body_cls=""):
    badge = f'<span class="badge idle" id="{badge_id}">--</span>' if badge_id else ""
    foot = ""
    if foot_left or foot_right:
        foot = f'<div class="w-foot"><span>{foot_left}</span><span>{foot_right}</span></div>'
    return (
        f'<section class="w {cls}">'
        f'<div class="w-head"><div class="w-icon tone-{icon_tone}">{icon(icon_name)}</div>'
        f'<div><div class="w-title">{title}</div><div class="w-sub">{sub}</div></div>{badge}</div>'
        f'<div class="w-body {body_cls}">{body}</div>{foot}</section>'
    )


def tile(key, label, icon_name, tone):
    return (
        f'<div class="tile idle" id="tile-{key}">'
        f'<div class="tile-ic tone-{tone}">{icon(icon_name)}</div>'
        f'<div style="min-width:0"><div class="tile-l">{label}</div>'
        f'<div class="tile-s" id="tile-{key}-s">--</div></div></div>'
    )


def matrix(tiles, title="System Health &amp; Diagnostics Matrix"):
    return (
        f'<div class="section-h"><span class="ic">{icon("shield")}</span>{title}</div>'
        f'<div class="matrix">{"".join(tiles)}</div>'
    )


def motor_switch(prefix, large=False):
    seg = "seg lg" if large else "seg"
    state = "state-big lg" if large else "state-big"
    return (
        f'<div class="{state}" id="{prefix}-state">--</div>'
        f'<div class="{seg}" id="{prefix}-seg" role="group" aria-label="Pump switch">'
        f'<button type="button" data-v="0" onclick="sendMotor(false, \'{prefix}-result\')">Off</button>'
        f'<button type="button" data-v="1" onclick="sendMotor(true, \'{prefix}-result\')">On</button>'
        f'</div><div class="cmd-result" id="{prefix}-result" aria-live="polite"></div>'
    )


def gauge_body(prefix, unit, large=False, extra=""):
    size = " lg" if large else ""
    return (
        f'<div class="gauge{size}" id="{prefix}-g"></div>'
        f'<div class="readout{size}"><b id="{prefix}-v">--</b><small>{unit}</small></div>'
        f'<span class="pill idle" id="{prefix}-pill">--</span>{extra}'
    )


def kpi(value_id, unit):
    return f'<div class="kpi"><span id="{value_id}">--</span><small>{unit}</small></div>'


def trend_widget(title, sub, trend_id, icon_tone="blue", cls=""):
    return widget(
        title, sub, "trend", f'<div class="trend" id="{trend_id}"></div>',
        f'Window: <b id="{trend_id}-win">--</b>',
        f'Samples: <b id="{trend_id}-n">--</b>',
        icon_tone=icon_tone, cls=cls, body_cls="left",
    )


WIND_MAX = UI_LIMITS["wind"]["max"]


# ----------------------------------------------------------------------
# PAGES
# ----------------------------------------------------------------------

@app.get("/")
def home_page():
    # Time-scale graphs widget
    graph_widget_html = f"""
    <style>
    .time-scale {{display: flex; gap: 0.5rem; margin: 1rem 0;}}
    .time-scale button {{padding: 0.5rem 1rem; border: 1px solid #cbd5e1; background: #f1f5f9; border-radius: 6px;
                        cursor: pointer; font-size: 0.875rem; font-weight: 600; color: #1e293b; transition: all 0.2s;}}
    .time-scale button:hover {{background: #e2e8f0;}}
    .time-scale button.active {{background: #2563eb; color: white; border-color: #2563eb;}}
    .graph-container {{position: relative; height: 280px; margin-top: 1rem;}}

    </style>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>
    
    <script>
    let charts = {{}};
    
    function loadGraph(scale, metric) {{
        fetch('/api/graph/' + scale + '?metric=' + metric, {{cache: 'no-store'}})
            .then(r => r.json())
            .then(res => drawChart(scale, metric, res.data))
            .catch(e => console.error('Graph load error:', e));
        
        document.querySelectorAll('#gs-' + metric + ' .time-scale button').forEach(b => 
            b.classList.remove('active'));
        event.target.classList.add('active');
    }}
    
    function drawChart(scale, metric, data) {{
        const cid = 'canvas-' + metric;
        const ctx = document.getElementById(cid);
        if (!ctx) return;
        
        const labels = data.map(d => {{
            const dt = new Date(d.timestamp * 1000);
            if (scale === '1d') return dt.toLocaleTimeString('en-US', {{hour: '2-digit', minute: '2-digit'}});
            return d.date || d.week;
        }});
        
        let values;
        if (metric === 'temperature') {{
            values = data.map(d => d.temperature !== undefined ? d.temperature : d.temperature_avg || 0);
        }} else if (metric === 'humidity') {{
            values = data.map(d => d.humidity !== undefined ? d.humidity : d.humidity_avg || 0);
        }} else if (metric === 'power') {{
            values = data.map(d => d.power || 0);
        }}
        
        if (charts[cid]) charts[cid].destroy();
        
        const colors = {{temperature: '#f59e0b', humidity: '#0891b2', power: '#7c3aed'}};
        const color = colors[metric] || '#2563eb';
        
        charts[cid] = new Chart(ctx, {{
            type: 'line',
            data: {{
                labels: labels,
                datasets: [{{
                    label: metric.charAt(0).toUpperCase() + metric.slice(1),
                    data: values,
                    borderColor: color,
                    backgroundColor: color + '20',
                    tension: 0.4,
                    fill: true,
                    borderWidth: 2,
                    pointRadius: 0,
                    pointHoverRadius: 4
                }}]
            }},
            options: {{
                responsive: true,
                maintainAspectRatio: false,
                plugins: {{legend: {{display: false}}}},
                scales: {{
                    y: {{beginAtZero: false, grid: {{color: '#f1f5f9'}}}},
                    x: {{grid: {{color: '#f1f5f9'}}}}
                }}
            }}
        }});
    }}
    
    // Load initial graphs
    window.addEventListener('DOMContentLoaded', () => {{
        ['temperature', 'humidity', 'power'].forEach(m => {{
            loadGraph('1d', m);
        }});
    }});
    </script>
    """
    
    wind_body = gauge_body("ov-wind", "m/s")

    env_body = (
        '<div class="env-row">'
        '<div class="thermo" id="ov-thermo"></div>'
        '<div class="env-col"><span class="kv-l">Temperature</span>'
        '<div class="big"><span id="ov-temp">--</span><small>°C</small></div>'
        '<span class="pill idle" id="ov-temp-pill">--</span></div>'
        '<div class="vsep"></div>'
        '<div class="ring" id="ov-hum-ring"></div>'
        '</div>'
    )

    pump_body = (
        '<div class="kv"><span class="kv-l">Active power</span><span class="kv-v blue" id="ov-power">--</span></div>'
        '<div class="band" id="ov-power-band"></div>'
        '<div class="kv"><span class="kv-l">Voltage</span><span class="kv-v idle" id="ov-volt">--</span></div>'
        '<div class="band" id="ov-volt-band"></div>'
        '<div class="kv"><span class="kv-l">Current</span><span class="kv-v idle" id="ov-curr">--</span></div>'
        '<div class="band" id="ov-curr-band"></div>'
    )

    body = (
        '<div class="widgets">'
        + widget("Pump Control", 'Poll: <span class="js-poll">--</span>', "toggle", motor_switch("ov"),
                 '<span id="ov-motor-fl">--</span>', '<span class="fv idle" id="ov-motor-fr">--</span>',
                 icon_tone="dark")
        + widget("Wind Speed", "Cup anemometer", "wind", wind_body,
                 f'Range: <b>0 to {WIND_MAX} m/s</b>', '<span class="fv idle" id="ov-bft">--</span>',
                 badge_id="ov-wind-badge", icon_tone="cyan")
        + widget("Environment", "Temperature, humidity and pressure", "thermometer", env_body,
                 'Pressure: <b><span id="ov-press">--</span> hPa</b>', '<span class="fv idle" id="ov-press-fv">--</span>',
                 badge_id="ov-env-badge", icon_tone="amber")
        + widget("Pump Load", "Active power, voltage and current", "zap", pump_body,
                 "Pump state", '<span class="fv idle" id="ov-pump-fr">--</span>',
                 badge_id="ov-pump-badge", icon_tone="violet", body_cls="left")
        + "</div>"
        + matrix([
            tile("motor", "Pump", "power", "green"),
            tile("link", "Modbus link", "radio", "cyan"),
            tile("wind", "Wind", "wind", "blue"),
            tile("temp", "Temperature", "thermometer", "amber"),
            tile("hum", "Humidity", "droplet", "cyan"),
            tile("press", "Pressure", "gauge", "violet"),
            tile("volt", "Voltage", "zap", "amber"),
            tile("curr", "Current", "activity", "red"),
            tile("load", "Load", "gauge", "violet"),
            tile("duty", "Duty cycle", "activity", "violet"),
            tile("run", "Current run", "clock", "blue"),
            tile("total", "Total running", "clock", "green"),
            tile("idle", "Total idle time", "clock", "red"),
        ])
        + f'<div class="section-h"><span class="ic">{icon("fan")}</span>Pump Activity</div>'
        + '<div class="widgets">'
        + widget("Current Downtime", "Since the pump last stopped", "clock", kpi("ov-down", "min"),
                 "Duration", '<b id="ov-down-sub">--</b>', icon_tone="red")
        + widget("State History", "Each bar is one PLC poll", "trend",
                 '<div class="strip" id="ov-strip"></div>'
                 '<div class="legend"><span><i style="background:#22c55e"></i>Running</span>'
                 '<span><i style="background:#fca5a5"></i>Stopped</span></div>',
                 'Window: <b id="ov-hist-win">--</b>', 'Duty: <b id="ov-duty">--</b>',
                 icon_tone="violet", cls="span-2", body_cls="left")
        + "</div>"
        + f'<div class="section-h"><span class="ic">{icon("trend")}</span>Time-Scale Graphs (Historical Data)</div>'
        + '<div class="widgets" style="grid-template-columns: 1fr;">'
        # ROW 1: Temperature + Humidity (MERGED, dual Y-axis)
        + widget("Temperature & Humidity", "1H/1D/1W/1M/1Y - °C / %RH", "thermometer",
                 '<div id="gs-temp-humid" class="time-scale" style="display:flex;justify-content:flex-end;gap:6px;margin-bottom:8px;">'
                 '<button onclick="loadMergedGraph(\'1h\', \'temp-humid\')" class="active">1H</button>'
                 '<button onclick="loadMergedGraph(\'1d\', \'temp-humid\')">1D</button>'
                 '<button onclick="loadMergedGraph(\'1w\', \'temp-humid\')">1W</button>'
                 '<button onclick="loadMergedGraph(\'1m\', \'temp-humid\')">1M</button>'
                 '<button onclick="loadMergedGraph(\'1y\', \'temp-humid\')">1Y</button>'
                 '</div><div class="graph-container"><canvas id="canvas-temp-humid"></canvas></div>',
                 "Needs 1h data", "Hourly avg", icon_tone="amber", body_cls="left")
        # ROW 2: Voltage + Current (MERGED, dual Y-axis)
        + widget("Voltage & Current", "1H/1D/1W/1M/1Y - V / A", "zap",
                 '<div id="gs-volt-curr" class="time-scale" style="display:flex;justify-content:flex-end;gap:6px;margin-bottom:8px;">'
                 '<button onclick="loadMergedGraph(\'1h\', \'volt-curr\')" class="active">1H</button>'
                 '<button onclick="loadMergedGraph(\'1d\', \'volt-curr\')">1D</button>'
                 '<button onclick="loadMergedGraph(\'1w\', \'volt-curr\')">1W</button>'
                 '<button onclick="loadMergedGraph(\'1m\', \'volt-curr\')">1M</button>'
                 '<button onclick="loadMergedGraph(\'1y\', \'volt-curr\')">1Y</button>'
                 '</div><div class="graph-container"><canvas id="canvas-volt-curr"></canvas></div>',
                 "Needs 1h data", "Hourly avg", icon_tone="amber", body_cls="left")
        + "</div>"
        + '<div class="widgets hist-pair" style="grid-template-columns: 1fr 1fr; gap: 16px;">'
        # ROW 3: Pump Power (SINGLE)
        + widget("Pump Power", "1H/1D/1W/1M/1Y - W", "gauge",
                 '<div id="gs-power" class="time-scale" style="display:flex;justify-content:flex-end;gap:6px;margin-bottom:8px;">'
                 '<button onclick="loadGraph(\'1h\', \'power\')" class="active">1H</button>'
                 '<button onclick="loadGraph(\'1d\', \'power\')">1D</button>'
                 '<button onclick="loadGraph(\'1w\', \'power\')">1W</button>'
                 '<button onclick="loadGraph(\'1m\', \'power\')">1M</button>'
                 '<button onclick="loadGraph(\'1y\', \'power\')">1Y</button>'
                 '</div><div class="graph-container"><canvas id="canvas-power"></canvas></div>',
                 "Needs 1h data", "Hourly avg", icon_tone="violet", body_cls="left")
        # ROW 4: Pressure (SINGLE)
        + widget("Pressure", "1H/1D/1W/1M/1Y - hPa", "cloud",
                 '<div id="gs-pressure" class="time-scale" style="display:flex;justify-content:flex-end;gap:6px;margin-bottom:8px;">'
                 '<button onclick="loadGraph(\'1h\', \'pressure\')" class="active">1H</button>'
                 '<button onclick="loadGraph(\'1d\', \'pressure\')">1D</button>'
                 '<button onclick="loadGraph(\'1w\', \'pressure\')">1W</button>'
                 '<button onclick="loadGraph(\'1m\', \'pressure\')">1M</button>'
                 '<button onclick="loadGraph(\'1y\', \'pressure\')">1Y</button>'
                 '</div><div class="graph-container"><canvas id="canvas-pressure"></canvas></div>',
                 "Needs 1h data", "Hourly avg", icon_tone="purple", body_cls="left")
        + "</div>"
    )

    script = r"""
    <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>
    <script>
    let charts = {};
    
    function loadGraph(scale, metric) {
        const btn = event.target;
        btn.disabled = true;
        
        fetch('/api/graph/' + scale + '?metric=' + metric, {cache: 'no-store'})
            .then(r => r.json())
            .then(res => {
                const cid = 'canvas-' + metric;
                const canvas = document.getElementById(cid);
                const container = canvas ? canvas.parentElement : null;
                
                if (!res.data || res.data.length === 0) {
                    if (container) {
                        container.innerHTML = '<div id="' + cid + '" style="display:none;"></div><div style="padding: 20px; text-align: center; color: #94a3b8;">No data yet. Waiting for ' + scale + ' data...</div>';
                    }
                } else {
                    if (container) {
                        container.innerHTML = '<canvas id="' + cid + '"></canvas>';
                    }
                    drawChart(scale, metric, res.data);
                }
            })
            .catch(e => {
                console.error('Graph error:', e);
                const cid = 'canvas-' + metric;
                const canvas = document.getElementById(cid);
                const container = canvas ? canvas.parentElement : null;
                if (container) {
                    container.innerHTML = '<div id="' + cid + '" style="display:none;"></div><div style="padding: 20px; text-align: center; color: #ef4444;">Error loading graph</div>';
                }
            })
            .finally(() => { btn.disabled = false; });
        
        document.querySelectorAll('#gs-' + metric + ' button').forEach(b => 
            b.classList.remove('active'));
        btn.classList.add('active');
    }
    
    function loadMergedGraph(scale, mergeType) {
        const btn = event.target;
        btn.disabled = true;
        const gsId = 'gs-' + mergeType;
        
        fetch('/api/graph/' + scale + '?metric=' + (mergeType === 'temp-humid' ? 'temperature' : 'voltage'), {cache: 'no-store'})
            .then(r => r.json())
            .then(res => {
                const cid = 'canvas-' + mergeType;
                const canvas = document.getElementById(cid);
                const container = canvas ? canvas.parentElement : null;
                
                if (!res.data || res.data.length === 0) {
                    if (container) {
                        container.innerHTML = '<div id="' + cid + '" style="display:none;"></div><div style="padding: 20px; text-align: center; color: #94a3b8;">No data yet. Waiting for ' + scale + ' data...</div>';
                    }
                } else {
                    if (container) {
                        container.innerHTML = '<canvas id="' + cid + '"></canvas>';
                    }
                    drawMergedChart(scale, mergeType, res.data);
                }
            })
            .catch(e => {
                console.error('Graph error:', e);
                const cid = 'canvas-' + mergeType;
                const canvas = document.getElementById(cid);
                const container = canvas ? canvas.parentElement : null;
                if (container) {
                    container.innerHTML = '<div id="' + cid + '" style="display:none;"></div><div style="padding: 20px; text-align: center; color: #ef4444;">Error loading graph</div>';
                }
            })
            .finally(() => { btn.disabled = false; });
        
        document.querySelectorAll('#' + gsId + ' button').forEach(b => 
            b.classList.remove('active'));
        btn.classList.add('active');
    }
    
    /* ===== Historical graphs: trend-card style + built-in wheel zoom ===== */
    (function () {
        const st = document.createElement('style');
        st.textContent =
            '.graph-container{position:relative;height:260px;margin-top:4px;cursor:grab;}' +
            '.graph-container.dragging{cursor:grabbing;}' +
            '.hist-hint{position:absolute;left:8px;bottom:2px;font-size:10px;color:#94a3b8;pointer-events:none;}' +
            '.widgets.hist-pair .graph-container{height:220px;}';
        document.head.appendChild(st);
    })();

    const HIST = {};             // cid -> {ts, series, start, end, scale}
    const HIST_MAX_POINTS = 150;  // same density as the trend cards

    function hexA(hex, a) {
        const h = hex.replace('#', '');
        const r = parseInt(h.substr(0, 2), 16), g = parseInt(h.substr(2, 2), 16), b = parseInt(h.substr(4, 2), 16);
        return 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';
    }

    function gradientFill(color) {
        return function (context) {
            const chart = context.chart, area = chart.chartArea;
            if (!area) return 'transparent';
            const g = chart.ctx.createLinearGradient(0, area.top, 0, area.bottom);
            g.addColorStop(0, hexA(color, 0.25));
            g.addColorStop(1, hexA(color, 0));
            return g;
        };
    }

    function fmtTime(ts, spanSec) {
        const dt = new Date(ts * 1000);
        if (spanSec <= 86400) {
            return dt.toLocaleTimeString('en-US', {hour: 'numeric', minute: '2-digit', hour12: true});
        }
        if (spanSec <= 10 * 86400) {
            return dt.toLocaleDateString('en-US', {month: 'short', day: 'numeric'}) + ', ' +
                   dt.toLocaleTimeString('en-US', {hour: 'numeric', hour12: true});
        }
        return dt.toLocaleDateString('en-US', {month: 'short', day: 'numeric'});
    }

    // Average raw points into at most maxPts buckets (smooth curve like the trend cards)
    function bucketAvg(ts, arrs, s, e, maxPts) {
        const n = e - s, step = Math.max(1, Math.ceil(n / maxPts));
        const outT = [], outA = arrs.map(() => []);
        for (let i = s; i < e; i += step) {
            const j = Math.min(e, i + step);
            outT.push(ts[Math.floor((i + j - 1) / 2)]);
            arrs.forEach((a, k) => {
                let sum = 0, c = 0;
                for (let q = i; q < j; q++) {
                    const v = a[q];
                    if (v !== null && !isNaN(v)) { sum += v; c++; }
                }
                outA[k].push(c ? sum / c : null);
            });
        }
        return {t: outT, a: outA};
    }

    // Y-axis hugs min/max with 15% padding, exactly like Trend.draw()
    function fitRange(vals) {
        const v = vals.filter(x => x !== null && !isNaN(x));
        if (!v.length) return {min: 0, max: 1};
        let lo = Math.min(...v), hi = Math.max(...v);
        if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
        const pad = (hi - lo) * 0.15;
        return {min: lo - pad, max: hi + pad};
    }

    function tickFmt(v) {
        const a = Math.abs(v);
        return a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : v.toFixed(2);
    }

    function renderHist(cid) {
        const H = HIST[cid], chart = charts[cid];
        if (!H || !chart) return;
        const s = H.start, e = H.end;
        const span = H.ts[e - 1] - H.ts[s];
        const {t, a} = bucketAvg(H.ts, H.series, s, e, HIST_MAX_POINTS);
        chart.data.labels = t.map(x => fmtTime(x, span));
        chart.data.datasets.forEach((ds, k) => {
            ds.data = a[k];
            const r = fitRange(a[k]);
            const ax = chart.options.scales[ds.yAxisID];
            ax.min = r.min; ax.max = r.max;
        });
        chart.update('none');
        const hint = document.getElementById(cid + '-hint');
        if (hint) {
            const zoomed = (e - s) < H.ts.length;
            hint.textContent = zoomed ? 'Zoomed · double-click to reset' : 'Scroll to zoom · drag to pan';
        }
    }

    function bindZoom(cid) {
        const canvas = document.getElementById(cid);
        if (!canvas || canvas._histBound) return;
        canvas._histBound = true;
        const box = canvas.parentElement;
        if (box && !document.getElementById(cid + '-hint')) {
            const h = document.createElement('div');
            h.id = cid + '-hint'; h.className = 'hist-hint';
            box.appendChild(h);
        }

        canvas.addEventListener('wheel', ev => {
            const H = HIST[cid], chart = charts[cid];
            if (!H || !chart || !chart.chartArea) return;
            ev.preventDefault();
            const area = chart.chartArea, total = H.ts.length;
            const len = H.end - H.start;
            const frac = Math.min(1, Math.max(0, (ev.offsetX - area.left) / (area.right - area.left)));
            const anchor = H.start + frac * len;
            const factor = ev.deltaY < 0 ? 0.75 : 1.33;
            let newLen = Math.round(len * factor);
            newLen = Math.max(Math.min(20, total), Math.min(total, newLen));
            let ns = Math.round(anchor - frac * newLen);
            ns = Math.max(0, Math.min(total - newLen, ns));
            H.start = ns; H.end = ns + newLen;
            renderHist(cid);
        }, {passive: false});

        let drag = null;
        canvas.addEventListener('mousedown', ev => {
            const H = HIST[cid];
            if (!H) return;
            drag = {x: ev.clientX, s: H.start, e: H.end};
            box && box.classList.add('dragging');
        });
        window.addEventListener('mousemove', ev => {
            if (!drag) return;
            const H = HIST[cid], chart = charts[cid];
            if (!H || !chart || !chart.chartArea) return;
            const w = chart.chartArea.right - chart.chartArea.left;
            const len = drag.e - drag.s, total = H.ts.length;
            let shift = Math.round(-(ev.clientX - drag.x) / w * len);
            let ns = Math.max(0, Math.min(total - len, drag.s + shift));
            if (ns !== H.start) { H.start = ns; H.end = ns + len; renderHist(cid); }
        });
        window.addEventListener('mouseup', () => {
            drag = null;
            box && box.classList.remove('dragging');
        });
        canvas.addEventListener('dblclick', () => {
            const H = HIST[cid];
            if (!H) return;
            H.start = 0; H.end = H.ts.length;
            renderHist(cid);
        });
    }

    function yAxis(color, position, title, showGrid) {
        return {
            type: 'linear', display: true, position: position,
            grid: showGrid ? {color: 'rgba(148,163,184,0.18)', drawBorder: false, borderDash: [3, 3]}
                           : {drawOnChartArea: false, drawBorder: false},
            ticks: {font: {size: 10}, color: color, maxTicksLimit: 5, callback: tickFmt},
            title: {display: !!title, text: title, font: {size: 10}, color: color}
        };
    }

    function trendDataset(label, color, axisId) {
        return {
            label: label, data: [], yAxisID: axisId,
            borderColor: color, backgroundColor: gradientFill(color),
            fill: 'start', tension: 0.35, borderWidth: 2,
            pointRadius: 0, pointHoverRadius: 4, pointHitRadius: 8,
            pointBackgroundColor: color, pointBorderColor: '#fff', spanGaps: true
        };
    }

    function buildHistChart(cid, scale, data, defs) {
        const canvas = document.getElementById(cid);
        if (!canvas || canvas.tagName !== 'CANVAS') return;
        if (charts[cid]) charts[cid].destroy();

        const rows = data.slice().sort((p, q) => p.timestamp - q.timestamp);
        HIST[cid] = {
            ts: rows.map(d => d.timestamp),
            series: defs.map(df => rows.map(df.get)),
            start: 0, end: rows.length, scale: scale
        };

        const scales = {
            x: {
                grid: {display: false, drawBorder: false},
                ticks: {font: {size: 10}, color: '#64748b', maxTicksLimit: 6, maxRotation: 0, autoSkip: true}
            }
        };
        defs.forEach((df, k) => {
            scales[df.axis] = yAxis(df.color, k === 0 ? 'left' : 'right', defs.length > 1 ? df.label : '', k === 0);
        });

        charts[cid] = new Chart(canvas, {
            type: 'line',
            data: {labels: [], datasets: defs.map(df => trendDataset(df.label, df.color, df.axis))},
            options: {
                responsive: true, maintainAspectRatio: false, animation: false,
                interaction: {mode: 'index', intersect: false},
                plugins: {
                    legend: {display: defs.length > 1, position: 'top', align: 'end',
                             labels: {font: {size: 11}, boxWidth: 10, boxHeight: 10, usePointStyle: true}},
                    tooltip: {
                        backgroundColor: 'rgba(15,23,42,0.85)', padding: 10,
                        callbacks: {
                            label: c => ' ' + c.dataset.label + ': ' +
                                (c.parsed.y === null ? '--' : c.parsed.y.toFixed(2)) + ' ' + defs[c.datasetIndex].unit
                        }
                    }
                },
                scales: scales
            }
        });
        bindZoom(cid);
        renderHist(cid);
    }

    const num0 = v => { const x = parseFloat(v); return isNaN(x) ? null : x; };


    const METRIC_DEFS = {
        temperature: {label: 'Temperature', unit: '°C',  color: '#f59e0b', get: d => num0(d.temperature ?? d.temperature_avg)},
        humidity:    {label: 'Humidity',    unit: '%RH', color: '#0891b2', get: d => num0(d.humidity ?? d.humidity_avg)},
        pressure:    {label: 'Pressure',    unit: 'hPa', color: '#8b5cf6', get: d => num0(d.pressure ?? d.pressure_avg)},
        power:       {label: 'Pump Power',  unit: 'W',   color: '#7c3aed', get: d => num0(d.power ?? d.motor_power_avg)},
        voltage:     {label: 'Voltage',     unit: 'V',   color: '#eab308', get: d => num0(d.voltage ?? d.motor_voltage_avg)},
        current:     {label: 'Current',     unit: 'A',   color: '#ef4444', get: d => num0(d.current ?? d.motor_current_avg)}
    };

    function drawChart(scale, metric, data) {
        const m = METRIC_DEFS[metric];
        if (!m) return;
        buildHistChart('canvas-' + metric, scale, data, [Object.assign({axis: 'y'}, m)]);
    }

    function drawMergedChart(scale, mergeType, data) {
        const pair = mergeType === 'temp-humid' ? ['temperature', 'humidity'] : ['voltage', 'current'];
        buildHistChart('canvas-' + mergeType, scale, data, [
            Object.assign({axis: 'y'},  METRIC_DEFS[pair[0]]),
            Object.assign({axis: 'y1'}, METRIC_DEFS[pair[1]])
        ]);
    }

    Gauge.mount("ov-wind-g", {min: 0, max: LIMITS.wind.max, zones: Z.wind(), ticks: T.wind(), decimals: 1});
    Thermo.mount("ov-thermo");
    Ring.mount("ov-hum-ring", {label: "% RH"});
    Band.mount("ov-power-band", B.power());
    Band.mount("ov-volt-band", B.voltage());
    Band.mount("ov-curr-band", B.current());

    async function refresh() {
        const res = await Promise.all([getJSON("/api/all"), getJSON("/api/history")]);
        const d = res[0], h = res[1];
        status(d);

        /* motor */
        renderMotor("ov", d);
        setHTML("ov-motor-fl", d.motor_control
            ? "Up: <b>" + fmtDuration(d.motor_uptime_min) + "</b>"
            : "Down: <b>" + fmtDuration(d.motor_downtime_min) + "</b>");
        const tot = num(d.motor_total_running_min);
        setFv("ov-motor-fr", [tot === null ? "idle" : "ok", tot === null ? "--" : "Total " + (tot / 60).toFixed(1) + " h"]);

        /* wind */
        const ws = Status.wind(d.wind_speed);
        Gauge.set("ov-wind-g", d.wind_speed);
        setText("ov-wind-v", fmt(d.wind_speed, 1));
        setPill("ov-wind-pill", ws);
        setBadge("ov-wind-badge", ws);
        setFv("ov-bft", ws);

        /* environment */
        const ts = Status.temperature(d.temperature), hs = Status.humidity(d.humidity), ps = Status.pressure(d.pressure);
        Thermo.set("ov-thermo", d.temperature);
        setText("ov-temp", fmt(d.temperature, 1));
        setPill("ov-temp-pill", ts);
        Ring.set("ov-hum-ring", d.humidity, 100, hs[0] === "ok" ? "#0891b2" : TONE_COLOR[hs[0]], fmt(d.humidity, 0));
        setText("ov-press", fmt(d.pressure, 1));
        setFv("ov-press-fv", ps);
        setBadge("ov-env-badge", summary([ts, hs, ps]));

        /* pump */
        const vs = Status.voltage(d.motor_voltage), cs = Status.current(d.motor_current), ls = Status.load(d.motor_power);
        const p = fmtPower(d.motor_power);
        setHTML("ov-power", p.v + "<small>" + p.u + "</small>");
        setHTML("ov-volt", fmt(d.motor_voltage, 1) + "<small>V</small>");
        setClass("ov-volt", "kv-v", vs[0]);
        setHTML("ov-curr", fmt(d.motor_current, 2) + "<small>A</small>");
        setClass("ov-curr", "kv-v", cs[0]);
        Band.set("ov-power-band", d.motor_power);
        Band.set("ov-volt-band", d.motor_voltage);
        Band.set("ov-curr-band", d.motor_current);
        setBadge("ov-pump-badge", summary([vs, cs, ls]));
        setFv("ov-pump-fr", Status.motor(d));

        /* health matrix */
        setTile("motor", Status.motor(d));
        setTile("link", Status.link(d));
        setTile("wind", ws);
        setTile("temp", ts);
        setTile("hum", hs);
        setTile("press", ps);
        setTile("volt", vs);
        setTile("curr", cs);


        /* pump activity */
        setText("ov-down", fmt(d.motor_downtime_min, 1));
        setText("ov-down-sub", fmtDuration(d.motor_downtime_min));
        const states = h.map((x) => x.motor_control).slice(-150);
        drawStrip("ov-strip", states);
        const valid = states.filter((x) => x !== null && x !== undefined);
        const duty = valid.length ? valid.filter(Boolean).length / valid.length * 100 : null;
        setText("ov-duty", duty === null ? "--" : duty.toFixed(0) + "%");
        setText("ov-hist-win", windowLabel(states.length));
        const conn = !!d.connected;
        setTile("load", ls);
        setTile("duty", duty === null ? ["idle", "No data"] : ["ok", duty.toFixed(0) + "% running"]);
        setTile("run", !conn ? ["idle", "Unknown"] : d.motor_control
            ? ["ok", "Up " + fmtDuration(d.motor_uptime_min)]
            : ["warn", "Down " + fmtDuration(d.motor_downtime_min)]);
        setTile("total", tot === null ? ["idle", "No data"] : ["ok", (tot / 60).toFixed(2) + " h"]);
        const idle = num(d.motor_total_idle_min);
        setTile("idle", idle === null ? ["idle", "No data"] : ["warn", (idle / 60).toFixed(2) + " h"]);
    }
    
    // Load initial graphs
    setTimeout(() => {
        // Load merged graphs
        ['temp-humid', 'volt-curr'].forEach(mergeType => {
            const btn = document.querySelector('#gs-' + mergeType + ' button.active');
            if (btn) {
                const metric = mergeType === 'temp-humid' ? 'temperature' : 'voltage';
                fetch('/api/graph/1h?metric=' + metric, {cache: 'no-store'})
                    .then(r => r.json())
                    .then(res => {
                        console.log('Graph data for ' + mergeType + ': ' + res.data.length + ' points');
                        if (res.data && res.data.length > 0) {
                            drawMergedChart('1h', mergeType, res.data);
                        } else {
                            console.warn('No data for ' + mergeType);
                        }
                    })
                    .catch(e => console.error('Initial load error for ' + mergeType + ':', e));
            }
        });
        
        // Load single graphs
        ['pressure', 'power'].forEach(m => {
            const btn = document.querySelector('#gs-' + m + ' button.active');
            if (btn) {
                fetch('/api/graph/1h?metric=' + m, {cache: 'no-store'})
                    .then(r => r.json())
                    .then(res => {
                        console.log('Graph data for ' + m + ': ' + res.data.length + ' points');
                        if (res.data && res.data.length > 0) {
                            drawChart('1h', m, res.data);
                        } else {
                            console.warn('No data for ' + m);
                        }
                    })
                    .catch(e => console.error('Initial load error for ' + m + ':', e));
            }
        });
    }, 1000);
    
    startLoop(refresh);
    </script>
    """
    return render_page("Overview", "overview", body, script)


@app.get("/motor")
def motor_page():
    body = (
        '<div class="widgets">'
        + widget("Pump Control", 'Poll: <span class="js-poll">--</span>', "toggle", motor_switch("mc", large=True),
                 "Write register", f"<b>{MOTOR_CONTROL_WRITE_ADDRESS}</b> (holding)",
                 badge_id="mc-badge", icon_tone="dark")
        + "</div>"
    )

    script = r"""
    <script>
    async function refresh() {
        const d = await getJSON("/api/motor");
        status(d);
        renderMotor("mc", d);
    }
    startLoop(refresh);
    </script>
    """
    return render_page("Pump Control", "motor", body, script)


@app.get("/anemometer")
def anemometer_page():
    body = (
        '<div class="widgets">'
        + widget("Wind Speed", 'Poll: <span class="js-poll">--</span>', "wind", gauge_body("an", "m/s", large=True),
                 f"Range: <b>0 to {WIND_MAX} m/s</b>", '<span class="fv idle" id="an-fv">--</span>',
                 badge_id="an-badge", icon_tone="cyan")
        + "</div>"
    )

    script = r"""
    <script>
    Gauge.mount("an-g", {min: 0, max: LIMITS.wind.max, zones: Z.wind(), ticks: T.wind(), decimals: 1});

    async function refresh() {
        const d = await getJSON("/api/anemometer");
        status(d);
        const ws = Status.wind(d.wind_speed);
        Gauge.set("an-g", d.wind_speed);
        setText("an-v", fmt(d.wind_speed, 1));
        setPill("an-pill", ws);
        setBadge("an-badge", ws);
        setFv("an-fv", ws);
    }
    startLoop(refresh);
    </script>
    """
    return render_page("Anemometer", "anemometer", body, script)


@app.get("/temperature")
def temperature_page():
    temp_body = (
        '<div class="env-row">'
        '<div class="thermo lg" id="en-thermo"></div>'
        '<div class="env-col"><div class="big xl"><span id="en-t">--</span><small>°C</small></div>'
        '<span class="pill idle" id="en-t-pill">--</span></div>'
        '</div><div class="band" id="en-t-band"></div>'
    )
    hum_body = (
        '<div class="ring lg" id="en-h-ring"></div>'
        '<span class="pill idle" id="en-h-pill">--</span>'
        '<div class="band" id="en-h-band"></div>'
    )
    press_body = gauge_body("en-p", "hPa")

    body = (
        '<div class="widgets cols-3">'
        + widget("Temperature", 'Poll: <span class="js-poll">--</span>', "thermometer", temp_body,
                 "Dew point", '<b id="en-dew">--</b>',
                 badge_id="en-t-badge", icon_tone="amber")
        + widget("Humidity", "Relative humidity", "droplet", hum_body,
                 "Comfort band", f'<b>{UI_LIMITS["humidity"]["low"]} to {UI_LIMITS["humidity"]["high"]} %RH</b>',
                 badge_id="en-h-badge", icon_tone="cyan")
        + widget("Pressure", "Barometric pressure", "gauge", press_body,
                 "In kPa", '<b id="en-kpa">--</b>',
                 badge_id="en-p-badge", icon_tone="violet")
        + "</div>"
    )

    script = r"""
    <script>
    Thermo.mount("en-thermo");
    Band.mount("en-t-band", B.temperature());
    Ring.mount("en-h-ring", {label: "% RH", stroke: 8});
    Band.mount("en-h-band", B.humidity());
    Gauge.mount("en-p-g", {min: LIMITS.pressure.min, max: LIMITS.pressure.max, zones: Z.pressure(), ticks: T.pressure()});

    async function refresh() {
        const d = await getJSON("/api/temperature");
        status(d);

        const ts = Status.temperature(d.temperature), hs = Status.humidity(d.humidity), ps = Status.pressure(d.pressure);

        Thermo.set("en-thermo", d.temperature);
        setText("en-t", fmt(d.temperature, 1));
        setPill("en-t-pill", ts);
        setBadge("en-t-badge", ts);
        Band.set("en-t-band", d.temperature);
        const dp = dewPoint(d.temperature, d.humidity);
        setText("en-dew", dp === null ? "--" : dp.toFixed(1) + " °C");

        Ring.set("en-h-ring", d.humidity, 100, hs[0] === "ok" ? "#0891b2" : TONE_COLOR[hs[0]], fmt(d.humidity, 1));
        setPill("en-h-pill", hs);
        setBadge("en-h-badge", hs);
        Band.set("en-h-band", d.humidity);

        Gauge.set("en-p-g", d.pressure);
        setText("en-p-v", fmt(d.pressure, 1));
        setPill("en-p-pill", ps);
        setBadge("en-p-badge", ps);
        const p = num(d.pressure);
        setText("en-kpa", p === null ? "--" : (p / 10).toFixed(2) + " kPa");
    }
    startLoop(refresh);
    </script>
    """
    return render_page("Environment", "temperature", body, script)


@app.get("/pump")
def pump_page():
    power_body = (
        '<div class="ring lg" id="pu-load"></div>'
        '<div class="readout" style="margin-top:0"><b id="pu-power">--</b><small id="pu-power-u">W</small></div>'
        '<div class="band" id="pu-p-band"></div>'
    )

    body = (
        '<div class="widgets">'
        + widget("Voltage", 'Poll: <span class="js-poll">--</span>', "zap", gauge_body("pu-v", "V"),
                 "Nominal", f'<b>{UI_LIMITS["voltage"]["nominal"]} V</b>',
                 badge_id="pu-v-badge", icon_tone="amber")
        + widget("Current", "Scaled x" + str(CURRENT_SCALE), "activity", gauge_body("pu-c", "A"),
                 "Trip level", f'<b>{UI_LIMITS["current"]["trip"]} A</b>',
                 badge_id="pu-c-badge", icon_tone="red")
        + widget("Power", "Voltage x current", "gauge", power_body,
                 "Rated", f'<b>{UI_LIMITS["power"]["max"]} W</b>',
                 badge_id="pu-p-badge", icon_tone="violet")
        + "</div>"
    )

    script = r"""
    <script>
    Gauge.mount("pu-v-g", {min: LIMITS.voltage.min, max: LIMITS.voltage.max, zones: Z.voltage(), ticks: T.voltage()});
    Gauge.mount("pu-c-g", {min: 0, max: LIMITS.current.max, zones: Z.current(), ticks: T.current(), decimals: 1});
    Ring.mount("pu-load", {label: "of rated", stroke: 8});
    Band.mount("pu-p-band", B.power());

    async function refresh() {
        const d = await getJSON("/api/all");
        status(d);

        const vs = Status.voltage(d.motor_voltage), cs = Status.current(d.motor_current), ls = Status.load(d.motor_power);

        Gauge.set("pu-v-g", d.motor_voltage);
        setText("pu-v-v", fmt(d.motor_voltage, 1));
        setPill("pu-v-pill", vs);
        setBadge("pu-v-badge", vs);

        Gauge.set("pu-c-g", d.motor_current);
        setText("pu-c-v", fmt(d.motor_current, 3));
        setPill("pu-c-pill", cs);
        setBadge("pu-c-badge", cs);

        const w = num(d.motor_power), pct = w === null ? null : w / LIMITS.power.max * 100;
        Ring.set("pu-load", pct, 100, TONE_COLOR[ls[0]], pct === null ? "--" : pct.toFixed(0) + "%");
        const p = fmtPower(d.motor_power);
        setText("pu-power", p.v);
        setText("pu-power-u", p.u);
        Band.set("pu-p-band", d.motor_power);
        setBadge("pu-p-badge", [ls[0], ls[0] === "ok" ? "Normal" : ls[1]]);
    }
    startLoop(refresh);
    </script>
    """
    return render_page("Pump Monitor", "pump", body, script)


# ----------------------------------------------------------------------
# STARTUP
# ----------------------------------------------------------------------

if __name__ == "__main__":
    init_db()
    
    polling_thread = threading.Thread(target=plc_polling_loop, daemon=True)
    polling_thread.start()

    print("")
    print("======================================================")
    print(" PLC HTTP Dashboard (with Idle Time Tracking)")
    print("======================================================")
    print(" Overview    : http://localhost:5000/")
    print(" Motor       : http://localhost:5000/motor")
    print(" Anemometer  : http://localhost:5000/anemometer")
    print(" Environment : http://localhost:5000/temperature")
    print(" Pump        : http://localhost:5000/pump")
    print("======================================================")
    print("")

    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    finally:
        save_runtime_state()
        if _relay_off_active:
            print("[RELAY] Shutting down during a relay cycle, switching relay back ON.")
            relay_on_with_retry()
        try:
            if plc_client:
                plc_client.close()
        except Exception:
            pass