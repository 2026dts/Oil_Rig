"""
PLC -> USR-W630 (Modbus over TCP) -> ThingsBoard bridge
--------------------------------------------------------
Reads sensor registers from the PLC via the USR-W630 serial-to-ethernet
converter, converts raw register values into real-world engineering units,
pushes them to ThingsBoard as telemetry, and listens for RPC commands from
the ThingsBoard dashboard to turn the motor ON/OFF (writes back to the PLC).

Install dependencies first:
    pip install pymodbus paho-mqtt --break-system-packages   (if on Linux/managed env)
    pip install pymodbus paho-mqtt                            (Windows venv, normal case)
"""

import time
import threading
import json
import paho.mqtt.client as mqtt
from pymodbus.client import ModbusTcpClient

# ======================================================================
# 1. CONNECTION SETTINGS -- fill these in
# ======================================================================

PLC_IP = "10.158.91.46"      # USR-W630 IP (Server mode, TCP, port 8899)
PLC_PORT = 8899
PLC_SLAVE_ID = 1

THINGSBOARD_HOST = "allcad-chennai.selfip.com"
THINGSBOARD_MQTT_PORT = 1883           # default ThingsBoard MQTT port
THINGSBOARD_ACCESS_TOKEN = "feQhNs3O5m16WyQE1zMB"

READ_INTERVAL_SEC = 2

# ======================================================================
# 2. REGISTER MAP + SCALING
#    Temperature, Humidity, Pressure, Gas Resistance are CONFIRMED from the
#    7Semi ENV-485 datasheet (registers 0x00-0x05, mirrored by the PLC into
#    holding registers 4096-4101).
#    Wind Speed/Direction and Motor Voltage/Current/Power (registers
#    4102-4106) are still UNCONFIRMED -- no datasheet for the anemometer or
#    current/voltage sensor yet. Update SIMPLE_REGISTERS below once you have
#    those datasheets.
# ======================================================================

BASE_ADDRESS = 4096
REGISTER_COUNT = 12

# Simple (single-register) values: (key, index, scale, unit)
SIMPLE_REGISTERS = [
    ("temperature",     0,  0.01,  "°C"),    # CONFIRMED: raw / 100
    ("humidity",        1,  0.01,  "%RH"),   # CONFIRMED: raw / 100
    ("wind_speed",       6,  0.1,   "m/s"),   # UNCONFIRMED -- verify with anemometer datasheet
    ("wind_direction",   7,  1,     ""),      # raw code (2=Right, 3=Left) -- no unit
    ("motor_voltage",    8,  1,     "V"),     # UNCONFIRMED -- verify with voltage sensor datasheet. Raw register is already in whole volts (matches ~10-12V supply, see test1.py output), NOT tenths -- previously scaled by 0.1 which gave 1.0-1.1V instead of ~10-11V.
    ("motor_current",    9,  0.4,   "A"),     # RESCALED: raw ~9-10 was being shown as 9-10A directly (too high for a 12V motor). scale=0.4 maps raw 9-10 -> ~3.6-4.0A, within the expected 2-5A range. APPROXIMATE -- verify with a clamp meter for precise calibration.
    ("motor_power_plc_raw", 10,  1,  "W"),     # PLC's own internal power calc -- kept only for reference/diagnostics, NOT used for the "motor_power" telemetry key anymore (see calculated version below)
    ("motor_control",   11,  1,     ""),      # raw 0/1 state, no scaling
]

# Combined 32-bit values: (key, high_index, low_index, scale, unit)
# CONFIRMED from 7Semi datasheet: value = (High << 16) | Low
COMBINED_REGISTERS = [
    ("pressure",  2, 3, 0.01, "hPa"),  # raw combined is in Pa -> /100 for hPa
    ("gas_resistance", 4, 5, 1,    "Ohm"),
]

# Wind direction register (index 7) reports rotation sense, not compass
# heading: 2 = clockwise ("Right"), 3 = anticlockwise ("Left").
WIND_DIRECTION_MAP = {
    2: "Right",
    3: "Left",
}

# Register address used to WRITE motor ON/OFF command back to the PLC.
# Confirm with your ladder logic whether this is a holding register or a coil.
MOTOR_CONTROL_WRITE_ADDRESS = BASE_ADDRESS + 11   # 4107

# Lookup of key -> unit, built from both register maps, used only for
# console display. Telemetry sent to ThingsBoard stays numeric-only.
UNITS = {key: unit for key, *_rest, unit in SIMPLE_REGISTERS}
UNITS.update({key: unit for key, *_rest, unit in COMBINED_REGISTERS})
UNITS["motor_power"] = "W"   # calculated field (V x I), not in SIMPLE_REGISTERS anymore

# Stored motor state for widget RPC responses.
current_motor_state = 0

# IAQ baseline tracking -- gas resistance in "clean air" is the highest
# stable value seen. IAQ score is derived from how far the current
# reading has dropped below that baseline. This is an approximation --
# not Bosch's proprietary BSEC algorithm, which additionally uses
# temperature/humidity compensation curves that aren't public.
gas_baseline = None
IAQ_BASELINE_LEARN_SAMPLES = 50   # number of readings to establish baseline

# --- Stale-data detection (auto reconnect) ---
# If the USR-W630 <-> PLC link desyncs, the PLC/USR sometimes keeps
# returning the exact same register snapshot forever instead of a fresh
# read. We detect that by comparing the raw register list across
# consecutive reads: if it hasn't changed at all for STALE_THRESHOLD
# consecutive reads, we force-close and reopen the Modbus TCP connection
# to resync, without touching the PLC itself.
_last_raw_regs = None
_stale_count = 0
STALE_THRESHOLD = 10          # consecutive identical reads before reconnecting
connection_needs_reset = False


# ======================================================================
# 3. MODBUS READ + SCALE
# ======================================================================

def read_plc_data(client):
    rr = client.read_holding_registers(
        address=BASE_ADDRESS,
        count=REGISTER_COUNT,
        device_id=PLC_SLAVE_ID
    )

    if rr is None or rr.isError():
        print("Modbus Read Error:", rr)
        return None

    regs = rr.registers

    # --- Stale-data check (priority: wind_speed register only) ---
    # Wind speed (index 6) is the field that most reliably changes on a
    # live link -- so we watch THAT register specifically for staleness,
    # rather than the whole 12-register block. If it repeats the same
    # value for STALE_THRESHOLD consecutive reads, flag a reconnect.
    global _last_raw_regs, _stale_count, connection_needs_reset
    wind_speed_raw = regs[6]
    if _last_raw_regs is not None and wind_speed_raw == _last_raw_regs:
        _stale_count += 1
    else:
        _stale_count = 0
    _last_raw_regs = wind_speed_raw

    if _stale_count >= STALE_THRESHOLD:
        print(f"[WARN] Same register data repeated {STALE_THRESHOLD} times in a row -- "
              f"flagging connection for reconnect.")
        connection_needs_reset = True
        _stale_count = 0
    # --- end stale-data check ---

    data = {}

    # Simple single-register values
    for key, idx, scale, unit in SIMPLE_REGISTERS:
        raw = regs[idx]
        # handle negative values represented as unsigned 16-bit (two's complement)
        if raw > 32767:
            raw -= 65536
        value = round(raw * scale, 3)
        data[key] = value

    # Motor power, computed from our own calibrated voltage/current
    # (P = V x I) instead of trusting the PLC's separate raw power
    # register, since that register was scaled against the PLC's own
    # raw ADC values and no longer matches after we recalibrated
    # motor_voltage/motor_current above.
    data["motor_power"] = round(data["motor_voltage"] * data["motor_current"], 2)

    # Combined 32-bit values (pressure, gas resistance) -- per 7Semi datasheet:
    # Value = (HighWord << 16) | LowWord
    for key, hi_idx, lo_idx, scale, unit in COMBINED_REGISTERS:
        high = regs[hi_idx]
        low = regs[lo_idx]
        combined = (high << 16) | low
        value = round(combined * scale, 3)
        data[key] = value

    # Wind direction as a readable rotation-sense label (2=Right, 3=Left)
    # Read directly from the raw register (index 7) -- not from the
    # already-processed `data` dict -- to avoid any scale/rounding
    # side effects changing the raw 2/3 code.
    raw_direction = regs[7]
    data["wind_direction_text"] = WIND_DIRECTION_MAP.get(raw_direction, "Unknown")

    # Derived IAQ (air quality) index from gas resistance
    iaq_score, iaq_label = compute_iaq(data["gas_resistance"])
    data["iaq_index"] = iaq_score
    data["iaq_label"] = iaq_label

    return data


def compute_iaq(gas_resistance_ohm):
    """
    Approximate IAQ (0-500 scale, lower = better air quality) from gas
    resistance. Maintains a rolling baseline of the highest resistance
    seen ("cleanest air" reference) and scores the current reading by
    how far it has dropped below that baseline.

    NOTE: this is a simplified heuristic, not Bosch's proprietary BSEC
    algorithm -- treat it as a relative trend indicator, not a
    certified air-quality measurement.
    """
    global gas_baseline

    if gas_resistance_ohm <= 0:
        return 0, "Unknown"

    # Establish / slowly adapt the baseline upward if we see cleaner air
    if gas_baseline is None or gas_resistance_ohm > gas_baseline:
        gas_baseline = gas_resistance_ohm

    # % drop from baseline -- higher drop = more VOCs = worse air
    drop_pct = max(0.0, (gas_baseline - gas_resistance_ohm) / gas_baseline * 100)

    # Map 0-100% drop onto a 0-500 IAQ scale
    iaq_score = round(min(drop_pct, 100) * 5, 1)

    if iaq_score <= 50:
        label = "Excellent"
    elif iaq_score <= 100:
        label = "Good"
    elif iaq_score <= 150:
        label = "Moderate"
    elif iaq_score <= 200:
        label = "Poor"
    else:
        label = "Very Poor"

    return iaq_score, label


def write_motor_control(client, state: int):
    """state: 1 = ON, 0 = OFF"""
    result = client.write_register(
        address=MOTOR_CONTROL_WRITE_ADDRESS,
        value=state,
        device_id=PLC_SLAVE_ID
    )
    if result.isError():
        print("Motor write failed:", result)
        return False
    print(f"Motor command sent: {'ON' if state else 'OFF'}")
    return True


# ======================================================================
# 4. THINGSBOARD MQTT CLIENT (telemetry + RPC)
# ======================================================================

plc_client = ModbusTcpClient(host=PLC_IP, port=PLC_PORT, framer='ascii', timeout=3)


def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to ThingsBoard, rc =", rc)
    client.subscribe("v1/devices/me/rpc/request/+")


def on_message(client, userdata, msg):
    # RPC request arrives here when the dashboard button is pressed
    try:
        request_id = msg.topic.split("/")[-1]
        payload = json.loads(msg.payload.decode())
        method = payload.get("method")
        params = payload.get("params")

        print(f"RPC received -> method: {method}, params: {params}")

        global current_motor_state

        if method == "getValue":
            client.publish(
                f"v1/devices/me/rpc/response/{request_id}",
                json.dumps(bool(current_motor_state))
            )

        elif method == "setMotorState":
            state = 1 if params else 0
            success = write_motor_control(plc_client, state)
            if success:
                current_motor_state = state
            response = {"success": success, "state": state}
            client.publish(f"v1/devices/me/rpc/response/{request_id}", json.dumps(response))

    except Exception as e:
        print("RPC handling error:", e)


tb_client = mqtt.Client()
tb_client.username_pw_set(THINGSBOARD_ACCESS_TOKEN)
tb_client.on_connect = on_connect
tb_client.on_message = on_message


def mqtt_loop():
    tb_client.connect(THINGSBOARD_HOST, THINGSBOARD_MQTT_PORT, keepalive=60)
    tb_client.loop_forever()


# ======================================================================
# 5. MAIN LOOP -- read PLC, push telemetry
# ======================================================================

def main():
    if not plc_client.connect():
        print("PLC (USR-W630) Connection Failed")
        return

    print("Connected to PLC via USR-W630")

    # Start MQTT (ThingsBoard) in a background thread so RPC listening
    # doesn't block the sensor read loop
    mqtt_thread = threading.Thread(target=mqtt_loop, daemon=True)
    mqtt_thread.start()

    try:
        while True:
            data = read_plc_data(plc_client)

            # --- Auto-reconnect if staleness was flagged inside read_plc_data ---
            global connection_needs_reset
            if connection_needs_reset:
                print("Reconnecting to PLC (USR-W630) to clear stale data...")
                try:
                    plc_client.close()
                except Exception as e:
                    print("Error while closing connection:", e)
                time.sleep(1)
                if plc_client.connect():
                    print("Reconnected to PLC successfully.")
                else:
                    print("Reconnect attempt failed -- will retry on next stale detection.")
                connection_needs_reset = False
            # --- end auto-reconnect ---

            if data:
                print("---------------------------")
                for k, v in data.items():
                    unit = UNITS.get(k, "")
                    print(f"{k:15s}: {v} {unit}".rstrip())

                # Push to ThingsBoard as telemetry (numeric values only, no units)
                tb_client.publish("v1/devices/me/telemetry", json.dumps(data))

            time.sleep(READ_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        plc_client.close()
        tb_client.disconnect()


if __name__ == "__main__":
    main()