"""
Diagnostic script: logs RAW PLC register values with timestamps to help
determine whether the "wind_speed stuck / 0" issue originates:
  (a) upstream, on the PLC/sensor side (anemometer <-> Delta PLC link), or
  (b) on the TCP/RS232 side (USR-W630 <-> Python link)

This is read-only: it never writes to the PLC and never publishes to
ThingsBoard, so it's safe to run alongside oil_rig.py or on its own.

How to use:
  1. Run this for at least 15-30 minutes, ideally during a period when
     you know the wind is actually blowing -- otherwise a real 0 reading
     looks identical to a stuck 0 reading.
  2. Watch the console. Every read prints ALL 12 raw registers, not just
     wind_speed, so you can see whether wind_speed (index 6) is
     misbehaving in isolation while the other registers (temperature,
     humidity, pressure, gas resistance) keep changing normally.
  3. Press Ctrl+C to stop. A summary prints showing how often EACH
     register's raw value changed between consecutive successful reads.

How to read the summary:
  - If wind_speed changes far less often than temperature/humidity,
    AND the modbus read itself never errors out (error_count stays 0),
    that points to the PLC not refreshing that specific register --
    i.e. an upstream problem between the anemometer and the PLC, not
    your Python/TCP code.
  - If error_count is high, or ALL registers freeze together at the
    same time, that instead points to the TCP/RS232 link itself.
"""

import time
from datetime import datetime
from pymodbus.client import ModbusTcpClient

PLC_IP = "10.158.91.46"
PLC_PORT = 8899
PLC_SLAVE_ID = 1
BASE_ADDRESS = 4096
REGISTER_COUNT = 12
POLL_INTERVAL_SEC = 2
LOG_FILE = "wind_speed_diagnostic_log.csv"

LABELS = [
    "temperature", "humidity", "pressure_hi", "pressure_lo",
    "gas_hi", "gas_lo", "wind_speed", "wind_direction",
    "motor_voltage", "motor_current", "motor_power_raw", "motor_control",
]

client = ModbusTcpClient(host=PLC_IP, port=PLC_PORT, framer='ascii', timeout=3)


def main():
    if not client.connect():
        print("Could not connect to PLC/USR-W630")
        return

    print(f"Connected. Logging raw registers every {POLL_INTERVAL_SEC}s to {LOG_FILE}")
    print("Press Ctrl+C to stop and see the summary.\n")

    last_values = None
    change_counts = {label: 0 for label in LABELS}
    total_reads = 0
    error_count = 0

    with open(LOG_FILE, "w") as f:
        f.write("timestamp," + ",".join(LABELS) + "\n")

        try:
            while True:
                rr = client.read_holding_registers(
                    address=BASE_ADDRESS, count=REGISTER_COUNT, device_id=PLC_SLAVE_ID
                )
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                if rr is None or rr.isError():
                    error_count += 1
                    print(f"[{ts}] MODBUS READ ERROR: {rr}")
                else:
                    regs = rr.registers
                    total_reads += 1
                    f.write(ts + "," + ",".join(str(r) for r in regs) + "\n")
                    f.flush()

                    print(f"[{ts}] " + " ".join(f"{l}={v}" for l, v in zip(LABELS, regs)))

                    if last_values is not None:
                        for label, prev, cur in zip(LABELS, last_values, regs):
                            if cur != prev:
                                change_counts[label] += 1
                    last_values = regs

                time.sleep(POLL_INTERVAL_SEC)

        except KeyboardInterrupt:
            print("\n--- SUMMARY ---")
            print(f"Total successful reads: {total_reads}   Modbus errors: {error_count}")
            if total_reads > 1:
                for label in LABELS:
                    pct = 100 * change_counts[label] / (total_reads - 1)
                    print(f"{label:18s}: changed on {change_counts[label]}/{total_reads - 1} reads ({pct:.1f}%)")
            print(f"\nFull raw log saved to {LOG_FILE} for further analysis.")
        finally:
            client.close()


if __name__ == "__main__":
    main()