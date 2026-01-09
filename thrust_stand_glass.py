import sys
import os
import time
import threading
import queue
import csv
import re
from datetime import datetime

import serial


# -----------------------
# HARD-CODED CONFIG
# -----------------------
SERIAL_PORT = "COM6"
BAUD_RATE   = 921600
LOG_DIR     = "logs"

# Your data format:
# t,throttle,load1,load2,load3,current,voltage,power
DATA_LINE_RE = re.compile(
    r"^\d+,\d+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+$"
)

CSV_HEADER = [
    "measurement_time_ms",
    "throttle_us",
    "load1",
    "load2",
    "load3",
    "current_A",
    "voltage_V",
    "power_W",
]


# -----------------------
# Recorder
# -----------------------
class Recorder:
    def __init__(self, log_dir):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.is_recording = False
        self.file = None
        self.writer = None
        self.path = None

        self.rows = 0
        self.last_data_line = None

    def start(self):
        if self.is_recording:
            print("[rec] already recording")
            return

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.path = os.path.join(self.log_dir, f"run_{ts}.csv")

        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(CSV_HEADER)
        self.file.flush()

        self.rows = 0
        self.last_data_line = None
        self.is_recording = True
        print(f"[rec] START -> {self.path}")

    def stop(self):
        if not self.is_recording:
            print("[rec] not recording")
            return

        self.file.flush()
        self.file.close()
        self.file = None
        self.writer = None
        self.is_recording = False
        print(f"[rec] STOP  -> {self.path} ({self.rows} rows)")

    def write(self, line):
        if not self.is_recording:
            return
        self.writer.writerow(line.split(","))
        self.rows += 1
        self.last_data_line = line

        # Flush periodically (much faster than flushing every line)
        if (self.rows % 25) == 0:
            self.file.flush()


# -----------------------
# Threads
# -----------------------
def serial_reader(ser, out_q, stop_evt):
    buf = bytearray()
    while not stop_evt.is_set():
        try:
            data = ser.read(512)
        except serial.SerialException as e:
            out_q.put(f"[py] serial error: {e}")
            break

        if not data:
            continue

        buf.extend(data)
        while b"\n" in buf:
            raw, _, buf = buf.partition(b"\n")
            line = raw.decode(errors="replace").strip()
            if line:
                out_q.put(line)

    out_q.put("[py] reader exited")


def console_input(cmd_q, stop_evt):
    while not stop_evt.is_set():
        try:
            cmd = input("> ").strip()
        except (KeyboardInterrupt, EOFError):
            cmd_q.put("quit")
            break
        if cmd:
            cmd_q.put(cmd)


# -----------------------
# Main
# -----------------------
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print("Error opening serial port:", e)
        sys.exit(1)

    print(f"[py] Connected to {SERIAL_PORT} @ {BAUD_RATE}")
    print("Commands:")
    print("  record start | record stop")
    print("  show                  (prints next 10 data lines)")
    print("  status                (prints last data line + row count)")
    print("  tare")
    print("  thrust <num>")
    print("  scale <s1> <s2> <s3>")
    print("  quit")

    stop_evt = threading.Event()
    line_q = queue.Queue()
    cmd_q = queue.Queue()

    threading.Thread(target=serial_reader, args=(ser, line_q, stop_evt), daemon=True).start()
    threading.Thread(target=console_input, args=(cmd_q, stop_evt), daemon=True).start()

    recorder = Recorder(LOG_DIR)

    # Show-next-N-lines control
    show_remaining = 0

    try:
        while not stop_evt.is_set():

            # Handle incoming serial lines
            while not line_q.empty():
                line = line_q.get()

                is_data = bool(DATA_LINE_RE.match(line))
                if is_data:
                    recorder.write(line)

                    # Print only the next N data lines when requested
                    if show_remaining > 0:
                        print(line)
                        show_remaining -= 1
                else:
                    # Always print non-data lines (OK / ERR / prompts)
                    print(line)

            # Handle user commands
            while not cmd_q.empty():
                cmd = cmd_q.get().strip()
                low = cmd.lower()

                if low in ("quit", "exit"):
                    stop_evt.set()
                    break

                elif low in ("record start", "start"):
                    recorder.start()

                elif low in ("record stop", "stop"):
                    recorder.stop()

                elif low == "show":
                    show_remaining = 10
                    print("[py] showing next 10 data lines")

                elif low == "status":
                    print(f"[py] recording={recorder.is_recording}, rows={recorder.rows}")
                    if recorder.last_data_line is not None:
                        print(f"[py] last: {recorder.last_data_line}")
                    else:
                        print("[py] last: (none yet)")

                else:
                    # Send verbatim to device
                    try:
                        ser.write((cmd + "\n").encode())
                    except Exception as e:
                        print(f"[py] write error: {e}")

            time.sleep(0.01)

    finally:
        if recorder.is_recording:
            recorder.stop()
        stop_evt.set()
        ser.close()
        print("[py] closed")


if __name__ == "__main__":
    main()
