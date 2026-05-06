#!/usr/bin/env python3
"""
ads1115_test.py — Read ADS1115 test output from Nano over serial.

The Nano sketch (ads1115_test.ino) emits lines like:
    RAW:13200 MV:2475 N:42

Opens /dev/ttyUSB0 at 38400 with HUPCL suppressed (same pattern as
inno_pilot_bridge.py) so the Nano does NOT reset when this script exits.

Usage:
    python3 /path/to/ads1115_test.py [--port /dev/ttyUSB0]

Stop inno-pilot-bridge / inno-pilot-socat before running so the port is free:
    sudo systemctl stop inno-pilot-bridge inno-pilot-socat
"""

import sys
import time
import termios
import argparse
import serial  # pyserial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 38400


def open_serial_no_reset(port: str, baud: int) -> serial.Serial:
    """Open serial port without triggering Nano reset via DTR/HUPCL.

    HUPCL (Hang Up on Close) drops DTR on close → Nano reset.
    We clear it via termios immediately after opening, same technique
    used by inno_pilot_bridge.py.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 2.0
    ser.dtr = False   # do not assert DTR before open
    ser.rts = False
    ser.open()

    # Clear HUPCL in the kernel's termios flags.
    fd = ser.fileno()
    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~termios.HUPCL          # cflag: clear HUPCL
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    return ser


def parse_line(text: str) -> dict | None:
    """Parse 'RAW:X MV:Y N:Z' into a dict, or None if unrecognised."""
    parts = {}
    for token in text.split():
        if ":" in token:
            key, _, val = token.partition(":")
            parts[key] = val
    if "RAW" in parts and "MV" in parts:
        return parts
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Read ADS1115 test data from Nano")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port (default: /dev/ttyUSB0)")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {BAUD} baud (HUPCL suppressed)...")
    try:
        ser = open_serial_no_reset(args.port, BAUD)
    except serial.SerialException as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    print("Waiting for Nano output (Ctrl-C to stop)...\n")
    print(f"{'Time':8s}  {'RAW':>7s}  {'mV':>6s}  {'N':>6s}  Raw line")
    print("-" * 50)

    try:
        while True:
            line = ser.readline()
            if not line:
                print("[timeout — no data from Nano]")
                continue
            text = line.decode("ascii", errors="replace").rstrip()
            ts = time.strftime("%H:%M:%S")

            parsed = parse_line(text)
            if parsed:
                raw = parsed.get("RAW", "?")
                mv  = parsed.get("MV",  "?")
                n   = parsed.get("N",   "?")
                print(f"{ts}  {raw:>7}  {mv:>6}  {n:>6}  {text}")
            else:
                # Pass-through for startup messages (FOUND/MISS probes etc.)
                print(f"{ts}  {'':>7}  {'':>6}  {'':>6}  {text}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
