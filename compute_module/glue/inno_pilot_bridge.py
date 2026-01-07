#!/usr/bin/env python3
import time
import serial
from pypilot.client import pypilotClient  # watch/receive/set API

# Serial devices
NANO_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"  # real Nano USB serial
PILOT_PORT = "/dev/ttyINNOPILOT_BRIDGE"                               # PTY side that talks to pypilot
BAUD       = 38400

# Nano -> Bridge button event protocol
BUTTON_EVENT_CODE = 0xE0
BTN_EVT_MINUS10 = 1
BTN_EVT_MINUS1  = 2
BTN_EVT_TOGGLE  = 3
BTN_EVT_PLUS10  = 4
BTN_EVT_PLUS1   = 5

# Bridge -> Nano state protocol
AP_ENABLED_CODE = 0xE1  # value: 0/1

# How often to refresh AP state to Nano (keepalive + on change)
AP_STATE_PERIOD_S = 0.5


def crc8_msb(data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
    """CRC-8 MSB-first, poly 0x31, init 0xFF (matches Arduino crc8 in pypilot motor code)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def build_frame(code: int, value: int) -> bytes:
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    body = bytes([code, lo, hi])
    return body + bytes([crc8_msb(body)])


def clamp_heading(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


def main():
    # Use localhost explicitly (fewer zeroconf surprises)
    client = pypilotClient('127.0.0.1')

    # Watch values we need. (poll happens inside receive())
    client.watch('ap.enabled', True)
    client.watch('ap.heading_command', True)

    ap_enabled = None      # bool
    heading_cmd = None     # float

    # Open serial ports
    nano = serial.Serial(NANO_PORT, BAUD, timeout=0.01)
    pilot = serial.Serial(PILOT_PORT, BAUD, timeout=0.01)

    # Buffer for Nano->pilot framing
    rxbuf = bytearray()

    last_ap_sent = None
    last_ap_sent_ts = 0.0

    print("Inno-Pilot: bridge started")
    print(f"Inno-Pilot: NANO_PORT={NANO_PORT}")
    print(f"Inno-Pilot: PILOT_PORT={PILOT_PORT}")

    while True:
        now = time.monotonic()

        # ---- Pump pypilot client + update cached values ----
        try:
            msgs = client.receive(0)  # non-blocking poll
        except Exception as e:
            msgs = {}
            print("Inno-Pilot: pypilot receive error:", e)

        if 'ap.enabled' in msgs:
            try:
                ap_enabled = bool(msgs['ap.enabled'])
            except Exception:
                pass

        if 'ap.heading_command' in msgs:
            try:
                heading_cmd = float(msgs['ap.heading_command'])
            except Exception:
                pass

        # ---- Push AP enabled state down to Nano (keepalive + on change) ----
        # If we don't know yet, default to OFF (0). This still acts as a "Nano online" keepalive.
        ap_to_send = 0 if ap_enabled is None else (1 if ap_enabled else 0)

        need_send = (
            (last_ap_sent is None) or
            (ap_to_send != last_ap_sent) or
            ((now - last_ap_sent_ts) >= AP_STATE_PERIOD_S)
        )
        if need_send:
            try:
                nano.write(build_frame(AP_ENABLED_CODE, ap_to_send))
                last_ap_sent = ap_to_send
                last_ap_sent_ts = now
                # comment out if too chatty:
                # print(f"Inno-Pilot: sent AP state to Nano: {ap_to_send}")
            except Exception as e:
                print("Inno-Pilot: failed sending AP state to Nano:", e)

        # ---- Data from pypilot -> Nano ----
        try:
            data_from_pilot = pilot.read(256)
            if data_from_pilot:
                nano.write(data_from_pilot)
        except Exception as e:
            print("Inno-Pilot: pilot->nano read/write error:", e)

        # ---- Data from Nano -> pypilot ----
        try:
            data_from_nano = nano.read(256)
            if data_from_nano:
                rxbuf.extend(data_from_nano)

                # Frames are fixed 4 bytes
                while len(rxbuf) >= 4:
                    f = bytes(rxbuf[:4])
                    del rxbuf[:4]

                    code = f[0]
                    value = f[1] | (f[2] << 8)

                    # Always forward raw frame to pypilot unchanged
                    pilot.write(f)

                    # Only treat as a button event if CRC is valid
                    if code == BUTTON_EVENT_CODE:
                        body = f[:3]
                        crc  = f[3]
                        if crc8_msb(body) != crc:
                            continue

                        print(f"Inno-Pilot: button event from Nano: {value}")

                        # Handle button events via pypilot API
                        try:
                            if value == BTN_EVT_TOGGLE:
                                # Toggle ap.enabled; if unknown, enable.
                                target = True if ap_enabled is None else (not ap_enabled)
                                client.set('ap.enabled', target)
                                print(f"Inno-Pilot: set ap.enabled -> {target}")

                            elif value in (BTN_EVT_MINUS10, BTN_EVT_MINUS1, BTN_EVT_PLUS10, BTN_EVT_PLUS1):
                                if heading_cmd is None:
                                    continue

                                delta = 0.0
                                if value == BTN_EVT_MINUS10: delta = -10.0
                                elif value == BTN_EVT_MINUS1:  delta = -1.0
                                elif value == BTN_EVT_PLUS1:   delta =  1.0
                                elif value == BTN_EVT_PLUS10:  delta = 10.0

                                new_cmd = clamp_heading(heading_cmd + delta)
                                client.set('ap.heading_command', new_cmd)
                                print(f"Inno-Pilot: set ap.heading_command -> {new_cmd:.1f}")

                        except Exception as e:
                            print("Inno-Pilot: button event handling error:", e)

        except Exception as e:
            print("Inno-Pilot: nano->pilot read/write error:", e)

        time.sleep(0.01)


if __name__ == "__main__":
    main()
