#!/usr/bin/env python3
"""
inno_pilot_bridge.py — relay between pypilot and the Nano servo controller,
with a TCP listener on port 8555 for the inno-remote wireless handheld.

Architecture:
  pypilot (port 23322) <-> bridge <-> PTY pair <-> Nano USB serial
  inno-remote (Wi-Fi)  <-> bridge (TCP port 8555)

The bridge:
  - Transparently relays pypilot servo frames to/from the Nano.
  - Intercepts Nano button events and translates them to pypilot API calls.
  - Accepts a single TCP connection from the inno-remote at a time.
  - Pushes pypilot telemetry to the remote and accepts commands from it.
"""
import os
import select
import socket
import time
import serial
from pypilot.client import pypilotClient

# ---------------------------------------------------------------------------
# Inno-Pilot version (must match Nano firmware + remote firmware)
# ---------------------------------------------------------------------------
INNOPILOT_VERSION = "v0.2.0_B"

# ---------------------------------------------------------------------------
# Serial devices
# ---------------------------------------------------------------------------
NANO_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"
PILOT_PORT = "/dev/ttyINNOPILOT_BRIDGE"
BAUD       = 38400

# ---------------------------------------------------------------------------
# Nano serial frame codes  (6-byte wire: MAGIC1 MAGIC2 CODE LO HI CRC)
# ---------------------------------------------------------------------------

# Bridge -> Nano commands
AP_ENABLED_CODE            = 0xE1  # value: 0/1
PILOT_HEADING_CODE         = 0xE2  # imu.heading * 10 (uint16, 0-3600)
PILOT_COMMAND_CODE         = 0xE3  # ap.heading_command * 10 (uint16)
PILOT_RUDDER_CODE          = 0xE4  # rudder.angle * 10 (int16 as two's-complement uint16)
PILOT_RUDDER_PORT_LIM_CODE = 0xE5  # +rudder.range * 10 (int16)
PILOT_RUDDER_STBD_LIM_CODE = 0xE6  # -rudder.range * 10 (int16)
MANUAL_MODE_CODE           = 0xE7  # manual-from-remote active: 0/1
MANUAL_RUD_TARGET_CODE     = 0xE8  # manual rudder target: 0-1000 (tenths of a percent)

# Nano -> Bridge telemetry / events
BUTTON_EVENT_CODE = 0xE0
BTN_EVT_MINUS10   = 1
BTN_EVT_MINUS1    = 2
BTN_EVT_TOGGLE    = 3
BTN_EVT_PLUS10    = 4
BTN_EVT_PLUS1     = 5

# Bridge <-> Nano keepalive framing
BRIDGE_MAGIC1         = 0xA5
BRIDGE_MAGIC2         = 0x5A
BRIDGE_HELLO_CODE     = 0xF0
BRIDGE_HELLO_ACK_CODE = 0xF1
BRIDGE_HELLO_VALUE    = 0xBEEF

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
HELLO_PERIOD_S    = 1.0   # Nano keepalive — Nano requires 3 frames within 5 s
AP_STATE_PERIOD_S = 0.5   # minimum resend interval for AP-enabled state
TELEM_PERIOD_S    = 0.2   # 5 Hz telemetry to Nano and remote

# ---------------------------------------------------------------------------
# Remote TCP server
# ---------------------------------------------------------------------------
REMOTE_TCP_PORT        = 8555
REMOTE_TIMEOUT_S       = 6.0   # disconnect if no data received for this long
REMOTE_PING_PERIOD_S   = 2.0   # send PONG keepalive to remote every N seconds

# ---------------------------------------------------------------------------
# Probe / discovery constants
# ---------------------------------------------------------------------------
PROBE_INITIAL_DELAY_S = 1.0
PROBE_RETRIES         = 5
PROBE_RETRY_DELAY_S   = 3.0


# ===========================================================================
# Serial helpers
# ===========================================================================

def open_serial_no_reset(port: str, baud: int, timeout: float) -> serial.Serial:
    """Open serial port while keeping DTR/RTS low to reduce Arduino resets."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.dsrdtr = False
    ser.rtscts = False
    ser.open()
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    return ser


def crc8_msb(data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
    """CRC-8 MSB-first, poly 0x31, init 0xFF (matches Arduino crc8)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def build_frame(code: int, value_u16: int) -> bytes:
    lo = value_u16 & 0xFF
    hi = (value_u16 >> 8) & 0xFF
    body = bytes([code, lo, hi])
    return body + bytes([crc8_msb(body)])


def wrap_frame(frame: bytes) -> bytes:
    return bytes([BRIDGE_MAGIC1, BRIDGE_MAGIC2]) + frame


def send_nano_frame(nano: serial.Serial, code: int, value_u16: int) -> None:
    nano.write(wrap_frame(build_frame(code, value_u16)))


def extract_wrapped_frames(buf: bytearray) -> list[bytes]:
    """Pull all complete 6-byte frames out of buf (mutates buf in place)."""
    frames: list[bytes] = []
    while len(buf) >= 2:
        if buf[0] != BRIDGE_MAGIC1 or buf[1] != BRIDGE_MAGIC2:
            del buf[0]
            continue
        if len(buf) < 6:
            break
        frame = bytes(buf[2:6])
        del buf[:6]
        frames.append(frame)
    return frames


# ===========================================================================
# Nano discovery / probe
# ===========================================================================

def probe_nano_port(port: str, timeout_s: float = 0.5) -> bool:
    try:
        with open_serial_no_reset(port, BAUD, timeout=0.1) as probe:
            time.sleep(PROBE_INITIAL_DELAY_S)
            for attempt in range(PROBE_RETRIES):
                probe.reset_input_buffer()
                probe.write(wrap_frame(build_frame(BRIDGE_HELLO_CODE, BRIDGE_HELLO_VALUE)))
                buf = bytearray()
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    chunk = probe.read(64)
                    if chunk:
                        buf.extend(chunk)
                        for frame in extract_wrapped_frames(buf):
                            code = frame[0]
                            value = frame[1] | (frame[2] << 8)
                            if code == BRIDGE_HELLO_ACK_CODE and value == BRIDGE_HELLO_VALUE:
                                return True
                if attempt < (PROBE_RETRIES - 1):
                    time.sleep(PROBE_RETRY_DELAY_S)
            return False
    except Exception:
        return False


def find_nano_port() -> str:
    by_id = "/dev/serial/by-id"
    candidates: list[str] = []
    if os.path.isdir(by_id):
        for entry in sorted(os.listdir(by_id)):
            path = os.path.join(by_id, entry)
            if os.path.islink(path):
                if os.path.realpath(path) == "/dev/ttyINNOPILOT":
                    continue
                candidates.append(path)
    if NANO_PORT not in candidates and os.path.exists(NANO_PORT):
        candidates.append(NANO_PORT)

    if os.path.exists(NANO_PORT):
        return NANO_PORT

    for candidate in candidates:
        if probe_nano_port(candidate):
            return candidate

    raise RuntimeError(f"Unable to find Nano on serial ports: {candidates}")


# ===========================================================================
# Encoding helpers
# ===========================================================================

def clamp_heading(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


def enc_deg10_u16(deg: float) -> int:
    """0..360 deg -> 0..3600 (uint16)."""
    v = int(round(clamp_heading(float(deg)) * 10.0))
    return max(0, min(3600, v)) & 0xFFFF


def enc_deg10_i16(deg: float) -> int:
    """Signed degrees -> int16 tenths as two's-complement uint16."""
    v = int(round(float(deg) * 10.0))
    v = max(-32768, min(32767, v))
    return v & 0xFFFF


# ===========================================================================
# TCP server helpers
# ===========================================================================

def setup_tcp_server() -> socket.socket:
    """Create and return the non-blocking TCP listening socket."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", REMOTE_TCP_PORT))
    srv.listen(1)
    srv.setblocking(False)
    print(f"[bridge] TCP remote server listening on port {REMOTE_TCP_PORT}", flush=True)
    return srv


def remote_send(sock: socket.socket, line: str) -> bool:
    """Send a text line to the remote.  Returns False if the send failed."""
    try:
        sock.sendall((line + "\n").encode())
        return True
    except OSError:
        return False


def close_remote(sock: socket.socket) -> None:
    try:
        sock.close()
    except OSError:
        pass


def process_remote_line(
    line: str,
    client: pypilotClient,
    remote_sock: socket.socket,
    ap_enabled,
    heading_cmd,
) -> tuple:
    """
    Parse and execute one newline-stripped command from the inno-remote.

    Commands handled:
      PING            -> reply PONG
      ESTOP           -> ap.enabled = False  (highest priority)
      BTN TOGGLE      -> toggle ap.enabled
      BTN -10|-1|+1|+10 -> adjust ap.heading_command

    Returns updated (ap_enabled, heading_cmd).
    """
    parts = line.strip().split()
    if not parts:
        return ap_enabled, heading_cmd

    cmd = parts[0].upper()

    if cmd == "PING":
        remote_send(remote_sock, "PONG")

    elif cmd == "ESTOP":
        client.set("ap.enabled", False)
        ap_enabled = False
        print("[bridge] Remote -> ESTOP: ap.enabled=False", flush=True)

    elif cmd == "BTN":
        if len(parts) < 2:
            print("[bridge] Remote BTN: missing argument, ignored", flush=True)
            return ap_enabled, heading_cmd

        arg = parts[1].upper()

        if arg == "TOGGLE":
            target = True if ap_enabled is None else (not ap_enabled)
            client.set("ap.enabled", target)
            ap_enabled = target
            print(f"[bridge] Remote BTN TOGGLE -> ap.enabled={target}", flush=True)

        else:
            try:
                delta = float(arg)
            except ValueError:
                print(f"[bridge] Remote BTN: unrecognised arg '{arg}', ignored", flush=True)
                return ap_enabled, heading_cmd

            if heading_cmd is None:
                print("[bridge] Remote BTN: heading_cmd unknown, ignored", flush=True)
                return ap_enabled, heading_cmd

            new_cmd = clamp_heading(heading_cmd + delta)
            client.set("ap.heading_command", new_cmd)
            heading_cmd = new_cmd
            print(f"[bridge] Remote BTN {delta:+.0f} -> heading_command={new_cmd:.1f}", flush=True)

    else:
        # Unknown command — log and ignore; future steps will add MODE, RUD, DB
        print(f"[bridge] Remote: unknown command '{cmd}', ignored", flush=True)

    return ap_enabled, heading_cmd


# ===========================================================================
# Main
# ===========================================================================

def main() -> None:
    # ---- pypilot client ----
    client = pypilotClient()
    client.watch('ap.enabled', True)
    client.watch('imu.heading', True)
    client.watch('ap.heading_command', True)
    client.watch('rudder.angle', True)
    client.watch('rudder.range', 1.0)   # periodic so calibration changes propagate

    ap_enabled   = None
    heading_cmd  = None
    heading      = None   # imu.heading
    rudder_angle = None
    rudder_range = None

    # ---- serial ports ----
    nano_port = find_nano_port()
    nano  = open_serial_no_reset(nano_port, BAUD, timeout=0.01)
    pilot = open_serial_no_reset(PILOT_PORT, BAUD, timeout=0.01)

    nano_buf  = bytearray()
    pilot_buf = bytearray()

    # ---- telemetry / keepalive timestamps ----
    last_hello_ts    = 0.0
    last_ap_sent     = None
    last_ap_sent_ts  = 0.0
    last_telem_ts    = 0.0

    # ---- TCP remote state ----
    tcp_server  = setup_tcp_server()
    remote_sock = None          # the one connected client socket (or None)
    remote_buf  = bytearray()   # line-assembly buffer for incoming TCP data
    remote_last_rx_ts = 0.0     # monotonic time of last data received from remote

    print(f"[bridge] Inno-Pilot Bridge {INNOPILOT_VERSION}", flush=True)
    print("[bridge] Starting main loop", flush=True)

    while True:
        now = time.monotonic()

        # ================================================================
        # 1. Nano keepalive (BRIDGE_HELLO every 1 s)
        # ================================================================
        if (now - last_hello_ts) >= HELLO_PERIOD_S:
            send_nano_frame(nano, BRIDGE_HELLO_CODE, BRIDGE_HELLO_VALUE)
            last_hello_ts = now

        # ================================================================
        # 2. Receive pypilot values
        # ================================================================
        # Ensure pypilot socket can't block our loop (flush() can hang)
        try:
            if client.connection and hasattr(client.connection, 'socket'):
                client.connection.socket.settimeout(0.1)
        except Exception:
            pass
        try:
            msgs = client.receive(0)  # non-blocking
        except Exception:
            msgs = {}

        if "ap.enabled" in msgs:
            try:
                ap_enabled = bool(msgs["ap.enabled"])
            except Exception:
                pass

        if "ap.heading_command" in msgs:
            try:
                heading_cmd = float(msgs["ap.heading_command"])
            except Exception:
                pass

        if "imu.heading" in msgs:
            try:
                heading = float(msgs["imu.heading"])
            except Exception:
                pass

        if "rudder.angle" in msgs:
            try:
                rudder_angle = float(msgs["rudder.angle"])
            except Exception:
                pass

        if "rudder.range" in msgs:
            try:
                rudder_range = float(msgs["rudder.range"])
            except Exception:
                pass

        # ================================================================
        # 3. Push AP-enabled state to Nano (on change + periodic keepalive)
        # ================================================================
        if ap_enabled is not None:
            need_send = (
                last_ap_sent is None
                or ap_enabled != last_ap_sent
                or (now - last_ap_sent_ts) >= AP_STATE_PERIOD_S
            )
            if need_send:
                send_nano_frame(nano, AP_ENABLED_CODE, 1 if ap_enabled else 0)
                if last_ap_sent is None or ap_enabled != last_ap_sent:
                    print(f"[bridge] Pypilot -> Nano: ap.enabled={ap_enabled}", flush=True)
                last_ap_sent    = ap_enabled
                last_ap_sent_ts = now

        # ================================================================
        # 4. Push telemetry to Nano at 5 Hz
        # ================================================================
        if (now - last_telem_ts) >= TELEM_PERIOD_S:
            last_telem_ts = now

            # ---- Nano telemetry (binary frames) ----
            if heading is not None:
                send_nano_frame(nano, PILOT_HEADING_CODE, enc_deg10_u16(heading))

            if heading_cmd is not None:
                send_nano_frame(nano, PILOT_COMMAND_CODE, enc_deg10_u16(heading_cmd))

            if rudder_angle is not None:
                send_nano_frame(nano, PILOT_RUDDER_CODE, enc_deg10_i16(rudder_angle))

            if rudder_range is not None:
                port_lim =  abs(rudder_range)
                stbd_lim = -abs(rudder_range)
                send_nano_frame(nano, PILOT_RUDDER_PORT_LIM_CODE, enc_deg10_i16(port_lim))
                send_nano_frame(nano, PILOT_RUDDER_STBD_LIM_CODE, enc_deg10_i16(stbd_lim))

            # ---- Remote telemetry (text lines to TCP client) ----
            if remote_sock is not None:
                ok = True
                if ap_enabled is not None:
                    ok = ok and remote_send(remote_sock, f"AP {1 if ap_enabled else 0}")
                if heading is not None:
                    ok = ok and remote_send(remote_sock, f"HDG {heading:.1f}")
                if heading_cmd is not None:
                    ok = ok and remote_send(remote_sock, f"CMD {heading_cmd:.1f}")
                if rudder_angle is not None:
                    ok = ok and remote_send(remote_sock, f"RDR {rudder_angle:.1f}")
                if not ok:
                    print("[bridge] TCP remote: send failed during telemetry, disconnecting", flush=True)
                    close_remote(remote_sock)
                    remote_sock = None
                    remote_buf  = bytearray()

        # ================================================================
        # 5. Relay: pypilot -> Nano
        # ================================================================
        data_from_pilot = pilot.read(256)
        if data_from_pilot:
            pilot_buf.extend(data_from_pilot)
            while len(pilot_buf) >= 4:
                raw_frame = bytes(pilot_buf[:4])
                del pilot_buf[:4]
                nano.write(wrap_frame(raw_frame))

        # ================================================================
        # 6. Relay: Nano -> pypilot  (intercept button events)
        # ================================================================
        try:
            data_from_nano = nano.read(256)
        except serial.serialutil.SerialException as exc:
            print(f"[bridge] ERROR: Nano serial read failed: {exc}", flush=True)
            raise SystemExit(1) from exc

        if data_from_nano:
            nano_buf.extend(data_from_nano)

            for f in extract_wrapped_frames(nano_buf):
                code  = f[0]
                value = f[1] | (f[2] << 8)

                # Forward raw frame to pypilot unchanged
                pilot.write(f)

                # Intercept button events and translate to pypilot API calls
                if code == BUTTON_EVENT_CODE:
                    print(f"[bridge] Nano -> API: BUTTON_EVENT value={value}", flush=True)
                    try:
                        if value == BTN_EVT_TOGGLE:
                            target = True if ap_enabled is None else (not ap_enabled)
                            client.set("ap.enabled", target)
                            ap_enabled = target  # optimistic local cache

                        elif value in (BTN_EVT_MINUS10, BTN_EVT_MINUS1,
                                       BTN_EVT_PLUS10,  BTN_EVT_PLUS1):
                            if heading_cmd is None:
                                continue
                            delta = {
                                BTN_EVT_MINUS10: -10.0,
                                BTN_EVT_MINUS1:  -1.0,
                                BTN_EVT_PLUS1:    1.0,
                                BTN_EVT_PLUS10:  10.0,
                            }[value]
                            new_cmd = clamp_heading(heading_cmd + delta)
                            client.set("ap.heading_command", new_cmd)
                            heading_cmd = new_cmd  # optimistic cache

                    except Exception as e:
                        print(f"[bridge] Button event error: {e}", flush=True)

        # ================================================================
        # 7. TCP remote: accept new connections / handle existing client
        # ================================================================
        watch_socks = [tcp_server]
        if remote_sock is not None:
            watch_socks.append(remote_sock)

        readable, _, _ = select.select(watch_socks, [], [], 0)

        for s in readable:
            if s is tcp_server:
                # New inbound connection
                conn, addr = tcp_server.accept()
                if remote_sock is not None:
                    print(f"[bridge] TCP remote: rejected {addr} (already connected)", flush=True)
                    conn.close()
                else:
                    conn.setblocking(False)
                    remote_sock = conn
                    remote_buf  = bytearray()
                    remote_last_rx_ts = now
                    print(f"[bridge] TCP remote: connected from {addr}", flush=True)

            elif s is remote_sock:
                try:
                    data = remote_sock.recv(512)
                    if data:
                        remote_buf.extend(data)
                        remote_last_rx_ts = now
                        # Parse all complete newline-terminated lines
                        while b"\n" in remote_buf:
                            nl = remote_buf.index(b"\n")
                            line = remote_buf[:nl].decode("ascii", errors="replace")
                            del remote_buf[:nl + 1]
                            ap_enabled, heading_cmd = process_remote_line(
                                line, client, remote_sock, ap_enabled, heading_cmd
                            )
                    else:
                        # Peer closed connection cleanly
                        print("[bridge] TCP remote: disconnected (peer closed)", flush=True)
                        close_remote(remote_sock)
                        remote_sock = None
                        remote_buf  = bytearray()
                except OSError as exc:
                    print(f"[bridge] TCP remote: socket error ({exc}), disconnecting", flush=True)
                    close_remote(remote_sock)
                    remote_sock = None
                    remote_buf  = bytearray()

        # Check for remote connection timeout (no data within REMOTE_TIMEOUT_S)
        if remote_sock is not None and (now - remote_last_rx_ts) >= REMOTE_TIMEOUT_S:
            print("[bridge] TCP remote: timeout — no data received, disconnecting", flush=True)
            close_remote(remote_sock)
            remote_sock = None
            remote_buf  = bytearray()

        time.sleep(0.01)


if __name__ == "__main__":
    main()
