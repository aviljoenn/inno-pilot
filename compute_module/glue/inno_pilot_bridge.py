#!/usr/bin/env python3
"""
inno_pilot_bridge.py — relay between pypilot and the Nano servo controller,
with a TCP listener on port 8555 for the inno-remote wireless handheld.

Architecture:
  pypilot (port 23322) <-> bridge <-> PTY pair <-> Nano USB serial
  inno-remote (Wi-Fi)  <-> bridge (TCP port 8555)

Threading model:
  - pypilot_worker (daemon thread): owns all pypilotClient interaction.
    Calls client.receive() (which may block on pselect) without stalling
    the main loop.  Outbound set() calls are posted via set_queue.
  - Main thread: Nano serial, TCP remote, HELLO keepalive.
    Reads pypilot state from PypilotState under state_lock.

Mode state machine (BridgeState.mode):
  IDLE   — AP disengaged, no remote manual control
  AP     — autopilot engaged via pypilot
  MANUAL — remote has manual steering authority; AP locked out
  Transitions:
    BTN_TOGGLE / ESTOP       -> IDLE<->AP (rejected in MANUAL with WARN_AP_PRESSED)
    MODE MANUAL (TCP)        -> any -> MANUAL  (disengages AP)
    MODE AUTO   (TCP)        -> MANUAL -> IDLE
    TCP disconnect in MANUAL -> MANUAL -> IDLE + WARN_STEER_LOSS to Nano
"""
import logging
import os
import queue
import select
import signal
import socket
import threading
import time
import serial
from dataclasses import dataclass
from typing import Optional
from pypilot.client import pypilotClient

# ---------------------------------------------------------------------------
# Logging — default INFO; toggle to DEBUG at runtime with:
#   kill -USR1 $(pgrep -f inno_pilot_bridge)
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("bridge")
log_pp = logging.getLogger("pypilot")      # pypilot-worker messages


def _toggle_log_level(signum, frame):       # noqa: ARG001
    """SIGUSR1 handler: flip between INFO and DEBUG for all bridge loggers."""
    root = logging.getLogger()
    if root.level <= logging.DEBUG:
        root.setLevel(logging.INFO)
        log.info("Log level toggled -> INFO")
    else:
        root.setLevel(logging.DEBUG)
        log.info("Log level toggled -> DEBUG")


# SIGUSR1 is only available on Unix (Pi), harmless to skip on Windows dev
if hasattr(signal, "SIGUSR1"):
    signal.signal(signal.SIGUSR1, _toggle_log_level)

# ---------------------------------------------------------------------------
# Inno-Pilot version (must match Nano firmware + remote firmware)
# ---------------------------------------------------------------------------
INNOPILOT_VERSION   = "v0.2.0_B7"
INNOPILOT_BUILD_NUM = 7  # increment with each push during development

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
# NOTE: 0xE7 is RESET_CODE in pypilot servo protocol — do NOT use for bridge commands
MANUAL_RUD_TARGET_CODE     = 0xE8  # manual rudder target: 0-1000 (0=full port, 1000=full stbd)
MANUAL_MODE_CODE           = 0xE9  # remote manual mode active: 0/1  (was 0xE7, changed to avoid RESET_CODE conflict)
WARNING_CODE               = 0xEA  # warning type to display/beep on Nano

# Warning subtypes sent with WARNING_CODE
WARN_NONE        = 0
WARN_AP_PRESSED  = 1   # AP toggle rejected during MANUAL: 1-beep + 5s OLED flash
WARN_STEER_LOSS  = 2   # TCP dropped in MANUAL: continuous beep, STOP required

# Nano -> Bridge telemetry / events
BUTTON_EVENT_CODE = 0xE0
BTN_EVT_MINUS10   = 1
BTN_EVT_MINUS1    = 2
BTN_EVT_TOGGLE    = 3
BTN_EVT_PLUS10    = 4
BTN_EVT_PLUS1     = 5
BTN_EVT_STOP      = 6

# Nano -> Bridge new telemetry
BUZZER_STATE_CODE = 0xEB  # Nano->Bridge: buzzer on(1)/off(0)

# Nano -> Bridge pypilot result codes (parsed here, also forwarded to pypilot)
FLAGS_CODE        = 0x8F  # Nano flags word — bridge relays fault bits to remote

# Bridge <-> Nano keepalive framing
BRIDGE_MAGIC1         = 0xA5
BRIDGE_MAGIC2         = 0x5A
BRIDGE_HELLO_CODE     = 0xF0
BRIDGE_HELLO_ACK_CODE = 0xF1
BRIDGE_HELLO_VALUE    = 0xBEEF
BRIDGE_VERSION_CODE   = 0xF2  # Bridge -> Nano: build number (uint16)

# ---------------------------------------------------------------------------
# Bridge mode identifiers
# ---------------------------------------------------------------------------
MODE_IDLE   = "IDLE"
MODE_AP     = "AP"
MODE_MANUAL = "MANUAL"

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

# ---------------------------------------------------------------------------
# pypilot reconnect constants
# ---------------------------------------------------------------------------
PYPILOT_RECONNECT_DELAY_S = 3.0


# ===========================================================================
# Shared pypilot state (owned by pypilot_worker, read by main thread)
# ===========================================================================

@dataclass
class PypilotState:
    """Thread-safe container for pypilot values.  Always access under state_lock."""
    ap_enabled:   Optional[bool]  = None
    heading:      Optional[float] = None
    heading_cmd:  Optional[float] = None
    rudder_angle: Optional[float] = None
    rudder_range: Optional[float] = None
    connected:    bool            = False


# ===========================================================================
# Bridge-local state (main thread only, no locking needed)
# ===========================================================================

@dataclass
class BridgeState:
    """Mode state machine and bridge-local bookkeeping."""
    mode:              str  = MODE_IDLE  # IDLE / AP / MANUAL
    manual_rud_target: int  = 500        # 0-1000, sent as MANUAL_RUD_TARGET_CODE
    nano_buzzer_on:    bool = False      # last known Nano buzzer state


# ===========================================================================
# pypilot daemon thread
# ===========================================================================

def pypilot_worker(
    state: PypilotState,
    lock: threading.Lock,
    set_q: "queue.Queue[tuple]",
) -> None:
    """
    Daemon thread: owns all pypilotClient interaction.
    receive() may block on pselect; the 10ms sleep prevents spinning
    which would starve the main loop on the single-core Pi Zero.
    """
    while True:
        try:
            client = pypilotClient()
            client.watch('ap.enabled', True)
            client.watch('imu.heading', True)
            client.watch('ap.heading_command', True)
            client.watch('rudder.angle', True)
            client.watch('rudder.range', 1.0)
            log_pp.info("Connected to pypilot")

            while True:
                # Drain outbound set() queue before blocking on receive()
                while True:
                    try:
                        key, val = set_q.get_nowait()
                        client.set(key, val)
                    except queue.Empty:
                        break

                msgs = client.receive(0)
                # Sleep to yield CPU back to main thread on single-core Pi Zero
                time.sleep(0.01)

                with lock:
                    state.connected = True
                    if "ap.enabled" in msgs:
                        try:
                            state.ap_enabled = bool(msgs["ap.enabled"])
                        except Exception:
                            pass
                    if "ap.heading_command" in msgs:
                        try:
                            state.heading_cmd = float(msgs["ap.heading_command"])
                        except Exception:
                            pass
                    if "imu.heading" in msgs:
                        try:
                            state.heading = float(msgs["imu.heading"])
                        except Exception:
                            pass
                    if "rudder.angle" in msgs:
                        try:
                            state.rudder_angle = float(msgs["rudder.angle"])
                        except Exception:
                            pass
                    if "rudder.range" in msgs:
                        try:
                            state.rudder_range = float(msgs["rudder.range"])
                        except Exception:
                            pass

        except Exception as exc:
            log_pp.warning("Connection lost: %s — retrying in %ds", exc, PYPILOT_RECONNECT_DELAY_S)
            with lock:
                state.connected = False
            time.sleep(PYPILOT_RECONNECT_DELAY_S)


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
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", REMOTE_TCP_PORT))
    srv.listen(1)
    srv.setblocking(False)
    log.info("TCP remote server listening on port %d", REMOTE_TCP_PORT)
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


# ===========================================================================
# Remote disconnect handler
# ===========================================================================

def handle_remote_disconnect(
    nano: serial.Serial,
    bstate: BridgeState,
    set_q: "queue.Queue[tuple]",
    pstate: PypilotState,
    plock: threading.Lock,
) -> None:
    """
    Called whenever the remote TCP connection drops (timeout, peer-closed, error).
    If the remote was in MANUAL mode this is a steering-loss event: force the
    Nano out of manual mode, trigger the Nano steer-loss alarm, disengage AP.
    """
    if bstate.mode == MODE_MANUAL:
        send_nano_frame(nano, MANUAL_MODE_CODE, 0)          # exit manual on Nano
        send_nano_frame(nano, WARNING_CODE, WARN_STEER_LOSS) # trigger Nano alarm
        set_q.put(("ap.enabled", False))
        with plock:
            pstate.ap_enabled = False
        log.warning("Remote disconnected in MANUAL mode — STEER LOSS triggered")
        bstate.mode = MODE_IDLE


# ===========================================================================
# Remote command parser
# ===========================================================================

def process_remote_line(
    line: str,
    set_q: "queue.Queue[tuple]",
    remote_sock: socket.socket,
    pstate: PypilotState,
    plock: threading.Lock,
    bstate: BridgeState,
    nano: serial.Serial,
) -> None:
    """
    Parse and execute one newline-stripped command from the inno-remote.

    Commands handled:
      PING              -> reply PONG
      ESTOP             -> ap.enabled=False, exit MANUAL if active
      BTN TOGGLE        -> toggle AP (rejected with WARN_AP_PRESSED in MANUAL mode)
      BTN -10|-1|+1|+10 -> adjust ap.heading_command (ignored in MANUAL)
      MODE MANUAL       -> enter remote manual steering
      MODE AUTO         -> exit remote manual steering
      RUD <pct>         -> rudder target 0-100% (only in MANUAL mode)
    """
    parts = line.strip().split()
    if not parts:
        return

    cmd = parts[0].upper()

    if cmd == "PING":
        remote_send(remote_sock, "PONG")

    elif cmd == "ESTOP":
        set_q.put(("ap.enabled", False))
        with plock:
            pstate.ap_enabled = False
        if bstate.mode == MODE_MANUAL:
            send_nano_frame(nano, MANUAL_MODE_CODE, 0)
            bstate.mode = MODE_IDLE
        elif bstate.mode == MODE_AP:
            bstate.mode = MODE_IDLE
        log.info("Remote -> ESTOP: ap.enabled=False")

    elif cmd == "BTN":
        if len(parts) < 2:
            return
        arg = parts[1].upper()

        if arg == "TOGGLE":
            if bstate.mode == MODE_MANUAL:
                # AP engage rejected while remote has manual authority
                send_nano_frame(nano, WARNING_CODE, WARN_AP_PRESSED)
                remote_send(remote_sock, "WARN AP_PRESSED")
                log.warning("Remote BTN TOGGLE rejected in MANUAL mode")
            else:
                with plock:
                    current = pstate.ap_enabled
                target = True if current is None else (not current)
                set_q.put(("ap.enabled", target))
                with plock:
                    pstate.ap_enabled = target  # optimistic cache
                if target:
                    bstate.mode = MODE_AP
                else:
                    bstate.mode = MODE_IDLE
                log.info("Remote BTN TOGGLE -> ap.enabled=%s", target)

        else:
            if bstate.mode == MODE_MANUAL:
                # Heading adjustments have no meaning in manual mode
                return
            try:
                delta = float(arg)
            except ValueError:
                log.warning("Remote BTN: unrecognised arg '%s', ignored", arg)
                return
            with plock:
                current_cmd = pstate.heading_cmd
            if current_cmd is None:
                log.warning("Remote BTN: heading_cmd unknown, ignored")
                return
            new_cmd = clamp_heading(current_cmd + delta)
            set_q.put(("ap.heading_command", new_cmd))
            with plock:
                pstate.heading_cmd = new_cmd
            log.info("Remote BTN %+.0f -> heading_command=%.1f", delta, new_cmd)

    elif cmd == "MODE":
        if len(parts) < 2:
            return
        arg = parts[1].upper()

        if arg == "MANUAL":
            # Disable AP, grant manual authority to remote
            set_q.put(("ap.enabled", False))
            with plock:
                pstate.ap_enabled = False
            send_nano_frame(nano, MANUAL_MODE_CODE, 1)
            bstate.mode = MODE_MANUAL
            bstate.manual_rud_target = 500  # reset to centre
            log.info("Remote -> MODE MANUAL: manual steering active")

        elif arg == "AUTO":
            if bstate.mode == MODE_MANUAL:
                send_nano_frame(nano, MANUAL_MODE_CODE, 0)
                bstate.mode = MODE_IDLE
                log.info("Remote -> MODE AUTO: manual steering released")

    elif cmd == "RUD":
        # Rudder target percentage: 0.0 = full port, 100.0 = full stbd
        if bstate.mode != MODE_MANUAL:
            return  # only honoured in MANUAL mode
        if len(parts) < 2:
            return
        try:
            pct = float(parts[1])
        except ValueError:
            log.warning("Remote RUD: invalid value '%s', ignored", parts[1])
            return
        pct = max(0.0, min(100.0, pct))
        target_0_1000 = int(round(pct * 10.0))  # 0-1000
        bstate.manual_rud_target = target_0_1000
        send_nano_frame(nano, MANUAL_RUD_TARGET_CODE, target_0_1000)

    else:
        log.warning("Remote: unknown command '%s', ignored", cmd)


# ===========================================================================
# Main
# ===========================================================================

def main() -> None:
    # ---- shared pypilot state + inter-thread communication ----
    pstate     = PypilotState()
    plock      = threading.Lock()
    set_q: "queue.Queue[tuple]" = queue.Queue()

    # ---- start pypilot daemon thread ----
    t = threading.Thread(
        target=pypilot_worker,
        args=(pstate, plock, set_q),
        daemon=True,
        name="pypilot-worker",
    )
    t.start()
    log.info("pypilot worker thread started")

    # ---- serial ports ----
    nano_port = find_nano_port()
    nano  = open_serial_no_reset(nano_port, BAUD, timeout=0.01)
    pilot = open_serial_no_reset(PILOT_PORT, BAUD, timeout=0.01)

    nano_buf  = bytearray()
    pilot_buf = bytearray()

    # ---- bridge state (mode state machine) ----
    bstate = BridgeState()

    # ---- telemetry / keepalive timestamps ----
    last_hello_ts    = 0.0
    last_ap_sent     = None
    last_ap_sent_ts  = 0.0
    last_telem_ts    = 0.0

    # ---- TCP remote state ----
    tcp_server  = setup_tcp_server()
    remote_sock = None
    remote_buf  = bytearray()
    remote_last_rx_ts = 0.0

    # ---- snapshot of previous ap_enabled to detect external changes ----
    prev_ap_enabled = None

    log.info("Inno-Pilot Bridge %s", INNOPILOT_VERSION)
    log.info("Starting main loop")

    while True:
        now = time.monotonic()

        # ================================================================
        # 1. Nano keepalive (BRIDGE_HELLO every 1 s)
        # ================================================================
        if (now - last_hello_ts) >= HELLO_PERIOD_S:
            send_nano_frame(nano, BRIDGE_HELLO_CODE, BRIDGE_HELLO_VALUE)
            send_nano_frame(nano, BRIDGE_VERSION_CODE, INNOPILOT_BUILD_NUM)
            last_hello_ts = now
            log.debug("HELLO + VERSION sent to Nano")

        # ================================================================
        # 2. Snapshot shared pypilot state for this iteration
        # ================================================================
        with plock:
            ap_enabled   = pstate.ap_enabled
            heading_cmd  = pstate.heading_cmd
            heading      = pstate.heading
            rudder_angle = pstate.rudder_angle
            rudder_range = pstate.rudder_range

        # Track pypilot-external AP changes to keep mode in sync
        if prev_ap_enabled != ap_enabled:
            if ap_enabled and bstate.mode == MODE_IDLE:
                bstate.mode = MODE_AP
            elif not ap_enabled and bstate.mode == MODE_AP:
                bstate.mode = MODE_IDLE
            prev_ap_enabled = ap_enabled

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
                    log.info("Pypilot -> Nano: ap.enabled=%s", ap_enabled)
                last_ap_sent    = ap_enabled
                last_ap_sent_ts = now

        # ================================================================
        # 4. Push telemetry to Nano and Remote at 5 Hz
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

            log.debug("Telemetry -> Nano: hdg=%s cmd=%s rud=%s",
                      heading, heading_cmd, rudder_angle)

            # ---- Remote telemetry (text lines to TCP client) ----
            if remote_sock is not None:
                ok = True
                if ap_enabled is not None:
                    ok = ok and remote_send(remote_sock, f"AP {1 if ap_enabled else 0}")
                ok = ok and remote_send(remote_sock, f"MODE {bstate.mode}")
                if heading is not None:
                    ok = ok and remote_send(remote_sock, f"HDG {heading:.1f}")
                if heading_cmd is not None:
                    ok = ok and remote_send(remote_sock, f"CMD {heading_cmd:.1f}")
                if rudder_angle is not None:
                    ok = ok and remote_send(remote_sock, f"RDR {rudder_angle:.1f}")
                # Rudder percentage (useful in MANUAL mode for remote display)
                if rudder_angle is not None and rudder_range is not None and rudder_range > 0:
                    rudder_pct = (rudder_angle + rudder_range) / (2.0 * rudder_range) * 100.0
                    rudder_pct = max(0.0, min(100.0, rudder_pct))
                    ok = ok and remote_send(remote_sock, f"RDR_PCT {rudder_pct:.1f}")
                if not ok:
                    log.warning("TCP remote: send failed during telemetry, disconnecting")
                    handle_remote_disconnect(nano, bstate, set_q, pstate, plock)
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
        # 6. Relay: Nano -> pypilot  (intercept events; parse telemetry)
        # ================================================================
        try:
            data_from_nano = nano.read(256)
        except serial.serialutil.SerialException as exc:
            log.error("Nano serial read failed: %s", exc)
            raise SystemExit(1) from exc

        if data_from_nano:
            nano_buf.extend(data_from_nano)

            for f in extract_wrapped_frames(nano_buf):
                code  = f[0]
                value = f[1] | (f[2] << 8)

                # Forward raw frame to pypilot unchanged
                pilot.write(f)

                if code == BUTTON_EVENT_CODE:
                    log.debug("Nano -> API: BUTTON_EVENT value=%d", value)
                    try:
                        if value == BTN_EVT_TOGGLE:
                            if bstate.mode == MODE_MANUAL:
                                # AP toggle rejected while remote has manual authority
                                send_nano_frame(nano, WARNING_CODE, WARN_AP_PRESSED)
                                if remote_sock is not None:
                                    remote_send(remote_sock, "WARN AP_PRESSED")
                                log.warning("Nano BTN TOGGLE rejected in MANUAL mode")
                            else:
                                with plock:
                                    current = pstate.ap_enabled
                                target = True if current is None else (not current)
                                set_q.put(("ap.enabled", target))
                                with plock:
                                    pstate.ap_enabled = target
                                if target:
                                    bstate.mode = MODE_AP
                                else:
                                    bstate.mode = MODE_IDLE

                        elif value in (BTN_EVT_MINUS10, BTN_EVT_MINUS1,
                                       BTN_EVT_PLUS10,  BTN_EVT_PLUS1):
                            if bstate.mode == MODE_MANUAL:
                                pass  # heading adjustments ignored in manual mode
                            else:
                                with plock:
                                    current_cmd = pstate.heading_cmd
                                if current_cmd is None:
                                    continue
                                delta = {
                                    BTN_EVT_MINUS10: -10.0,
                                    BTN_EVT_MINUS1:  -1.0,
                                    BTN_EVT_PLUS1:    1.0,
                                    BTN_EVT_PLUS10:  10.0,
                                }[value]
                                new_cmd = clamp_heading(current_cmd + delta)
                                set_q.put(("ap.heading_command", new_cmd))
                                with plock:
                                    pstate.heading_cmd = new_cmd

                        elif value == BTN_EVT_STOP:
                            # STOP pressed on Nano: disengage AP and exit any manual mode
                            set_q.put(("ap.enabled", False))
                            with plock:
                                pstate.ap_enabled = False
                            if bstate.mode == MODE_MANUAL:
                                send_nano_frame(nano, MANUAL_MODE_CODE, 0)
                            bstate.mode = MODE_IDLE
                            log.info("Nano STOP -> ap.enabled=False, mode=IDLE")

                    except Exception as e:
                        log.error("Button event error: %s", e)

                elif code == FLAGS_CODE:
                    # Relay Nano fault flags to remote (remote can display fault indicators)
                    if remote_sock is not None:
                        remote_send(remote_sock, f"FLAGS {value}")

                elif code == BUZZER_STATE_CODE:
                    bstate.nano_buzzer_on = (value != 0)

        # ================================================================
        # 7. TCP remote: accept new connections / handle existing client
        # ================================================================
        watch_socks = [tcp_server]
        if remote_sock is not None:
            watch_socks.append(remote_sock)

        readable, _, _ = select.select(watch_socks, [], [], 0)

        for s in readable:
            if s is tcp_server:
                conn, addr = tcp_server.accept()
                if remote_sock is not None:
                    log.warning("TCP remote: rejected %s (already connected)", addr)
                    conn.close()
                else:
                    conn.setblocking(False)
                    remote_sock = conn
                    remote_buf  = bytearray()
                    remote_last_rx_ts = now
                    log.info("TCP remote: connected from %s", addr)

            elif s is remote_sock:
                try:
                    data = remote_sock.recv(512)
                    if data:
                        remote_buf.extend(data)
                        remote_last_rx_ts = now
                        while b"\n" in remote_buf:
                            nl = remote_buf.index(b"\n")
                            line = remote_buf[:nl].decode("ascii", errors="replace")
                            del remote_buf[:nl + 1]
                            process_remote_line(
                                line, set_q, remote_sock,
                                pstate, plock, bstate, nano
                            )
                    else:
                        log.info("TCP remote: disconnected (peer closed)")
                        handle_remote_disconnect(nano, bstate, set_q, pstate, plock)
                        close_remote(remote_sock)
                        remote_sock = None
                        remote_buf  = bytearray()
                except OSError as exc:
                    log.warning("TCP remote: socket error (%s), disconnecting", exc)
                    handle_remote_disconnect(nano, bstate, set_q, pstate, plock)
                    close_remote(remote_sock)
                    remote_sock = None
                    remote_buf  = bytearray()

        # Check for remote connection timeout
        if remote_sock is not None and (now - remote_last_rx_ts) >= REMOTE_TIMEOUT_S:
            log.warning("TCP remote: timeout — no data received, disconnecting")
            handle_remote_disconnect(nano, bstate, set_q, pstate, plock)
            close_remote(remote_sock)
            remote_sock = None
            remote_buf  = bytearray()

        time.sleep(0.01)


if __name__ == "__main__":
    main()
