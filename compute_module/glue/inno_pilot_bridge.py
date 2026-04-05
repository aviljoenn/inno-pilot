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
import copy
import json
import logging
import logging.handlers
import os
import queue
import select
import signal
import socket
import threading
import time
import serial
import termios
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
INNOPILOT_VERSION   = "v1.2.0_B29"
INNOPILOT_BUILD_NUM = 29  # increment with each push during development

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
PILOT_HEADING_CODE         = 0xE2  # imu.heading * 10 (uint16, 0-3600)
PILOT_COMMAND_CODE         = 0xE3  # ap.heading_command * 10 (uint16)
PILOT_RUDDER_CODE          = 0xE4  # rudder.angle * 10 (int16 as two's-complement uint16)
PILOT_RUDDER_PORT_LIM_CODE = 0xE5  # +rudder.range * 10 (int16)
PILOT_RUDDER_STBD_LIM_CODE = 0xE6  # -rudder.range * 10 (int16)
# NOTE: 0xE7 is RESET_CODE in pypilot servo protocol — do NOT use for bridge commands

# pypilot servo protocol codes (relayed verbatim to Nano via PILOT_PORT)
PYPILOT_COMMAND_CODE   = 0xC7  # motor position command — implies AP engaged
PYPILOT_DISENGAGE_CODE = 0x68  # motor disengage — implies AP off
MANUAL_RUD_TARGET_CODE     = 0xE8  # manual rudder target: 0-1000 (0=full port, 1000=full stbd)
MANUAL_MODE_CODE           = 0xE9  # remote manual mode active: 0/1  (was 0xE7, changed to avoid RESET_CODE conflict)
WARNING_CODE               = 0xEA  # warning type to display/beep on Nano

# Warning subtypes sent with WARNING_CODE
WARN_NONE        = 0
WARN_AP_PRESSED  = 1   # AP toggle rejected during MANUAL: 1-beep + 5s OLED flash
WARN_STEER_LOSS  = 2   # TCP dropped in MANUAL: continuous beep, STOP required

# Nano -> Bridge telemetry / events
PIN_STATE_CODE    = 0xE1  # H-bridge pin state change: bits [2]=D9/EN, [1]=D3/LPWM, [0]=D2/RPWM
BUTTON_EVENT_CODE = 0xE0

# B26: motor activation reason diagnostic — sent once when D9 goes LOW→HIGH
MOTOR_REASON_CODE = 0xEE
# reason field (bits [3:0] of value):
_MRSN = {1: "manual_phys(btn)", 2: "delta_jog(stale_cmd)", 3: "ap_active",
         4: "rm_driving", 5: "rm_braking"}
BTN_EVT_MINUS10   = 1
BTN_EVT_MINUS1    = 2
BTN_EVT_TOGGLE    = 3
BTN_EVT_PLUS10    = 4
BTN_EVT_PLUS1     = 5
BTN_EVT_STOP      = 6

# Nano -> Bridge new telemetry
BUZZER_STATE_CODE = 0xEB  # Nano->Bridge: buzzer on(1)/off(0)
COMMS_DIAG_CODE       = 0xEC  # Nano->Bridge: comms diagnostics (lo=err_window_sum, hi=crit_consec_s)
COMMS_ERR_DETAIL_CODE = 0xED  # Nano->Bridge: error detail (lo=corrupt code, hi=rx_crc)

# Bridge -> Nano: feature enable bitmask (0xEF)
FEATURES_CODE             = 0xEF  # Bridge->Nano: uint8 feature bitmask (sent on startup + settings change)
FEATURE_LIMIT_SWITCHES    = 0x01  # use D7/D8 NC limit switches
FEATURE_TEMP_SENSOR       = 0x02  # DS18B20 temperature fault detection
FEATURE_PI_VOLTAGE        = 0x04  # A3 Pi supply voltage fault detection
FEATURE_BATTERY_VOLTAGE   = 0x08  # A0 main Vin over/under-voltage fault detection
FEATURE_CURRENT_SENSOR    = 0x10  # A1 motor current telemetry
FEATURE_ON_BOARD_BUTTONS  = 0x20  # physical button ladder on A6 is wired

# Nano -> Bridge pypilot result codes (parsed here, also forwarded to pypilot)
FLAGS_CODE        = 0x8F  # Nano flags word — bridge relays fault bits to remote

# Flag bit masks for bridge-side parsing of FLAGS_CODE value
FLAG_COMMS_WARN   = 0x1000  # error rate elevated (WARN level)
FLAG_COMMS_CRIT   = 0x2000  # error rate unsafe, AP auto-disengaged (CRITICAL level)

# RCT (Remote Control Test) frame codes — ignored by production motor_simple.ino
RCT_SETTING_CODE        = 0xEE  # both dirs: HI=setting_index (0-6), LO=uint8 value
RCT_TARGET_CODE         = 0xF3  # Pi -> Nano: target pct×10 (0-1000; 500 = midships)
RCT_RESULT_STOP_CODE    = 0xF4  # Nano -> Pi: first_stop_adc after full-power coast
RCT_RESULT_PULSES_CODE  = 0xF5  # Nano -> Pi: fine pulse count used to reach deadband
RCT_RESULT_FINAL_CODE   = 0xF6  # Nano -> Pi: final_adc inside (or nearest to) deadband
RCT_RDR_PCT_CODE        = 0xF7  # Nano -> Pi: ratify live rudder position pct×10 (0-1000)
RCT_HZ_CODE             = 0xF8  # Nano -> Pi: ratify loop Hz (sent once per second)

# RCT settings — stored on Pi as JSON, pushed to Nano on bridge startup
RCT_SETTINGS_PATH = "/etc/inno-pilot/rct_settings.json"

# ---------------------------------------------------------------------------
# Pilot settings — user-configurable parameters, owned by bridge
# ---------------------------------------------------------------------------
PILOT_SETTINGS_PATH = "/var/lib/inno-pilot/settings.json"

_DEFAULT_PILOT_SETTINGS: dict = {
    "network": {
        "ip_mode": "dhcp",
        "ip":      "",
        "mask":    "255.255.255.0",
        "gateway": "",
        "dns1":    "8.8.8.8",
        "dns2":    "8.8.4.4",
    },
    "wifi": {
        "ssid": "",
        "key":  "",
    },
    "vessel": {
        "name":             "",
        "type":             "sail",
        "rudder_range_deg": 35,
    },
    "features": {
        "limit_switches":         False,
        "temp_sensor":            False,
        "pi_voltage_sensor":      False,
        "battery_voltage_sensor": False,
        "current_sensor":         False,
        "on_board_buttons":       False,
    },
    "autopilot": {
        "deadband_pct":           3.0,
        "pgain":                  1.0,
        "off_course_alarm_deg":   20,
        "rudder_limit_port_pct":  0,
        "rudder_limit_stbd_pct":  100,
    },
    "safety": {
        "auto_disengage_on_fault":  True,
        "comms_warn_threshold_pct": 10,
        "comms_crit_threshold_pct": 25,
    },
}

# In-memory pilot settings — loaded at startup, updated on SETTINGS SET.
# Only accessed from the main thread; no locking required.
_pilot_settings: dict = {}

# Ordered list of setting keys (index must match Nano's RCT_SETTING_NAMES order)
_RCT_KEYS = [
    "pwm_max",         # 0
    "coast_count_max", # 1
    "pwm_min",         # 2
    "coast_count_min", # 3
    "pulse_ms",        # 4
    "deadband",        # 5
    "pulse_delay_ms",  # 6
]
_RCT_DEFAULTS: dict = {
    "pwm_max":         255,
    "coast_count_max":  50,
    "pwm_min":         160,
    "coast_count_min":  10,
    "pulse_ms":         20,
    "deadband":         20,
    "pulse_delay_ms":  100,
}

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
TELEM_PERIOD_S    = 0.2   # 5 Hz telemetry to Nano and remote

# ---------------------------------------------------------------------------
# Remote TCP server
# ---------------------------------------------------------------------------
REMOTE_TCP_PORT        = 8555
REMOTE_TIMEOUT_S       = 6.0   # disconnect if no data received for this long
REMOTE_PING_PERIOD_S   = 2.0   # send PONG keepalive to remote every N seconds

# ---------------------------------------------------------------------------
# OTA firmware server (serves inno_remote.bin to remote on version mismatch)
# ---------------------------------------------------------------------------
OTA_HTTP_PORT     = 8556
OTA_SERVER_HOST   = "192.168.6.13"        # Pi fixed IP on boat LAN
OTA_FIRMWARE_DIR  = "/var/lib/inno-pilot/ota"
OTA_FIRMWARE_FILE = "inno_remote.bin"
OTA_FIRMWARE_PATH = f"{OTA_FIRMWARE_DIR}/{OTA_FIRMWARE_FILE}"

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
    # Comms-fault state (populated from Nano FLAGS and COMMS_DIAG frames)
    nano_comms_warn:      bool = False
    nano_comms_crit:      bool = False
    nano_err_window:      int  = 0       # errors in last 10-second window
    nano_crit_consec:     int  = 0       # consecutive seconds above CRIT threshold
    comms_disengage_sent: bool = False   # latch: prevents repeated disengage on same fault
    rct_target:            int  = 500        # last TGT sent to Nano (pct×10, 0-1000)
    rct_last_rdr_send:   float = 0.0        # monotonic time of last RDR_PCT forward to remote


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
    """Open serial port while keeping DTR/RTS low to avoid Arduino resets.

    Also clears the HUPCL termios flag so that DTR is not dropped when the
    port is later closed — otherwise the kernel drops DTR on close, which
    triggers the Nano's reset circuit and causes it to restart.
    """
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
    # Disable HUPCL: prevents DTR from dropping when the fd is closed.
    # Without this, every close() resets the Nano via its RC-differentiator
    # on the RESET pin, even if dtr=False was set during open.
    attrs = termios.tcgetattr(ser.fd)
    attrs[2] &= ~termios.HUPCL  # cflag: clear hang-up-on-close
    termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
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
    raw = wrap_frame(build_frame(code, value_u16))
    log.debug("nano TX  code=0x%02X val=%d  hex=%s", code, value_u16, raw.hex())
    nano.write(raw)


def extract_wrapped_frames(buf: bytearray) -> list[tuple[bytes, bool]]:
    """Pull all complete 6-byte frames out of buf (mutates buf in place).

    Returns list of (frame_4bytes, crc_ok) tuples. Callers must skip frames
    where crc_ok is False rather than forwarding corrupt data.
    """
    frames: list[tuple[bytes, bool]] = []
    while len(buf) >= 2:
        if buf[0] != BRIDGE_MAGIC1 or buf[1] != BRIDGE_MAGIC2:
            del buf[0]
            continue
        if len(buf) < 6:
            break
        frame = bytes(buf[2:6])
        del buf[:6]
        crc_ok = (crc8_msb(frame[:3]) == frame[3])
        frames.append((frame, crc_ok))
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
                send_nano_frame(probe, BRIDGE_HELLO_CODE, BRIDGE_HELLO_VALUE)
                buf = bytearray()
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    chunk = probe.read(64)
                    if chunk:
                        buf.extend(chunk)
                        for frame, crc_ok in extract_wrapped_frames(buf):
                            if not crc_ok:
                                continue
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
# RCT settings helpers
# ===========================================================================

def load_or_create_rct_settings() -> dict:
    """Load RCT settings from JSON, creating the file with defaults if absent."""
    if os.path.isfile(RCT_SETTINGS_PATH):
        try:
            with open(RCT_SETTINGS_PATH) as fh:
                data = json.load(fh)
            # Back-fill any missing keys added in later versions
            missing = {k: v for k, v in _RCT_DEFAULTS.items() if k not in data}
            if missing:
                data.update(missing)
                _save_rct_settings(data)
            return data
        except Exception as exc:
            log.warning("RCT settings load failed (%s) — recreating with defaults", exc)
    settings = dict(_RCT_DEFAULTS)
    _save_rct_settings(settings)
    log.info("RCT settings created at %s", RCT_SETTINGS_PATH)
    return settings


def _save_rct_settings(settings: dict) -> None:
    """Write RCT settings dict to JSON (creates parent dir if needed)."""
    os.makedirs(os.path.dirname(RCT_SETTINGS_PATH), exist_ok=True)
    with open(RCT_SETTINGS_PATH, "w") as fh:
        json.dump(settings, fh, indent=2)
    log.info("RCT settings saved to %s", RCT_SETTINGS_PATH)


def push_rct_settings_to_nano(nano: "serial.Serial", settings: dict) -> None:
    """Send all 7 RCT settings to the Nano as individual frames.

    Frame encoding: CODE=RCT_SETTING_CODE, uint16 = (index << 8) | uint8_value.
    Production motor_simple.ino ignores these frames silently.
    """
    for idx, key in enumerate(_RCT_KEYS):
        val = max(0, min(255, int(settings.get(key, _RCT_DEFAULTS[key]))))
        send_nano_frame(nano, RCT_SETTING_CODE, (idx << 8) | val)
    log.info("RCT settings pushed to Nano (%d frames)", len(_RCT_KEYS))


# ===========================================================================
# Pilot settings helpers
# ===========================================================================

def load_pilot_settings() -> dict:
    """Return pilot settings from file, merged over built-in defaults."""
    s = copy.deepcopy(_DEFAULT_PILOT_SETTINGS)
    if os.path.isfile(PILOT_SETTINGS_PATH):
        try:
            with open(PILOT_SETTINGS_PATH) as fh:
                saved = json.load(fh)
            for sec, vals in saved.items():
                if isinstance(s.get(sec), dict) and isinstance(vals, dict):
                    s[sec].update(vals)
                else:
                    s[sec] = vals
        except Exception as exc:
            log.warning("Pilot settings load failed (%s) — using defaults", exc)
    return s


def _persist_pilot_settings(settings: dict) -> None:
    """Write pilot settings to file, creating parent dirs if needed."""
    try:
        os.makedirs(os.path.dirname(PILOT_SETTINGS_PATH), exist_ok=True)
        with open(PILOT_SETTINGS_PATH, "w") as fh:
            json.dump(settings, fh, indent=2)
        log.info("Pilot settings saved \u2192 %s", PILOT_SETTINGS_PATH)
    except Exception as exc:
        log.error("Pilot settings save failed: %s", exc)


def send_features_to_nano(nano: serial.Serial, settings: dict) -> None:
    """Build feature bitmask from pilot settings and send FEATURES_CODE to Nano.

    Called at startup (after the 1-second Nano settle delay) and on every
    SETTINGS SET command so the Nano immediately reflects the new feature config.
    """
    feats = settings.get("features", {})
    mask: int = 0
    if feats.get("limit_switches",         False): mask |= FEATURE_LIMIT_SWITCHES
    if feats.get("temp_sensor",            False): mask |= FEATURE_TEMP_SENSOR
    if feats.get("pi_voltage_sensor",      False): mask |= FEATURE_PI_VOLTAGE
    if feats.get("battery_voltage_sensor", False): mask |= FEATURE_BATTERY_VOLTAGE
    if feats.get("current_sensor",         False): mask |= FEATURE_CURRENT_SENSOR
    if feats.get("on_board_buttons",       False): mask |= FEATURE_ON_BOARD_BUTTONS
    send_nano_frame(nano, FEATURES_CODE, mask)
    log.info(
        "Features -> Nano: 0x%02X  (limit=%s temp=%s pi_v=%s bat_v=%s curr=%s btn=%s)",
        mask,
        bool(mask & FEATURE_LIMIT_SWITCHES),
        bool(mask & FEATURE_TEMP_SENSOR),
        bool(mask & FEATURE_PI_VOLTAGE),
        bool(mask & FEATURE_BATTERY_VOLTAGE),
        bool(mask & FEATURE_CURRENT_SENSOR),
        bool(mask & FEATURE_ON_BOARD_BUTTONS),
    )


def _apply_pilot_settings(settings: dict, nano: "serial.Serial | None" = None) -> None:
    """Apply settings that the bridge uses at runtime.

    Called once at startup (nano=None, features sent separately after settle delay)
    and on every SETTINGS SET command (nano provided, features sent immediately).
    """
    db = settings.get("autopilot", {}).get("deadband_pct", 3.0)
    log.debug("Pilot settings applied: deadband=%.1f%%", db)
    if nano is not None:
        send_features_to_nano(nano, settings)


# ===========================================================================
# TCP server helpers
# ===========================================================================

def setup_ota_http_server() -> None:
    """Start a minimal HTTP file server in a daemon thread.

    Serves GET /inno_remote.bin from OTA_FIRMWARE_DIR on OTA_HTTP_PORT.
    Returns 404 if the file is not present (safe to call before binary is deployed).
    The server runs for the lifetime of the process.
    """
    import http.server
    import urllib.parse

    class _OTAHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            requested = urllib.parse.unquote(self.path.lstrip("/"))
            if requested != OTA_FIRMWARE_FILE:
                self.send_error(404)
                return
            try:
                with open(OTA_FIRMWARE_PATH, "rb") as fh:
                    data = fh.read()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            log.info("OTA: served %d bytes to %s", len(data), self.client_address[0])

        def log_message(self, fmt, *args):  # suppress default access-log noise
            pass

    class _OTAServer(http.server.HTTPServer):
        allow_reuse_address = True

    def _serve():
        srv = _OTAServer(("", OTA_HTTP_PORT), _OTAHandler)
        log.info("OTA HTTP server listening on port %d (firmware: %s)",
                 OTA_HTTP_PORT, OTA_FIRMWARE_PATH)
        srv.serve_forever()

    t = threading.Thread(target=_serve, daemon=True, name="ota-http")
    t.start()


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
      HELLO <ver>       -> log version match/mismatch, reply HELLO <bridge_ver>
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

    elif cmd == "HELLO":
        remote_ver = parts[1] if len(parts) > 1 else "unknown"
        if remote_ver != INNOPILOT_VERSION:
            log.warning("Remote version mismatch: remote=%s bridge=%s",
                        remote_ver, INNOPILOT_VERSION)
        else:
            log.info("Remote version: %s (match)", remote_ver)
        remote_send(remote_sock, f"HELLO {INNOPILOT_VERSION}")
        # Send current deadband so remote/web-remote shows the live value
        db_pct = _pilot_settings.get("autopilot", {}).get("deadband_pct", 3.0)
        remote_send(remote_sock, f"DB {db_pct:.1f}")
        # Offer OTA if remote is behind and firmware binary is available
        if remote_ver != INNOPILOT_VERSION and os.path.isfile(OTA_FIRMWARE_PATH):
            ota_url = f"http://{OTA_SERVER_HOST}:{OTA_HTTP_PORT}/{OTA_FIRMWARE_FILE}"
            remote_send(remote_sock, f"OTA {ota_url}")
            log.info("OTA offered to remote at %s", ota_url)

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
        if len(parts) < 2:
            return
        try:
            pct = float(parts[1])
        except ValueError:
            log.warning("Remote RUD: invalid value '%s', ignored", parts[1])
            return
        pct = max(0.0, min(100.0, pct))
        target_0_1000 = int(round(pct * 10.0))  # 0-1000
        # Always forward as RCT target regardless of bridge mode — the test sketch
        # uses this to show the live target on the Nano OLED.  This works even when
        # the bridge is in IDLE (e.g. after restart before MODE MANUAL is re-sent).
        bstate.rct_target = target_0_1000
        send_nano_frame(nano, RCT_TARGET_CODE, target_0_1000)
        # MANUAL_RUD_TARGET_CODE only applies in MANUAL mode
        if bstate.mode != MODE_MANUAL:
            return
        bstate.manual_rud_target = target_0_1000
        send_nano_frame(nano, MANUAL_RUD_TARGET_CODE, target_0_1000)

    elif cmd == "TGT":
        # RCT test target: 0.0-100.0 % (0=port limit, 50=midships, 100=stbd limit)
        # Forwarded to Nano as RCT_TARGET_CODE (pct×10, 0-1000).
        # Accepted in any bridge mode — the Nano test sketch decides when to act on it.
        if len(parts) < 2:
            return
        try:
            pct = float(parts[1])
        except ValueError:
            log.warning("Remote TGT: invalid value '%s', ignored", parts[1])
            return
        pct = max(0.0, min(100.0, pct))
        target_0_1000 = int(round(pct * 10.0))
        bstate.rct_target = target_0_1000
        send_nano_frame(nano, RCT_TARGET_CODE, target_0_1000)
        log.info("Remote TGT %.1f%% -> RCT_TARGET_CODE %d", pct, target_0_1000)

    elif cmd == "SETTINGS":
        sub = parts[1].upper() if len(parts) > 1 else ""

        if sub == "GET":
            # Reply with the full in-memory settings as compact single-line JSON
            body = json.dumps(_pilot_settings, separators=(",", ":"), ensure_ascii=True)
            remote_send(remote_sock, f"SETTINGS {body}")
            log.debug("SETTINGS GET handled (%d bytes)", len(body))

        elif sub == "SET":
            # Everything after "SETTINGS SET " is the JSON payload.
            # Split at most twice so spaces inside JSON strings are preserved.
            raw_parts = line.strip().split(None, 2)
            json_str = raw_parts[2] if len(raw_parts) >= 3 else ""
            try:
                new_settings = json.loads(json_str)
                if not isinstance(new_settings, dict):
                    raise ValueError("expected JSON object")
                # Merge into in-memory settings section by section
                for sec, vals in new_settings.items():
                    if isinstance(_pilot_settings.get(sec), dict) and isinstance(vals, dict):
                        _pilot_settings[sec].update(vals)
                    else:
                        _pilot_settings[sec] = vals
                _persist_pilot_settings(_pilot_settings)
                _apply_pilot_settings(_pilot_settings, nano=nano)
                remote_send(remote_sock, "SETTINGS OK")
                log.info("SETTINGS SET: saved and applied")
            except Exception as exc:
                log.warning("SETTINGS SET failed: %s", exc)
                remote_send(remote_sock, "SETTINGS ERR")
        else:
            log.warning("Remote SETTINGS: unknown sub-command '%s'", sub)

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

    # Load pilot settings (user-configurable parameters).
    _pilot_settings.update(load_pilot_settings())
    _apply_pilot_settings(_pilot_settings)
    log.info("Pilot settings loaded from %s", PILOT_SETTINGS_PATH)

    # Load RCT settings and push to Nano.  Production motor_simple.ino ignores
    # these frames; the remote_control_test sketch uses them.  We delay 1 s so
    # the Nano has time to complete its setup() before the frames arrive.
    rct_settings = load_or_create_rct_settings()
    time.sleep(1.0)
    # Safety: ensure Nano is not stuck in ratify mode from a previous run.
    send_nano_frame(nano, MANUAL_MODE_CODE, 0)
    push_rct_settings_to_nano(nano, rct_settings)
    # Push feature enables to Nano so it knows which sensors/alarms are active.
    send_features_to_nano(nano, _pilot_settings)

    nano_buf  = bytearray()
    pilot_buf = bytearray()

    # ---- bridge state (mode state machine) ----
    bstate = BridgeState()

    # ---- volatile diagnostic log (tmpfs — does not survive reboot) ----
    diag_log = logging.getLogger("bridge.diag")
    diag_log.setLevel(logging.DEBUG)
    diag_handler = logging.handlers.RotatingFileHandler(
        "/tmp/inno_pilot_comms_diag.log",
        maxBytes=256 * 1024,   # 256 KB max per file
        backupCount=1,         # keep 1 rotated backup (512 KB total)
    )
    diag_handler.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    ))
    diag_log.addHandler(diag_handler)
    diag_log.propagate = False  # do not echo to stdout/systemd journal

    # ---- telemetry / keepalive timestamps ----
    last_hello_ts    = 0.0
    last_telem_ts    = 0.0

    # ---- pypilot relay diagnostics ----
    relay_good  = 0  # CRC-valid frames forwarded to Nano
    relay_drop  = 0  # bytes dropped during realignment
    relay_log_ts = 0.0

    # ---- OTA firmware server (background daemon thread) ----
    setup_ota_http_server()

    # ---- TCP remote state ----
    tcp_server  = setup_tcp_server()
    remote_sock = None
    remote_buf  = bytearray()
    remote_last_rx_ts = 0.0

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
            heading_cmd  = pstate.heading_cmd
            heading      = pstate.heading
            rudder_angle = pstate.rudder_angle
            rudder_range = pstate.rudder_range

        # ================================================================
        # 3. Push telemetry to Nano and Remote at 5 Hz
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
                ok = ok and remote_send(remote_sock, f"AP {1 if bstate.mode == MODE_AP else 0}")
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
                if bstate.nano_comms_crit:
                    ok = ok and remote_send(remote_sock, "COMMS CRIT")
                elif bstate.nano_comms_warn:
                    ok = ok and remote_send(remote_sock, f"COMMS WARN {bstate.nano_err_window}")
                else:
                    ok = ok and remote_send(remote_sock, "COMMS OK")
                if not ok:
                    log.warning("TCP remote: send failed during telemetry, disconnecting")
                    handle_remote_disconnect(nano, bstate, set_q, pstate, plock)
                    close_remote(remote_sock)
                    remote_sock = None
                    remote_buf  = bytearray()

        # ================================================================
        # 4. Relay: pypilot -> Nano (CRC-validated, self-aligning)
        # ================================================================
        data_from_pilot = pilot.read(256)
        if data_from_pilot:
            log.debug("pilot RX  %d bytes: %s", len(data_from_pilot),
                      data_from_pilot.hex())
            pilot_buf.extend(data_from_pilot)
            while len(pilot_buf) >= 4:
                candidate = bytes(pilot_buf[:4])
                crc_calc = crc8_msb(candidate[:3])
                if crc_calc == candidate[3]:
                    # Valid pypilot frame — wrap with magic header and forward
                    wrapped = wrap_frame(candidate)
                    nano.write(wrapped)
                    log.debug("pilot->nano RELAY  code=0x%02X  raw=%s  wrapped=%s",
                              candidate[0], candidate.hex(), wrapped.hex())
                    del pilot_buf[:4]
                    relay_good += 1

                    # Sync bridge mode from pypilot's own engage/disengage signals.
                    # This keeps bstate.mode correct when pypilot changes AP state
                    # independently (fault, external client, etc.).
                    relay_code = candidate[0]
                    if relay_code == PYPILOT_DISENGAGE_CODE and bstate.mode == MODE_AP:
                        bstate.mode = MODE_IDLE
                        with plock:
                            pstate.ap_enabled = False
                        log.info("pypilot relay: DISENGAGE received — mode -> IDLE")
                    elif relay_code == PYPILOT_COMMAND_CODE and bstate.mode == MODE_IDLE:
                        bstate.mode = MODE_AP
                        with plock:
                            pstate.ap_enabled = True
                        log.info("pypilot relay: COMMAND received — mode -> AP")
                else:
                    # Alignment error — drop one byte and rescan
                    log.debug("pilot relay DROP  byte=0x%02X  cand=%s"
                              "  crc_got=0x%02X crc_exp=0x%02X",
                              pilot_buf[0], candidate.hex(),
                              candidate[3], crc_calc)
                    del pilot_buf[0]
                    relay_drop += 1

            # Log relay stats every 30 s (reset counters each window)
            if relay_good + relay_drop > 0 and (now - relay_log_ts) >= 30.0:
                total    = relay_good + relay_drop
                drop_pct = relay_drop / total * 100.0
                if drop_pct > 5.0:
                    log.warning("pypilot relay: %.1f%% byte drop rate (%d/%d) — possible upstream issue",
                                drop_pct, relay_drop, total)
                else:
                    log.info("pypilot relay stats: %d forwarded, %d bytes dropped (%.1f%%)",
                             relay_good, relay_drop, drop_pct)
                relay_good   = 0
                relay_drop   = 0
                relay_log_ts = now

        # ================================================================
        # 5. Relay: Nano -> pypilot  (intercept events; parse telemetry)
        # ================================================================
        try:
            data_from_nano = nano.read(256)
        except serial.serialutil.SerialException as exc:
            log.error("Nano serial read failed: %s", exc)
            raise SystemExit(1) from exc

        if data_from_nano:
            log.debug("nano RX  %d bytes: %s", len(data_from_nano),
                      data_from_nano.hex())
            nano_buf.extend(data_from_nano)

            for f, crc_ok in extract_wrapped_frames(nano_buf):
                if not crc_ok:
                    log.warning("Nano->bridge CRC error: frame=%s expected=0x%02X got=0x%02X",
                                f.hex(), crc8_msb(f[:3]), f[3])
                    diag_log.warning(
                        "BRIDGE_CRC_ERR frame=%s code=0x%02X expected=0x%02X got=0x%02X",
                        f.hex(), f[0], crc8_msb(f[:3]), f[3],
                    )
                    continue  # drop corrupt frame — do not forward to pypilot

                code  = f[0]
                value = f[1] | (f[2] << 8)

                log.debug("nano->pilot RELAY  code=0x%02X val=%d  hex=%s",
                          code, value, f.hex())

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

                elif code == PIN_STATE_CODE:
                    # H-bridge pin state change from Nano (D2/D3/D9)
                    d2 = bool(value & 0x01)   # D2  RPWM  (stbd direction)
                    d3 = bool(value & 0x02)   # D3  LPWM  (port direction)
                    d9 = bool(value & 0x04)   # D9  EN/PWM (motor enabled)
                    if d9:
                        direction = "STBD" if d2 else ("PORT" if d3 else "EN-only?")
                    else:
                        direction = "STOP"
                    log.debug(
                        "Nano motor pins: D2(RPWM)=%d D3(LPWM)=%d D9(EN)=%d  [%s]",
                        d2, d3, d9, direction,
                    )

                elif code == MOTOR_REASON_CODE:
                    # B26: one-shot diagnostic sent when D9 goes LOW→HIGH.
                    # Decodes which branch activated the motor + key state at that moment.
                    reason_id   = value & 0x0F
                    ap_en       = bool(value & 0x10)
                    cmd_recent  = bool(value & 0x20)
                    pi_alive_f  = bool(value & 0x40)
                    man_ov      = bool(value & 0x80)
                    cmd_approx  = ((value >> 8) & 0xFF) * 8  # approx last_command_val
                    reason_str  = _MRSN.get(reason_id, f"unknown({reason_id})")
                    log.debug(
                        "MOTOR START reason=%s  ap_en=%d cmd_recent=%d pi_alive=%d "
                        "manual_ov=%d  last_cmd≈%d (delta≈%+d)",
                        reason_str, ap_en, cmd_recent, pi_alive_f, man_ov,
                        cmd_approx, cmd_approx - 1000,
                    )

                elif code == FLAGS_CODE:
                    # Parse comms fault bits (bridge-side detection plane)
                    bstate.nano_comms_warn = bool(value & FLAG_COMMS_WARN)
                    bstate.nano_comms_crit = bool(value & FLAG_COMMS_CRIT)

                    # Auto-disengage AP on CRITICAL comms fault
                    if bstate.nano_comms_crit and not bstate.comms_disengage_sent:
                        set_q.put(("ap.enabled", False))
                        with plock:
                            pstate.ap_enabled = False
                        if bstate.mode == MODE_AP:
                            bstate.mode = MODE_IDLE
                        bstate.comms_disengage_sent = True
                        log.warning("COMMS CRITICAL: auto-disengage AP (Nano flags=0x%04X)", value)

                    # Clear disengage latch once comms fully recovers
                    if not bstate.nano_comms_warn and not bstate.nano_comms_crit:
                        bstate.comms_disengage_sent = False

                    # Relay flags to remote
                    if remote_sock is not None:
                        remote_send(remote_sock, f"FLAGS {value}")

                elif code == BUZZER_STATE_CODE:
                    bstate.nano_buzzer_on = (value != 0)

                elif code == COMMS_DIAG_CODE:
                    bstate.nano_err_window  = value & 0xFF
                    bstate.nano_crit_consec = (value >> 8) & 0xFF
                    if bstate.nano_err_window > 0:
                        log.info("Nano comms diag: %d errors/10s, %d consecutive CRIT seconds",
                                 bstate.nano_err_window, bstate.nano_crit_consec)
                    else:
                        log.debug("Nano comms diag: clean (0 errors/10s)")

                elif code == COMMS_ERR_DETAIL_CODE:
                    err_code_byte = value & 0xFF
                    err_rx_crc    = (value >> 8) & 0xFF
                    diag_log.warning(
                        "ERR_DETAIL code=0x%02X rx_crc=0x%02X err_window=%d crit_s=%d",
                        err_code_byte, err_rx_crc,
                        bstate.nano_err_window, bstate.nano_crit_consec,
                    )

                elif code == RCT_SETTING_CODE:
                    # Nano reports a setting it changed in adjust mode — update Pi JSON
                    idx = (value >> 8) & 0xFF
                    val =  value       & 0xFF
                    if idx < len(_RCT_KEYS):
                        key = _RCT_KEYS[idx]
                        rct_settings[key] = val
                        _save_rct_settings(rct_settings)
                        log.info("RCT setting update from Nano: %s=%d", key, val)

                elif code == RCT_RESULT_STOP_CODE:
                    log.info("RCT result: first_stop_adc=%d", value)
                    if remote_sock is not None:
                        remote_send(remote_sock, f"RCT_STOP {value}")

                elif code == RCT_RESULT_PULSES_CODE:
                    log.info("RCT result: pulse_count=%d", value)
                    if remote_sock is not None:
                        remote_send(remote_sock, f"RCT_PULSES {value}")

                elif code == RCT_RESULT_FINAL_CODE:
                    log.info("RCT result: final_adc=%d", value)
                    if remote_sock is not None:
                        remote_send(remote_sock, f"RCT_FINAL {value}")

                elif code == RCT_RDR_PCT_CODE:
                    # Ratify live position: throttled to 50 ms (20 Hz) so a fast Nano
                    # ratify loop (>100 Hz) does not flood the remote TCP connection.
                    if remote_sock is not None:
                        now = time.monotonic()
                        if now - bstate.rct_last_rdr_send >= 0.05:
                            remote_send(remote_sock, f"RDR_PCT {value / 10.0:.1f}")
                            bstate.rct_last_rdr_send = now

                elif code == RCT_HZ_CODE:
                    # Ratify loop Hz: forward to remote for display on MODE line
                    if remote_sock is not None:
                        remote_send(remote_sock, f"HZ {value}")

        # ================================================================
        # 6. TCP remote: accept new connections / handle existing client
        # ================================================================
        watch_socks = [tcp_server]
        if remote_sock is not None:
            watch_socks.append(remote_sock)

        readable, _, _ = select.select(watch_socks, [], [], 0)

        for s in readable:
            if s is tcp_server:
                conn, addr = tcp_server.accept()
                if remote_sock is not None:
                    # If the same IP reconnects (e.g. after OTA reboot or TCP half-close),
                    # replace the stale socket rather than rejecting the new connection.
                    try:
                        curr_ip = remote_sock.getpeername()[0]
                    except OSError:
                        curr_ip = None
                    if addr[0] == curr_ip:
                        log.warning("TCP remote: %s reconnected, replacing stale connection", addr[0])
                        handle_remote_disconnect(nano, bstate, set_q, pstate, plock)
                        close_remote(remote_sock)
                        remote_buf = bytearray()
                    else:
                        log.warning("TCP remote: rejected %s (already connected from %s)", addr, curr_ip)
                        conn.close()
                        conn = None
                if conn is not None and remote_sock is None:
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
