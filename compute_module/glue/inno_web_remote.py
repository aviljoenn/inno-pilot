#!/usr/bin/env python3
"""
inno_web_remote.py — Browser-based Inno-Remote UI (port 8888).

Connects to inno-pilot-bridge on TCP port 8555 as a standard remote
client (same HELLO/PING/BTN/MODE/RUD protocol as the ESP32 handheld),
then serves a single-page web app over HTTP + Server-Sent Events.

No external dependencies — stdlib only.

Usage:
    python3 /usr/local/bin/inno_web_remote.py
    Open http://192.168.6.13:8888/ in a browser on the boat LAN.
"""

import json
import logging
import math
import os
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("web-remote")

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WEB_PORT          = 8888           # HTTP / SSE port for browser clients
BRIDGE_HOST       = "127.0.0.1"
BRIDGE_PORT       = 8555           # inno-pilot-bridge TCP remote port
PING_PERIOD_S     = 2.0
RECONNECT_DELAY_S = 5.0
# Sent in HELLO handshake.  Bridge logs mismatch but stays connected.
INNOPILOT_VERSION = "v1.2.0_B29"

# ---------------------------------------------------------------------------
# Settings persistence — /var/lib/inno-pilot/settings.json
# ---------------------------------------------------------------------------
SETTINGS_FILE = "/var/lib/inno-pilot/settings.json"

_DEFAULT_SETTINGS: dict = {
    "network": {
        "ip_mode": "dhcp",          # "dhcp" or "static"
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
        "type":             "sail",  # "sail" or "power"
        "rudder_range_deg": 35,      # full rudder throw each side (degrees)
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
        "pgain":                  1.0,  # proportional heading gain
        "off_course_alarm_deg":   20,   # degrees off-course before alert
        "rudder_limit_port_pct":  0,    # software port stop (%)
        "rudder_limit_stbd_pct":  100,  # software stbd stop (%)
    },
    "safety": {
        "auto_disengage_on_fault":  True,
        "comms_warn_threshold_pct": 10,  # CRC error rate % for WARN
        "comms_crit_threshold_pct": 25,  # CRC error rate % for CRIT
    },
}


def _load_settings() -> dict:
    """Return settings from file merged over built-in defaults."""
    import copy
    s = copy.deepcopy(_DEFAULT_SETTINGS)
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE) as fh:
                saved = json.load(fh)
            for sec, vals in saved.items():
                if isinstance(s.get(sec), dict) and isinstance(vals, dict):
                    s[sec].update(vals)
                else:
                    s[sec] = vals
        except Exception as exc:
            log.warning("Settings load failed: %s", exc)
    return s


def _persist_settings(settings: dict) -> None:
    """Write settings dict to file, creating parent dirs if needed."""
    try:
        os.makedirs(os.path.dirname(SETTINGS_FILE), exist_ok=True)
        with open(SETTINGS_FILE, "w") as fh:
            json.dump(settings, fh, indent=2)
        log.info("Settings saved \u2192 %s", SETTINGS_FILE)
    except Exception as exc:
        log.error("Settings save failed: %s", exc)


# ---------------------------------------------------------------------------
# Shared state — written by bridge thread, read by HTTP handlers
# ---------------------------------------------------------------------------
_state: dict = {
    "connected":  False,
    "mode":       "IDLE",   # IDLE | AP | MANUAL  (from bridge MODE telemetry)
    "ap":         0,
    "hdg":        None,     # float degrees
    "cmd":        None,     # float degrees (AP heading command)
    "rdr":        None,     # float degrees (actual rudder angle from pypilot)
    "rdr_pct":    None,     # float 0–100  (actual rudder position %)
    "db":         3.0,
    "comms":      "OK",     # OK | WARN | CRIT
    "warn":       None,
    "bridge_ver": None,
    "version":    INNOPILOT_VERSION,
}
_state_lock = threading.Lock()

# One Queue per connected SSE browser
_sse_subs: list[queue.Queue] = []
_sse_lock  = threading.Lock()

# Commands flowing from browser → bridge
_cmd_q: "queue.Queue[str]" = queue.Queue(maxsize=200)

# Responses to SETTINGS GET / SETTINGS SET received from the bridge.
# HTTP handler drains this queue before issuing a request, then waits up to
# SETTINGS_TIMEOUT_S for the bridge to reply.  Maxsize=4 catches stale replies.
_settings_resp_q: "queue.Queue[Optional[dict]]" = queue.Queue(maxsize=4)
SETTINGS_TIMEOUT_S = 4.0

# When cleared, the bridge thread drops the TCP connection and stops reconnecting.
# Set by default (connected); cleared when the mode toggle is moved to OFF.
_bridge_active = threading.Event()
_bridge_active.set()


def _snap() -> dict:
    with _state_lock:
        return dict(_state)


def _update(**kw) -> None:
    """Update shared state and push a snapshot to every SSE subscriber."""
    with _state_lock:
        _state.update(kw)
        snap = dict(_state)
    _broadcast(snap)


def _broadcast(snap: dict) -> None:
    with _sse_lock:
        dead = []
        for q in _sse_subs:
            try:
                q.put_nowait(snap)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _sse_subs.remove(q)
            log.debug("SSE: dropped slow client")


# ---------------------------------------------------------------------------
# Bridge TCP client — reconnecting, runs in a daemon thread
# ---------------------------------------------------------------------------

def _parse_bridge_line(line: str) -> None:
    """Parse one telemetry line received from the bridge."""
    parts = line.split()
    if not parts:
        return
    c = parts[0].upper()

    if c == "PONG":
        pass  # keepalive, no state change

    elif c == "HELLO":
        _update(bridge_ver=(parts[1] if len(parts) > 1 else "?"))

    elif c == "AP":
        try:
            _update(ap=int(parts[1]))
        except (IndexError, ValueError):
            pass

    elif c == "MODE":
        # Bridge sends MODE IDLE|AP|MANUAL every telemetry cycle
        if len(parts) > 1:
            _update(mode=parts[1].upper())

    elif c in ("HDG", "CMD", "RDR", "DB"):
        try:
            _update(**{c.lower(): float(parts[1])})
        except (IndexError, ValueError):
            pass

    elif c == "RDR_PCT":
        try:
            _update(rdr_pct=float(parts[1]))
        except (IndexError, ValueError):
            pass

    elif c == "COMMS":
        # COMMS OK | COMMS WARN <detail> | COMMS CRIT
        if len(parts) > 1:
            _update(comms=parts[1].upper())

    elif c == "WARN":
        _update(warn=(parts[1] if len(parts) > 1 else None))

    elif c == "SETTINGS":
        # Response to SETTINGS GET: "SETTINGS <json>"
        # Response to SETTINGS SET: "SETTINGS OK" or "SETTINGS ERR"
        if len(parts) < 2:
            return
        sub = parts[1].upper()
        if sub in ("OK", "ERR"):
            # Ack for a SET command — signal waiting HTTP handler
            try:
                _settings_resp_q.put_nowait({"_ack": sub})
            except queue.Full:
                pass
        else:
            # Must be JSON payload (SETTINGS GET response).
            # The entire remainder of the line (after "SETTINGS ") is JSON.
            json_str = line[len("SETTINGS "):].strip()
            try:
                data = json.loads(json_str)
                _settings_resp_q.put_nowait(data)
            except (json.JSONDecodeError, queue.Full) as exc:
                log.warning("SETTINGS response parse error: %s", exc)

    else:
        log.debug("Bridge unknown: %s", line)


def bridge_client() -> None:
    """
    Persistent TCP client to inno-pilot-bridge.
    Reconnects automatically after any failure.
    When _bridge_active is cleared (mode OFF), drops the current connection
    and does not reconnect until the event is set again.
    Runs forever as a daemon thread.
    """
    while True:
        # Don't attempt connection while mode is OFF
        if not _bridge_active.is_set():
            time.sleep(0.2)
            continue

        sock: Optional[socket.socket] = None
        intentional_disconnect = False
        try:
            log.info("Connecting to bridge %s:%d", BRIDGE_HOST, BRIDGE_PORT)
            sock = socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout=5.0)
            sock.settimeout(0.1)
            _update(connected=True)
            sock.sendall(f"HELLO {INNOPILOT_VERSION}\n".encode())

            buf = b""
            last_ping = time.monotonic()

            while True:
                # Drop connection immediately if mode switched to OFF
                if not _bridge_active.is_set():
                    log.info("Bridge: mode OFF — dropping TCP connection")
                    intentional_disconnect = True
                    break

                now = time.monotonic()
                if now - last_ping >= PING_PERIOD_S:
                    sock.sendall(b"PING\n")
                    last_ping = now

                # Forward any queued commands from browser to bridge
                while True:
                    try:
                        sock.sendall((_cmd_q.get_nowait() + "\n").encode())
                    except queue.Empty:
                        break

                # Read telemetry from bridge
                try:
                    chunk = sock.recv(512)
                    if not chunk:
                        raise OSError("Bridge EOF")
                    buf += chunk
                except socket.timeout:
                    pass

                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    _parse_bridge_line(raw.decode(errors="replace").strip())

        except Exception as exc:
            log.warning("Bridge lost: %s", exc)

        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            if intentional_disconnect:
                # Mode OFF — stay disconnected; state already set by _handle_command
                log.info("Bridge disconnected (mode OFF)")
            else:
                # Unexpected loss — reset to IDLE and schedule reconnect
                _update(
                    connected=False, hdg=None, cmd=None,
                    rdr=None, rdr_pct=None, ap=0,
                    mode="IDLE", comms="OK", warn=None,
                )
                log.info("Reconnecting in %ds...", RECONNECT_DELAY_S)
                time.sleep(RECONNECT_DELAY_S)


# ---------------------------------------------------------------------------
# Ship's wheel SVG — generated once at startup
# ---------------------------------------------------------------------------

def _make_wheel_svg() -> str:
    """Return SVG markup for a wooden ship's wheel (8 spokes, brass hub)."""
    parts: list[str] = []

    # Drop shadow
    parts.append('<ellipse cx="102" cy="103" rx="86" ry="86" fill="rgba(0,0,0,0.28)"/>')

    # Rim — three layered stroked circles give a rounded 3-D wood appearance
    parts.append('<circle cx="100" cy="100" r="84" fill="none" stroke="#2A1408" stroke-width="20"/>')
    parts.append('<circle cx="100" cy="100" r="84" fill="none" stroke="#8B5E3C" stroke-width="14"/>')
    parts.append('<circle cx="100" cy="100" r="84" fill="none" stroke="#C49A6C" stroke-width="5"/>')
    parts.append('<circle cx="100" cy="100" r="76" fill="none" stroke="#1A0C04" stroke-width="1.5"/>')

    # 8 spokes + handle knobs at 45° intervals
    for i in range(8):
        a = math.radians(i * 45)
        ix = 100 + 22 * math.cos(a)
        iy = 100 + 22 * math.sin(a)
        ox = 100 + 73 * math.cos(a)
        oy = 100 + 73 * math.sin(a)
        kx = 100 + 80 * math.cos(a)
        ky = 100 + 80 * math.sin(a)

        if i == 6:
            # ── King spoke: Turk's Head rope-woven appearance ──
            # Dark shadow base
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#1A0C04" stroke-width="11" stroke-linecap="round"/>'
            )
            # Rope base (manila/cream)
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#C8922A" stroke-width="7" stroke-linecap="round"/>'
            )
            # Woven Turk's Head strand pattern — alternating X crossings along the spoke.
            # King spoke is vertical: (100,78) → (100,27), length 51 px.
            # 7 crossings spaced every 7 px give seamless tiling (strand half-height = 3.5 px).
            rw = 3.5   # half-width  of strand from spoke centre
            rh = 3.5   # half-height of each crossing
            s_dark = "#6B4010"   # shadow side of strand
            s_mid  = "#D4A050"   # lit side of strand
            s_hi   = "#F0D090"   # specular highlight
            for step in range(7):
                yc = 30.5 + step * 7.0
                if step % 2 == 0:
                    # "\" strand lies on top of "/" strand
                    parts.append(
                        f'<line x1="{100+rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100-rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_dark}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100-rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100+rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_mid}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100-rw+0.5:.1f}" y1="{yc-rh+0.5:.1f}" '
                        f'x2="{100+rw-1.0:.1f}" y2="{yc+rh-1.0:.1f}" '
                        f'stroke="{s_hi}" stroke-width="0.9" stroke-linecap="round" opacity="0.65"/>'
                    )
                else:
                    # "/" strand lies on top of "\" strand
                    parts.append(
                        f'<line x1="{100-rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100+rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_dark}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100+rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100-rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_mid}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100+rw-0.5:.1f}" y1="{yc-rh+0.5:.1f}" '
                        f'x2="{100-rw+1.0:.1f}" y2="{yc+rh-1.0:.1f}" '
                        f'stroke="{s_hi}" stroke-width="0.9" stroke-linecap="round" opacity="0.65"/>'
                    )
        else:
            # Normal spoke — layered for wood-grain depth
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#2A1408" stroke-width="9" stroke-linecap="round"/>'
            )
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#8B5E3C" stroke-width="5.5" stroke-linecap="round"/>'
            )
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#C09060" stroke-width="2" stroke-linecap="round"/>'
            )

        # Handle knob
        parts.append(
            f'<circle cx="{kx:.1f}" cy="{ky:.1f}" r="10" '
            f'fill="#2A1408" stroke="#1A0C04" stroke-width="1"/>'
        )
        parts.append(
            f'<circle cx="{kx:.1f}" cy="{ky:.1f}" r="7.5" fill="#7A5030"/>'
        )
        # Highlight
        parts.append(
            f'<circle cx="{kx - 1.5:.1f}" cy="{ky - 1.5:.1f}" r="3" '
            f'fill="#C09060" opacity="0.6"/>'
        )

    # Brass center hub
    parts.append('<circle cx="100" cy="100" r="23" fill="#2A1408" stroke="#1A0C04" stroke-width="2"/>')
    parts.append('<circle cx="100" cy="100" r="20" fill="#C49020"/>')
    parts.append('<circle cx="100" cy="100" r="14" fill="#DAA520" stroke="#8B6010" stroke-width="1"/>')
    parts.append('<circle cx="100" cy="100" r="7"  fill="#F5D060"/>')
    parts.append('<circle cx="97"  cy="97"  r="3"  fill="#FFF8C0" opacity="0.55"/>')

    return "\n        ".join(parts)


_WHEEL_SVG = _make_wheel_svg()

# ---------------------------------------------------------------------------
# Embedded single-page web app
# ---------------------------------------------------------------------------
_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Inno-Remote</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%}
body{
  background:#0d0f1e;
  display:flex;
  align-items:flex-start;
  justify-content:center;
  padding:10px 6px 24px;
  font-family:'Segoe UI',Roboto,Arial,sans-serif;
  min-height:100vh;
}

/* ── Remote outer case ── */
.remote{
  background:linear-gradient(160deg,#f5f5f5 0%,#dcdcdc 100%);
  border-radius:26px;
  box-shadow:
    0 30px 80px rgba(0,0,0,0.65),
    inset 0 1px 0 rgba(255,255,255,0.9),
    inset 0 -2px 6px rgba(0,0,0,0.12);
  padding:16px 14px 20px;
  width:340px;
  max-width:98vw;
  display:flex;
  flex-direction:column;
  gap:13px;
  -webkit-user-select:none;user-select:none;
}

/* ── OLED display ── */
.oled{
  background:#06081a;
  border-radius:9px;
  padding:10px 12px 9px;
  box-shadow:
    inset 0 3px 10px rgba(0,0,0,0.9),
    0 0 0 3px #1e2040,
    0 0 0 5px #2e3060,
    0 0 0 7px #131326;
  display:flex;
  flex-direction:column;
  gap:5px;
}
.oled-title{
  color:#00d4ff;
  font-size:1.2em;
  font-weight:700;
  text-align:center;
  letter-spacing:3px;
  text-shadow:0 0 10px #00bfff,0 0 22px rgba(0,191,255,0.35);
}

/* Rudder position bar */
.rdr-bar{
  position:relative;
  height:16px;
  display:flex;
  align-items:center;
  padding:0 4px;
}
.rdr-track{
  width:100%;
  height:2px;
  background:repeating-linear-gradient(90deg,
    #1a4060 0,#1a4060 4px,transparent 4px,transparent 8px);
  position:relative;
}
.rdr-center-tick{
  position:absolute;
  left:50%;
  top:-4px;
  width:2px;
  height:10px;
  background:#2a6090;
  transform:translateX(-50%);
}
.rdr-marker{
  position:absolute;
  width:12px;
  height:12px;
  background:#00d4ff;
  box-shadow:0 0 6px #00bfff;
  top:50%;
  left:50%;
  transform:translate(-50%,-50%);
  transition:left 0.12s ease;
}

/* line-height:1.5 pre-reserves height for the 1.5× AP label so layout never shifts */
.oled-mode{color:#ccc;font-size:0.8em;letter-spacing:1px;line-height:1.5;display:flex;align-items:flex-end}
.oled-mode b{color:#00d4ff}
/* AP label in AUTO mode — 1.5× the surrounding text size */
.oled-mode .ap-label{font-size:1.5em;font-weight:700;color:#00d4ff;line-height:1}
.oled-data{
  color:#00d4ff;
  font-size:0.76em;
  font-weight:700;
  font-family:'Courier New',monospace;
  display:flex;
  justify-content:space-between;
}
.oled-btns{
  display:flex;
  justify-content:center;
  gap:4px;
  margin-top:1px;
}
.oled-btn{
  background:#0a1428;
  color:#7aabcc;
  border:1px solid #1e3850;
  border-radius:3px;
  font-size:0.7em;
  padding:2px 7px;
  cursor:pointer;
  font-family:'Courier New',monospace;
  touch-action:manipulation;
}
.oled-btn:active{background:#1a3060}
/* Bottom row of OLED screen: version left, conn centred */
.oled-status{
  display:flex;
  align-items:center;
  position:relative;
  font-family:'Courier New',monospace;
  letter-spacing:1px;
  margin-top:2px;
}
#o-ver{
  font-size:0.63em;
  color:#6ee0ff;
}
#o-conn{
  position:absolute;
  left:50%;
  transform:translateX(-50%);
  font-size:0.945em;
  font-weight:700;
  white-space:nowrap;
}
.ok{color:#00cc70}
.warn{color:#ffaa00}
.crit{color:#ff3030;animation:blink .45s step-end infinite}
@keyframes blink{50%{opacity:0}}

/* ── Physical button row ── */
.btn-row{
  display:flex;
  justify-content:space-between;
  gap:5px;
}
.hw-btn{
  flex:1;
  height:50px;
  border:none;
  border-radius:8px;
  font-size:1.2em;
  font-weight:800;
  cursor:pointer;
  color:#fff;
  box-shadow:0 5px 0 rgba(0,0,0,0.38),0 6px 10px rgba(0,0,0,0.28);
  transition:transform .07s,box-shadow .07s;
  display:flex;align-items:center;justify-content:center;
  touch-action:manipulation;
}
.hw-btn:active{transform:translateY(4px);box-shadow:0 1px 0 rgba(0,0,0,0.38),0 2px 4px rgba(0,0,0,0.28)}
.hw-btn.b1{background:linear-gradient(180deg,#ff5a5a,#c00000)}
.hw-btn.b2{background:linear-gradient(180deg,#cacaca,#909090);color:#111}
.hw-btn.b3{background:linear-gradient(180deg,#444444,#111111)}
.hw-btn.b4{background:linear-gradient(180deg,#55cc55,#1a8a1a)}
.hw-btn.b5{background:linear-gradient(180deg,#55cc55,#1a8a1a)}

/* ── STOP + toggle row ── */
.action-row{
  display:flex;
  align-items:center;
  justify-content:space-between;
  padding:0 10px;
}

/* STOP button */
.stop-wrap{
  position:relative;
  width:96px;
  height:96px;
}
.stop-ring{
  position:absolute;
  inset:0;
  border-radius:50%;
  background:radial-gradient(circle at 35% 28%,#dddddd,#9a9a9a 55%,#686868);
  box-shadow:0 4px 14px rgba(0,0,0,0.5),inset 0 1px 0 rgba(255,255,255,0.55);
}
.stop-btn{
  position:absolute;
  inset:8px;
  border-radius:50%;
  border:none;
  cursor:pointer;
  background:radial-gradient(circle at 38% 28%,#ff7070,#cc0000 55%,#7a0000);
  color:#fff;
  font-size:1.05em;
  font-weight:900;
  letter-spacing:2px;
  box-shadow:0 6px 0 #550000,0 8px 18px rgba(0,0,0,0.5),
             inset 0 2px 4px rgba(255,255,255,0.18);
  transition:transform .07s,box-shadow .07s;
  touch-action:manipulation;
}
.stop-btn:active{
  transform:translateY(4px);
  box-shadow:0 2px 0 #550000,0 4px 8px rgba(0,0,0,0.5),
             inset 0 2px 4px rgba(255,255,255,0.18);
}

/* ── 4-position mode radio selector ── */
.mode-radio-group{display:flex;flex-direction:column;gap:12px;justify-content:center}
.mode-radio{display:flex;align-items:center;gap:10px;cursor:pointer;user-select:none;-webkit-user-select:none}
.mrd-dot{
  width:22px;height:22px;border-radius:50%;
  border:2px solid #888;background:#ccc;
  position:relative;flex-shrink:0;
  transition:border-color .18s,background .18s;
}
.mrd-dot::after{
  content:'';position:absolute;inset:4px;border-radius:50%;
  background:#007aff;transform:scale(0);
  transition:transform .22s cubic-bezier(.34,1.56,.64,1);
}
.mode-radio.active .mrd-dot{border-color:#007aff;background:#d8eaff}
.mode-radio.active .mrd-dot::after{transform:scale(1)}
.mrd-lbl{
  font-size:0.7em;font-weight:600;color:#999;
  letter-spacing:0.5px;line-height:1.1;
  transition:color .15s,font-weight .15s;
}
.mode-radio.active .mrd-lbl{color:#111;font-weight:800}
.mode-radio:hover .mrd-lbl{color:#444}
/* OFF mode: blank the OLED content area */
.oled.blank-mode .oled-title,
.oled.blank-mode .rdr-bar,
.oled.blank-mode .oled-mode,
.oled.blank-mode .oled-data,
.oled.blank-mode .oled-status{visibility:hidden}

/* ── Ship's wheel ── */
.wheel-section{
  display:flex;
  flex-direction:column;
  align-items:center;
  gap:5px;
}
.wheel-wrap{
  width:210px;
  height:210px;
  cursor:grab;
  touch-action:none;
  filter:saturate(0.3) brightness(0.5);
  transition:filter .35s;
}
.wheel-wrap.active{
  filter:saturate(1) brightness(1);
}
.wheel-wrap.active:active{cursor:grabbing}
#wheel-svg{width:100%;height:100%;display:block}
.wheel-lbl{
  font-size:.76em;
  color:#888;
  text-align:center;
}
.wheel-lbl b{color:#444}

/* ── "No bridge" overlay ── */
.no-bridge-overlay{
  position:fixed;
  inset:0;
  background:rgba(6,8,28,.88);
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
  gap:14px;
  z-index:200;
  -webkit-backdrop-filter:blur(5px);
  backdrop-filter:blur(5px);
}
.no-bridge-overlay h2{
  font-size:1.8em;
  color:#ff4040;
  letter-spacing:3px;
}
.no-bridge-overlay p{color:#aaa;font-size:.9em}
.no-bridge-overlay.hidden{display:none}

@media(max-width:350px){
  .remote{width:100vw;border-radius:0;padding:10px 8px 16px}
}

/* ── Gear / settings button ─────────────────────────────────────────── */
.settings-footer{display:flex;justify-content:flex-start;padding:0 4px 2px}
.gear-btn{
  width:38px;height:38px;border-radius:50%;
  background:#111;border:1.5px solid #333;
  color:#fff;font-size:1.3em;line-height:1;
  cursor:pointer;display:flex;align-items:center;justify-content:center;
  box-shadow:0 3px 8px rgba(0,0,0,.55),inset 0 1px 0 rgba(255,255,255,.07);
  transition:transform .1s,background .15s,border-color .15s;
  touch-action:manipulation;
}
.gear-btn:active{transform:scale(.88)}
.gear-btn.settings-open{
  background:#0d2a40;border-color:#00bfff;color:#00d4ff;
  box-shadow:0 0 8px rgba(0,191,255,.45),0 3px 8px rgba(0,0,0,.4);
}

/* ── Settings warning toast ─────────────────────────────────────────── */
.sw-toast{
  position:fixed;top:50%;left:50%;
  transform:translate(-50%,-50%) scale(.85);
  background:rgba(155,50,0,.97);color:#fff;
  padding:11px 20px;border-radius:9px;
  font-size:.88em;font-weight:700;text-align:center;line-height:1.5;
  opacity:0;pointer-events:none;z-index:500;
  transition:opacity .18s,transform .18s;
  box-shadow:0 6px 24px rgba(0,0,0,.7);
}
.sw-toast.visible{opacity:1;transform:translate(-50%,-50%) scale(1)}

/* ── Settings overlay panel ─────────────────────────────────────────── */
.sov{
  position:fixed;inset:0;background:rgba(4,6,22,.92);
  display:flex;align-items:center;justify-content:center;
  z-index:400;
  -webkit-backdrop-filter:blur(4px);backdrop-filter:blur(4px);
}
.sov.hidden{display:none}
.spanel{
  background:linear-gradient(170deg,#0e1226,#080b18);
  border:1px solid #1e2a50;border-radius:13px;
  box-shadow:0 24px 70px rgba(0,0,0,.85),0 0 0 1px rgba(0,191,255,.08);
  width:318px;max-width:96vw;max-height:88vh;
  display:flex;flex-direction:column;overflow:hidden;
}
.shdr{
  background:linear-gradient(90deg,#0c1830,#101e40);
  border-bottom:1px solid #1a2840;
  padding:9px 14px 7px;flex-shrink:0;
}
.shdr-title{
  color:#00d4ff;font-size:1.0em;font-weight:700;
  letter-spacing:3px;text-shadow:0 0 10px rgba(0,191,255,.4);
  display:block;
}
.shdr-hint{
  color:#354860;font-size:.62em;font-family:'Courier New',monospace;
  letter-spacing:.5px;margin-top:2px;display:block;
}
.sbody{overflow-y:auto;padding:6px 12px 10px;flex:1;overscroll-behavior:contain}
.ss-title{
  color:#3090b8;font-size:.68em;font-weight:700;letter-spacing:2px;
  margin:12px 0 4px;padding-bottom:3px;border-bottom:1px solid #152030;
}
.ss-title:first-child{margin-top:4px}
.sf-row{
  display:flex;align-items:center;justify-content:space-between;
  padding:5px 5px;border-radius:5px;border:1px solid transparent;gap:6px;
  transition:border-color .13s,background .13s;
}
.sf-row.focused{
  background:rgba(0,80,160,.20);border-color:#0080c0;
  box-shadow:0 0 7px rgba(0,128,192,.28);
}
.sf-row.sf-hidden{display:none}
.sf-lbl{color:#90a8c0;font-size:.76em;min-width:108px;flex-shrink:0}
.sf-eval{
  color:#00d4ff;font-size:.8em;font-family:'Courier New',monospace;
  font-weight:700;min-width:64px;text-align:right;
}
.sf-inp{
  background:#080f20;border:1px solid #1e3450;border-radius:4px;
  color:#00d4ff;font-family:'Courier New',monospace;font-size:.79em;
  padding:3px 6px;width:130px;outline:none;
}
.sf-inp:focus{border-color:#0090d0;box-shadow:0 0 5px rgba(0,144,208,.35)}
.sf-bool{display:flex;gap:4px}
.sf-bb{
  padding:3px 9px;border-radius:4px;border:1px solid #1e3450;
  background:#080f20;color:#4a6a80;font-size:.72em;font-weight:700;
  cursor:pointer;touch-action:manipulation;font-family:'Courier New',monospace;
  transition:background .1s,color .1s,border-color .1s;
}
.sf-bb.bb-on {background:#082010;border-color:#00a030;color:#00cc50}
.sf-bb.bb-off{background:#200808;border-color:#a02020;color:#cc3030}
.sftr{border-top:1px solid #152030;padding:7px 12px;display:flex;gap:8px;flex-shrink:0}
.sftr-btn{
  flex:1;padding:7px 0;border-radius:6px;border:none;
  font-size:.8em;font-weight:700;cursor:pointer;letter-spacing:1.5px;
  touch-action:manipulation;
}
.sftr-btn.sv {background:linear-gradient(180deg,#0d4020,#061808);color:#00cc50;border:1px solid #0a4020}
.sftr-btn.cx {background:linear-gradient(180deg,#280c0c,#140404);color:#cc3030;border:1px solid #3a1010}
</style>
</head>
<body>

<!-- Disconnected overlay (visible until SSE delivers connected=true) -->
<div class="no-bridge-overlay" id="overlay">
  <h2>NO BRIDGE</h2>
  <p>Connecting to inno-pilot-bridge\u2026</p>
  <p id="dots">\u25cf</p>
</div>

<div class="remote">

  <!-- OLED display panel -->
  <div class="oled">
    <div class="oled-title">Inno-Web-Remote</div>

    <!-- Rudder position bar -->
    <div class="rdr-bar">
      <div class="rdr-track">
        <div class="rdr-center-tick"></div>
        <div class="rdr-marker" id="rdr-marker"></div>
      </div>
    </div>

    <div class="oled-mode" id="oled-mode-row">MODE: <b id="o-mode">IDLE</b></div>

    <div class="oled-data">
      <span>CMD:&nbsp;<span id="o-cmd">---</span>&deg;</span>
      <span>RDR:&nbsp;<span id="o-rdr">---</span>%</span>
      <span>Head:&nbsp;<span id="o-hdg">---</span></span>
    </div>

    <div class="oled-status">
      <span id="o-ver">---</span>
      <span id="o-conn" class="warn">CONNECTING\u2026</span>
    </div>

  </div>

  <!-- Physical button row -->
  <div class="btn-row">
    <button class="hw-btn b1" data-cmd="BTN -10">&laquo;</button>
    <button class="hw-btn b2" data-cmd="BTN -1">&lsaquo;</button>
    <button class="hw-btn b3" data-cmd="BTN TOGGLE">|</button>
    <button class="hw-btn b4" data-cmd="BTN +1">&rsaquo;</button>
    <button class="hw-btn b5" data-cmd="BTN +10">&raquo;</button>
  </div>

  <!-- STOP button + 3-position mode toggle -->
  <div class="action-row">
    <div class="stop-wrap">
      <div class="stop-ring"></div>
      <button class="stop-btn" id="stop-btn">STOP</button>
    </div>

    <div class="mode-radio-group" id="mode-radio-group">
      <div class="mode-radio" data-action="auto">
        <span class="mrd-dot"></span><span class="mrd-lbl">AUTO</span>
      </div>
      <div class="mode-radio" data-action="remote">
        <span class="mrd-dot"></span><span class="mrd-lbl">REMOTE</span>
      </div>
      <div class="mode-radio" data-action="on">
        <span class="mrd-dot"></span><span class="mrd-lbl">ON</span>
      </div>
      <div class="mode-radio active" data-action="off">
        <span class="mrd-dot"></span><span class="mrd-lbl">OFF</span>
      </div>
    </div>
  </div>

  <!-- Ship's wheel (rotatable, active in MANUAL mode) -->
  <div class="wheel-section">
    <div class="wheel-wrap" id="wheel-wrap">
      <svg id="wheel-svg" viewBox="0 0 200 200" xmlns="http://www.w3.org/2000/svg">
        $$WHEEL_SVG$$
      </svg>
    </div>
    <div class="wheel-lbl">Rudder: <b id="wheel-pct">--</b>% &mdash; drag wheel in REMOTE mode</div>
  </div>

  <!-- Settings gear button — only active while toggle is in OFF position -->
  <div class="settings-footer">
    <button class="gear-btn" id="gear-btn" title="Settings (OFF mode only)">&#9881;</button>
  </div>

</div><!-- .remote -->

<!-- Settings warning toast -->
<div class="sw-toast" id="sw-toast">Settings only<br>available in OFF mode</div>

<!-- Settings overlay -->
<div class="sov hidden" id="sov">
  <div class="spanel">
    <div class="shdr">
      <span class="shdr-title">&#9881; SETTINGS</span>
      <span class="shdr-hint">Tap a field to edit &nbsp;&bull;&nbsp; SAVE to apply</span>
    </div>
    <div class="sbody" id="sbody">

      <div class="ss-title">NETWORK</div>
      <div class="sf-row" data-sfid="ip_mode">
        <span class="sf-lbl">IP Mode</span>
        <span class="sf-eval" id="sf-ip_mode">DHCP</span>
      </div>
      <div class="sf-row sf-hidden" data-sfid="ip">
        <span class="sf-lbl">IP Address</span>
        <input class="sf-inp" type="text" id="sf-ip" placeholder="192.168.x.x">
      </div>
      <div class="sf-row sf-hidden" data-sfid="mask">
        <span class="sf-lbl">Subnet Mask</span>
        <input class="sf-inp" type="text" id="sf-mask" placeholder="255.255.255.0">
      </div>
      <div class="sf-row sf-hidden" data-sfid="gateway">
        <span class="sf-lbl">Gateway</span>
        <input class="sf-inp" type="text" id="sf-gateway" placeholder="192.168.x.1">
      </div>
      <div class="sf-row sf-hidden" data-sfid="dns1">
        <span class="sf-lbl">DNS 1</span>
        <input class="sf-inp" type="text" id="sf-dns1" placeholder="8.8.8.8">
      </div>
      <div class="sf-row sf-hidden" data-sfid="dns2">
        <span class="sf-lbl">DNS 2</span>
        <input class="sf-inp" type="text" id="sf-dns2" placeholder="8.8.4.4">
      </div>

      <div class="ss-title">WI-FI</div>
      <div class="sf-row" data-sfid="ssid">
        <span class="sf-lbl">SSID</span>
        <input class="sf-inp" type="text" id="sf-ssid" placeholder="Network name">
      </div>
      <div class="sf-row" data-sfid="key">
        <span class="sf-lbl">Password</span>
        <input class="sf-inp" type="password" id="sf-key" placeholder="Wi-Fi key">
      </div>

      <div class="ss-title">VESSEL</div>
      <div class="sf-row" data-sfid="name">
        <span class="sf-lbl">Boat Name</span>
        <input class="sf-inp" type="text" id="sf-name" placeholder="My Boat">
      </div>
      <div class="sf-row" data-sfid="type">
        <span class="sf-lbl">Vessel Type</span>
        <span class="sf-eval" id="sf-type">SAIL</span>
      </div>
      <div class="sf-row" data-sfid="rudder_range_deg">
        <span class="sf-lbl">Rudder Range (&#176;)</span>
        <input class="sf-inp" type="number" id="sf-rudder_range_deg" min="10" max="60" step="1">
      </div>

      <div class="ss-title">CONNECTIONS &amp; FEATURES</div>
      <div class="sf-row" data-sfid="limit_switches">
        <span class="sf-lbl">Limit Switches</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="limit_switches" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="limit_switches" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="temp_sensor">
        <span class="sf-lbl">Temp Sensor</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="temp_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="temp_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="pi_voltage_sensor">
        <span class="sf-lbl">Pi Voltage</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="pi_voltage_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="pi_voltage_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="battery_voltage_sensor">
        <span class="sf-lbl">Battery Voltage</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="battery_voltage_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="battery_voltage_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="current_sensor">
        <span class="sf-lbl">Current Sensor</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="current_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="current_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="on_board_buttons">
        <span class="sf-lbl">On-Board Buttons</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="on_board_buttons" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="on_board_buttons" data-bval="false">OFF</button>
        </div>
      </div>

      <div class="ss-title">AUTOPILOT</div>
      <div class="sf-row" data-sfid="deadband_pct">
        <span class="sf-lbl">Deadband (%)</span>
        <input class="sf-inp" type="number" id="sf-deadband_pct" min="0.5" max="20" step="0.5">
      </div>
      <div class="sf-row" data-sfid="pgain">
        <span class="sf-lbl">P-Gain</span>
        <input class="sf-inp" type="number" id="sf-pgain" min="0.1" max="5.0" step="0.1">
      </div>
      <div class="sf-row" data-sfid="off_course_alarm_deg">
        <span class="sf-lbl">Off-Course Alarm (&#176;)</span>
        <input class="sf-inp" type="number" id="sf-off_course_alarm_deg" min="5" max="90" step="1">
      </div>
      <div class="sf-row" data-sfid="rudder_limit_port_pct">
        <span class="sf-lbl">Port Limit (%)</span>
        <input class="sf-inp" type="number" id="sf-rudder_limit_port_pct" min="0" max="45" step="1">
      </div>
      <div class="sf-row" data-sfid="rudder_limit_stbd_pct">
        <span class="sf-lbl">Stbd Limit (%)</span>
        <input class="sf-inp" type="number" id="sf-rudder_limit_stbd_pct" min="55" max="100" step="1">
      </div>

      <div class="ss-title">SAFETY</div>
      <div class="sf-row" data-sfid="auto_disengage_on_fault">
        <span class="sf-lbl">Auto-Disengage</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="auto_disengage_on_fault" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="auto_disengage_on_fault" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="comms_warn_threshold_pct">
        <span class="sf-lbl">Comms WARN (%)</span>
        <input class="sf-inp" type="number" id="sf-comms_warn_threshold_pct" min="1" max="50" step="1">
      </div>
      <div class="sf-row" data-sfid="comms_crit_threshold_pct">
        <span class="sf-lbl">Comms CRIT (%)</span>
        <input class="sf-inp" type="number" id="sf-comms_crit_threshold_pct" min="5" max="90" step="1">
      </div>

    </div><!-- .sbody -->
    <div class="sftr">
      <button class="sftr-btn sv"  id="sov-save">SAVE</button>
      <button class="sftr-btn cx" id="sov-cancel">CANCEL</button>
    </div>
  </div><!-- .spanel -->
</div><!-- .sov -->

<script>
'use strict';

// ── Shared state ──────────────────────────────────────────────────────────
var gMode      = 'IDLE';
var gConnected = false;
var gApOn      = false;      // true when AP is actually engaged (d.ap === 1)
var gTogglePos = 'off';      // mode radio position: 'auto' | 'remote' | 'on' | 'off'
var wheelAngle = 0;      // accumulated rotation in degrees, clamped to ±MAX_DEG
var isDragging = false;
var prevPtrAngle = null;
var lastRudSend  = 0;
var MAX_DEG = 150;       // ±150° maps 0..100% rudder
var gManualRudPct = 50.0;  // commanded rudder % in MANUAL mode (0–100%)
var gRdrPct       = null;  // latest rdr_pct from bridge telemetry
var gJogTimer     = null;  // setInterval handle for hold-jog repeat
var gJogHoldTimer = null;  // setTimeout handle for jog hold-delay
var gSettings     = {};    // settings loaded from /settings endpoint
var gSettingsOpen = false; // true while settings panel is visible

// ── Command sender ────────────────────────────────────────────────────────
function sendCmd(cmd) {
  fetch('/command', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({cmd: cmd})
  }).catch(function() {});
}

// ── SSE subscription ──────────────────────────────────────────────────────
var es = new EventSource('/events');
es.onmessage = function(e) { updateUI(JSON.parse(e.data)); };
es.onerror   = function()  { setConnected(false); };

// ── UI updater ────────────────────────────────────────────────────────────
function updateUI(d) {
  gConnected = !!d.connected;
  gMode      = d.mode || 'IDLE';
  gApOn      = !!d.ap;
  gRdrPct    = d.rdr_pct != null ? d.rdr_pct : null;

  // Suppress overlay when connected, when mode is OFF (intentional disconnect),
  // or when the toggle has left OFF but the bridge hasn't reconnected yet.
  setConnected(gConnected || gMode === 'OFF' || gTogglePos !== 'off');

  // OLED mode line — AUTO: show AP ON/OFF with oversized "AP"; other modes: plain label
  var modeRow = document.getElementById('oled-mode-row');
  if (gMode === 'AP' && gApOn) {
    modeRow.innerHTML = 'MODE: <span class="ap-label">AP</span>\u00a0ON';
  } else {
    modeRow.innerHTML = 'MODE: <b id="o-mode">' + (gMode || 'IDLE') + '</b>';
  }

  // Heading / RDR / CMD — always displayed
  document.getElementById('o-hdg').textContent =
    d.hdg != null ? d.hdg.toFixed(1) : '---';
  document.getElementById('o-rdr').textContent =
    d.rdr_pct != null ? d.rdr_pct.toFixed(1) : '---';
  document.getElementById('o-cmd').textContent =
    d.cmd != null ? d.cmd.toFixed(1) : '---';

  // Rudder position bar (uses rdr_pct; falls back to 50% centre)
  var pct = d.rdr_pct != null ? Math.max(0, Math.min(100, d.rdr_pct)) : 50;
  document.getElementById('rdr-marker').style.left = pct + '%';

  // Version + comms status
  document.getElementById('o-ver').textContent = d.bridge_ver || d.version || '---';
  var connEl = document.getElementById('o-conn');
  if (d.connected) {
    var comms = (d.comms || 'OK').toUpperCase();
    if (comms === 'CRIT') {
      connEl.textContent = 'COMMS CRIT';
      connEl.className = 'crit';
    } else if (comms === 'WARN') {
      connEl.textContent = 'COMMS WARN';
      connEl.className = 'warn';
    } else {
      connEl.textContent = 'CONNECTED';
      connEl.className = 'ok';
    }
  } else if (gMode === 'OFF') {
    connEl.textContent = 'OFFLINE';
    connEl.className = 'warn';   // amber — intentional, not a fault
  } else {
    connEl.textContent = 'NO BRIDGE';
    connEl.className = 'crit';
  }

  // Mode radio selector — driven by physical position, not bridge mode
  setToggle(gTogglePos);
  // Blank the OLED content panel when in OFF mode
  document.querySelector('.oled').classList.toggle('blank-mode', gTogglePos === 'off');

  // Wheel active state + position sync
  var ww = document.getElementById('wheel-wrap');
  if (gMode === 'MANUAL') {
    ww.classList.add('active');
    // In MANUAL: keep wheel at drag position, don't override with telemetry
  } else {
    ww.classList.remove('active');
    // Sync wheel visual to actual rudder position
    if (d.rdr_pct != null) {
      wheelAngle = (d.rdr_pct - 50) / 50 * MAX_DEG;
      document.getElementById('wheel-svg').style.transform =
        'rotate(' + wheelAngle + 'deg)';
    }
    document.getElementById('wheel-pct').textContent =
      d.rdr_pct != null ? d.rdr_pct.toFixed(0) : '--';
  }
}

function setConnected(ok) {
  document.getElementById('overlay').classList.toggle('hidden', ok);
}

function setToggle(m) {
  // Sync animated radio button active state
  document.querySelectorAll('.mode-radio').forEach(function(el) {
    el.classList.toggle('active', el.dataset.action === m);
  });

  // Physical button labels — shown in AUTO and REMOTE modes only
  var btns = [
    document.querySelector('.hw-btn.b1'),
    document.querySelector('.hw-btn.b2'),
    document.querySelector('.hw-btn.b3'),
    document.querySelector('.hw-btn.b4'),
    document.querySelector('.hw-btn.b5'),
  ];
  if (m === 'auto') {
    btns[0].textContent = '-10';
    btns[1].textContent = '-1';
    btns[2].textContent = 'Go';
    btns[3].textContent = '+1';
    btns[4].textContent = '+10';
  } else if (m === 'remote') {
    btns[0].innerHTML = '&laquo;';
    btns[1].innerHTML = '&lsaquo;';
    btns[2].innerHTML = '|';
    btns[3].innerHTML = '&rsaquo;';
    btns[4].innerHTML = '&raquo;';
  } else {
    // OFF and ON: no button labels
    btns.forEach(function(b) { b.textContent = ''; });
  }
}

// ── Manual-mode jog helpers ───────────────────────────────────────────────
// jogRudder: adjust gManualRudPct by deltaPct, sync wheel visual, send RUD.
function jogRudder(deltaPct) {
  gManualRudPct = Math.max(0, Math.min(100, gManualRudPct + deltaPct));
  wheelAngle = (gManualRudPct - 50) / 50 * MAX_DEG;
  wheelSvg.style.transform = 'rotate(' + wheelAngle + 'deg)';
  document.getElementById('wheel-pct').textContent = Math.round(gManualRudPct);
  sendCmd('RUD ' + gManualRudPct.toFixed(1));
}

// startJog: immediate first jog, then after 300 ms hold delay repeat at intervalMs.
function startJog(deltaPct, intervalMs) {
  stopJog();
  jogRudder(deltaPct);
  gJogHoldTimer = setTimeout(function() {
    gJogHoldTimer = null;
    gJogTimer = setInterval(function() {
      if (gMode !== 'MANUAL') { stopJog(); return; }
      // Stop repeating when the limit is already reached
      var next = gManualRudPct + deltaPct;
      if (next < 0 || next > 100) { stopJog(); return; }
      jogRudder(deltaPct);
    }, intervalMs);
  }, 300);
}

function stopJog() {
  if (gJogTimer     !== null) { clearInterval(gJogTimer);     gJogTimer     = null; }
  if (gJogHoldTimer !== null) { clearTimeout(gJogHoldTimer);  gJogHoldTimer = null; }
}

// ── Button wiring ─────────────────────────────────────────────────────────
// MANUAL mode: B1=port 10% jog (hold=continuous), B2=port 1° jog (hold=500ms),
//              B3=centre rudder, B4=stbd 1° (hold=500ms), B5=stbd 10% (hold=continuous).
// AUTO/IDLE mode: send BTN commands as before.
(function() {
  var btnCfg = [
    { cls: 'b1', cmd: 'BTN -10',     delta: -10, interval: 200 },
    { cls: 'b2', cmd: 'BTN -1',      delta: -1,  interval: 500 },
    { cls: 'b3', cmd: 'BTN TOGGLE',  delta: null },
    { cls: 'b4', cmd: 'BTN +1',      delta: 1,   interval: 500 },
    { cls: 'b5', cmd: 'BTN +10',     delta: 10,  interval: 200 },
  ];

  btnCfg.forEach(function(cfg) {
    var el = document.querySelector('.hw-btn.' + cfg.cls);
    if (!el) return;

    function onPress() {
      if (gMode === 'MANUAL') {
        if (cfg.delta === null) {
          // B3: centre rudder immediately
          stopJog();
          gManualRudPct = 50.0;
          wheelAngle    = 0;
          wheelSvg.style.transform = 'rotate(0deg)';
          document.getElementById('wheel-pct').textContent = '50';
          sendCmd('RUD 50.0');
        } else {
          startJog(cfg.delta, cfg.interval);
        }
      } else {
        sendCmd(cfg.cmd);
      }
    }

    function onRelease() {
      // Only stop jog for directional buttons; centre has no hold behaviour
      if (gMode === 'MANUAL' && cfg.delta !== null) stopJog();
    }

    el.addEventListener('mousedown',   function(e) { e.preventDefault(); onPress(); });
    el.addEventListener('mouseup',     onRelease);
    el.addEventListener('mouseleave',  onRelease);
    el.addEventListener('touchstart',  function(e) { e.preventDefault(); onPress(); }, {passive: false});
    el.addEventListener('touchend',    onRelease);
    el.addEventListener('touchcancel', onRelease);
  });
}());

document.getElementById('stop-btn').addEventListener('click', function() {
  sendCmd('ESTOP');
});
document.getElementById('stop-btn').addEventListener('touchstart', function(e) {
  e.preventDefault(); sendCmd('ESTOP');
}, {passive: false});

// Mode radio button clicks
document.querySelectorAll('.mode-radio').forEach(function(el) {
  el.addEventListener('click', function() { handleToggleAction(el.dataset.action); });
  el.addEventListener('touchstart', function(e) {
    e.preventDefault(); handleToggleAction(el.dataset.action);
  }, {passive: false});
});

function handleToggleAction(action) {
  if (action === 'auto') {
    // Enter AUTO-ready state; AP starts OFF.  User presses Go to engage/disengage AP.
    gTogglePos = 'auto';
    if (gMode !== 'AP') sendCmd('MODE AUTO');

  } else if (action === 'off') {
    if (gApOn)                    sendCmd('BTN TOGGLE');  // disengage AP before disconnect
    if (gTogglePos === 'remote')  sendCmd('MODE AUTO');   // exit REMOTE before disconnect
    gTogglePos = 'off';
    // Delay gives the preceding command time to reach the bridge before the TCP
    // connection is dropped by MODE OFF (bridge loop polls every ~100 ms).
    setTimeout(function() { sendCmd('MODE OFF'); }, 300);

  } else if (action === 'on') {
    // Observer mode: connect (or reconnect), show telemetry, no steering control.
    if (gApOn)                    sendCmd('BTN TOGGLE');  // disengage AP
    if (gTogglePos === 'remote')  sendCmd('MODE AUTO');   // exit REMOTE steering → IDLE
    gTogglePos = 'on';
    // Send MODE ON to wake the bridge thread if it was idle due to mode OFF;
    // the server handles it locally without forwarding a control command.
    sendCmd('MODE ON');

  } else if (action === 'remote') {
    // REMOTE mode: helm wheel lights up, skipper commands the rudder.
    gTogglePos = 'remote';
    if (gMode !== 'MANUAL') {
      sendCmd('MODE MANUAL');
      // Initialise commanded position from actual rudder (or midships if unknown)
      gManualRudPct = gRdrPct !== null ? gRdrPct : 50.0;
      wheelAngle = (gManualRudPct - 50) / 50 * MAX_DEG;
      document.getElementById('wheel-svg').style.transform = 'rotate(' + wheelAngle + 'deg)';
      document.getElementById('wheel-pct').textContent = Math.round(gManualRudPct);
    }
  }
}

// ── Ship's wheel drag interaction ─────────────────────────────────────────
var wheelWrap = document.getElementById('wheel-wrap');
var wheelSvg  = document.getElementById('wheel-svg');

function wheelCenter() {
  var r = wheelWrap.getBoundingClientRect();
  return {x: r.left + r.width / 2, y: r.top + r.height / 2};
}

function ptrAngleDeg(cx, cy) {
  var c = wheelCenter();
  return Math.atan2(cy - c.y, cx - c.x) * 180 / Math.PI;
}

function wheelStart(cx, cy) {
  isDragging   = true;
  prevPtrAngle = ptrAngleDeg(cx, cy);
}

function wheelMove(cx, cy) {
  if (!isDragging || gMode !== 'MANUAL') return;
  var curr  = ptrAngleDeg(cx, cy);
  var delta = curr - prevPtrAngle;
  // Unwrap angle discontinuity at ±180°
  if (delta >  180) delta -= 360;
  if (delta < -180) delta += 360;
  prevPtrAngle = curr;

  wheelAngle = Math.max(-MAX_DEG, Math.min(MAX_DEG, wheelAngle + delta));
  wheelSvg.style.transform = 'rotate(' + wheelAngle + 'deg)';

  var pct = (wheelAngle + MAX_DEG) / (2 * MAX_DEG) * 100;
  gManualRudPct = pct;  // keep button jog state in sync with wheel drag
  document.getElementById('wheel-pct').textContent = Math.round(pct);

  // Send RUD at max 5 Hz
  var now = Date.now();
  if (now - lastRudSend >= 200) {
    sendCmd('RUD ' + pct.toFixed(1));
    lastRudSend = now;
  }
}

function wheelEnd() {
  isDragging   = false;
  prevPtrAngle = null;
}

// Mouse events
wheelWrap.addEventListener('mousedown', function(e) {
  e.preventDefault(); wheelStart(e.clientX, e.clientY);
});
window.addEventListener('mousemove', function(e) { wheelMove(e.clientX, e.clientY); });
window.addEventListener('mouseup',   function()  { wheelEnd(); });

// Touch events
wheelWrap.addEventListener('touchstart', function(e) {
  e.preventDefault(); wheelStart(e.touches[0].clientX, e.touches[0].clientY);
}, {passive: false});
window.addEventListener('touchmove', function(e) {
  if (isDragging) { e.preventDefault(); wheelMove(e.touches[0].clientX, e.touches[0].clientY); }
}, {passive: false});
window.addEventListener('touchend', function() { wheelEnd(); });

// ── Settings panel ───────────────────────────────────────────────────────────
// Field registry — drives population, B-button navigation, and bool/enum control.
// type: 'text'|'password'|'number'|'bool'|'enum'
// onVal/offVal: stored value toggled by tapping the enum field.
// dep: {id, val} — field hidden unless named field equals val.
var SF = [
  // Network
  {id:'ip_mode', sec:'network', type:'enum',     onVal:'static', offVal:'dhcp'},
  {id:'ip',      sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'mask',    sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'gateway', sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'dns1',    sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'dns2',    sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  // Wi-Fi
  {id:'ssid',    sec:'wifi',    type:'text'},
  {id:'key',     sec:'wifi',    type:'password'},
  // Vessel
  {id:'name',             sec:'vessel',   type:'text'},
  {id:'type',             sec:'vessel',   type:'enum',   onVal:'sail', offVal:'power'},
  {id:'rudder_range_deg', sec:'vessel',   type:'number'},
  // Features
  {id:'limit_switches',         sec:'features', type:'bool'},
  {id:'temp_sensor',            sec:'features', type:'bool'},
  {id:'pi_voltage_sensor',      sec:'features', type:'bool'},
  {id:'battery_voltage_sensor', sec:'features', type:'bool'},
  {id:'current_sensor',         sec:'features', type:'bool'},
  {id:'on_board_buttons',       sec:'features', type:'bool'},
  // Autopilot
  {id:'deadband_pct',           sec:'autopilot', type:'number'},
  {id:'pgain',                  sec:'autopilot', type:'number'},
  {id:'off_course_alarm_deg',   sec:'autopilot', type:'number'},
  {id:'rudder_limit_port_pct',  sec:'autopilot', type:'number'},
  {id:'rudder_limit_stbd_pct',  sec:'autopilot', type:'number'},
  // Safety
  {id:'auto_disengage_on_fault',  sec:'safety', type:'bool'},
  {id:'comms_warn_threshold_pct', sec:'safety', type:'number'},
  {id:'comms_crit_threshold_pct', sec:'safety', type:'number'},
];

function sfGet(f) { return (gSettings[f.sec] || {})[f.id]; }
function sfSet(f, v) {
  if (!gSettings[f.sec]) gSettings[f.sec] = {};
  gSettings[f.sec][f.id] = v;
}

// Returns visible (non-hidden) field list, honouring dep conditions.
function sfVisibleList() {
  return SF.filter(function(f) {
    if (!f.dep) return true;
    var d = SF.find(function(x) { return x.id === f.dep.id; });
    return d && String(sfGet(d)) === f.dep.val;
  });
}

// Show/hide rows for fields with dep conditions.
function sfSyncVisibility() {
  SF.forEach(function(f) {
    if (!f.dep) return;
    var d = SF.find(function(x) { return x.id === f.dep.id; });
    var show = d && String(sfGet(d)) === f.dep.val;
    var row = document.querySelector('.sf-row[data-sfid="' + f.id + '"]');
    if (row) row.classList.toggle('sf-hidden', !show);
  });
}

// Render ON/OFF button styles for a bool field.
function sfBoolRender(fid, isOn) {
  document.querySelectorAll('.sf-bb[data-boolid="' + fid + '"]').forEach(function(b) {
    b.classList.remove('bb-on', 'bb-off');
    if (b.dataset.bval === 'true'  &&  isOn) b.classList.add('bb-on');
    if (b.dataset.bval === 'false' && !isOn) b.classList.add('bb-off');
  });
}

// Populate all controls from gSettings.
function sfApplyToUI() {
  SF.forEach(function(f) {
    var v = sfGet(f);
    if (v === undefined || v === null) return;
    if (f.type === 'bool') {
      sfBoolRender(f.id, !!v);
    } else if (f.type === 'enum') {
      var el = document.getElementById('sf-' + f.id);
      if (el) el.textContent = String(v).toUpperCase();
    } else {
      var el = document.getElementById('sf-' + f.id);
      if (el) el.value = v;
    }
  });
}

// Read text/number/password inputs back into gSettings.
// (bool and enum values are written directly to gSettings on interaction.)
function sfCollect() {
  SF.forEach(function(f) {
    if (f.type === 'bool' || f.type === 'enum') return;
    var el = document.getElementById('sf-' + f.id);
    if (!el) return;
    if (!gSettings[f.sec]) gSettings[f.sec] = {};
    gSettings[f.sec][f.id] = (f.type === 'number') ? (parseFloat(el.value) || 0) : el.value;
  });
}


function openSettings() {
  if (gTogglePos !== 'off') {
    // Flash warning — settings only accessible in OFF mode
    var t = document.getElementById('sw-toast');
    t.classList.add('visible');
    setTimeout(function() { t.classList.remove('visible'); }, 2200);
    return;
  }
  gSettingsOpen = true;
  document.getElementById('gear-btn').classList.add('settings-open');
  document.getElementById('sov').classList.remove('hidden');
  setToggle('off'); // re-renders buttons (override inside setToggle shows settings labels)
  fetch('/settings')
    .then(function(r) { return r.json(); })
    .then(function(s) {
      gSettings = s;
      sfApplyToUI();
      sfSyncVisibility();
    })
    .catch(function() {});
}

function closeSettings(save) {
  if (save) {
    sfCollect();
    fetch('/settings', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(gSettings)
    }).catch(function() {});
  }
  gSettingsOpen = false;
  document.getElementById('gear-btn').classList.remove('settings-open');
  document.getElementById('sov').classList.add('hidden');
  setToggle('off'); // restore button labels (empty in OFF mode, since gSettingsOpen is now false)
}

// Settings event wiring
document.getElementById('gear-btn').addEventListener('click', openSettings);
document.getElementById('gear-btn').addEventListener('touchstart', function(e) {
  e.preventDefault(); openSettings();
}, {passive:false});
document.getElementById('sov-save').addEventListener('click',   function() { closeSettings(true); });
document.getElementById('sov-cancel').addEventListener('click', function() { closeSettings(false); });

// Bool buttons: direct click updates gSettings immediately
document.querySelectorAll('.sf-bb').forEach(function(b) {
  b.addEventListener('click', function() {
    var fid = b.dataset.boolid;
    var val = b.dataset.bval === 'true';
    var f = SF.find(function(x) { return x.id === fid; });
    if (f) { sfSet(f, val); sfBoolRender(fid, val); }
  });
});

// Field row click: focus the input for text/number/password fields
document.querySelectorAll('.sf-row').forEach(function(row) {
  row.addEventListener('click', function(e) {
    var inp = row.querySelector('.sf-inp');
    if (inp && e.target !== inp) inp.focus();
  });
});
// Enum field: tap the displayed value to toggle between the two options
document.querySelectorAll('.sf-eval').forEach(function(el) {
  el.style.cursor = 'pointer';
  el.addEventListener('click', function(e) {
    e.stopPropagation();
    var row = el.parentElement;
    while (row && !row.classList.contains('sf-row')) { row = row.parentElement; }
    if (!row) return;
    var fid = row.dataset.sfid;
    var f = SF.find(function(x) { return x.id === fid; });
    if (!f || f.type !== 'enum') return;
    var nv = (sfGet(f) === f.onVal) ? f.offVal : f.onVal;
    sfSet(f, nv);
    el.textContent = nv.toUpperCase();
    sfSyncVisibility();
  });
});

// ── No-bridge overlay animation ───────────────────────────────────────────
var dotN = 1;
setInterval(function() {
  dotN = (dotN % 3) + 1;
  var d = document.getElementById('dots');
  if (d) d.textContent = '\u25cf'.repeat(dotN);
}, 550);
</script>
</body>
</html>""".replace("$$WHEEL_SVG$$", _WHEEL_SVG)

# ---------------------------------------------------------------------------
# HTTP request handler
# ---------------------------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):  # suppress default access log noise
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_page()
        elif self.path == "/events":
            self._serve_sse()
        elif self.path == "/health":
            self._serve_health()
        elif self.path == "/settings":
            self._serve_settings()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/command":
            self._handle_command()
        elif self.path == "/settings":
            self._handle_settings_post()
        else:
            self.send_error(404)

    # ---- GET / ----

    def _serve_page(self) -> None:
        body = _HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    # ---- GET /events (SSE) ----

    def _serve_sse(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type",    "text/event-stream")
        self.send_header("Cache-Control",   "no-cache")
        self.send_header("Connection",      "keep-alive")
        self.send_header("X-Accel-Buffering", "no")  # disable nginx buffering
        self.end_headers()

        client_q: queue.Queue = queue.Queue(maxsize=60)
        with _sse_lock:
            _sse_subs.append(client_q)

        try:
            # Send current state immediately on connect
            snap = _snap()
            self.wfile.write(f"data: {json.dumps(snap)}\n\n".encode())
            self.wfile.flush()

            while True:
                try:
                    event = client_q.get(timeout=15)
                    self.wfile.write(f"data: {json.dumps(event)}\n\n".encode())
                    self.wfile.flush()
                except queue.Empty:
                    # SSE keepalive comment (prevents proxy/browser timeout)
                    self.wfile.write(b": ka\n\n")
                    self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with _sse_lock:
                try:
                    _sse_subs.remove(client_q)
                except ValueError:
                    pass

    # ---- POST /command ----

    def _handle_command(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length) if length else b""
        try:
            cmd_str: str = str(json.loads(body)["cmd"])
        except (json.JSONDecodeError, KeyError, ValueError, TypeError):
            self.send_response(400)
            self.end_headers()
            return

        tok = cmd_str.upper().split()

        # ── MODE OFF: intentional TCP disconnect ──────────────────────────────
        # Handled entirely locally — do not forward to bridge.
        if len(tok) >= 2 and tok[0] == "MODE" and tok[1] == "OFF":
            _bridge_active.clear()          # signal bridge thread to drop connection
            # Drain any pending commands so nothing sneaks through after disconnect
            while True:
                try:
                    _cmd_q.get_nowait()
                except queue.Empty:
                    break
            _update(
                connected=False, hdg=None, cmd=None,
                rdr=None, rdr_pct=None, ap=0,
                mode="OFF", comms="OK", warn=None,
            )
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
            return

        # ── MODE ON: observer mode — reconnect without taking control ─────────
        # Re-enables the bridge thread without forwarding any mode command to
        # the bridge itself.  Bridge stays in IDLE; web remote observes only.
        if len(tok) >= 2 and tok[0] == "MODE" and tok[1] == "ON":
            _bridge_active.set()          # wake bridge thread if idle (was mode OFF)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
            return

        # ── Any active command: ensure bridge is (re)connected ────────────────
        # This wakes the bridge thread if it was idle due to mode OFF.
        _bridge_active.set()

        # Optimistic local state update for instant UI feedback before
        # bridge confirms via telemetry (bridge updates within ~200 ms)
        if tok and tok[0] == "ESTOP":
            _update(mode="IDLE", ap=0)
        elif len(tok) >= 2 and tok[0] == "MODE":
            if tok[1] == "MANUAL":
                _update(mode="MANUAL")
            elif tok[1] == "AUTO":
                _update(mode="IDLE")
        elif len(tok) >= 2 and tok[0] == "BTN" and tok[1] == "TOGGLE":
            with _state_lock:
                cur_ap   = _state["ap"]
                cur_mode = _state["mode"]
            if cur_mode != "MANUAL":
                if cur_ap:
                    _update(ap=0, mode="IDLE")
                else:
                    _update(ap=1, mode="AP")

        # Forward command to bridge via _cmd_q (bridge thread reads this)
        try:
            _cmd_q.put_nowait(cmd_str)
        except queue.Full:
            log.warning("Command queue full, dropping: %s", cmd_str)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"ok":true}')

    # ---- GET /health ----

    def _serve_health(self) -> None:
        snap = _snap()
        body = json.dumps({
            "status":           "ok",
            "bridge_connected": snap["connected"],
            "mode":             snap["mode"],
        }).encode()
        self.send_response(200)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ---- GET /settings ----

    def _serve_settings(self) -> None:
        """Fetch current settings from the bridge via TCP.

        Sends SETTINGS GET through the command queue and waits for the bridge
        to reply with SETTINGS <json>.  Falls back to the local settings file
        when the bridge is not connected or the response times out.
        """
        # Drain any stale responses from a previous request
        while True:
            try:
                _settings_resp_q.get_nowait()
            except queue.Empty:
                break

        data: Optional[dict] = None
        if _snap()["connected"]:
            try:
                _cmd_q.put_nowait("SETTINGS GET")
            except queue.Full:
                log.warning("Settings GET: cmd_q full, falling back to file")
            else:
                try:
                    resp = _settings_resp_q.get(timeout=SETTINGS_TIMEOUT_S)
                    # If it's a dict without an _ack key it's the GET response
                    if isinstance(resp, dict) and "_ack" not in resp:
                        data = resp
                        log.debug("Settings fetched from bridge (%d keys)", len(resp))
                    else:
                        log.warning("Settings GET: unexpected response %s", resp)
                except queue.Empty:
                    log.warning("Settings GET: bridge timed out, using local file")

        if data is None:
            data = _load_settings()

        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control",  "no-cache")
        self.end_headers()
        self.wfile.write(body)

    # ---- POST /settings ----

    def _handle_settings_post(self) -> None:
        """Send new settings to the bridge via TCP; bridge saves and applies them.

        Falls back to writing the local settings file when not connected.
        """
        length = int(self.headers.get("Content-Length", 0))
        raw    = self.rfile.read(length) if length else b""
        try:
            settings = json.loads(raw)
            if not isinstance(settings, dict):
                raise ValueError("expected JSON object")
        except Exception as exc:
            log.warning("Settings POST: bad request — %s", exc)
            self.send_response(400)
            self.end_headers()
            return

        saved_via_bridge = False
        if _snap()["connected"]:
            # Drain stale responses
            while True:
                try:
                    _settings_resp_q.get_nowait()
                except queue.Empty:
                    break

            # Compact JSON — must be a single line (no embedded newlines)
            json_line = json.dumps(settings, separators=(",", ":"), ensure_ascii=True)
            try:
                _cmd_q.put_nowait(f"SETTINGS SET {json_line}")
            except queue.Full:
                log.warning("Settings POST: cmd_q full, falling back to file")
            else:
                try:
                    resp = _settings_resp_q.get(timeout=SETTINGS_TIMEOUT_S)
                    if isinstance(resp, dict) and resp.get("_ack") == "OK":
                        saved_via_bridge = True
                        log.debug("Settings saved via bridge")
                    else:
                        log.warning("Settings POST: bridge replied %s", resp)
                except queue.Empty:
                    log.warning("Settings POST: bridge timed out, falling back to file")

        if not saved_via_bridge:
            # Bridge unavailable — write local file so settings survive reconnect
            _persist_settings(settings)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"ok":true}')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    # Start bridge client thread
    t = threading.Thread(target=bridge_client, daemon=True, name="bridge-client")
    t.start()
    log.info("Bridge client thread started (target: %s:%d)", BRIDGE_HOST, BRIDGE_PORT)

    # Start HTTP server
    server = ThreadingHTTPServer(("", WEB_PORT), _Handler)
    log.info("Web remote listening on http://0.0.0.0:%d/", WEB_PORT)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down")


if __name__ == "__main__":
    main()
