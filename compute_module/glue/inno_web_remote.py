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
INNOPILOT_VERSION = "v1.2.0_B9"

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

.oled-mode{color:#ccc;font-size:0.8em;letter-spacing:1px}
.oled-mode b{color:#00d4ff}
.oled-data{
  color:#00d4ff;
  font-size:0.76em;
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
.oled-status{
  color:#007aaa;
  font-size:0.63em;
  text-align:center;
  letter-spacing:1px;
  margin-top:2px;
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

/* ── 3-position toggle switch ── */
.toggle-assembly{display:flex;align-items:center;gap:14px;flex-direction:row-reverse}
.toggle-track{
  width:38px;
  height:126px;
  background:linear-gradient(to bottom,#b8b8b8,#888888);
  border-radius:19px;
  border:2px solid #686868;
  position:relative;
  box-shadow:inset 0 2px 5px rgba(0,0,0,0.45),0 2px 4px rgba(0,0,0,0.2);
  cursor:pointer;
  flex-shrink:0;
}
/* Detent marks */
.toggle-track::before,.toggle-track::after{
  content:'';
  position:absolute;
  left:5px;
  right:5px;
  height:2px;
  background:rgba(0,0,0,0.25);
}
.toggle-track::before{top:43px}
.toggle-track::after{top:87px}

.toggle-lever{
  position:absolute;
  width:34px;
  height:34px;
  border-radius:50%;
  background:radial-gradient(circle at 38% 32%,#eeeeee 0%,#bbbbbb 60%,#999999 100%);
  left:-1px;
  box-shadow:0 2px 6px rgba(0,0,0,0.55);
  transition:top .22s cubic-bezier(.34,1.56,.64,1);
  top:46px; /* default: OFF */
}
.toggle-lever.pos-auto  {top:4px}
.toggle-lever.pos-off   {top:46px}
.toggle-lever.pos-manual{top:88px}

.toggle-labels{
  display:flex;
  flex-direction:column;
  justify-content:space-between;
  height:126px;
}
.tgl-lbl{
  font-size:0.7em;
  font-weight:600;
  color:#999;
  cursor:pointer;
  letter-spacing:0.5px;
  line-height:1.1;
  transition:color .15s;
}
.tgl-lbl:hover{color:#444}
.tgl-lbl.active{color:#111;font-weight:800}

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
    <div class="oled-title">Inno-Remote</div>

    <!-- Rudder position bar -->
    <div class="rdr-bar">
      <div class="rdr-track">
        <div class="rdr-center-tick"></div>
        <div class="rdr-marker" id="rdr-marker"></div>
      </div>
    </div>

    <div class="oled-mode">MODE: <b id="o-mode">IDLE</b></div>

    <div class="oled-data">
      <span>Head:&nbsp;<span id="o-hdg">---</span></span>
      <span><span id="o-right-pre">RDR:&nbsp;</span><span id="o-right-val">---</span><span id="o-right-suf">%</span></span>
    </div>

    <!-- OLED button row (same commands as physical buttons) -->
    <div class="oled-btns">
      <button class="oled-btn" data-cmd="BTN -10">&laquo;</button>
      <button class="oled-btn" data-cmd="BTN -1">&lsaquo;</button>
      <button class="oled-btn" data-cmd="BTN TOGGLE">|</button>
      <button class="oled-btn" data-cmd="BTN +1">&rsaquo;</button>
      <button class="oled-btn" data-cmd="BTN +10">&raquo;</button>
    </div>

    <div class="oled-status">
      <span id="o-ver">---</span>&nbsp;|&nbsp;<span id="o-conn" class="warn">CONNECTING\u2026</span>
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

    <div class="toggle-assembly">
      <div class="toggle-track" id="toggle-track">
        <div class="toggle-lever pos-off" id="toggle-lever"></div>
      </div>
      <div class="toggle-labels">
        <span class="tgl-lbl"        id="lbl-auto"   data-action="auto">AUTO</span>
        <span class="tgl-lbl active" id="lbl-off"    data-action="off">OFF</span>
        <span class="tgl-lbl"        id="lbl-manual" data-action="manual">MANUAL</span>
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
    <div class="wheel-lbl">Rudder: <b id="wheel-pct">--</b>% &mdash; drag wheel in MANUAL mode</div>
  </div>

</div><!-- .remote -->

<script>
'use strict';

// ── Shared state ──────────────────────────────────────────────────────────
var gMode      = 'IDLE';
var gConnected = false;
var wheelAngle = 0;      // accumulated rotation in degrees, clamped to ±MAX_DEG
var isDragging = false;
var prevPtrAngle = null;
var lastRudSend  = 0;
var MAX_DEG = 150;       // ±150° maps 0..100% rudder

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

  // Suppress "NO BRIDGE" overlay when mode is OFF (intentional disconnect)
  setConnected(gConnected || gMode === 'OFF');

  // OLED mode line
  document.getElementById('o-mode').textContent = gMode;

  // Heading
  document.getElementById('o-hdg').textContent =
    d.hdg != null ? d.hdg.toFixed(1) : '---';

  // Right OLED field: CMD (degrees) in AP mode, RDR_PCT (%) otherwise
  var pre = document.getElementById('o-right-pre');
  var val = document.getElementById('o-right-val');
  var suf = document.getElementById('o-right-suf');
  if (gMode === 'AP' && d.cmd != null) {
    pre.textContent = 'CMD:\u00a0';
    val.textContent = d.cmd.toFixed(1);
    suf.textContent = '\u00b0';
  } else {
    pre.textContent = 'RDR:\u00a0';
    val.textContent = d.rdr_pct != null ? d.rdr_pct.toFixed(1) : '---';
    suf.textContent = '%';
  }

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

  // 3-position toggle
  setToggle(gMode);

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
  var lever  = document.getElementById('toggle-lever');
  var lblA   = document.getElementById('lbl-auto');
  var lblO   = document.getElementById('lbl-off');
  var lblM   = document.getElementById('lbl-manual');

  lever.className = 'toggle-lever';
  lblA.classList.remove('active');
  lblO.classList.remove('active');
  lblM.classList.remove('active');

  if (m === 'AP') {
    lever.classList.add('pos-auto');
    lblA.classList.add('active');
  } else if (m === 'MANUAL') {
    lever.classList.add('pos-manual');
    lblM.classList.add('active');
  } else {
    lever.classList.add('pos-off');
    lblO.classList.add('active');
  }
}

// ── Button wiring ─────────────────────────────────────────────────────────
document.querySelectorAll('[data-cmd]').forEach(function(el) {
  el.addEventListener('click', function() { sendCmd(el.dataset.cmd); });
  el.addEventListener('touchstart', function(e) {
    e.preventDefault(); sendCmd(el.dataset.cmd);
  }, {passive: false});
});

document.getElementById('stop-btn').addEventListener('click', function() {
  sendCmd('ESTOP');
});
document.getElementById('stop-btn').addEventListener('touchstart', function(e) {
  e.preventDefault(); sendCmd('ESTOP');
}, {passive: false});

// Toggle label clicks
document.querySelectorAll('.tgl-lbl').forEach(function(el) {
  el.addEventListener('click', function() { handleToggleAction(el.dataset.action); });
  el.addEventListener('touchstart', function(e) {
    e.preventDefault(); handleToggleAction(el.dataset.action);
  }, {passive: false});
});

// Toggle track click — position in track determines action
document.getElementById('toggle-track').addEventListener('click', function(e) {
  var rect = e.currentTarget.getBoundingClientRect();
  var frac = (e.clientY - rect.top) / rect.height;
  if      (frac < 0.33) handleToggleAction('auto');
  else if (frac > 0.67) handleToggleAction('manual');
  else                   handleToggleAction('off');
});

function handleToggleAction(action) {
  if (action === 'auto') {
    // Engage AP (only if not already in AP)
    if (gMode !== 'AP') sendCmd('BTN TOGGLE');

  } else if (action === 'off') {
    if (gMode === 'AP')     sendCmd('BTN TOGGLE');   // disengage AP before disconnect
    if (gMode === 'MANUAL') sendCmd('MODE AUTO');    // exit manual before disconnect
    // Delay gives the preceding command time to reach the bridge before the TCP
    // connection is dropped by MODE OFF (bridge loop polls every ~100 ms).
    setTimeout(function() { sendCmd('MODE OFF'); }, 300);

  } else if (action === 'manual') {
    if (gMode !== 'MANUAL') {
      sendCmd('MODE MANUAL');
      // Reset wheel to midships on entering manual
      wheelAngle = 0;
      document.getElementById('wheel-svg').style.transform = 'rotate(0deg)';
      document.getElementById('wheel-pct').textContent = '50';
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
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/command":
            self._handle_command()
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
