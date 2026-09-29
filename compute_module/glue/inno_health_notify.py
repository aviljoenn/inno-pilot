#!/usr/bin/env python3
"""
inno_health_notify.py — Inno-Pilot health monitoring and Telegram notifications.

Boot behaviour
--------------
Waits BOOT_SETTLE_S for services to stabilise, runs a 10-ping gateway test,
sends a full health snapshot via Telegram, then enters the periodic loop.
Every important event is logged (Python logging → journalctl) for audit.

Periodic behaviour
------------------
A single non-blocking loop runs one long-lived `ping` process (default every
1 s, settable 1-30 s in Settings, forced to 0.5 s while the remote is in Debug
mode).  Every ping result is recorded and evaluated the moment it arrives into
a WARN_WINDOW_S (60 s) sliding-window packet-loss monitor.  State changes
(WARN / CLEAR) are logged with structured markers and trigger Telegram alerts.
This measures the Pi -> WiFi router path only (outward pings); the
browser -> Pi path is measured separately by the web remote itself.

Packet-loss state machine
--------------------------
  OK  → WARN  when the WARN_WINDOW_S (60 s) moving avg > PACKET_LOSS_WARN_PCT (2 %)
  WARN→ OK    when that avg stays ≤ 2 % for CLEAR_WINDOW_S (60 s)
              clearance only accrues while pings are actually succeeding;
              100 % loss (gateway unreachable) does NOT count toward clearance.

All state transitions, anomalies and service faults are written to the Python
logger (captured by journalctl under the inno-health-notify unit).
Every tick the monitor also publishes its state to NET_STATUS_FILE (tmpfs) as
JSON; inno_web_remote.py reads that file to show the Pi -> router state on the
remote (replaces the old journal tail).
"""

import collections
import json
import os
import re
import select
import signal
import socket
import subprocess
import time
import urllib.request
from typing import Optional

# ---------------------------------------------------------------------------
# Config paths (must match inno_web_remote.py)
# ---------------------------------------------------------------------------
TELEGRAM_CONF = "/home/innopilot/.pypilot/telegram.conf"
SETTINGS_FILE = "/var/lib/inno-pilot/settings.json"

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
BOOT_SETTLE_S        = 45     # wait after systemd start before boot report
BOOT_PING_COUNT      = 10     # gateway pings in one-shot boot test
PING_INTERVAL_DEFAULT_S = 1.0 # standard Pi->router ping interval
PING_INTERVAL_MIN_S     = 1.0     # Settings range is 1..30 s
PING_INTERVAL_MAX_S     = 30.0
PING_INTERVAL_DEBUG_S   = 0.5     # forced while the remote is in Debug mode (finer metrics)
MIN_LOST_TO_WARN     = 2      # a single lost ping never alerts, however few samples yet
MIN_WINDOW_SAMPLES   = 20     # never judge loss on fewer samples than this; the window
                              # stretches to MIN_WINDOW_SAMPLES x interval for slow pings
CFG_POLL_S           = 1.0    # how often the loop re-reads settings / debug flag
GW_REFRESH_S         = 10.0   # how often the default gateway is re-checked
SAMPLE_S             = 60.0   # temperature / RSSI sampling period for period reports
LOOP_MAX_WAIT_S      = 0.25   # longest the loop ever waits => bounds shutdown latency
PACKET_LOSS_WARN_PCT = 2.0    # moving-avg threshold for WARN (2 % is acceptable for AP control)
WARN_WINDOW_S        = 60     # sliding window for moving average (was 10 min — too slow to react)
CLEAR_WINDOW_S       = 60     # time below threshold required to clear (was 30 min)
# tmpfs (RAM) so a 10 s rewrite cadence never wears the SD card.  World-readable.
NET_STATUS_FILE      = "/dev/shm/inno-pilot-net.json"
# Existence of this tmpfs file (touched by inno_web_remote.py) means Debug is on.
DEBUG_FLAG_FILE      = "/dev/shm/inno-pilot-debug"

INNO_SERVICES = [
    ("bridge",     "inno-pilot-bridge"),
    ("web-remote", "inno-pilot-web-remote"),
    ("socat",      "inno-pilot-socat"),
    ("autopilot",  "pypilot"),
    ("tailscale",  "tailscaled"),
    ("sshd",       "ssh"),
]

# ---------------------------------------------------------------------------
# Logging (captured by journalctl under inno-health-notify.service)
# ---------------------------------------------------------------------------
import logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("inno_health")

# ---------------------------------------------------------------------------
# Telegram helpers
# ---------------------------------------------------------------------------

def _read_conf() -> tuple:
    """Return (token, chat_id) from telegram.conf, or (None, None) on error."""
    try:
        with open(TELEGRAM_CONF) as fh:
            d = json.load(fh)
        return d.get("token"), str(d.get("chat_id", ""))
    except Exception:
        return None, None


def _vessel_name() -> str:
    """Return vessel name from settings, or empty string if not configured."""
    try:
        with open(SETTINGS_FILE) as fh:
            return json.load(fh).get("vessel", {}).get("name", "").strip()
    except Exception:
        return ""


def _send(text: str) -> None:
    """Fire-and-forget Telegram message; silent on any failure.
    Prepends vessel name so messages are identifiable per boat.
    """
    token, chat_id = _read_conf()
    if not token or not chat_id:
        return
    name = _vessel_name()
    if name:
        text = f"[{name}] {text}"
    try:
        data = json.dumps({"chat_id": chat_id, "text": text}).encode()
        req  = urllib.request.Request(
            f"https://api.telegram.org/bot{token}/sendMessage",
            data=data,
            headers={"Content-Type": "application/json"},
        )
        urllib.request.urlopen(req, timeout=10)
    except Exception as exc:
        log.warning("Telegram send failed: %s", exc)

# ---------------------------------------------------------------------------
# Settings helpers
# ---------------------------------------------------------------------------

def _read_interval() -> int:
    """Return notifications.health_interval_min from settings (default 0)."""
    try:
        with open(SETTINGS_FILE) as fh:
            s = json.load(fh)
        return max(0, int(s.get("notifications", {}).get("health_interval_min", 0)))
    except Exception:
        return 0


def _read_ping_interval() -> float:
    """Return notifications.ping_interval_s (default 1.0, clamped 1-30 s)."""
    try:
        with open(SETTINGS_FILE) as fh:
            s = json.load(fh)
        v = float(s.get("notifications", {}).get("ping_interval_s", PING_INTERVAL_DEFAULT_S))
        return max(PING_INTERVAL_MIN_S, min(PING_INTERVAL_MAX_S, v))
    except Exception:
        return PING_INTERVAL_DEFAULT_S

# ---------------------------------------------------------------------------
# Packet-loss sliding-window state machine
# ---------------------------------------------------------------------------

class _NetMonitor:
    """
    Per-ping sliding-window packet-loss monitor driving the OK <-> WARN state
    machine.  Every ping result (reply or timeout) is recorded and evaluated the
    moment it arrives via record().

    Each deque entry is (monotonic_ts, ok, rtt_ms_or_None).
    """

    def __init__(self) -> None:
        self._window: collections.deque = collections.deque()
        self._state   = "OK"
        self._interval = PING_INTERVAL_DEFAULT_S
        self._clear_start: Optional[float] = None  # when below-threshold run began

    def set_interval(self, seconds: float) -> None:
        """Tell the monitor the current ping interval (sizes the window)."""
        self._interval = seconds

    def window_s(self) -> float:
        """Window length: WARN_WINDOW_S, stretched so it always holds enough
        samples to be statistically meaningful at slow ping intervals."""
        return max(float(WARN_WINDOW_S), MIN_WINDOW_SAMPLES * self._interval)

    def reset(self) -> None:
        """Clean slate (e.g. gateway changed — old samples are about another path)."""
        self._window.clear()
        self._state = "OK"
        self._clear_start = None

    def lost_count(self) -> int:
        """Number of lost pings currently in the window."""
        return sum(1 for _, ok, _ in self._window if not ok)

    def moving_avg_pct(self) -> Optional[float]:
        """Packet-loss % over the current window, or None if no data."""
        n = len(self._window)
        if n == 0:
            return None
        return round(self.lost_count() / n * 100, 2)

    def _over(self, avg: float) -> bool:
        """True when loss breaches: above the threshold AND at least
        MIN_LOST_TO_WARN pings lost (so one stray drop in a young window, where
        1 ping is >2 % of a handful of samples, can never raise an alert)."""
        return avg > PACKET_LOSS_WARN_PCT and self.lost_count() >= MIN_LOST_TO_WARN

    def avg_rtt_ms(self) -> Optional[float]:
        """Mean round-trip of the replies in the window, or None."""
        rtts = [r for _, ok, r in self._window if ok and r is not None]
        return round(sum(rtts) / len(rtts), 1) if rtts else None

    def _prune(self, now: float) -> None:
        w = self.window_s()
        while self._window and (now - self._window[0][0]) > w:
            self._window.popleft()

    def no_data(self) -> None:
        """No gateway: nothing to measure. Pause any clearance progress."""
        self._clear_start = None

    def record(self, ok: bool, rtt_ms: Optional[float]) -> Optional[str]:
        """
        Record one ping result and check for state transitions.
        Returns 'WARN', 'CLEAR', or None (no transition).
        Nothing is judged until MIN_WINDOW_SAMPLES results are in the window.
        """
        now = time.monotonic()
        self._window.append((now, ok, rtt_ms))
        self._prune(now)
        if len(self._window) < MIN_WINDOW_SAMPLES:
            return None
        avg = self.moving_avg_pct()
        if avg is None:
            return None

        if self._state == "OK":
            if self._over(avg):
                self._state       = "WARN"
                self._clear_start = None
                return "WARN"

        elif self._state == "WARN":
            if self._over(avg):
                # Still above threshold — reset any clearance progress
                # (this also covers a dead gateway: 100 % loss never clears)
                self._clear_start = None
            elif self._clear_start is None:
                self._clear_start = now
            elif (now - self._clear_start) >= CLEAR_WINDOW_S:
                self._state       = "OK"
                self._clear_start = None
                return "CLEAR"

        return None


class _PingStream:
    """
    One long-lived `ping` process read WITHOUT blocking.

    Why a stream instead of ping batches: a batch call blocks the caller for the
    whole batch.  Here the child is spawned once (and restarted only on a change
    of gateway or interval); the main loop drains whatever output has arrived
    with a non-blocking read and evaluates each result immediately.

    `-O` makes iputils ping print "no answer yet for icmp_seq=N" as soon as the
    next echo is sent with N still unanswered, so losses show up promptly.
    Unprivileged (no raw socket / sysctl needed), same as the old batch ping.
    """

    def __init__(self, host: str, interval: float) -> None:
        self.host     = host
        self.interval = interval
        self._buf     = b""
        self._done: set = set()     # seqs already counted (lost or replied)
        self._max_seq = 0
        self.proc = subprocess.Popen(
            ["ping", "-O", "-n", "-W", "1", "-i", str(interval), host],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        os.set_blocking(self.proc.stdout.fileno(), False)

    def fileno(self) -> int:
        return self.proc.stdout.fileno()

    def alive(self) -> bool:
        return self.proc.poll() is None

    def close(self) -> None:
        try:
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        try:
            self.proc.stdout.close()
        except Exception:
            pass

    def poll(self) -> list:
        """Return [(ok, rtt_ms_or_None), ...] for everything that has arrived.
        Never blocks."""
        try:
            chunk = os.read(self.fileno(), 4096)
        except (BlockingIOError, InterruptedError):
            return []
        except OSError:
            return []
        self._buf += chunk
        out = []
        *lines, self._buf = self._buf.split(b"\n")
        for raw in lines:
            line = raw.decode("ascii", "replace")
            m = re.search(r"icmp_seq=(\d+)", line)
            if not m or "DUP!" in line:
                continue                      # banner / summary / duplicate reply
            seq = int(m.group(1))
            if seq in self._done:
                continue                      # late reply to a seq already counted lost
            self._done.add(seq)
            self._max_seq = max(self._max_seq, seq)
            t = re.search(r"time=([\d.]+)", line)
            out.append((True, float(t.group(1))) if t else (False, None))
        if len(self._done) > 512:            # bound memory on long runs
            self._done = {q for q in self._done if q >= self._max_seq - 256}
        return out


# ---------------------------------------------------------------------------
# System probes
# ---------------------------------------------------------------------------

def _svc_status(unit: str) -> str:
    """Return systemd unit active state: 'active', 'failed', 'inactive', etc."""
    try:
        out = subprocess.check_output(
            ["systemctl", "is-active", unit],
            text=True, timeout=3
        ).strip()
        return out
    except subprocess.CalledProcessError as exc:
        return (exc.stdout or "inactive").strip()
    except Exception:
        return "?"


def _cpu_temp() -> Optional[float]:
    """Return CPU temperature in °C, or None."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as fh:
            return int(fh.read().strip()) / 1000.0
    except Exception:
        return None


def _mem_free_mb() -> Optional[int]:
    """Return MemAvailable in MiB, or None."""
    try:
        with open("/proc/meminfo") as fh:
            for line in fh:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) // 1024
    except Exception:
        pass
    return None


def _wifi_rssi() -> Optional[int]:
    """Return wlan0 signal level in dBm, or None if not connected."""
    try:
        out = subprocess.check_output(
            ["iw", "dev", "wlan0", "link"],
            text=True, timeout=3, stderr=subprocess.DEVNULL
        )
        m = re.search(r"signal:\s*(-\d+)\s*dBm", out)
        return int(m.group(1)) if m else None
    except Exception:
        return None


def _default_gateway() -> Optional[str]:
    """Return the default gateway IP address, or None.

    Reads /proc/net/route (a plain file read — no subprocess, so it is cheap
    enough for the main loop); falls back to `ip route` if that fails."""
    try:
        with open("/proc/net/route") as fh:
            next(fh)                                   # header
            for line in fh:
                f = line.split()
                # default route: Destination 00000000, RTF_GATEWAY (0x2) set
                if len(f) >= 4 and f[1] == "00000000" and int(f[3], 16) & 2:
                    return socket.inet_ntoa(bytes.fromhex(f[2])[::-1])
        return None
    except Exception:
        pass
    try:
        out = subprocess.check_output(
            ["ip", "route", "show", "default"],
            text=True, timeout=3
        )
        m = re.search(r"default via (\S+)", out)
        return m.group(1) if m else None
    except Exception:
        return None


def _ping(host: str, count: int, interval: float = 1.0) -> dict:
    """
    Ping host <count> times at <interval> seconds.
    Uses a single ping process — efficient even at 0.5 s interval.
    Returns dict: sent / received / lost / loss_pct / avg_ms.
    """
    result: dict = {
        "sent": count, "received": 0, "lost": count,
        "loss_pct": 100.0, "avg_ms": None,
    }
    if not host:
        return result
    try:
        out = subprocess.check_output(
            ["ping", "-c", str(count), "-i", str(interval), "-W", "1", host],
            text=True,
            timeout=count * interval + 5,
            stderr=subprocess.DEVNULL,
        )
        m = re.search(r"(\d+) packets transmitted, (\d+) received", out)
        if m:
            result["sent"]     = int(m.group(1))
            result["received"] = int(m.group(2))
            result["lost"]     = result["sent"] - result["received"]
            result["loss_pct"] = (
                round(result["lost"] / result["sent"] * 100, 1)
                if result["sent"] else 100.0
            )
        m2 = re.search(r"rtt .* = [\d.]+/([\d.]+)/", out)
        if m2:
            result["avg_ms"] = float(m2.group(1))
    except Exception:
        pass
    return result


def _clock_synced() -> bool:
    """Return True if NTP/chrony reports clock is synchronized."""
    try:
        out = subprocess.check_output(
            ["timedatectl", "show", "--property=NTPSynchronized", "--value"],
            text=True, timeout=3
        ).strip()
        return out.lower() == "yes"
    except Exception:
        return False


def _journal_warn_err(since: str) -> dict:
    """Count WARNING+ journal entries for inno-pilot and related services."""
    counts = {"warn": 0, "err": 0}
    for _, unit in INNO_SERVICES:
        try:
            out = subprocess.check_output(
                ["journalctl", "-u", unit, "--since", since,
                 "-p", "warning", "--no-pager", "-o", "cat"],
                text=True, timeout=5, stderr=subprocess.DEVNULL,
            )
            for line in out.splitlines():
                if not line.strip():
                    continue
                if re.search(r"\bERR(OR)?\b|error", line, re.IGNORECASE):
                    counts["err"] += 1
                else:
                    counts["warn"] += 1
        except Exception:
            pass
    return counts


def _kernel_checks(since: Optional[str] = None) -> dict:
    """Scan kernel journal for undervoltage, SD I/O errors and FS errors.
    Pass since=None to scan the entire current boot.
    """
    result = {"undervoltage": 0, "sd_errors": 0, "fs_errors": 0}
    args   = ["journalctl", "-k"]
    args  += ["--since", since] if since else ["--boot"]
    args  += ["--no-pager", "-o", "cat"]
    try:
        out = subprocess.check_output(args, text=True, timeout=8,
                                      stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            if re.search(r"under.?voltage|voltage drop", line, re.IGNORECASE):
                result["undervoltage"] += 1
            if re.search(r"mmc\d.*error|I/O error.*mmcblk", line, re.IGNORECASE):
                result["sd_errors"] += 1
            if re.search(r"EXT4-fs error|forced fsck", line, re.IGNORECASE):
                result["fs_errors"] += 1
    except Exception:
        pass
    return result


def _fs_remounted_ro() -> bool:
    """Return True if root filesystem was remounted read-only this boot."""
    try:
        out = subprocess.check_output(
            ["journalctl", "--boot", "--no-pager", "-o", "cat",
             "--grep", "Remounting filesystem read-only"],
            text=True, timeout=5, stderr=subprocess.DEVNULL,
        )
        return bool(out.strip())
    except Exception:
        return False

# ---------------------------------------------------------------------------
# Message builders
# ---------------------------------------------------------------------------

def _hostname() -> str:
    try:
        return socket.gethostname()
    except Exception:
        return "innopilot"


def _fmt_ping(p: dict) -> str:
    avg = f"{p['avg_ms']:.1f} ms" if p["avg_ms"] is not None else "n/a"
    return f"PING  {p['sent']} sent, {p['lost']} lost ({p['loss_pct']}%), avg {avg}"


def _svc_line(label: str, status: str) -> str:
    tag = "[OK]" if status == "active" else "[!!]"
    return f"  {tag} {label}: {status}"


def build_boot_report(ping_result: dict) -> str:
    """Full health snapshot for the boot notification. Also logs all anomalies."""
    host   = _hostname()
    temp   = _cpu_temp()
    mem    = _mem_free_mb()
    rssi   = _wifi_rssi()
    synced = _clock_synced()
    gw     = _default_gateway()
    kernel = _kernel_checks()
    ro_fs  = _fs_remounted_ro()

    # --- Audit log: system state at boot ---
    log.info("BOOT: host=%s temp=%s mem=%s rssi=%s clock_synced=%s gw=%s",
             host,
             f"{temp:.1f}C" if temp is not None else "n/a",
             f"{mem}MB" if mem is not None else "n/a",
             f"{rssi}dBm" if rssi is not None else "n/a",
             synced, gw or "none")

    if not synced:
        log.warning("BOOT: Clock NOT synchronized — timestamps unreliable")
    if rssi is None:
        log.warning("BOOT: WiFi not connected (wlan0 link down)")
    if gw is None:
        log.warning("BOOT: No default gateway found")
    if kernel["undervoltage"]:
        log.warning("BOOT: Undervoltage events detected: %d", kernel["undervoltage"])
    if kernel["sd_errors"]:
        log.warning("BOOT: SD card I/O errors detected: %d", kernel["sd_errors"])
    if kernel["fs_errors"]:
        log.warning("BOOT: Filesystem errors detected: %d", kernel["fs_errors"])
    if ro_fs:
        log.warning("BOOT: Root filesystem was remounted read-only (SD corruption risk)")
    if mem is not None and mem < 50:
        log.warning("BOOT: Low memory at boot: %d MB free", mem)
    if temp is not None and temp > 80.0:
        log.warning("BOOT: High CPU temperature at boot: %.1f C", temp)

    for label, unit in INNO_SERVICES:
        s = _svc_status(unit)
        if s == "active":
            log.info("BOOT: Service %s: %s", label, s)
        else:
            log.warning("BOOT: Service %s: %s", label, s)

    # --- Build message ---
    lines = [
        f"Inno-Pilot BOOT  —  {host}",
        "─────────────────────────────",
        f"CPU temp : {f'{temp:.1f} C' if temp is not None else 'n/a'}",
        f"RAM free : {f'{mem} MB free' if mem is not None else 'n/a'}",
        f"Clock    : {'synced' if synced else 'NOT synced  [!!]'}",
        f"WiFi     : {f'{rssi} dBm' if rssi is not None else 'n/a'}",
        "",
        "SERVICES",
    ]
    for label, unit in INNO_SERVICES:
        lines.append(_svc_line(label, _svc_status(unit)))

    lines += ["", "NETWORK"]
    if gw:
        lines += [f"  Gateway {gw}", f"  {_fmt_ping(ping_result)}"]
    else:
        lines.append("  No default gateway")

    lines += ["", "SYSTEM"]
    lines.append(f"  Undervoltage : {kernel['undervoltage'] or 'none'}")
    lines.append(f"  SD errors    : {kernel['sd_errors'] or 'none'}")
    lines.append(f"  FS errors    : {kernel['fs_errors'] or 'none'}")
    if ro_fs:
        lines.append("  [!!] Root filesystem remounted read-only")

    return "\n".join(lines)


def build_period_report(
    since_ts: str,
    interval_min: int,
    ping_result: dict,
    temp_min: Optional[float],
    temp_max: Optional[float],
    rssi_samples: list,
    gw: Optional[str],
    net_state: str,
) -> str:
    """Summary health report for a completed monitoring period. Logs anomalies."""
    host    = _hostname()
    kernel  = _kernel_checks(since_ts)
    jcounts = _journal_warn_err(since_ts)

    # --- Audit log: period summary ---
    log.info("PERIOD %dmin: ping=%s net_state=%s journal_warn=%d journal_err=%d",
             interval_min, _fmt_ping(ping_result).replace("PING  ", ""),
             net_state, jcounts["warn"], jcounts["err"])
    if kernel["undervoltage"]:
        log.warning("PERIOD: Undervoltage events: %d", kernel["undervoltage"])
    if kernel["sd_errors"]:
        log.warning("PERIOD: SD card I/O errors: %d", kernel["sd_errors"])
    if kernel["fs_errors"]:
        log.warning("PERIOD: Filesystem errors: %d", kernel["fs_errors"])
    if jcounts["err"]:
        log.warning("PERIOD: Journal errors from inno-pilot services: %d", jcounts["err"])
    if temp_max is not None and temp_max > 80.0:
        log.warning("PERIOD: Peak CPU temperature: %.1f C", temp_max)

    # --- Build message ---
    lines = [
        "Inno-Pilot Health",
        f"Period: last {interval_min} min",
        "─────────────────────────────",
    ]

    lines.append(
        f"CPU temp : {f'{temp_min:.1f}–{temp_max:.1f} C' if temp_min is not None else 'n/a'}"
    )
    if rssi_samples:
        avg_rssi = int(sum(rssi_samples) / len(rssi_samples))
        lines.append(f"WiFi     : {avg_rssi} dBm avg ({len(rssi_samples)} samples)")
    else:
        lines.append("WiFi     : n/a")

    lines += ["", "SERVICES"]
    failed = [(lbl, _svc_status(u)) for lbl, u in INNO_SERVICES]
    failed = [(lbl, s) for lbl, s in failed if s != "active"]
    if failed:
        for lbl, s in failed:
            lines.append(_svc_line(lbl, s))
    else:
        lines.append("  [OK] All running")

    lines += ["", "NETWORK"]
    if gw:
        lines += [f"  Gateway {gw}", f"  {_fmt_ping(ping_result)}"]
        if net_state == "WARN":
            lines.append("  [!!] Packet loss warning active")
    else:
        lines.append("  No default gateway")

    lines += ["", "ERRORS (period)"]
    lines.append(f"  Journal WARN : {jcounts['warn'] or 'none'}")
    lines.append(f"  Journal ERR  : {jcounts['err'] or 'none'}")
    lines.append(f"  Undervoltage : {kernel['undervoltage'] or 'none'}")
    lines.append(f"  SD errors    : {kernel['sd_errors'] or 'none'}")
    lines.append(f"  FS errors    : {kernel['fs_errors'] or 'none'}")

    return "\n".join(lines)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    _shutdown = [False]

    def _sig(n, f):  # noqa: ANN001
        _shutdown[0] = True

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT,  _sig)

    log.info("inno_health_notify starting — settling %ds", BOOT_SETTLE_S)

    # Wait for network and services to settle after boot
    for _ in range(BOOT_SETTLE_S):
        if _shutdown[0]:
            return
        time.sleep(1)

    # --- Boot report: 10 pings, clean slate ---
    gw = _default_gateway()
    ping_interval = _read_ping_interval()
    boot_ping = _ping(gw, BOOT_PING_COUNT, ping_interval) if gw else {
        "sent": 0, "received": 0, "lost": 0, "loss_pct": 0.0, "avg_ms": None,
    }
    _send(build_boot_report(boot_ping))

    # --- Periodic loop ---
    # One single-threaded, non-blocking loop.  Each pass: (1) housekeeping that is
    # due (settings, gateway, samples, period report), (2) start/keep the ping
    # stream, (3) drain whatever ping results have arrived and record + evaluate
    # + publish each immediately, (4) wait — but only in select(), for at most
    # LOOP_MAX_WAIT_S, so shutdown and setting changes are noticed promptly.
    net_monitor   = _NetMonitor()   # clean slate — no history from previous run
    gw            = _default_gateway()
    stream: Optional[_PingStream] = None
    stream_restart_ok_at = 0.0      # rate-limit respawns of a dead ping process

    interval_min  = _read_interval()
    ping_interval = _read_ping_interval()
    debug_on      = os.path.exists(DEBUG_FLAG_FILE)
    eff_interval  = PING_INTERVAL_DEBUG_S if debug_on else ping_interval
    net_monitor.set_interval(eff_interval)

    now = time.monotonic()
    next_cfg    = now + CFG_POLL_S
    next_gw     = now + GW_REFRESH_S
    next_sample = now + SAMPLE_S

    # Accumulated stats for the current period report
    ping_sent_acc = ping_recv_acc = ping_ms_count = 0
    ping_ms_sum   = 0.0
    temp_samples: list = []
    rssi_samples: list = []
    period_start  = time.strftime("%Y-%m-%d %H:%M:%S")
    period_t0     = now

    while not _shutdown[0]:
        now = time.monotonic()

        # ---- settings / debug flag (cheap; once per CFG_POLL_S) ----
        if now >= next_cfg:
            next_cfg = now + CFG_POLL_S
            new_min   = _read_interval()
            ping_interval = _read_ping_interval()
            debug_on  = os.path.exists(DEBUG_FLAG_FILE)
            new_eff   = PING_INTERVAL_DEBUG_S if debug_on else ping_interval
            if new_eff != eff_interval:
                log.info("PING interval %.1fs -> %.1fs%s", eff_interval, new_eff,
                         " (debug)" if debug_on else "")
                eff_interval = new_eff
                net_monitor.set_interval(eff_interval)
                if stream is not None:          # respawn at the new interval
                    stream.close()
                    stream = None
            if new_min != interval_min:
                log.info("PERIOD: Interval changed — resetting accumulator")
                interval_min = new_min
                ping_sent_acc = ping_recv_acc = ping_ms_count = 0
                ping_ms_sum   = 0.0
                temp_samples, rssi_samples = [], []
                period_start  = time.strftime("%Y-%m-%d %H:%M:%S")
                period_t0     = now

        # ---- gateway (file read, once per GW_REFRESH_S) ----
        if now >= next_gw:
            next_gw = now + GW_REFRESH_S
            new_gw = _default_gateway()
            if new_gw != gw:
                log.info("NETWORK: Default gateway %s -> %s", gw, new_gw)
                gw = new_gw
                net_monitor.reset()     # old samples describe a different path
                if stream is not None:
                    stream.close()
                    stream = None

        # ---- ping stream lifecycle ----
        if gw and stream is not None and not stream.alive():
            # ping exited (e.g. network unreachable).  Count it as a lost ping
            # and respawn no faster than once a second.
            stream.close()
            stream = None
            stream_restart_ok_at = now + 1.0
            _record_ping(False, None, net_monitor, gw)
            ping_sent_acc += 1
        if gw and stream is None and now >= stream_restart_ok_at:
            try:
                stream = _PingStream(gw, eff_interval)
            except Exception as exc:  # noqa: BLE001
                log.warning("NETWORK: could not start ping: %s", exc)
                stream_restart_ok_at = now + 5.0
        if not gw:
            net_monitor.no_data()

        # ---- drain results that have arrived; record + evaluate each at once ----
        if stream is not None:
            for ok, rtt in stream.poll():
                _record_ping(ok, rtt, net_monitor, gw)
                ping_sent_acc += 1
                if ok:
                    ping_recv_acc += 1
                    ping_ms_sum   += rtt
                    ping_ms_count += 1

        # ---- temperature / RSSI samples for period reports ----
        if now >= next_sample:
            next_sample = now + SAMPLE_S
            t = _cpu_temp()
            if t is not None:
                temp_samples.append(t)
            r = _wifi_rssi()
            if r is not None:
                rssi_samples.append(r)

        # ---- period report ----
        if interval_min > 0 and (now - period_t0) >= interval_min * 60:
            ping_lost = ping_sent_acc - ping_recv_acc
            period_ping = {
                "sent":     ping_sent_acc,
                "received": ping_recv_acc,
                "lost":     ping_lost,
                "loss_pct": round(ping_lost / ping_sent_acc * 100, 1) if ping_sent_acc else 0.0,
                "avg_ms":   round(ping_ms_sum / ping_ms_count, 1) if ping_ms_count else None,
            }
            _send(build_period_report(
                period_start, interval_min,
                period_ping,
                min(temp_samples) if temp_samples else None,
                max(temp_samples) if temp_samples else None,
                rssi_samples, gw,
                net_monitor._state,
            ))
            ping_sent_acc = ping_recv_acc = ping_ms_count = 0
            ping_ms_sum   = 0.0
            temp_samples, rssi_samples = [], []
            period_start  = time.strftime("%Y-%m-%d %H:%M:%S")
            period_t0     = now

        # ---- wait: only ever inside select(), never a bare sleep ----
        if stream is not None:
            try:
                select.select([stream.fileno()], [], [], LOOP_MAX_WAIT_S)
            except (OSError, ValueError):
                time.sleep(0.05)   # fd vanished mid-select; loop will respawn
        else:
            select.select([], [], [], LOOP_MAX_WAIT_S)

    if stream is not None:
        stream.close()


def _record_ping(ok: bool, rtt_ms: Optional[float], monitor: "_NetMonitor",
                 gw: Optional[str]) -> None:
    """Record ONE ping result, evaluate it, log any transition, publish status.

    This is the single quick step run the moment a result arrives, so the alert
    state (and the remote's display, via NET_STATUS_FILE) never lags a ping.
    """
    transition = monitor.record(ok, rtt_ms)
    _handle_transition(transition, monitor, gw)


def _publish_net_status(monitor: "_NetMonitor", gw: Optional[str]) -> None:
    """Write the monitor's current state to NET_STATUS_FILE for the web remote.

    Atomic (tmp + rename) so the reader never sees a half-written file.  The
    reader treats a stale 'ts' as "Pi-side monitor not running".  Failures are
    logged at debug only — status publishing must never disturb monitoring.
    """
    try:
        tmp = NET_STATUS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({
                "ts":       time.time(),
                "state":    monitor._state,           # "OK" | "WARN"
                "loss_pct": monitor.moving_avg_pct(), # window average
                "avg_ms":   monitor.avg_rtt_ms(),     # window mean RTT
                "gateway":  gw,
            }, f)
        os.chmod(tmp, 0o644)
        os.replace(tmp, NET_STATUS_FILE)
    except Exception as exc:  # noqa: BLE001
        log.debug("net status publish failed: %s", exc)


def _handle_transition(transition: Optional[str], monitor: "_NetMonitor",
                       gw: Optional[str] = None) -> None:
    """Log and notify on packet-loss state transitions; publish current state."""
    _publish_net_status(monitor, gw)
    if transition == "WARN":
        avg = monitor.moving_avg_pct()
        log.warning(
            "PACKET_LOSS_WARN: Pi->router %ds avg %.1f%% exceeds threshold %.1f%%",
            WARN_WINDOW_S, avg, PACKET_LOSS_WARN_PCT,
        )
        _send(
            f"WIFI ALERT: Autopilot WiFi is weak — {avg:.1f}% of messages lost "
            f"over the last {WARN_WINDOW_S}s (limit {PACKET_LOSS_WARN_PCT:.0f}%)"
        )
    elif transition == "CLEAR":
        log.info(
            "PACKET_LOSS_CLEAR: Pi->router avg below %.1f%% for %d s — network OK",
            PACKET_LOSS_WARN_PCT, CLEAR_WINDOW_S,
        )
        _send(
            f"WIFI OK: Autopilot WiFi is back to normal "
            f"(under {PACKET_LOSS_WARN_PCT:.0f}% lost for {CLEAR_WINDOW_S}s)"
        )


if __name__ == "__main__":
    main()
