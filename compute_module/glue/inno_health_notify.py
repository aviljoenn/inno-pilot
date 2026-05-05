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
Each tick fires one ping batch that acts as the tick sleep (~TICK_S seconds).
Results feed a 10-minute sliding-window packet-loss monitor.  State changes
(WARN / CLEAR) are logged with structured markers and trigger Telegram alerts.

Packet-loss state machine
--------------------------
  OK  → WARN  when 10-min moving avg > PACKET_LOSS_WARN_PCT (2 %)
  WARN→ OK    when 10-min avg stays ≤ 2 % for CLEAR_WINDOW_S (30 min)
              clearance only accrues while pings are actually succeeding;
              100 % loss (gateway unreachable) does NOT count toward clearance.

All state transitions, anomalies and service faults are written to the Python
logger (captured by journalctl under the inno-health-notify unit).
inno_web_remote.py tails this journal to surface net_warn in the browser UI.
"""

import collections
import json
import os
import re
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
TICK_S               = 60     # nominal period between ping batches / samples
PACKET_LOSS_WARN_PCT = 2.0    # 10-min moving-avg threshold for WARN
WARN_WINDOW_S        = 600    # sliding window for moving average (10 min)
CLEAR_WINDOW_S       = 1800   # time below threshold required to clear (30 min)

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
    """Return notifications.ping_interval_s from settings (default 0.5, clamped 0.2–60)."""
    try:
        with open(SETTINGS_FILE) as fh:
            s = json.load(fh)
        v = float(s.get("notifications", {}).get("ping_interval_s", 0.5))
        return max(0.2, min(60.0, v))
    except Exception:
        return 0.5

# ---------------------------------------------------------------------------
# Packet-loss sliding-window state machine
# ---------------------------------------------------------------------------

class _NetMonitor:
    """
    Maintains a WARN_WINDOW_S sliding window of ping results and drives the
    OK ↔ WARN state machine.

    Each deque entry is (monotonic_ts, sent, received).
    """

    def __init__(self) -> None:
        self._window: collections.deque = collections.deque()
        self._state   = "OK"
        self._clear_start: Optional[float] = None  # when below-threshold run began

    def moving_avg_pct(self) -> Optional[float]:
        """Packet-loss % over the current window, or None if no data."""
        total_sent = sum(s for _, s, _ in self._window)
        total_recv = sum(r for _, _, r in self._window)
        if total_sent == 0:
            return None
        return round((total_sent - total_recv) / total_sent * 100, 2)

    def _prune(self, now: float) -> None:
        while self._window and (now - self._window[0][0]) > WARN_WINDOW_S:
            self._window.popleft()

    def update(self, sent: int, received: int) -> Optional[str]:
        """
        Record a batch result and check for state transitions.
        Returns 'WARN', 'CLEAR', or None (no transition).
        Clearance only accrues when sent > 0 (gateway actually reachable).
        """
        now = time.monotonic()
        self._window.append((now, sent, received))
        self._prune(now)

        avg       = self.moving_avg_pct()
        pings_ok  = sent > 0

        if avg is None:
            return None

        if self._state == "OK":
            if avg > PACKET_LOSS_WARN_PCT:
                self._state       = "WARN"
                self._clear_start = None
                return "WARN"

        elif self._state == "WARN":
            if avg > PACKET_LOSS_WARN_PCT:
                # Still above threshold — reset any clearance progress
                self._clear_start = None
            else:
                if not pings_ok:
                    # Gateway unreachable: 100 % loss doesn't count toward clearance
                    self._clear_start = None
                else:
                    if self._clear_start is None:
                        self._clear_start = now
                    elif (now - self._clear_start) >= CLEAR_WINDOW_S:
                        self._state       = "OK"
                        self._clear_start = None
                        return "CLEAR"

        return None

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
    """Return the default gateway IP address, or None."""
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
    net_monitor   = _NetMonitor()   # clean slate — no history from previous run
    gw            = _default_gateway()
    ping_interval = _read_ping_interval()

    # Accumulated stats for the current period report
    ping_sent_acc    = 0
    ping_recv_acc    = 0
    ping_ms_sum      = 0.0
    ping_ms_count    = 0
    temp_samples: list = []
    rssi_samples: list = []
    period_start     = time.strftime("%Y-%m-%d %H:%M:%S")
    ticks_done       = 0

    while not _shutdown[0]:
        interval_min  = _read_interval()
        ping_interval = _read_ping_interval()

        if interval_min <= 0:
            # Boot-only mode — still run pings for the monitor, just no period reports
            n = max(1, int(TICK_S / ping_interval))
            if gw:
                p = _ping(gw, n, ping_interval)
                transition = net_monitor.update(p["sent"], p["received"])
                _handle_transition(transition, net_monitor)
                # Refresh gateway each tick
                gw = _default_gateway()
            else:
                time.sleep(TICK_S)
                gw = _default_gateway()
                if gw:
                    log.info("NETWORK: Default gateway appeared: %s", gw)
            continue

        ticks_needed = max(1, (interval_min * 60) // TICK_S)

        # Ping batch acts as the tick sleep (~TICK_S seconds)
        n = max(1, int(TICK_S / ping_interval))
        if gw:
            p = _ping(gw, n, ping_interval)
        else:
            time.sleep(TICK_S)
            gw = _default_gateway()
            if gw:
                log.info("NETWORK: Default gateway appeared: %s", gw)
            p = {"sent": 0, "received": 0, "lost": 0, "loss_pct": 100.0, "avg_ms": None}

        # Feed monitor and check for state transitions
        transition = net_monitor.update(p["sent"], p["received"])
        _handle_transition(transition, net_monitor)

        # Accumulate ping stats for period report
        ping_sent_acc += p["sent"]
        ping_recv_acc += p["received"]
        if p["avg_ms"] is not None:
            ping_ms_sum   += p["avg_ms"]
            ping_ms_count += 1

        # Temperature and RSSI samples
        t = _cpu_temp()
        if t is not None:
            temp_samples.append(t)
        r = _wifi_rssi()
        if r is not None:
            rssi_samples.append(r)

        ticks_done += 1

        # Check if interval changed mid-period; if so, reset without sending
        if _read_interval() != interval_min:
            log.info("PERIOD: Interval changed mid-period — resetting accumulator")
            ping_sent_acc = ping_recv_acc = ping_ms_count = 0
            ping_ms_sum   = 0.0
            temp_samples  = []
            rssi_samples  = []
            period_start  = time.strftime("%Y-%m-%d %H:%M:%S")
            ticks_done    = 0
            gw = _default_gateway()
            continue

        if ticks_done >= ticks_needed:
            # Full period complete — build and send report
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

            # Reset accumulators
            ping_sent_acc = ping_recv_acc = ping_ms_count = 0
            ping_ms_sum   = 0.0
            temp_samples  = []
            rssi_samples  = []
            period_start  = time.strftime("%Y-%m-%d %H:%M:%S")
            ticks_done    = 0
            gw = _default_gateway()


def _handle_transition(transition: Optional[str], monitor: "_NetMonitor") -> None:
    """Log and notify on packet-loss state transitions."""
    if transition == "WARN":
        avg = monitor.moving_avg_pct()
        log.warning(
            "PACKET_LOSS_WARN: 10-min avg %.1f%% exceeds threshold %.1f%%",
            avg, PACKET_LOSS_WARN_PCT,
        )
        _send(
            f"NETWORK ALERT: Packet loss {avg:.1f}% "
            f"(10-min avg, threshold {PACKET_LOSS_WARN_PCT:.0f}%)"
        )
    elif transition == "CLEAR":
        log.info(
            "PACKET_LOSS_CLEAR: avg below %.1f%% for %d min — network OK",
            PACKET_LOSS_WARN_PCT, CLEAR_WINDOW_S // 60,
        )
        _send(
            f"NETWORK OK: Packet loss cleared "
            f"(below {PACKET_LOSS_WARN_PCT:.0f}% for {CLEAR_WINDOW_S // 60} min)"
        )


if __name__ == "__main__":
    main()
