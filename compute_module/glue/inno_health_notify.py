#!/usr/bin/env python3
"""
inno_health_notify.py — Inno-Pilot health monitoring and Telegram notifications.

Sends a full health report on boot, then optionally repeats at a configured
interval.  The interval is read from /var/lib/inno-pilot/settings.json
(notifications.health_interval_min).  0 = boot-only.

The main loop ticks every 60 s so interval changes in settings take effect
within one tick without requiring a service restart.
"""

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
BOOT_SETTLE_S   = 45    # wait after systemd start before sending boot report
TICK_S          = 60    # main loop tick; also period between ping accumulations
BOOT_PING_COUNT = 10    # pings in one-shot boot gateway test
TICK_PING_COUNT = 5     # pings accumulated each tick for period reports

INNO_SERVICES = [
    ("bridge",     "inno-pilot-bridge"),
    ("web-remote", "inno-pilot-web-remote"),
    ("socat",      "inno-pilot-socat"),
    ("pypilot",    "pypilot"),
    ("tailscale",  "tailscaled"),
    ("sshd",       "ssh"),
]

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


def _send(text: str) -> None:
    """Fire-and-forget Telegram message; silent on any failure."""
    token, chat_id = _read_conf()
    if not token or not chat_id:
        return
    try:
        data = json.dumps({"chat_id": chat_id, "text": text}).encode()
        req  = urllib.request.Request(
            f"https://api.telegram.org/bot{token}/sendMessage",
            data=data,
            headers={"Content-Type": "application/json"},
        )
        urllib.request.urlopen(req, timeout=10)
    except Exception:
        pass

# ---------------------------------------------------------------------------
# Settings helper
# ---------------------------------------------------------------------------

def _read_interval() -> int:
    """Return notifications.health_interval_min from settings (default 0)."""
    try:
        with open(SETTINGS_FILE) as fh:
            s = json.load(fh)
        return max(0, int(s.get("notifications", {}).get("health_interval_min", 0)))
    except Exception:
        return 0

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


def _ping(host: str, count: int) -> dict:
    """Ping host <count> times.  Returns dict: sent/received/lost/loss_pct/avg_ms."""
    result: dict = {
        "sent": count, "received": 0, "lost": count,
        "loss_pct": 100.0, "avg_ms": None,
    }
    if not host:
        return result
    try:
        out = subprocess.check_output(
            ["ping", "-c", str(count), "-W", "1", host],
            text=True, timeout=count + 5, stderr=subprocess.DEVNULL
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
    """Count WARNING+ journal entries for all inno-pilot and related services."""
    counts = {"warn": 0, "err": 0}
    units  = [u for _, u in INNO_SERVICES]
    for unit in units:
        try:
            out = subprocess.check_output(
                ["journalctl", "-u", unit, "--since", since,
                 "-p", "warning", "--no-pager", "-o", "cat"],
                text=True, timeout=5, stderr=subprocess.DEVNULL
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
    """
    Scan kernel journal for undervoltage, SD card I/O errors, and FS errors.
    Pass since=None to scan the whole current boot.
    """
    result = {"undervoltage": 0, "sd_errors": 0, "fs_errors": 0}
    args = ["journalctl", "-k"]
    if since:
        args += ["--since", since]
    else:
        args += ["--boot"]
    args += ["--no-pager", "-o", "cat"]
    try:
        out = subprocess.check_output(args, text=True, timeout=8, stderr=subprocess.DEVNULL)
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
    """Return True if the root filesystem was remounted read-only (SD corruption indicator)."""
    try:
        out = subprocess.check_output(
            ["journalctl", "--boot", "--no-pager", "-o", "cat",
             "--grep", "Remounting filesystem read-only"],
            text=True, timeout=5, stderr=subprocess.DEVNULL
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
        return "pypilotpi"


def _fmt_ping(p: dict) -> str:
    avg = f"{p['avg_ms']:.1f} ms" if p["avg_ms"] is not None else "n/a"
    return f"{p['sent']} sent, {p['lost']} lost ({p['loss_pct']}%), avg {avg}"


def _svc_line(label: str, status: str) -> str:
    tag = "[OK]" if status == "active" else "[!!]"
    return f"  {tag} {label}: {status}"


def build_boot_report() -> str:
    """Full health snapshot for the boot notification."""
    host    = _hostname()
    temp    = _cpu_temp()
    mem     = _mem_free_mb()
    rssi    = _wifi_rssi()
    synced  = _clock_synced()
    gw      = _default_gateway()
    kernel  = _kernel_checks()           # whole current boot
    ro_fs   = _fs_remounted_ro()

    lines = [
        f"Inno-Pilot BOOT  —  {host}",
        "─────────────────────────────",
    ]

    # System
    temp_s = f"{temp:.1f} C" if temp is not None else "n/a"
    mem_s  = f"{mem} MB free" if mem is not None else "n/a"
    rssi_s = f"{rssi} dBm" if rssi is not None else "n/a"
    lines += [
        f"CPU temp : {temp_s}",
        f"RAM free : {mem_s}",
        f"Clock    : {'synced' if synced else 'NOT synced  [!!]'}",
        f"WiFi     : {rssi_s}",
    ]

    # Services
    lines += ["", "SERVICES"]
    for label, unit in INNO_SERVICES:
        lines.append(_svc_line(label, _svc_status(unit)))

    # Network
    lines += ["", "NETWORK"]
    if gw:
        p = _ping(gw, BOOT_PING_COUNT)
        lines += [
            f"  Gateway {gw}",
            f"  {_fmt_ping(p)}",
        ]
    else:
        lines.append("  No default gateway")

    # System health
    lines += ["", "SYSTEM"]
    lines.append(f"  Undervoltage : {kernel['undervoltage'] or 'none'}")
    lines.append(f"  SD errors    : {kernel['sd_errors'] or 'none'}")
    lines.append(f"  FS errors    : {kernel['fs_errors'] or 'none'}")
    if ro_fs:
        lines.append("  [!!] Root filesystem was remounted read-only")

    return "\n".join(lines)


def build_period_report(
    since_ts: str,
    interval_min: int,
    ping_result: dict,
    temp_min: Optional[float],
    temp_max: Optional[float],
    rssi_samples: list,
    gw: Optional[str],
) -> str:
    """Summary health report for a completed monitoring period."""
    host    = _hostname()
    kernel  = _kernel_checks(since_ts)
    jcounts = _journal_warn_err(since_ts)

    lines = [
        f"Inno-Pilot Health  —  {host}",
        f"Period: last {interval_min} min",
        "─────────────────────────────",
    ]

    # System stats
    if temp_min is not None and temp_max is not None:
        lines.append(f"CPU temp : {temp_min:.1f}–{temp_max:.1f} C")
    else:
        lines.append("CPU temp : n/a")

    if rssi_samples:
        avg_rssi = int(sum(rssi_samples) / len(rssi_samples))
        lines.append(f"WiFi     : {avg_rssi} dBm avg ({len(rssi_samples)} samples)")
    else:
        lines.append("WiFi     : n/a")

    # Services — only list non-active ones; show "all OK" if everything is fine
    lines += ["", "SERVICES"]
    failed = [(lbl, _svc_status(u)) for lbl, u in INNO_SERVICES]
    failed = [(lbl, s) for lbl, s in failed if s != "active"]
    if failed:
        for lbl, s in failed:
            lines.append(_svc_line(lbl, s))
    else:
        lines.append("  [OK] All running")

    # Network
    lines += ["", "NETWORK"]
    if gw:
        lines.append(f"  Gateway {gw}")
        lines.append(f"  {_fmt_ping(ping_result)}")
    else:
        lines.append("  No default gateway")

    # Errors
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
    # Graceful shutdown on SIGTERM (systemd stop) or Ctrl-C
    _shutdown = [False]

    def _handle_sig(n, f):   # noqa: ANN001
        _shutdown[0] = True

    signal.signal(signal.SIGTERM, _handle_sig)
    signal.signal(signal.SIGINT,  _handle_sig)

    # Wait for network and inno-pilot services to settle after boot
    for _ in range(BOOT_SETTLE_S):
        if _shutdown[0]:
            return
        time.sleep(1)

    # Send boot report
    _send(build_boot_report())

    # Periodic reporting loop — ticks every TICK_S seconds.
    # Accumulates ping, temp, and RSSI samples over each interval period.
    gw = _default_gateway()

    while not _shutdown[0]:
        interval_min = _read_interval()

        if interval_min <= 0:
            # Boot-only mode — idle until the user sets an interval
            time.sleep(TICK_S)
            continue

        # Accumulate stats over the configured interval
        ticks_needed   = max(1, (interval_min * 60) // TICK_S)
        period_start   = time.strftime("%Y-%m-%d %H:%M:%S")

        ping_sent      = 0
        ping_received  = 0
        ping_ms_sum    = 0.0
        ping_ms_count  = 0
        temp_samples: list  = []
        rssi_samples: list  = []
        interval_changed   = False

        for _ in range(ticks_needed):
            if _shutdown[0]:
                return
            time.sleep(TICK_S)

            # Restart accumulation if the user changed the interval mid-period
            if _read_interval() != interval_min:
                interval_changed = True
                break

            # Ping accumulation
            if gw:
                p = _ping(gw, TICK_PING_COUNT)
                ping_sent     += p["sent"]
                ping_received += p["received"]
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

        if interval_changed:
            # Don't send a partial-period report; restart the outer loop
            continue

        # Build aggregated ping result
        ping_lost = ping_sent - ping_received
        ping_result = {
            "sent":     ping_sent,
            "received": ping_received,
            "lost":     ping_lost,
            "loss_pct": round(ping_lost / ping_sent * 100, 1) if ping_sent else 0.0,
            "avg_ms":   round(ping_ms_sum / ping_ms_count, 1) if ping_ms_count else None,
        }

        temp_min = min(temp_samples) if temp_samples else None
        temp_max = max(temp_samples) if temp_samples else None

        _send(build_period_report(
            period_start, interval_min,
            ping_result, temp_min, temp_max,
            rssi_samples, gw,
        ))

        # Refresh gateway after each period (DHCP may reassign)
        gw = _default_gateway()


if __name__ == "__main__":
    main()
