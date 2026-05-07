"""board_conf.py — Python loader for /var/lib/inno-pilot/board.conf.

Reads the shell-sourceable KEY="VALUE" file produced by detect_arduino.sh and
returns the values as a dict.  Used by inno_pilot_bridge.py (and any future
Python consumer) to pick up the detected Arduino board's port and baud rate
without hardcoding them.

The file format is intentionally a strict subset of shell syntax:
  - One KEY=VALUE assignment per line
  - VALUE may be quoted with double quotes (which are stripped)
  - Lines starting with '#' and blank lines are ignored
  - No multi-line strings, no shell expansion, no continuation lines

Typical use:

    from board_conf import load
    cfg = load()
    nano_port = cfg["INNO_BOARD_BYID_REAL"]
    baud      = int(cfg["INNO_BOARD_BAUD"])
"""
from __future__ import annotations

import os
from typing import Dict

DEFAULT_PATH = "/var/lib/inno-pilot/board.conf"

REQUIRED_KEYS = (
    "INNO_BOARD_FQBN",
    "INNO_BOARD_PORT",
    "INNO_BOARD_BYID",
    "INNO_BOARD_BAUD",
)


class BoardConfError(RuntimeError):
    """Raised when board.conf is missing, unreadable, or incomplete."""


def load(path: str = DEFAULT_PATH) -> Dict[str, str]:
    """Parse board.conf and return a dict of all KEY=VALUE entries.

    Raises BoardConfError if the file is missing or any required key is absent.
    """
    if not os.path.isfile(path):
        raise BoardConfError(
            f"board config {path} missing — run detect_arduino.sh"
        )

    cfg: Dict[str, str] = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key, sep, value = line.partition("=")
            if not sep:
                # Line without '=' — skip silently; lets future fields without
                # values not break older parsers.
                continue
            key = key.strip()
            value = value.strip()
            # Strip a single pair of surrounding double quotes if present
            if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
                value = value[1:-1]
            cfg[key] = value

    # Derive BYID_REAL if missing (matches board_conf.sh behaviour)
    cfg.setdefault(
        "INNO_BOARD_BYID_REAL", cfg.get("INNO_BOARD_BYID", "") + ".real"
    )

    missing = [k for k in REQUIRED_KEYS if k not in cfg or not cfg[k]]
    if missing:
        raise BoardConfError(
            f"board config {path} incomplete — missing keys: {', '.join(missing)}"
        )

    return cfg


if __name__ == "__main__":
    # Diagnostic entry point: `python3 board_conf.py` prints all loaded values.
    import sys
    try:
        cfg = load()
    except BoardConfError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    for k in sorted(cfg):
        print(f"{k}={cfg[k]}")
