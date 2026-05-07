#!/bin/bash
# detect_arduino.sh — auto-detect which Arduino is wired to the Pi.
#
# Probes connected USB-serial devices, identifies the board (Nano old/new
# bootloader vs Uno R3 vs Uno clone) by USB descriptor + bootloader handshake,
# and writes the result to /var/lib/inno-pilot/board.conf for downstream
# scripts (inno_deploy.sh, inno_pilot_bridge.py, fixlink, etc.) to source.
#
# Algorithm (see top-level CLAUDE.md and the spec discussion for full details):
#
#   1. Stop bridge / socat / fixlink so they don't hold the port.
#   2. Enumerate /dev/serial/by-id/ entries that look like an Arduino USB-UART.
#   3. For each candidate, ask `arduino-cli board list` first — genuine Uno R3
#      reports its FQBN directly via the 16U2's USB VID/PID.
#   4. For boards arduino-cli can't identify (CH340 clones / FTDI Nanos),
#      probe the bootloader with `avrdude -n` (no-write) at 115200 then 57600
#      baud.  First responding baud rate distinguishes new-bootloader Nanos
#      and Unos (115200) from old-bootloader Nanos (57600).
#   5. Write board.conf atomically (tmp + mv) so consumers never see a
#      partial file.
#
# Safety:
#   - `avrdude -n` does NOT write flash; the probe is non-destructive.
#   - HUPCL drops DTR on close, resetting the Nano.  We pre-clear HUPCL with
#     `stty -hupcl` and sleep between probes so each attempt starts from a
#     known state.
#   - Caller is responsible for restarting the services we stopped.
#
# Exit codes:
#   0  board.conf written (or matches existing, on --dry-run)
#   1  no Arduino found
#   2  multiple boards responded (ambiguous; user must unplug extras)
#   3  probe error (avrdude missing, etc.)

set -euo pipefail

# ─── configurable paths ───────────────────────────────────────────────────────
BOARD_CONF="${INNO_BOARD_CONF:-/var/lib/inno-pilot/board.conf}"
BOARD_CONF_TMP="${BOARD_CONF}.tmp"

# Runtime serial baud (NOT the bootloader baud).  Same on Nano and Uno: the
# Nano sketch and the bridge talk at 38400 8N1.  Kept here so it's traceable.
RUNTIME_BAUD=38400

# Build flags applied during arduino-cli compile.  See top-level CLAUDE.md:
# the 128-byte RX buffer override is required to survive bridge telemetry
# bursts while the OLED I2C draw blocks loop().
DEFAULT_BUILD_FLAGS="build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128"

# Bootloader probe parameters
PROBE_BAUDS=(115200 57600)        # new-bootloader / Uno first, old-Nano second
PROBE_TIMEOUT=5                   # seconds per avrdude attempt
PROBE_RESET_SETTLE=5              # seconds between attempts (Nano setup() is ~3s)

# ─── flags ────────────────────────────────────────────────────────────────────
REDETECT=false
DRY_RUN=false
VERBOSE=false

usage() {
    cat <<EOF
Usage: $0 [--redetect] [--dry-run] [--verbose]
  --redetect  Ignore existing $BOARD_CONF and probe fresh
  --dry-run   Print detection result, do not write board.conf
  --verbose   Show every probe attempt
EOF
}

for arg in "$@"; do
    case "$arg" in
        --redetect) REDETECT=true ;;
        --dry-run)  DRY_RUN=true ;;
        --verbose)  VERBOSE=true ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "unknown arg: $arg" >&2; usage; exit 3 ;;
    esac
done

# All status output goes to stderr so command-substitution callers
# (e.g. probe_avrdude) can use stdout exclusively for return values.
log()  { echo "[detect] $*" >&2; }
vlog() { $VERBOSE && echo "[detect] $*" >&2 || true; }
die()  { echo "[detect] ERROR: $*" >&2; exit "${2:-3}"; }

# ─── short-circuit if cached and valid ────────────────────────────────────────
if ! $REDETECT && [[ -f "$BOARD_CONF" ]]; then
    # Source it in a subshell so we don't pollute our env with stale values
    if (
        # shellcheck disable=SC1090
        source "$BOARD_CONF"
        [[ -n "${INNO_BOARD_BYID:-}" ]] && [[ -e "$INNO_BOARD_BYID" ]]
    ); then
        log "board.conf already valid — skipping detection (use --redetect to force)."
        exit 0
    else
        log "board.conf present but BYID no longer exists — re-detecting."
    fi
fi

# ─── pre-flight checks ────────────────────────────────────────────────────────
command -v avrdude >/dev/null 2>&1 \
    || die "avrdude not found on PATH — install via 'sudo apt-get install -y avrdude'" 3

# ─── stop services that hold the serial port ──────────────────────────────────
# Each `stop_if_active` is best-effort; an absent service is fine.
stop_if_active() {
    local svc="$1"
    if systemctl list-unit-files "${svc}.service" &>/dev/null \
       && systemctl is-active --quiet "$svc" 2>/dev/null; then
        vlog "stopping $svc"
        sudo systemctl stop "$svc" || true
    fi
}
stop_if_active inno-pilot-bridge
stop_if_active inno-pilot-fixlink
stop_if_active inno-pilot-socat

sleep 1   # let kernel release the port

# ─── enumerate candidate ports ────────────────────────────────────────────────
# Match any USB-serial device that could plausibly be an Arduino.  We
# deliberately accept generic CH340/FTDI/Silicon Labs adapters because the
# Inno-Pilot Nano clones use CH340.  arduino-cli + avrdude probes will
# eliminate non-Arduino dongles.
shopt -s nullglob
CANDIDATES=()
for sym in /dev/serial/by-id/usb-Arduino*-if00* \
           /dev/serial/by-id/usb-1a86_USB_Serial-if00* \
           /dev/serial/by-id/usb-FTDI_*-if00* \
           /dev/serial/by-id/usb-Silicon_Labs_*-if00*; do
    # Skip the *.real fixlink alternates: when fixlink has run, the original
    # by-id symlink points at a PTY and the actual USB device is at <byid>.real.
    # Probe BYID.real directly, never the now-redirected BYID itself.
    case "$sym" in
        *.real) ;;  # accept .real entries
        *)
            # If a .real sibling exists, fixlink has already redirected this
            # by-id; the bare name now points at /dev/pts/* and is not the
            # Arduino.  Skip it — its .real sibling will be picked up below.
            [[ -e "${sym}.real" ]] && continue
            ;;
    esac
    # Final guard: only accept candidates whose resolved target is a real
    # tty (USB or ACM), never a pty.
    target="$(readlink -f "$sym")"
    case "$target" in
        /dev/ttyUSB*|/dev/ttyACM*) CANDIDATES+=("$sym") ;;
        *) vlog "skipping $sym -> $target (not a USB/ACM tty)" ;;
    esac
done
shopt -u nullglob

if (( ${#CANDIDATES[@]} == 0 )); then
    die "no Arduino-like USB serial device found in /dev/serial/by-id/ — is it plugged in?" 1
fi

log "found ${#CANDIDATES[@]} candidate port(s):"
for c in "${CANDIDATES[@]}"; do log "  - $c"; done

# ─── helpers ──────────────────────────────────────────────────────────────────
classify_byid() {
    # Echoes a USB-hint tag based on the by-id symlink name.
    local byid="$1"
    case "$(basename "$byid")" in
        *Arduino*)         echo "uno-genuine" ;;
        *1a86_USB_Serial*) echo "ch340"       ;;  # Nano clone or Uno clone
        *FTDI*)            echo "nano-ftdi"   ;;
        *Silicon_Labs*)    echo "nano-cp2102" ;;
        *)                 echo "unknown"     ;;
    esac
}

probe_arduino_cli() {
    # If arduino-cli reports a definite FQBN for this port, echo it.
    # Empty output = arduino-cli couldn't identify it (the common CH340 case).
    local port_real="$1"
    command -v arduino-cli >/dev/null 2>&1 || return 0
    arduino-cli board list --format json 2>/dev/null \
        | python3 -c '
import json, sys
target = sys.argv[1]
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
# arduino-cli output schema differs across versions: try both shapes
ports = data.get("detected_ports") or data
if isinstance(ports, dict):
    ports = ports.get("ports", [])
for entry in ports:
    p = entry.get("port") or entry
    addr = p.get("address", "")
    if addr != target:
        continue
    matches = entry.get("matching_boards") or p.get("matching_boards") or []
    for m in matches:
        fqbn = m.get("fqbn") or m.get("FQBN")
        if fqbn:
            print(fqbn)
            sys.exit(0)
' "$port_real" 2>/dev/null || true
}

probe_avrdude() {
    # Returns the first baud rate at which avrdude completes the bootloader
    # handshake, or empty string if neither responds.
    local port="$1"
    # Pre-clear HUPCL so the probe's own close doesn't reset the Nano before
    # we move on to the next attempt.
    stty -F "$port" -hupcl 2>/dev/null || true

    for baud in "${PROBE_BAUDS[@]}"; do
        vlog "  trying $port at $baud baud..."
        if timeout "$PROBE_TIMEOUT" avrdude \
                -p atmega328p -c arduino -P "$port" -b "$baud" \
                -n -qq 2>/dev/null; then
            vlog "  handshake OK at $baud"
            echo "$baud"
            return 0
        fi
        # Sleep between attempts: avrdude's close drops DTR even with HUPCL
        # cleared briefly, and the Nano spends ~3s in setup() after reset.
        sleep "$PROBE_RESET_SETTLE"
    done
    return 1
}

# ─── probe each candidate ─────────────────────────────────────────────────────
RESPONDERS=()    # parallel arrays: byid, port_real, fqbn, variant, method
RESPONDER_PORTS=()
RESPONDER_FQBNS=()
RESPONDER_VARIANTS=()
RESPONDER_METHODS=()

for byid in "${CANDIDATES[@]}"; do
    port_real="$(readlink -f "$byid")"
    hint="$(classify_byid "$byid")"
    log "probing $byid (-> $port_real, hint=$hint)"

    # Normalise to the canonical (non-.real) by-id name.  The renderer derives
    # BYID_REAL by appending .real, so storing the canonical form keeps the two
    # fields consistent regardless of whether fixlink has run yet.
    case "$byid" in
        *.real) byid_canonical="${byid%.real}" ;;
        *)      byid_canonical="$byid" ;;
    esac

    # Step 1: arduino-cli direct identification
    fqbn_cli="$(probe_arduino_cli "$port_real" || true)"
    if [[ -n "$fqbn_cli" ]]; then
        log "  arduino-cli identified FQBN=$fqbn_cli"
        case "$fqbn_cli" in
            *uno*)  variant="uno-genuine" ;;
            *nano*) variant="nano-new"    ;;
            *)      variant="unknown"     ;;
        esac
        RESPONDERS+=("$byid_canonical")
        RESPONDER_PORTS+=("$port_real")
        RESPONDER_FQBNS+=("$fqbn_cli")
        RESPONDER_VARIANTS+=("$variant")
        RESPONDER_METHODS+=("arduino-cli-board-list")
        continue
    fi

    # Step 2: bootloader handshake probe
    detected_baud="$(probe_avrdude "$port_real" || true)"
    if [[ -z "$detected_baud" ]]; then
        log "  no bootloader response — not an Arduino (or bootloader corrupt)"
        continue
    fi

    # Map (USB hint, bootloader baud) → (FQBN, variant)
    case "$hint:$detected_baud" in
        uno-genuine:115200)
            fqbn="arduino:avr:uno";              variant="uno-genuine" ;;
        ch340:115200|nano-ftdi:115200|nano-cp2102:115200)
            # CH340 at 115200: could be new-bootloader Nano OR Uno clone.
            # We default to nano-new because that's the documented Inno-Pilot
            # hardware.  If a fleet ever uses CH340 Uno clones, bump
            # INNO_BOARD_VARIANT manually after detection.
            fqbn="arduino:avr:nano:cpu=atmega328"; variant="nano-new" ;;
        *:57600)
            fqbn="arduino:avr:nano:cpu=atmega328old"; variant="nano-old" ;;
        *)
            fqbn="arduino:avr:nano";              variant="unknown" ;;
    esac

    log "  detected variant=$variant FQBN=$fqbn (baud=$detected_baud)"
    RESPONDERS+=("$byid_canonical")
    RESPONDER_PORTS+=("$port_real")
    RESPONDER_FQBNS+=("$fqbn")
    RESPONDER_VARIANTS+=("$variant")
    RESPONDER_METHODS+=("avrdude-probe-$detected_baud")
done

# ─── decide ───────────────────────────────────────────────────────────────────
n="${#RESPONDERS[@]}"
if (( n == 0 )); then
    die "no Arduino bootloader responded on any candidate port" 1
fi
if (( n > 1 )); then
    log "ERROR: multiple boards responded (ambiguous):"
    for ((i=0; i<n; i++)); do
        log "  ${RESPONDERS[$i]} -> ${RESPONDER_FQBNS[$i]} (${RESPONDER_VARIANTS[$i]})"
    done
    log "Unplug all but the intended board and re-run."
    exit 2
fi

CHOSEN_BYID="${RESPONDERS[0]}"
CHOSEN_PORT="${RESPONDER_PORTS[0]}"
CHOSEN_FQBN="${RESPONDER_FQBNS[0]}"
CHOSEN_VARIANT="${RESPONDER_VARIANTS[0]}"
CHOSEN_METHOD="${RESPONDER_METHODS[0]}"

log "Detected: $CHOSEN_VARIANT on $CHOSEN_PORT"
log "  FQBN  = $CHOSEN_FQBN"
log "  BYID  = $CHOSEN_BYID"
log "  baud  = $RUNTIME_BAUD"
log "  via   = $CHOSEN_METHOD"

# ─── render board.conf ────────────────────────────────────────────────────────
NOW="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

render_conf() {
    cat <<EOF
# Auto-generated by detect_arduino.sh — do not edit by hand.
# Delete this file or pass --redetect to detect_arduino.sh / inno_deploy.sh
# to re-probe (e.g. after a hardware swap).
INNO_BOARD_FQBN="$CHOSEN_FQBN"
INNO_BOARD_VARIANT="$CHOSEN_VARIANT"
INNO_BOARD_PORT="$CHOSEN_PORT"
INNO_BOARD_BYID="$CHOSEN_BYID"
INNO_BOARD_BYID_REAL="${CHOSEN_BYID}.real"
INNO_BOARD_BAUD="$RUNTIME_BAUD"
INNO_BOARD_BUILD_FLAGS="$DEFAULT_BUILD_FLAGS"
INNO_BOARD_DETECTED_AT="$NOW"
INNO_BOARD_DETECT_METHOD="$CHOSEN_METHOD"
EOF
}

if $DRY_RUN; then
    log "--dry-run — not writing $BOARD_CONF.  Would write:"
    render_conf
    exit 0
fi

# Atomic write: tmp file, fsync via mv, fix ownership.
sudo mkdir -p "$(dirname "$BOARD_CONF")"
render_conf | sudo tee "$BOARD_CONF_TMP" >/dev/null
sudo mv -f "$BOARD_CONF_TMP" "$BOARD_CONF"
sudo chown innopilot:innopilot "$BOARD_CONF" 2>/dev/null || true
sudo chmod 644 "$BOARD_CONF"

log "wrote $BOARD_CONF"
exit 0
