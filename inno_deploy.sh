#!/bin/bash
# =============================================================================
# Inno-Pilot full release deployment script
# Resides at ~/inno_deploy.sh on the Pi (OUTSIDE the repo to prevent git pull
# from modifying this script while it is running).
#
# Usage:
#   bash ~/inno_deploy.sh [branch]
#
# If branch is omitted, the current branch of ~/inno-pilot is used.
#
# What this script does:
#   1. git pull the repo
#   2. Pre-compile Nano firmware (while services are still up — no port needed)
#   3. Stop all inno-pilot + pypilot services
#   4. Deploy glue (bridge, web-remote, systemd units, OTA binary)
#   5. Flash the Nano via arduino-cli
#   6. Wait for the Nano to boot after flash
#   7. Start all services in dependency order
#   8. Print a final status summary
# =============================================================================

set -euo pipefail

# --------------- Configurable paths / names ---------------
REPO_DIR="$HOME/inno-pilot"
GLUE_DEPLOY="$REPO_DIR/compute_module/glue/deploy_inno_pilot_glue.sh"
NANO_SKETCH_DIR="$REPO_DIR/servo_motor_control/arduino/motor_simple"
ARDUINO_CLI="/usr/local/bin/arduino-cli"
NANO_PORT="/dev/ttyUSB0"
NANO_FQBN="arduino:avr:nano"
NANO_BUILD_FLAG="build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128"
NANO_BOOT_WAIT_S=6    # seconds to wait for Nano to finish its setup() splash after flash
BRIDGE_SETTLE_S=3     # seconds to let the bridge stabilise before starting pypilot
# ----------------------------------------------------------

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { echo "[ERROR] $*" >&2; exit 1; }
info() { echo; echo ">>> $*"; echo; }

# --------------- arduino-cli invocation ---------------
# Pi .12 (Pi5): arduino:avr core lives under /root/.arduino15; must use sudo.
# Pi .13 (Pi Zero): core lives under innopilot's own ~/.arduino15; no sudo needed.
# Detect by checking the core hardware directory on disk — avoids triggering a
# network index download (which would hang silently and OOM the Pi).
if [ -d "$HOME/.arduino15/packages/arduino/hardware/avr" ]; then
    ARDUINO=("$ARDUINO_CLI")
    ARDUINO15_ROOT="$HOME/.arduino15"
    log "arduino-cli: using user core (~/.arduino15)"
else
    ARDUINO=(sudo "$ARDUINO_CLI")
    ARDUINO15_ROOT="/root/.arduino15"
    log "arduino-cli: user core absent, falling back to sudo (root core)"
fi
# ------------------------------------------------------

# --------------- ARMv6 avrdude compatibility fix ---------------
# arduino-cli downloads avrdude compiled for armhf (ARMv7+). On ARMv6 (Pi Zero),
# executing an armhf binary causes SIGILL ("illegal instruction"). The system
# avrdude (apt package avrdude) is compiled for ARMv6 and works correctly.
# This function replaces the bundled binary with a symlink to the system one.
# The symlink is idempotent and survives reboots; it is only overwritten if
# arduino-cli explicitly reinstalls its avrdude package.
fix_avrdude_for_armv6() {
    local bundled_bin bundled_conf
    bundled_bin=$(find "$ARDUINO15_ROOT/packages/arduino/tools/avrdude" \
                  -name avrdude -type f 2>/dev/null | head -1)
    if [[ -z "$bundled_bin" ]]; then
        # Core tools not yet downloaded — nothing to fix; upload would fail anyway.
        log "avrdude compat: bundled avrdude not found in $ARDUINO15_ROOT — skipping fix."
        return
    fi

    # Ensure the system avrdude package is present.
    if ! command -v avrdude &>/dev/null; then
        log "avrdude compat: system avrdude not installed — installing via apt ..."
        sudo apt-get install -y avrdude
    fi
    local system_avrdude system_conf
    system_avrdude=$(command -v avrdude)
    # System avrdude config is at /etc/avrdude.conf
    system_conf="/etc/avrdude.conf"

    # --- binary ---
    if [[ -L "$bundled_bin" ]]; then
        log "avrdude compat: binary already symlinked ($(readlink "$bundled_bin")) — OK."
    else
        log "avrdude compat: symlinking binary -> $system_avrdude"
        if [[ "$bundled_bin" == /root/* ]]; then
            sudo ln -sf "$system_avrdude" "$bundled_bin"
        else
            ln -sf "$system_avrdude" "$bundled_bin"
        fi
    fi

    # --- config file ---
    # arduino-cli passes the bundled etc/avrdude.conf explicitly via -C. The 8.0.0
    # config format is incompatible with the system avrdude 7.x. Symlink it to the
    # system config so the versions stay in sync.
    bundled_conf="$(dirname "$(dirname "$bundled_bin")")/etc/avrdude.conf"
    if [[ -L "$bundled_conf" ]]; then
        log "avrdude compat: config already symlinked ($(readlink "$bundled_conf")) — OK."
    elif [[ ! -f "$system_conf" ]]; then
        log "avrdude compat: WARNING — system config $system_conf not found; upload may fail."
    else
        log "avrdude compat: symlinking config -> $system_conf"
        if [[ "$bundled_conf" == /root/* ]]; then
            sudo ln -sf "$system_conf" "$bundled_conf"
        else
            ln -sf "$system_conf" "$bundled_conf"
        fi
    fi

    log "avrdude compat: fix applied."
}
# ------------------------------------------------------

# Guard: refuse to run on the dev workstation
case "$(uname -m)" in
  arm* | aarch64) ;;
  *) die "This script is for the Raspberry Pi, not the dev workstation (arch: $(uname -m))." ;;
esac

# Determine branch
BRANCH="${1:-$(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD)}"

# ---------------------------------------------------------------------------
info "Step 1 — git pull (branch: $BRANCH)"
# ---------------------------------------------------------------------------
cd "$REPO_DIR"
git pull origin "$BRANCH"
log "Pull complete."

# ---------------------------------------------------------------------------
info "Step 2 — Pre-compile Nano firmware (services still running)"
# ---------------------------------------------------------------------------
# Compile does not touch /dev/ttyUSB0, so we can do this before stopping services
# to minimise the downtime window.
cd "$NANO_SKETCH_DIR"
log "Compiling $NANO_SKETCH_DIR ..."
"${ARDUINO[@]}" compile \
    --fqbn "$NANO_FQBN" \
    --build-property "$NANO_BUILD_FLAG" \
    .
log "Nano firmware compiled OK."

# ---------------------------------------------------------------------------
info "Step 3 — Stop services (consumers first, providers last)"
# ---------------------------------------------------------------------------
stop_svc() {
    local svc="$1"
    if systemctl list-unit-files "${svc}.service" &>/dev/null \
       && systemctl is-active --quiet "$svc" 2>/dev/null; then
        log "  Stopping $svc ..."
        sudo systemctl stop "$svc"
    else
        log "  $svc — already stopped or not installed, skipping."
    fi
}

stop_svc pypilot_web
stop_svc pypilot
stop_svc inno-pilot-web-remote
stop_svc inno-pilot-bridge
stop_svc inno-pilot-fixlink
stop_svc inno-pilot-socat

# ---------------------------------------------------------------------------
info "Step 4 — Deploy glue (bridge, web-remote, systemd units, OTA binary)"
# ---------------------------------------------------------------------------
bash "$GLUE_DEPLOY"
log "Glue deploy complete."

# ---------------------------------------------------------------------------
info "Step 5 — Flash Nano firmware"
# ---------------------------------------------------------------------------
# On ARMv6 (Pi Zero) arduino-cli bundles an armhf avrdude that crashes with
# SIGILL. Replace it with a symlink to the system avrdude before upload.
if [[ "$(uname -m)" == "armv6l" ]]; then
    fix_avrdude_for_armv6
fi

# NOTE: HUPCL is not relevant here because arduino-cli handles the reset pulse
# intentionally (needed for programming). After upload completes and arduino-cli
# closes the port, the Nano will reset once more — that is the boot we wait for
# in Step 6.
log "Flashing Nano on $NANO_PORT ..."
"${ARDUINO[@]}" upload \
    -p "$NANO_PORT" \
    --fqbn "$NANO_FQBN" \
    .
log "Flash complete."

# ---------------------------------------------------------------------------
info "Step 6 — Waiting ${NANO_BOOT_WAIT_S}s for Nano to complete setup() after flash"
# ---------------------------------------------------------------------------
# The Nano runs a ~3 s OLED splash in setup(). The bridge needs the Nano to be
# ready before sending FEATURES_CODE and HELLO, so give it ample time here.
sleep "$NANO_BOOT_WAIT_S"
log "Nano should be ready."

# ---------------------------------------------------------------------------
info "Step 7 — Start services (providers first, consumers last)"
# ---------------------------------------------------------------------------
start_svc() {
    local svc="$1"
    if systemctl list-unit-files "${svc}.service" &>/dev/null; then
        log "  Starting $svc ..."
        sudo systemctl start "$svc"
    else
        log "  $svc — unit not installed, skipping."
    fi
}

# socat must be up before fixlink and bridge use the PTY pair
start_svc inno-pilot-socat

# fixlink is a one-shot that redirects the USB symlink; it will go inactive after
# running — that is expected and not an error
start_svc inno-pilot-fixlink

# bridge grabs the real USB port and opens the TCP server on port 8555
start_svc inno-pilot-bridge

# web remote connects to the bridge's TCP server
start_svc inno-pilot-web-remote

# Let the bridge stabilise (FEATURES_CODE + HELLO exchange with Nano) before
# pypilot tries to grab the PTY
log "  Waiting ${BRIDGE_SETTLE_S}s for bridge to stabilise ..."
sleep "$BRIDGE_SETTLE_S"

# pypilot connects via the PTY symlink created by socat + fixlink
start_svc pypilot

# pypilot_web is optional (Pi5 second boat may have it; Pi Zero typically doesn't)
if systemctl list-unit-files pypilot_web.service &>/dev/null; then
    start_svc pypilot_web
fi

# ---------------------------------------------------------------------------
info "Step 8 — Service status summary"
# ---------------------------------------------------------------------------
SVCS=(inno-pilot-socat inno-pilot-fixlink inno-pilot-bridge inno-pilot-web-remote pypilot pypilot_web)
all_ok=true
for svc in "${SVCS[@]}"; do
    if ! systemctl list-unit-files "${svc}.service" &>/dev/null; then
        log "  $svc: (not installed)"
        continue
    fi
    status=$(systemctl is-active "$svc" 2>/dev/null || true)
    # fixlink is one-shot; inactive after running is normal
    if [[ "$svc" == "inno-pilot-fixlink" && "$status" == "inactive" ]]; then
        log "  $svc: inactive (one-shot — OK)"
    elif [[ "$status" == "active" ]]; then
        log "  $svc: active ✓"
    else
        log "  $svc: $status  ← CHECK THIS"
        all_ok=false
    fi
done

echo
if $all_ok; then
    log "=== Deployment complete — all services nominal ==="
else
    log "=== Deployment finished — one or more services need attention (see above) ==="
    exit 1
fi
