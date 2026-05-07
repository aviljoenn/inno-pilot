#!/usr/bin/env bash
# Inno-Pilot bootstrap installer
#
# One-command fresh install on any Raspberry Pi running Raspberry Pi OS Lite
# (Bullseye or Bookworm, 32-bit or 64-bit):
#
#   curl -fsSL https://raw.githubusercontent.com/aviljoenn/inno-pilot/master/install.sh | bash
#
# Safe to re-run: each phase is idempotent.

set -euo pipefail

# Prevent apt from opening interactive pagers (apt-listchanges, debconf, etc.)
export DEBIAN_FRONTEND=noninteractive

REPO_URL="https://github.com/aviljoenn/inno-pilot.git"
REPO_DIR="$HOME/inno-pilot"
LOG="$HOME/inno-pilot-install.log"

# ── helpers ───────────────────────────────────────────────────────────────────

step() { echo; echo "==> $*"; }
info() { echo "    $*"; }
die()  { echo "ERROR: $*" >&2; exit 1; }

# Tee all output to log file
exec > >(tee -a "$LOG") 2>&1
echo "=== Inno-Pilot install started $(date) ==="

# ── detect OS / hardware ──────────────────────────────────────────────────────

step "Detecting hardware and OS"

PI_MODEL="unknown"
if [ -f /proc/device-tree/model ]; then
    PI_MODEL="$(tr -d '\0' </proc/device-tree/model)"
fi
info "Pi model : $PI_MODEL"

OS_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME:-unknown}" | tr '[:upper:]' '[:lower:]')"
PYTHON_VER="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
info "OS       : $OS_CODENAME"
info "Python   : $PYTHON_VER"

# ── Phase 1 — base packages ───────────────────────────────────────────────────

step "Phase 1: installing base packages"

sudo apt-get update -y
sudo apt-get upgrade -y

# Packages common to all Pi OS versions
COMMON_PKGS=(
    git python3-pip python3-dev
    python3-numpy python3-scipy
    python3-serial python3-smbus
    i2c-tools socat
    libgles2-mesa-dev libgles2 libgbm-dev
    python3-setuptools
)

# Bookworm / Trixie (Debian 12/13) ship Flask 3.x via apt and block pip (PEP 668).
# Install web-stack packages via apt up front so dependencies.py never needs pip.
# Bullseye (Debian 11) still uses the old pip-based path.
case "$OS_CODENAME" in
    bookworm|trixie)
        EXTRA_PKGS=(
            python3-flask python3-flask-socketio python3-socketio
            python3-importlib-metadata
        )
        ;;
    *)
        # Bullseye or unknown: pigpio available; flask-socketio via pip in dependencies.py
        EXTRA_PKGS=(pigpio python3-flask)
        ;;
esac

# shellcheck disable=SC2086
sudo apt-get install -y "${COMMON_PKGS[@]}" "${EXTRA_PKGS[@]}"

step "Phase 1: enabling I2C"
sudo raspi-config nonint do_i2c 0

# ── Phase 2 — clone repo ──────────────────────────────────────────────────────

step "Phase 2: cloning inno-pilot repo"

if [ -d "$REPO_DIR/.git" ]; then
    info "Repo already exists — pulling latest master"
    git -C "$REPO_DIR" fetch origin
    git -C "$REPO_DIR" checkout master
    git -C "$REPO_DIR" pull --ff-only origin master
else
    git clone "$REPO_URL" "$REPO_DIR"
fi

# ── Phase 3 — install pypilot ─────────────────────────────────────────────────

step "Phase 3: installing pypilot (this takes several minutes on slow Pi hardware)"

cd "$REPO_DIR/compute_module/pypilot"
sudo python3 setup.py install

# dependencies.py (run inside setup.py above) installs pypilot_data, which copies a
# pyproject.toml into this directory.  On setuptools 78 (Trixie / Python 3.13) that
# pyproject.toml is treated as authoritative and overrides setup(packages=…), causing
# build_py to skip the pypilot Python source entirely — only the C extensions land in
# dist-packages.  Detect the condition, remove the interfering file, and re-run so the
# Python modules are properly installed.  The second pass is fast because the C objects
# are already compiled in build/.
if ! python3 -c "import pypilot.autopilot" 2>/dev/null; then
    info "pypilot Python modules missing (pyproject.toml interference) — re-running install"
    rm -f pyproject.toml
    sudo python3 setup.py install
fi

# ── Phase 4 — arduino-cli ────────────────────────────────────────────────────

step "Phase 4: installing arduino-cli"

if command -v arduino-cli >/dev/null 2>&1; then
    info "arduino-cli already installed: $(arduino-cli version)"
else
    # BINDIR controls where the arduino-cli installer places the binary.
    # env-var prefix on a piped command (VAR=x cmd | sh) only applies to the
    # left side of the pipe (curl), NOT to the right side (sh).  Download the
    # installer to a file first so we can pass BINDIR to sh directly.
    ARDUINO_TMP="$(mktemp -d)"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        -o "$ARDUINO_TMP/arduino-cli-install.sh"
    BINDIR="$ARDUINO_TMP" sh "$ARDUINO_TMP/arduino-cli-install.sh"
    sudo mv "$ARDUINO_TMP/arduino-cli" /usr/local/bin/arduino-cli
    rm -rf "$ARDUINO_TMP"
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

# ── Phase 5 — deploy glue layer ───────────────────────────────────────────────

step "Phase 5: deploying inno-pilot glue layer"

cd "$REPO_DIR"
bash compute_module/glue/deploy_inno_pilot_glue.sh

# ── Phase 6 — initialise pypilot config directory ────────────────────────────

step "Phase 6: initialising pypilot config"

if [ ! -d "$HOME/.pypilot" ]; then
    info "Starting pypilot briefly to generate default config..."
    python3 -m pypilot.autopilot &
    PYPILOT_PID=$!
    sleep 6
    kill "$PYPILOT_PID" 2>/dev/null || true
    wait "$PYPILOT_PID" 2>/dev/null || true
else
    info "~/.pypilot already exists, skipping init run"
fi

# Ensure servodevice points to the stable by-id path.
# mkdir -p guards against pypilot failing to start above (e.g. import error on first run).
mkdir -p "$HOME/.pypilot"
SERVOFILE="$HOME/.pypilot/servodevice"
echo '["/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",38400]' > "$SERVOFILE"
info "servodevice set to USB by-id path"

# ── Phase 7 — fix stale pypilot_client.conf ──────────────────────────────────

case "$OS_CODENAME" in
    bookworm|trixie)
        step "Phase 7: resetting pypilot_client.conf to localhost ($OS_CODENAME)"
        mkdir -p "$HOME/.pypilot"
        echo '{"host":"127.0.0.1","port":23322}' > "$HOME/.pypilot/pypilot_client.conf"
        info "pypilot_client.conf set to 127.0.0.1"
        ;;
esac

# ── Done ──────────────────────────────────────────────────────────────────────

echo
echo "============================================================"
echo " Inno-Pilot installation complete!"
echo " Log saved to: $LOG"
echo
echo " Next step (if Arduino Nano is connected):"
echo "   sudo systemctl stop pypilot inno-pilot-bridge inno-pilot-socat"
echo "   cd $REPO_DIR/servo_motor_control/arduino/motor_simple"
echo '   arduino-cli compile --fqbn arduino:avr:nano \'
echo '     --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" .'
echo "   arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano ."
echo
echo " Rebooting in 10 seconds — Ctrl-C to cancel."
echo "============================================================"

sleep 10
sudo reboot
