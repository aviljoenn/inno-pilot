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
    avrdude
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
# dist-packages.
#
# Naive remediation (rm pyproject.toml + re-run) loses the race: setup.py re-imports
# dependencies, which re-runs install_data.sh, which re-creates pyproject.toml BEFORE
# setup() is called.  The data idempotency check `os.path.exists('ui/compass.png')` no
# longer matches upstream pypilot_data's current layout (compass.png moved into a
# pypilot_data/ subdir), so install_data.sh runs unconditionally on every pass.
#
# Workaround: tell setup.py to skip `import dependencies` entirely by creating a stub
# `deps` file (the same marker dependencies.py writes on full success).  The first pass
# above has already fetched RTIMULib2 and pypilot_data, so the second pass only needs
# to install the pypilot Python modules and entry-point scripts.
if ! python3 -c "import pypilot.autopilot" 2>/dev/null; then
    info "pypilot Python modules missing — bypassing dependencies.py and retrying"
    rm -f pyproject.toml
    touch deps
    sudo python3 setup.py install
    rm -f deps
fi

# Hard verification — fail the install loudly here rather than letting Phase 6 crash
# on a partial pypilot.  /usr/local/bin/pypilot is the entry-point script systemd
# launches; if it's missing the autopilot will never start.
#
# The check must run with cwd outside the source tree.  Python prepends '' (cwd)
# to sys.path when running with -c, and the source tree contains pypilot/__init__.py
# but no compiled _linebuffer.so — so importing from source dir would falsely fail
# with "cannot import name '_linebuffer'".  At runtime systemd starts /usr/local/bin/pypilot
# with cwd=/, which resolves correctly via dist-packages.
( cd / && python3 -c "import pypilot.autopilot" ) \
    || die "pypilot Python package failed to install — autopilot would not start"
[ -x /usr/local/bin/pypilot ] \
    || die "pypilot entry-point script /usr/local/bin/pypilot was not created"

# Upstream pypilot ships pypilot.service in scripts/debian/etc/systemd/system/
# but its README expects the integrator to `sudo cp -r etc /` manually.  Our
# install.sh has to do that step explicitly, otherwise systemctl reports
# "Unit pypilot.service could not be found" after install and the autopilot
# never starts on boot.
#
# We only install pypilot.service itself (the autopilot core).  pypilot_web,
# pypilot_hat, and pypilot_boatimu are not used by Inno-Pilot and would only
# add maintenance burden if installed.
#
# The shipped unit hard-codes User=pi, which doesn't exist on Inno-Pilot Pis
# (we use innopilot).  Patch the User line in place during install.
PYPILOT_UNIT_SRC="$REPO_DIR/compute_module/pypilot/scripts/debian/etc/systemd/system/pypilot.service"
PYPILOT_UNIT_DST="/etc/systemd/system/pypilot.service"
if [ -f "$PYPILOT_UNIT_SRC" ]; then
    info "Installing pypilot.service unit (with User=innopilot)"
    sudo install -m 644 "$PYPILOT_UNIT_SRC" "$PYPILOT_UNIT_DST"
    sudo sed -i 's/^User=pi$/User=innopilot/' "$PYPILOT_UNIT_DST"
    sudo systemctl daemon-reload
    sudo systemctl enable pypilot.service
else
    info "WARNING: pypilot.service template not found at $PYPILOT_UNIT_SRC — autopilot will not auto-start"
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

# Place inno_deploy.sh outside the repo so future `git pull` cannot modify it
# mid-execution (the script self-updates by copying the repo version over this
# one and re-execing — see comments at the top of inno_deploy.sh).  Without
# this step a fresh install has no working OTA / manual-deploy path.
install -m 755 "$REPO_DIR/inno_deploy.sh" "$HOME/inno_deploy.sh"
info "inno_deploy.sh placed at $HOME/inno_deploy.sh"

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
