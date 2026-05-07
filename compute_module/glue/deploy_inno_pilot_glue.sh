#!/bin/bash
set -e

# Run this from anywhere inside the repository:
#   cd ~/inno-pilot
#   ./compute_module/glue/deploy_inno_pilot_glue.sh

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$REPO_DIR/../.." && pwd)"

echo "Inno-Pilot deploy (glue): using repo dir $REPO_DIR"

# Ensure socat is installed
if ! command -v socat >/dev/null 2>&1; then
  echo "ERROR: socat is not installed. Install with:"
  echo "  sudo apt-get update && sudo apt-get install -y socat"
  exit 1
fi

# Ensure /etc/inno-pilot exists and is writable by the bridge service user (innopilot)
echo "Ensuring /etc/inno-pilot config dir exists..."
sudo mkdir -p /etc/inno-pilot
sudo chown innopilot:innopilot /etc/inno-pilot

# Ensure /var/lib/inno-pilot exists and is writable by innopilot.
# Both the bridge and the web-remote write settings.json here; without this
# chown the directory is root-owned and neither process can create the file.
echo "Ensuring /var/lib/inno-pilot data dir is owned by innopilot..."
sudo mkdir -p /var/lib/inno-pilot
sudo chown innopilot:innopilot /var/lib/inno-pilot

# Copy bridge script
echo "Installing inno_pilot_bridge.py -> /usr/local/bin/"
sudo cp "$REPO_DIR/inno_pilot_bridge.py" /usr/local/bin/inno_pilot_bridge.py
sudo chmod 755 /usr/local/bin/inno_pilot_bridge.py

# Copy web remote script
echo "Installing inno_web_remote.py -> /usr/local/bin/"
sudo cp "$REPO_DIR/inno_web_remote.py" /usr/local/bin/inno_web_remote.py
sudo chmod 755 /usr/local/bin/inno_web_remote.py

# Copy health notify script
echo "Installing inno_health_notify.py -> /usr/local/bin/"
sudo cp "$REPO_DIR/inno_health_notify.py" /usr/local/bin/inno_health_notify.py
sudo chmod 755 /usr/local/bin/inno_health_notify.py

# Copy OTA firmware binary (if committed to repo)
OTA_BIN="$REPO_ROOT/inno-remote/firmware/inno_remote/ota/inno_remote.bin"
OTA_DEST_DIR="/var/lib/inno-pilot/ota"
if [ -f "$OTA_BIN" ]; then
  echo "Installing OTA firmware -> $OTA_DEST_DIR/"
  sudo mkdir -p "$OTA_DEST_DIR"
  sudo cp "$OTA_BIN" "$OTA_DEST_DIR/inno_remote.bin"
  sudo chmod 644 "$OTA_DEST_DIR/inno_remote.bin"
else
  echo "OTA firmware not found at $OTA_BIN — skipping (OTA unavailable until binary is committed)"
fi

# Copy symlink fix helper
echo "Installing inno_pilot_fix_symlink.sh -> /usr/local/sbin/"
sudo cp "$REPO_DIR/inno_pilot_fix_symlink.sh" /usr/local/sbin/inno_pilot_fix_symlink.sh
sudo chmod 755 /usr/local/sbin/inno_pilot_fix_symlink.sh

# Copy board-config loader (bash) and the Arduino auto-detect probe.
# Installed to /usr/local/sbin so they're in root's PATH and reachable by
# fixlink + inno_deploy.sh + future automation.
echo "Installing board_conf.sh + detect_arduino.sh -> /usr/local/sbin/"
sudo cp "$REPO_DIR/board_conf.sh"        /usr/local/sbin/board_conf.sh
sudo cp "$REPO_DIR/detect_arduino.sh"    /usr/local/sbin/detect_arduino.sh
sudo chmod 755 /usr/local/sbin/board_conf.sh /usr/local/sbin/detect_arduino.sh

# Copy board-config Python loader next to the bridge so `import board_conf`
# resolves via Python's automatic insertion of the script's parent directory
# into sys.path (no PYTHONPATH or site-packages plumbing required).
echo "Installing board_conf.py -> /usr/local/bin/"
sudo cp "$REPO_DIR/board_conf.py" /usr/local/bin/board_conf.py
sudo chmod 644 /usr/local/bin/board_conf.py

# Copy systemd units
echo "Installing systemd units -> /etc/systemd/system/"
sudo cp "$REPO_DIR/inno-pilot-socat.service"      /etc/systemd/system/inno-pilot-socat.service
sudo cp "$REPO_DIR/inno-pilot-fixlink.service"    /etc/systemd/system/inno-pilot-fixlink.service
sudo cp "$REPO_DIR/inno-pilot-bridge.service"     /etc/systemd/system/inno-pilot-bridge.service
sudo cp "$REPO_DIR/inno-pilot-web-remote.service" /etc/systemd/system/inno-pilot-web-remote.service
sudo cp "$REPO_DIR/inno-health-notify.service"    /etc/systemd/system/inno-health-notify.service

# Reload systemd
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable services
echo "Enabling Inno-Pilot services..."
sudo systemctl enable inno-pilot-socat.service
sudo systemctl enable inno-pilot-fixlink.service
sudo systemctl enable inno-pilot-bridge.service
sudo systemctl enable inno-pilot-web-remote.service
sudo systemctl enable inno-health-notify.service

# Ensure pypilot uses by-id name.  Source board.conf so the path matches the
# detected Arduino — falls back to the historical CH340 Nano path if the file
# is missing (existing .12/.13 fleet during transitional rollout).
SERVOFILE="$HOME/.pypilot/servodevice"
mkdir -p "$(dirname "$SERVOFILE")"
SERVO_BYID="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SERVO_BAUD="38400"
if [ -f /var/lib/inno-pilot/board.conf ]; then
  # shellcheck disable=SC1091
  source /var/lib/inno-pilot/board.conf
  SERVO_BYID="${INNO_BOARD_BYID:-$SERVO_BYID}"
  SERVO_BAUD="${INNO_BOARD_BAUD:-$SERVO_BAUD}"
  echo "Writing servodevice from board.conf: $SERVO_BYID @ $SERVO_BAUD"
else
  echo "board.conf not present yet — writing servodevice with default CH340 path"
fi
echo "[\"$SERVO_BYID\",$SERVO_BAUD]" > "$SERVOFILE"

# Make pypilot start after our services via drop-in override
echo "Configuring pypilot to start after Inno-Pilot services..."
sudo mkdir -p /etc/systemd/system/pypilot.service.d
sudo bash -c 'cat >/etc/systemd/system/pypilot.service.d/override.conf' <<EOV
[Unit]
After=inno-pilot-socat.service inno-pilot-fixlink.service inno-pilot-bridge.service
EOV

sudo systemctl daemon-reload

echo "Inno-Pilot glue deploy complete."
echo "Recommended: reboot to test full boot sequence:"
echo "  sudo reboot"
