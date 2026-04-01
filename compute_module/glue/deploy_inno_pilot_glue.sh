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

# Ensure /etc/inno-pilot exists and is writable by the bridge service user (aviljoen)
echo "Ensuring /etc/inno-pilot config dir exists..."
sudo mkdir -p /etc/inno-pilot
sudo chown aviljoen:aviljoen /etc/inno-pilot

# Copy bridge script
echo "Installing inno_pilot_bridge.py -> /usr/local/bin/"
sudo cp "$REPO_DIR/inno_pilot_bridge.py" /usr/local/bin/inno_pilot_bridge.py
sudo chmod 755 /usr/local/bin/inno_pilot_bridge.py

# Copy web remote script
echo "Installing inno_web_remote.py -> /usr/local/bin/"
sudo cp "$REPO_DIR/inno_web_remote.py" /usr/local/bin/inno_web_remote.py
sudo chmod 755 /usr/local/bin/inno_web_remote.py

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

# Copy systemd units
echo "Installing systemd units -> /etc/systemd/system/"
sudo cp "$REPO_DIR/inno-pilot-socat.service"      /etc/systemd/system/inno-pilot-socat.service
sudo cp "$REPO_DIR/inno-pilot-fixlink.service"    /etc/systemd/system/inno-pilot-fixlink.service
sudo cp "$REPO_DIR/inno-pilot-bridge.service"     /etc/systemd/system/inno-pilot-bridge.service
sudo cp "$REPO_DIR/inno-pilot-web-remote.service" /etc/systemd/system/inno-pilot-web-remote.service

# Reload systemd
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable services
echo "Enabling Inno-Pilot services..."
sudo systemctl enable inno-pilot-socat.service
sudo systemctl enable inno-pilot-fixlink.service
sudo systemctl enable inno-pilot-bridge.service
sudo systemctl enable inno-pilot-web-remote.service

# Ensure pypilot uses by-id name
SERVOFILE="$HOME/.pypilot/servodevice"
if [ -f "$SERVOFILE" ]; then
  echo "Ensuring servodevice uses by-id path..."
  echo '["/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",38400]' > "$SERVOFILE"
fi

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
