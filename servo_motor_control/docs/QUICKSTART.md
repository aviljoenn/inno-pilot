# Inno-Pilot Quickstart (Pi Zero + Nano + OLED)

This is the “do exactly this” bootstrap for a fresh Pi OS install.

Assumptions:
- Pi Zero is reachable via SSH.
- Repo is `https://github.com/aviljoenn/inno-pilot`.
- Nano is connected via USB (CH340) and shows up as `/dev/ttyUSB0`.
- You’re using Raspberry Pi OS (Bookworm) + systemd.
- You want everything to survive reboot and come up automatically.

---

## 0) Base OS prep

Download and burn Pi OS (Bookworm) Lite and burn to SD card from here
https://downloads.raspberrypi.com/raspios_oldstable_lite_arm64/images/raspios_oldstable_lite_arm64-2025-11-24/2025-11-24-raspios-bookworm-arm64-lite.img.xz

```bash
sudo apt-get update
sudo apt-get install -y \
  git \
  i2c-tools \
  python3 \
  python3-pip \
  python3-serial \
  socat
```

Enable I2C (if needed):
```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
sudo reboot
```

Verify IMU presence (example: LSM9DS1 shows 0x1e and 0x6b):
```bash
sudo i2cdetect -y 1
```

---

## 1) Clone the repo

```bash
cd ~
git clone https://github.com/aviljoenn/inno-pilot.git
cd ~/inno-pilot
```

---

## 2) Install/Update pypilot on the Pi (component)

If you already have pypilot installed and working, skip this section.

### 2.1 Install core build deps (safe defaults)
```bash
sudo apt-get install -y \
  python3-dev \
  python3-numpy \
  python3-scipy \
  python3-ujson \
  python3-pyudev \
  python3-zeroconf \
  python3-websocket \
  python3-flask \
  python3-flask-babel \
  gettext \
  swig
```

### 2.2 Install python packages needed by pypilot web
```bash
sudo python3 -m pip install --break-system-packages \
  "python-socketio<6" \
  "flask-socketio<6" \
  eventlet \
  importlib_metadata
```

### 2.3 Install RTIMULib2 (if not already installed)
If RTIMU import fails, build RTIMULib2 from the pypilot tree:
```bash
cd ~/inno-pilot/compute_module/pypilot/RTIMULib2/Linux/python
sudo python3 setup.py build
sudo python3 setup.py install
```

### 2.4 Install pypilot (pip install local)
```bash
cd ~/inno-pilot/compute_module/pypilot
sudo python3 -m pip install --break-system-packages .
```

Sanity:
```bash
which pypilot pypilot_web pypilot_client
```

---

## 3) Create pypilot systemd services (if not already)

If you already have working `/etc/systemd/system/pypilot.service` and `pypilot_web.service`, skip.

Example minimal services (edit paths if yours differ):
```bash
sudo tee /etc/systemd/system/pypilot.service >/dev/null <<'EOF'
[Unit]
Description=Pypilot Autopilot Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=aviljoen
ExecStart=/usr/bin/python3 /usr/local/bin/pypilot
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
```

```bash
sudo tee /etc/systemd/system/pypilot_web.service >/dev/null <<'EOF'
[Unit]
Description=Pypilot Web UI
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=aviljoen
ExecStart=/usr/bin/python3 /usr/local/bin/pypilot_web
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
```

Enable:
```bash
sudo systemctl daemon-reload
sudo systemctl enable pypilot pypilot_web
```

---

## 4) Install Arduino tooling (arduino-cli)

If arduino-cli is already installed and working, skip.

```bash
# If you already have arduino-cli in PATH, verify:
arduino-cli version
```

If not installed, install it (choose your preferred method).
Then install AVR core:
```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
```

---

## 5) Build + upload the Nano firmware (motor_simple)

### 5.1 Stop services that might hold /dev/ttyUSB0
```bash
sudo systemctl stop pypilot pypilot_web || true
sudo systemctl stop inno-pilot-bridge inno-pilot-socat inno-pilot-fixlink || true
```

### 5.2 Confirm Nano serial device
```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

### 5.3 Compile + upload
```bash
cd ~/inno-pilot/servo_motor_control/arduino/motor_simple
arduino-cli compile --fqbn arduino:avr:nano .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .
```

---

## 6) Deploy Inno-Pilot Glue (PTY + symlink + bridge)

This installs:
- `/usr/local/bin/inno_pilot_bridge.py`
- `/usr/local/sbin/inno_pilot_fix_symlink.sh`
- systemd units:
  - `inno-pilot-socat.service`
  - `inno-pilot-fixlink.service`
  - `inno-pilot-bridge.service`

Run deploy:
```bash
cd ~/inno-pilot
bash ./compute_module/glue/deploy_inno_pilot_glue.sh
```

Reboot recommended:
```bash
sudo reboot
```

---

## 7) After reboot: verify everything is up

### 7.1 Check services
```bash
systemctl status inno-pilot-socat --no-pager
systemctl status inno-pilot-fixlink --no-pager
systemctl status inno-pilot-bridge --no-pager
systemctl status pypilot --no-pager
systemctl status pypilot_web --no-pager
```

### 7.2 Verify PTYs exist
```bash
ls -l /dev/ttyINNOPILOT*
```

### 7.3 Verify by-id link redirect
```bash
ls -l /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0*
```

Expected shape:
- `usb-1a86_USB_Serial-if00-port0` -> `/dev/ttyINNOPILOT`
- `usb-1a86_USB_Serial-if00-port0.real` -> `../../ttyUSB0`

### 7.4 Verify open file handles
```bash
sudo lsof /dev/ttyINNOPILOT /dev/ttyINNOPILOT_BRIDGE \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real
```

You should see:
- `socat` holding the PTYs
- `inno_pilot_bridge.py` holding `ttyUSB0.real` and `/dev/ttyINNOPILOT_BRIDGE`
- `pypilot` holding `/dev/ttyINNOPILOT` (NOT ttyUSB0)

### 7.5 Verify pypilot sees servo values
```bash
pypilot_client servo.flags
pypilot_client ap.enabled
pypilot_client ap.heading_command
```

### 7.6 Tail the bridge (debug)
```bash
journalctl -u inno-pilot-bridge -f --no-pager
```

---

## 8) Normal update workflow (GitHub is source of truth)

When you change code on GitHub:

### 8.1 Stop services
```bash
sudo systemctl stop pypilot pypilot_web inno-pilot-bridge
sudo systemctl stop inno-pilot-socat inno-pilot-fixlink || true
```

### 8.2 Pull
```bash
cd ~/inno-pilot
git pull origin master
```

### 8.3 Redeploy glue (if glue changed)
```bash
bash ./compute_module/glue/deploy_inno_pilot_glue.sh
```

### 8.4 Rebuild + upload Nano sketch (if motor_simple changed)
```bash
cd ~/inno-pilot/servo_motor_control/arduino/motor_simple
arduino-cli compile --fqbn arduino:avr:nano .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .
```

### 8.5 Start services
```bash
sudo systemctl start inno-pilot-socat
sudo systemctl start inno-pilot-fixlink
sudo systemctl start inno-pilot-bridge
sudo systemctl start pypilot pypilot_web
```

---

## 9) Common failures & quick fixes

### Upload fails: “Device or resource busy”
Stop anything holding the serial port:
```bash
sudo systemctl stop pypilot pypilot_web inno-pilot-bridge
sudo lsof /dev/ttyUSB0
```

### pypilot binds to ttyUSB0 instead of PTY
The by-id redirect didn’t happen. Check:
```bash
systemctl status inno-pilot-fixlink --no-pager
ls -l /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0*
```

### Bridge running but not acting
Tail logs:
```bash
journalctl -u inno-pilot-bridge -f --no-pager
```

---

## 10) Notes
- Calibration lives in `~/.pypilot/pypilot.conf` and `~/.pypilot/servodevice`.
- Treat that folder as “operational state”; back it up when stable.
