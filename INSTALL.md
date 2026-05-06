# Inno-Pilot — Fresh Install Guide (Pi Zero 2W)

This guide covers building a new Inno-Pilot compute module from a blank SD card.
Target hardware: **Raspberry Pi Zero 2W**.

---

## Phase 1 — Flash the OS

**Use:** Raspberry Pi OS Lite **32-bit Bullseye** (not Bookworm, not 64-bit).

Why Bullseye 32-bit:
- pypilot's C extensions (RTIMULib2, wiringPi) build cleanly against it.
- `numpy`, `scipy`, `python3-serial` are available via `apt`.
- systemd is present — the inno-pilot glue layer requires it (3 service units).
- Avoids TinyCore/piCore (TinyPilot): Sean D'Epagnier's minimal distribution uses
  busybox init, not systemd, so the inno-pilot glue services cannot run there.

Flash steps:
1. Download **Raspberry Pi Imager**.
2. Choose *Raspberry Pi OS (Legacy, 32-bit) Lite — Bullseye*.
3. In Advanced Options (gear icon):
   - Set hostname (e.g. `autopilotzero`).
   - Enable SSH (password auth).
   - Set user `innopilot`, password `innopilot123`.
   - Configure Wi-Fi if needed.
4. Write to SD card.

---

## Phase 2 — First boot and base packages

SSH in once the Pi is on the network:

```bash
ssh innopilot@<pi-ip>
```

Update and install required packages:

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
  git python3-pip python3-dev python3-numpy python3-scipy \
  python3-serial python3-smbus i2c-tools socat \
  libgles2-mesa-dev libgles2 libgbm-dev pigpio
```

Enable I2C (for OLED and ADS1115 on the Nano side):

```bash
sudo raspi-config nonint do_i2c 0
```

---

## Phase 3 — Clone and install inno-pilot (includes pypilot)

The inno-pilot repo **is a fork of pypilot** — it contains pypilot's full history
and source tree plus all inno-pilot additions. A separate pypilot clone is not needed.

```bash
cd ~
git clone https://github.com/aviljoenn/inno-pilot.git inno-pilot
cd inno-pilot/compute_module/pypilot
sudo python3 setup.py install
```

`setup.py` lives at `compute_module/pypilot/` and installs pypilot and all its dependencies.
`compute_module/` contains all inno-pilot-specific additions on top.

---

## Phase 4 — Install arduino-cli (for future Nano firmware flashing)

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/
arduino-cli core update-index
arduino-cli core install arduino:avr
```

Verify:

```bash
arduino-cli version   # expect 1.x
```

---

## Phase 5 — Deploy the inno-pilot glue layer

The glue layer inserts a transparent bridge between pypilot and the Arduino Nano,
adds a TCP control server (port 8555) for the wireless remote, and manages a
PTY pair so pypilot never sees the raw USB device directly.

```bash
cd ~/inno-pilot
./compute_module/glue/deploy_inno_pilot_glue.sh
```

This script:
- Installs `inno_pilot_bridge.py` → `/usr/local/bin/`
- Installs `inno_web_remote.py` → `/usr/local/bin/`
- Installs `inno_pilot_fix_symlink.sh` → `/usr/local/sbin/`
- Installs 4 systemd units: `inno-pilot-socat`, `inno-pilot-fixlink`, `inno-pilot-bridge`, `inno-pilot-web-remote`
- Enables all four services
- Sets `~/.pypilot/servodevice` to the stable by-id USB path
- Creates a pypilot drop-in so pypilot starts after glue services
- Sets `/etc/inno-pilot/` and `/var/lib/inno-pilot/` to be owned by `innopilot`
  (required so the bridge and web-remote can read/write `settings.json`)

---

## Phase 6 — Configure pypilot

Start pypilot once to generate its default config:

```bash
python3 -m pypilot.autopilot &
sleep 5
kill %1
```

Verify `~/.pypilot/servodevice` contains:

```
["/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",38400]
```

(The deploy script sets this; the manual start above ensures the `~/.pypilot/` directory exists.)

---

## Phase 7 — Reboot and verify

```bash
sudo reboot
```

After ~60 s, SSH back in and check services:

```bash
sudo systemctl status inno-pilot-socat
sudo systemctl status inno-pilot-fixlink
sudo systemctl status inno-pilot-bridge
sudo systemctl status inno-pilot-web-remote
sudo systemctl status pypilot
```

All five should be `active`. Then check the bridge log for Nano comms:

```bash
journalctl -u inno-pilot-bridge -n 50 --no-pager
```

Look for lines like `Nano frame OK`, servo activity, or the TCP server binding on port 8555.

Check the web remote is up:

```bash
curl -s http://localhost:8888/health
# expect: {"status": "ok", "bridge_connected": true, ...}
```

Then open **http://192.168.6.13:8888/** in any browser on the boat LAN for the
web-based remote control UI. See [Phase 9a](#phase-9a--web-remote) below for
details.

---

## Phase 8 — Flash Arduino Nano firmware

**Stop services before touching the serial port:**

```bash
sudo systemctl stop pypilot inno-pilot-bridge inno-pilot-socat
```

Compile and flash from the Pi:

```bash
cd ~/inno-pilot/servo_motor_control/arduino/motor_simple
arduino-cli compile \
  --fqbn arduino:avr:nano \
  --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" \
  .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .
```

The `SERIAL_RX_BUFFER_SIZE=128` flag is **required** — the default 64-byte buffer
overflows during bridge telemetry bursts when the OLED I2C draw blocks `loop()`.

**Restart services after flashing:**

```bash
sudo systemctl start inno-pilot-socat inno-pilot-bridge pypilot
```

Allow ~5 s for the Nano to finish its boot splash before expecting normal frames.

---

## Phase 9a — Web remote

The **web remote** is a browser-based replica of the physical Inno-Remote UI that
runs on the Pi itself and is accessible from any phone, tablet, or laptop on the
boat LAN — no app install required.

**Port:** 8888 (HTTP + Server-Sent Events)
**URL:** `http://192.168.6.13:8888/`
**Service:** `inno-pilot-web-remote` (auto-started by systemd after deploy)

### What it provides

- OLED-style display: mode, heading, AP command or rudder %, rudder position bar
- 5 heading buttons: `<<` (−10°) `<` (−1°) `|` (AP toggle) `>` (+1°) `>>` (+10°)
- STOP button: sends ESTOP (immediate AP disengage)
- 3-position mode toggle: AUTO / OFF / MANUAL
- Draggable ship's wheel for manual rudder control in MANUAL mode
- "NO BRIDGE" overlay when the bridge TCP connection is down

### Architecture

```
Browser (HTTP+SSE) <-> inno_web_remote.py (port 8888)
                            |
                     TCP client (port 8555)
                            |
                    inno-pilot-bridge
```

The web remote connects to the bridge as a single TCP client using the same
protocol as the ESP32 handheld.  Only one TCP client is accepted by the bridge
at a time; when the web remote is active, the ESP32 remote cannot connect.

### Log

```bash
journalctl -u inno-pilot-web-remote -n 50 --no-pager
```

---

## Phase 9b — ESP32 wireless remote firmware

> **Note:** The ESP32 firmware currently has the Pi IP address **hardcoded**.
> Before flashing for a new boat, update the target IP in the firmware source,
> rebuild, and flash.

From the dev workstation (Git Bash, ESP-IDF env loaded):

```bash
source ~/esp-idf-env.sh
cd /c/gitsrc/inno-pilot/inno-remote/firmware/inno_remote
# Edit the hardcoded Pi IP if this is a new boat install
idf.py build
idf.py -p COMx flash
```

---

## Update workflow (ongoing)

When new code is pushed to GitHub:

```bash
# On the Pi:
sudo systemctl stop pypilot
sudo systemctl stop inno-pilot-bridge inno-pilot-fixlink inno-pilot-socat || true

cd ~/inno-pilot
git pull origin master   # or feature branch

./compute_module/glue/deploy_inno_pilot_glue.sh

sudo reboot
```

---

## Troubleshooting quick reference

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Nano silent after serial open | HUPCL reset (DTR drop) | `stty -F /dev/ttyUSB0 -hupcl`; wait 5 s |
| pypilot not finding servo | Wrong servodevice path | Check `~/.pypilot/servodevice`; re-run deploy script |
| Bridge not starting | socat PTY not ready | `systemctl status inno-pilot-socat`; check logs |
| High CRC error rate | Noisy USB cable or buffer overflow | Check `SERIAL_RX_BUFFER_SIZE=128` was used; replace cable |
| Remote can't connect | Wrong IP in ESP32 firmware | Rebuild firmware with correct Pi IP |
| Web remote shows NO BRIDGE | Bridge not running or web remote started first | `systemctl restart inno-pilot-web-remote`; check bridge log |
| Web remote 404 / no page | Service not deployed | Re-run deploy script; check `ls /usr/local/bin/inno_web_remote.py` |
| Settings not saved / always reset to defaults | `/var/lib/inno-pilot/` owned by root | `sudo chown innopilot:innopilot /var/lib/inno-pilot` (deploy script handles this from B32 onward) |
| Settings panel shows "⚠ Using local settings" even when bridge is running | Bridge client thread was in OFF mode during settings open (fixed in B32) | Deploy B32 or later |

---

## Appendix A — Pi5 / Bookworm reinstall notes

The main guide above targets **Pi Zero 2W + Bullseye**. If you are reinstalling on a
**Raspberry Pi 5 (or Pi 4/400) running Raspberry Pi OS Bookworm**, the procedure
differs in several places.  These differences were discovered during the April 2026
reinstall of the `.12` boat Pi5.

### OS / Python differences on Bookworm

- Python 3.12+ (sometimes 3.13) is the system interpreter.
- PEP 668 is enforced: `pip install` to the system site-packages is blocked with
  "externally-managed-environment". You **cannot** use `pip install` for the pypilot
  dependencies.
- Flask ships at version 3.x on Bookworm.  `flask.Markup` was removed in Flask 2.3
  and moved to `markupsafe`; the inno-pilot source already has the fix.

### Phase 2 — Base packages (Bookworm variant)

Replace the `apt-get install` from the main guide with:

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
  git python3-pip python3-dev python3-numpy python3-scipy \
  python3-serial python3-smbus i2c-tools socat \
  libgles2-mesa-dev libgles2 libgbm-dev \
  python3-flask python3-flask-socketio \
  python3-importlib-metadata
```

Note: `pigpio` is not available by default on Bookworm arm64 — omit it unless you
specifically need it (it is not required by the glue layer).

### Phase 3 — Install pypilot (Bookworm variant)

**Do NOT use `pip install`** for pypilot on Bookworm. The SWIG-generated C extension
modules (`linebuffer`, `arduino_servo`, `ugfx`) must be placed by the old-style
installer to the correct locations.  Use:

```bash
cd ~/inno-pilot/compute_module/pypilot
sudo python3 setup.py install
```

If `dependencies.py` fails with `externally-managed-environment` errors:

```bash
sudo apt-get install -y python3-importlib-metadata
sudo python3 dependencies.py   # retry after apt install
```

Then proceed with `sudo python3 setup.py install`.

### After install: fix pypilot_client.conf

After the first boot, check and correct the pypilot client config:

```bash
cat ~/.pypilot/pypilot_client.conf
```

If this shows any IP other than `127.0.0.1` (e.g. a stale `.13` address from a
previous installation), fix it:

```bash
echo '{"host":"127.0.0.1","port":23322}' > ~/.pypilot/pypilot_client.conf
sudo systemctl restart pypilot
```

pypilot's zeroconf discovery auto-writes this file when it finds a remote server on
the LAN.  On a Pi that was previously connected to another pypilot instance this file
will be stale.

### Nano cold-boot UART note

On cold power-on the Nano's UART TX does not initialise until a proper DTR
transition occurs. The bridge (B35+) now pulses DTR HIGH→LOW immediately after
opening the serial port to force UART initialisation. No manual intervention is
required — this is handled automatically.

**If you see "Bridge: waiting…" stuck on the OLED after a cold boot** with an older
bridge (pre-B35), the workaround is:

```bash
sudo systemctl restart inno-pilot-bridge
```

One restart is enough — the bridge holds DTR at the correct level after the first
start.

### Flask 2.3+ note

If `pypilot_web` fails with:
```
ImportError: cannot import name 'Markup' from 'flask'
```
the deployed `web.py` is from upstream pypilot (not the inno-pilot fork).  Replace it:

```bash
sudo cp ~/inno-pilot/compute_module/pypilot/web/web.py \
        /usr/local/lib/python3*/dist-packages/pypilot/web/web.py
sudo systemctl restart pypilot_web
```

This fix is already in the inno-pilot source; `sudo python3 setup.py install` will
place the correct file automatically.
