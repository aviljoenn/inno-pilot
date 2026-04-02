# Inno-Pilot Bridge – Pi Zero ↔ Nano ↔ pypilot

This directory contains the **Inno-Pilot-Bridge**: everything that connects the
Inno-Pilot servo controller (Nano) to any other logic instance connected to or running on the compute module (Raspberry Pi) where examples include, but are not limited to: Pypilot, inno-web-remote, inno-remote

## What this bridge does

- Keeps pypilot talking to the same **USB by-id** name it expects:
  `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- Moves the original by-id symlink to:
  `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real`
- Creates a PTY pair using `socat`:
  - `/dev/ttyINNOPILOT`
  - `/dev/ttyINNOPILOT_BRIDGE`
- Repoints the by-id name to `/dev/ttyINNOPILOT` (the PTY)
- Runs a bridge process that:
  - Connects the real Nano USB (`...port0.real`) to `/dev/ttyINNOPILOT_BRIDGE`
  - Forwards all servo frames unchanged
  - Uses a lightweight magic-byte wrapper between bridge ↔ Nano so pypilot
    cannot bind directly to the Nano when multiple USB devices are present
  - Listens for `BUTTON_EVENT_CODE` (0xE0) frames from the Nano and translates
    them into pypilot API calls:
    - `ap.enabled` toggle
    - `ap.heading_command` ±1/±10 degrees
  - Runs a TCP server on port **8555** for the ESP32-C3 wireless remote or the inno-web-remote web service listening on port **8888**:
    - Receives: `BTN TOGGLE/±1/±10`, `ESTOP`, `MODE MANUAL/AUTO`, `RUD 0-100%`
    - Sends: `AP`, `HDG`, `CMD`, `RDR`, `RDR_PCT`, `FLAGS`, `MODE`, `WARN` at ~1Hz
  - Manages a mode state machine (`IDLE / AP / MANUAL`):
    - In MANUAL mode, translates pot-based RUD commands to Nano motor frames
    - On TCP disconnect in MANUAL: triggers steer-loss warning on Nano (life-safety)
  - Parses `FLAGS_CODE` and `BUZZER_STATE_CODE` from Nano for remote relay
  - Validates CRC on all Nano→bridge frames; corrupt frames are dropped (never forwarded to pypilot)
  - Monitors Nano-reported comms error rate via `COMMS_DIAG_CODE` (0xEC) telemetry; auto-disengages
    AP if error rate exceeds safety threshold; sends `COMMS WARN`/`COMMS CRIT` to TCP remote

In short: pypilot still sees its normal servo device, but we have inserted a
transparent bridge in the middle that adds Inno-Pilot control head + wireless remote features.

## Files

- `inno_pilot_bridge.py`  
  Bridge between Nano and PTY. Forwards all traffic and handles button
  events via `pypilotClient`.

- `inno_pilot_fix_symlink.sh`  
  Moves the original by-id symlink to `.real` and points the original name at
  `/dev/ttyINNOPILOT`.

- `inno-pilot-socat.service`  
  systemd unit to create the PTY pair `/dev/ttyINNOPILOT` and
  `/dev/ttyINNOPILOT_BRIDGE`.

- `inno-pilot-fixlink.service`  
  systemd oneshot to run `inno_pilot_fix_symlink.sh` after socat starts.

- `inno-pilot-bridge.service`  
  systemd unit to run `inno_pilot_bridge.py` as user `aviljoen` (group `dialout`).

- `deploy_inno_pilot_bridge.sh`  
  Deployment helper that copies scripts into `/usr/local/bin` and
  `/usr/local/sbin`, installs/updates the systemd units, and ensures
  `pypilot.service` starts after the bridge.

- `CLAUDE.md`
  Guidance for Claude Code (and future humans) on how to maintain this bridge layer.

## Deployment (on the Pi Zero)

After updating the repo:

```bash
sudo systemctl stop pypilot pypilot_web
sudo systemctl stop inno-pilot-bridge inno-pilot-fixlink inno-pilot-socat || true

cd ~/inno-pilot
git pull origin master

./compute_module/bridge/deploy_inno_pilot_bridge.sh

sudo reboot
```

After reboot:

- inno-pilot-socat creates PTYs
- inno-pilot-fixlink adjusts the by-id symlink
- inno-pilot-bridge connects Nano ↔ PTY
- pypilot starts and uses the PTY via the by-id name
