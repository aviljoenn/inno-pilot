# Inno-Pilot Glue – Claude Code Guide

This file is for **Claude Code** (and future human maintainers) working on the
Inno-Pilot glue on the Pi Zero.

## Critical: Nano reset on serial close (HUPCL)

The Arduino Nano resets every time `/dev/ttyUSB0` is closed because Linux drops
DTR on close by default (HUPCL flag).  The bridge neutralises this in
`open_serial_no_reset()` via `termios.tcsetattr(..., ~termios.HUPCL)`.

**Do not remove or bypass that call.**  If you open the port anywhere else
(debug scripts, sniffers, `stty` probes), run `stty -F /dev/ttyUSB0 -hupcl`
before closing, or the Nano will reset and be silent for ~5 s.  See the
"Known hardware gotcha" section in the top-level CLAUDE.md for full details.

---

## Principles

- The **repository is the source of truth**.
- Do **not** hand-edit:
  - `/usr/local/bin/inno_pilot_bridge.py`
  - `/usr/local/sbin/inno_pilot_fix_symlink.sh`
  - `/etc/systemd/system/inno-pilot-*.service`
  - `/etc/systemd/system/pypilot.service.d/override.conf`
- Instead:
  - Edit files under `compute_module/glue/` in the repo.
  - Commit and push to GitHub.
  - On the Pi, `git pull` and run the deploy script.

## Update / deployment workflow

When changing the glue (bridge logic, units, etc.):

1. **Edit in the repo**
   Make changes under `compute_module/glue/` (this directory).

2. **Commit and push**
   From your dev machine or the Pi:

   ```bash
   git add compute_module/glue
   git commit -m "Update Inno-Pilot glue"
   git push origin master
   ```

3. **To deploy on the Compute Module**
   Run the following on the Pi:

   ```bash
   sudo systemctl stop pypilot pypilot_web
   sudo systemctl stop inno-pilot-bridge inno-pilot-fixlink inno-pilot-socat || true

   cd ~/inno-pilot
   git pull origin master

   ./compute_module/glue/deploy_inno_pilot_glue.sh

   sudo reboot
   ```
