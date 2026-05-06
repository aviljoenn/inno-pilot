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

## Critical: Nano UART cold-boot initialisation (DTR pulse required)

On a **cold power-on**, the CH340 USB-UART chip on the Nano starts with DTR LOW.
`open_serial_no_reset()` keeps DTR LOW (that is its purpose — no transition, no
reset), but the Nano's serial TX hardware does not initialise until it sees a proper
DTR pulse.

**Symptom:** Bridge sends frames continuously; Nano OLED shows "Bridge: waiting…"
indefinitely.  `journalctl` shows TX byte counts increasing but zero RX bytes.

**Fix already in place** (in `main()` of `inno_pilot_bridge.py`):

```python
nano.dtr = True
time.sleep(0.15)   # 150 ms — long enough for the RC differentiator on RESET
nano.dtr = False
log.info("DTR reset pulse sent to Nano")
```

HUPCL was already cleared by `open_serial_no_reset()`, so this explicit pulse is
the **only** reset the Nano sees during normal bridge operation.  The bridge then
waits 2 s (extended from 1 s) for the Nano to finish `setup()` before sending RCT
settings.

**Do not remove this pulse.**  It is required on every cold boot, not just after
power outages — any situation where DTR has been LOW since the Pi powered on triggers
this condition.

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
