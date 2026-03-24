# Changelog

All notable changes to the Inno-Pilot system are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/).
Version applies to all three components (Bridge, Nano, Remote) simultaneously.

## [Unreleased]

## [v0.1.0] — 2026-03-20 — Baseline

First tagged baseline. Captures the working state before inno-remote integration.

### Components
- **Bridge** (`compute_module/glue/inno_pilot_bridge.py`) — pypilot ↔ Nano relay via PTY/socat
- **Nano** (`servo_motor_control/arduino/motor_simple/motor_simple.ino`) — motor controller with clutch, OLED, buzzer
- **Remote** (`inno-remote/firmware/inno_remote/`) — ESP32-C3 handheld, demo/simulator mode only

### What works
- pypilot autopilot with heading hold via bridge relay
- Serial CRC-8 framed protocol (6-byte frames, 38400 baud) between bridge and Nano
- Motor H-bridge drive with current sensing and stall protection
- Clutch engage/disengage with buzzer feedback
- Rudder position sensing via potentiometer (ADC)
- Port and starboard limit detection
- OLED status display on Nano (heading, rudder, AP state)
- Remote: Wi-Fi STA with auto-reconnect, OLED UI, button ladder, demo/simulation mode
- Deployment via `deploy_inno_pilot_glue.sh` on Pi

### Known limitations
- Remote is not integrated (demo mode only, no TCP connection)
- No OTA update capability for remote
- No version checking between components
- Single installation config (no per-boat provisioning)
