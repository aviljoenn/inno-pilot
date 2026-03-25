# Changelog

All notable changes to the Inno-Pilot system are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/).
Version applies to all three components (Bridge, Nano, Remote) simultaneously.

## [Unreleased]

## [v0.2.0_B7] — 2026-03-25 — Remote Integration (Tasks #2–#10)

### Added
- **Bridge**: Mode state machine (`IDLE / AP / MANUAL`), `handle_remote_disconnect()` with steer-loss safety
- **Bridge**: TCP server on port 8555 handles `MODE MANUAL/AUTO`, `RUD xx.x`, `BTN TOGGLE/±1/±10`, `ESTOP`
- **Bridge**: Parses `FLAGS_CODE` and `BUZZER_STATE_CODE` from Nano, relays to Remote as TCP text
- **Bridge**: Sends `RDR_PCT`, `MODE`, `WARN AP_PRESSED` telemetry to Remote
- **Nano**: `MANUAL_MODE_CODE` (0xE9), `MANUAL_RUD_TARGET_CODE` (0xE8), `WARNING_CODE` (0xEA), `BUZZER_STATE_CODE` (0xEB)
- **Nano**: Two-press STOP logic — first press silences active alarm, second press is full emergency stop
- **Nano**: Remote manual steering via bang-bang motor drive to ADC target with deadband
- **Nano**: Steer-loss OLED display (flashing `!STEER LOSS`), `?AP Pressed?` warning (5s)
- **Nano**: Buzzer priority chain: steer-loss > hw-fault > ap-warn, with state reporting to bridge
- **Nano**: OLED row 4 shows `MAN: ON` when in remote MANUAL mode
- **Remote**: `on_bridge_rx()` TCP callback — parses `AP`, `HDG`, `CMD`, `RDR`, `RDR_PCT`, `FLAGS`, `MODE`, `WARN AP_PRESSED`
- **Remote**: Mode switch sends `MODE MANUAL` / `MODE AUTO` on physical transition
- **Remote**: Buttons send `BTN TOGGLE`, `BTN ±1/±10` in AUTO mode
- **Remote**: STOP button sends `ESTOP` to bridge on every press
- **Remote**: Pot sends `RUD xx.x` (0–100%) in MANUAL mode, throttled to 1% change or 200ms
- **Remote**: Bridge telemetry overrides local simulation when connected
- **Remote**: `?AP Rejected?` warning overlay on OLED (5s auto-expire)

### Changed
- `MANUAL_MODE_CODE` reassigned from 0xE7 to 0xE9 (0xE7 conflicts with pypilot `RESET_CODE`)
- Bridge `process_remote_line()` refactored to modify `BridgeState` directly
- Bridge pypilot worker thread with `time.sleep(0.01)` to prevent CPU starvation on Pi Zero (fix from B6)

### Fixed
- Pi: Offline issue (pypilot thread consuming 92% CPU on single-core Pi Zero, starving main loop)

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
