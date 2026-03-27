# Changelog

All notable changes to the Inno-Pilot system are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/).
Version applies to all three components (Bridge, Nano, Remote) simultaneously.

## [Unreleased]

## [v0.2.0_B18] — 2026-03-27 — OLED Simplification & Graphical Rudder Bar

### Changed
- **Nano**: OLED Row 1 removed — Pi online/offline status replaced by version-mismatch-by-exception warning in rows 2-3 (flashing `!VER MISMATCH!` + build numbers when bridge ≠ Nano build)
- **Nano**: Boot splash is now non-blocking — `loop()` runs during the 3 s splash (serial RX uninterrupted); old `delay(3000)` removed
- **Nano**: Row 4 replaces AP/Clutch dual-field with single `Helm:` field: `HAND` (manual jog), `AUTO` (AP engaged), `REMOTE` (TCP remote manual)
- **Nano**: Row 5 heading values always 3-digit zero-padded (e.g. `005`); `HDG:` right-justified on the display
- **Nano**: Row 6 replaced with graphical rudder bar — `P` on left, `S` on right, `I` indicator proportionally placed within [port_lim, stbd_lim]; `I` blinks over `P` or `S` when rudder exceeds a limit, with `?Rud>Limit` warning in rows 2-3
- **Nano**: Rows 2-3 warning priority updated: steer_loss > hw_fault > ver_mismatch > comms_crit > comms_warn > rud_overshoot > ap_pressed
- **Nano**: Removed unused `rudder_deg` local in `oled_draw()` (was never read)
- **Bridge**: Build bumped to B18 to match Nano

## [v0.2.0_B17] — 2026-03-27 — Fix CRC Errors: RX Buffer + OLED Throttle

### Fixed
- **Nano**: Root cause of sustained ~20 CRC errors/10s identified — OLED I2C draw blocking `loop()` long enough to overflow the 64-byte Arduino serial RX buffer during bridge telemetry bursts
- **Nano**: Increased serial RX buffer from 64 to 128 bytes via `--build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128"` (required on every compile)
- **Nano**: Reduced OLED refresh from 200ms (5 Hz) to 1000ms (1 Hz) — eliminates buffer pressure overlap with 5 Hz telemetry bursts
- **Nano**: Partial OLED update — rows 0-1 (version/Pi status) skip I2C writes when content unchanged, saving ~25% of draw time

## [v0.2.0_B16] — 2026-03-27 — Comms Error Diagnostic Logging

### Added
- **Nano**: `COMMS_ERR_DETAIL_CODE` (0xED) — sends corrupt frame CODE byte + received CRC to bridge on CRC error, rate-limited to 5/s (200 ms throttle, latest-wins 1-slot)
- **Bridge**: Volatile diagnostic log at `/tmp/inno_pilot_comms_diag.log` — 256 KB rotating, ms-precision timestamps, does not survive reboot
- **Bridge**: `COMMS_ERR_DETAIL_CODE` parser — extracts corrupt code + CRC, logs with err_window/crit_s context
- **Bridge**: Bridge-side CRC errors (Nano→Bridge direction) also logged to the same diagnostic file

## [v0.2.0_B15] — 2026-03-26 — Comms-Fault Detection & Safety System

### Added
- **Nano**: Sliding-window CRC error rate monitor — 10 × 1-second buckets, cached sum, evaluated every loop
- **Nano**: Two-level fault thresholds: WARN (≥5 errors/10 s) and CRITICAL (≥15/10 s held 3 consecutive seconds)
- **Nano**: Autonomous AP disengage (one-shot latch) on CRITICAL — motor neutral, clutch off, 0ms latency
- **Nano**: 1 Hz comms buzzer alarm (500ms on/off) — distinct from 100ms hw-alarm rapid beeps
- **Nano**: OLED comms-fault slots: `!COMMS ERR!` (flashing 2×) at CRITICAL, `?Comms:N err/10s` (static 1×) at WARN
- **Nano**: PTM two-press extended to silence comms buzzer (first press silence, second press STOP)
- **Nano**: `COMMS_DIAG_CODE` (0xEC) telemetry at 1 Hz — lo byte = err_window_sum, hi byte = crit_consec_s
- **Nano**: New flag bits `COMMS_WARN_FAULT=0x1000`, `COMMS_CRIT_FAULT=0x2000` in FLAGS word
- **Bridge**: Parse `COMMS_WARN_FAULT`/`COMMS_CRIT_FAULT` from FLAGS — auto-disengage AP via pypilot set_q on CRITICAL
- **Bridge**: `COMMS_DIAG_CODE` handler — logs elevated error rates at INFO, clean at DEBUG
- **Bridge**: CRC validation on Nano→bridge frames in `extract_wrapped_frames()` — corrupt frames dropped, not forwarded to pypilot
- **Bridge**: `COMMS OK` / `COMMS WARN N` / `COMMS CRIT` TCP telemetry to remote at 5 Hz
- **Bridge**: Relay diagnostics — drop-rate percentage logged every 30 s, warns at >5%

### Changed
- OLED fault priority: steer_loss > hw_fault > **comms_crit** > **comms_warn** > ap_pressed_warn > overlay
- Buzzer priority: steer_loss > hw_alarm > **comms_fault** (500ms/1Hz) > ap_pressed_warn

## [v0.2.0_B14] — 2026-03-26 — AP State Cleanup + Probe Refactor

### Changed
- **Bridge**: Removed `AP_ENABLED_CODE` (0xE1) send — AP engage/disengage is now derived from pypilot's native `COMMAND_CODE`/`DISENGAGE_CODE` relay frames (single source of truth)
- **Bridge**: Removed `ap.enabled` subscription from pypilot worker thread (redundant after above)
- **Bridge**: Relay loop now inspects forwarded pypilot frames to keep `bstate.mode` accurate when pypilot changes AP state externally
- **Bridge**: `probe_nano_port()` now uses `send_nano_frame()` — removes hand-rolled wrap/build duplication
- **Nano**: Removed `AP_ENABLED_CODE` handler; `ap_enabled_remote` set true on first `COMMAND_CODE`, cleared on `DISENGAGE_CODE`

## [v0.2.0_B11] — 2026-03-26 — Serial Reliability + HUPCL Fix

### Added
- **Bridge**: B8 — CRC-validated pypilot relay: wraps outbound pypilot frames in bridge framing + CRC before forwarding to Nano; prevents Nano CRC-error flood from raw pypilot data
- **Bridge**: B9 — Structured DEBUG logging for relay diagnostics (SIGUSR1 toggle)
- **Nano**: B10 — RX debug counters on OLED (CRC errors, raw bytes, valid frames)

### Fixed
- **Bridge**: B11 — HUPCL disabled on serial open via `termios.tcsetattr(~termios.HUPCL)` — prevents Nano reset (DTR-drop) when the port is closed. Documented in `CLAUDE.md` and top-level `CLAUDE.md`.

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
