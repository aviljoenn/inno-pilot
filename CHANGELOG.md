# Changelog

All notable changes to the Inno-Pilot system are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/).
Version applies to all three components (Bridge, Nano, Remote) simultaneously and the version of all components must always match regardless if a change was made in a component or not.

## [Unreleased]

## [v1.2.0_B27] — 2026-04-05 — Fix: motor activates in HAND mode on units without buttons

### Fixed
- **Nano**: Motor falsely activated in HAND mode when helm moved by hand on units with no
  physical buttons wired (e.g. .12 / Pi5 boat). Root cause: floating A6 pin combined with
  ADC S/H crosstalk from the preceding A2 rudder read caused `decode_button_from_adc()` to
  return BTN_B1, setting `manual_override=true` and driving the motor STBD (confirmed via
  MOTOR_REASON_CODE = `manual_phys(btn)` with `manual_ov=1`).

### Added
- **Nano**: `FEATURE_ON_BOARD_BUTTONS` (0x20) — when this feature flag is OFF, the Nano
  skips `analogRead(A6)` entirely and forces `raw_b = BTN_NONE`, eliminating the floating
  pin false-positive. Default OFF so units without buttons are safe out of the box.
- **Bridge**: `FEATURE_ON_BOARD_BUTTONS` constant and `on_board_buttons` settings key;
  included in `send_features_to_nano()` and logged at startup.
- **Web Remote**: "On-Board Buttons" ON/OFF toggle in SETTINGS → CONNECTIONS & FEATURES.
  Set ON for the .13 unit (has buttons), leave OFF for the .12 unit (no buttons).

## [v1.2.0_B20] — 2026-04-02 — Version sync across all components

### Changed
- **All**: Synchronised version to `v1.2.0_B20` across Bridge, Nano (motor_simple.ino), and ESP32 Remote — Bridge was at B15, Nano and Remote were at B7

## [v1.2.0_B1] — 2026-03-27 — OTA Update Pipeline + Version Reset

### Added
- **Remote**: OTA firmware update via TCP — on TCP connect, HELLO version mismatch triggers automatic OTA download from bridge HTTP server (`http://192.168.6.13:8556/inno_remote.bin`), flashes inactive OTA partition, and reboots; OLED shows "OTA UPDATE / Updating..." during transfer, "OTA FAILED / Check bridge" on error
- **Remote**: Task watchdog unregisters during OTA download to prevent false 10 s timeout during transfer
- **Bridge**: OTA HTTP file server on port 8556 — serves `inno_remote.bin` from `/var/lib/inno-pilot/ota/` in a daemon thread; returns 404 if binary not yet deployed (safe at startup)
- **Bridge**: HELLO handler now sends `OTA <url>` after `HELLO <ver>` reply when remote version mismatches and binary is present
- **Deploy**: `deploy_inno_pilot_glue.sh` now copies `ota/inno_remote.bin` from repo to `/var/lib/inno-pilot/ota/` on Pi
- **Repo**: `inno-remote/firmware/inno_remote/ota/` directory tracked in git; `.gitignore` exception allows committing the OTA-ready `.bin`

### Changed
- Version scheme reset from `v0.2.0_B21` (Bridge/Nano) / `v0.2.0_B19` (Remote) to `v1.2.0_B1` across all three components — build counter restarts at 1

## [v0.2.0_B21] — 2026-03-27 — OLED Fault Area Moved to Rows 1-2, Helm Always at Row 3

### Changed
- **Nano**: All fault/warning messages moved from rows 2-3 to rows 1-2 — 2X messages span rows 1+2, 1X messages use row 1
- **Nano**: Helm field (row 3) is now always drawn unconditionally — `row3_taken` guard removed entirely
- **Bridge**: Build bumped to B21 to match Nano

## [v0.2.0_B20] — 2026-03-27 — OLED Layout Compaction: Remove Diagnostics Row, Move Helm/Cmd/Rudder Up

### Changed
- **Nano**: Removed Rx diagnostics row (was row 3: `Rx:N Er:N A:N.N`) — no longer shown in normal or offline state
- **Nano**: Helm mode moved from row 4 to row 3 (guarded: hidden during 2X fault messages that span rows 2-3)
- **Nano**: Cmd/HDG row moved from row 5 to row 4
- **Nano**: Graphical rudder bar moved from row 6 to row 5; track now filled with `-` instead of spaces (`P---I---S`)
- **Nano**: Row 6 always cleared (was rudder bar); row 7 V/A/T unchanged
- **Nano**: Offline path now simply clears rows 3-6 (no diagnostics text in middle rows)
- **Bridge**: Build bumped to B20 to match Nano

## [v0.2.0_B19] — 2026-03-27 — Remote Safety: TCP Indicator, Comms Display, Watchdog, Buzzer, Version Handshake

### Added
- **Remote**: No-bridge connecting screen — when TCP is not established in normal mode, OLED shows "NO BRIDGE / Connecting..." instead of simulated instrument values; simulation is now strictly a demo-mode feature (hold ESTOP at boot)
- **Remote**: COMMS fault display — parses `COMMS WARN` / `COMMS CRIT` from bridge; mode line becomes `MODE: AUTO [W]` on WARN and `!COMMS FAULT!` on CRIT
- **Remote**: Buzzer drive — GPIO 8 (NPN transistor) now driven for three events:
  - ESTOP press: double-beep (100 ms on / 60 ms off / 100 ms on), highest priority
  - TCP bridge lost mid-session: triple short pulse alarm
  - COMMS CRIT active: continuous 500 ms on/off slow beep (matches Nano pattern)
- **Remote**: Task watchdog — `esp_task_wdt_add(NULL)` registered in `app_main`; fed by `esp_task_wdt_reset()` every 20 ms loop; 10 s timeout + panic in `sdkconfig.defaults`
- **Remote**: VERSION handshake — sends `HELLO v0.2.0_B19` on every new TCP connection; logs mismatch warning if bridge replies with a different version
- **Remote**: `tcp_client_set_connect_callback()` API — fires once per new TCP connection, used for HELLO send
- **Bridge**: HELLO command handler — parses remote version, logs match/mismatch at INFO/WARNING, replies `HELLO <bridge_version>`

### Changed
- All components bumped to B19 (Nano: version constant only, no logic changes)

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
