# Inno-Pilot Serial Protocols

## 1) Servo link framing (pypilot-compatible)

### Physical link
- UART over USB serial (Nano ↔ Pi)
- **38400 baud**, 8N1
- Binary frames

### Frame format (4 bytes)

[code] [value_lo] [value_hi] [crc8]


- `value` is little-endian: `value = lo | (hi << 8)`
- CRC is CRC-8 poly **0x31**, init **0xFF**, MSB-first
- The Arduino `crc.h` implementation is the reference

### Core pypilot codes (examples)
From Pi → Nano:
- `0xC7` COMMAND (0..2000, **1000=neutral**)
- `0x68` DISENGAGE
- other codes exist but are not required for basic operation

From Nano → Pi:
- `0x8F` FLAGS (bitmask)
- `0xA7` RUDDER_SENSE (scaled)
- `0x1C` CURRENT (typically *100)
- `0xB3` VOLTAGE (typically *100)
- `0xF9` CONTROLLER_TEMP (typically *100)

---

## 2) Inno-Pilot custom extensions (without breaking pypilot)

### 2.1 Nano → Bridge: Button events
These frames are sent by the Nano and also forwarded along the servo link (bridge interprets them too).

- `0xE0` BUTTON_EVENT
  - value = event id

Recommended event ids:
- 1 = -10°
- 2 = -1°
- 3 = toggle AP
- 4 = +10°
- 5 = +1°
- 6 = STOP (force AP off). Since the safety PR: always a full stop that also silences
  alarms (formerly two-press), and latched in the Nano — the Nano re-sends 6 every 0.5 s
  while pypilot keeps commanding, until pypilot's DISENGAGE stream has run for 0.6 s
- 7 = AP_LOST (Nano AP command watchdog: engaged, but no pypilot COMMAND frame for 1.5 s;
  the Nano has disengaged and raised STEER LOSS)

### 2.2 Bridge → Nano: AP state + telemetry
Bridge injects “state frames” so the Nano can display pypilot truth.

All values are **tenths** unless noted.

- `0xE1` AP_ENABLED
  - value: 0 (off) or 1 (on)

- `0xE2` PILOT_HEADING
  - value: `imu.heading * 10` (uint16, 0..3600)

- `0xE3` PILOT_COMMAND
  - value: `ap.heading_command * 10` (uint16, 0..3600)

- `0xE4` PILOT_RUDDER_ANGLE
  - value: `rudder.angle * 10` (int16, two’s complement)

- `0xE5` PILOT_RUDDER_PORT_LIMIT
  - value: **port limit** in degrees * 10 (int16)
  - typically negative (example: -40.0° ⇒ -400)

- `0xE6` PILOT_RUDDER_STBD_LIMIT
  - value: **stbd limit** in degrees * 10 (int16)
  - typically positive (example: +40.0° ⇒ +400)

Encoding note: For int16 values:
- cast to uint16 before placing lo/hi
- Nano must cast back to `int16_t` when reading

### 2.3 Bridge → Nano: Remote manual control and warnings (v0.2.0_B7)

- `0xE8` MANUAL_RUD_TARGET
  - value: 0..1000 (0=full port, 500=centre, 1000=full stbd)
  - Nano maps this to the rudder ADC range and drives the motor bang-bang

- `0xE9` MANUAL_MODE
  - value: 1 = enter MANUAL mode, 0 = exit MANUAL mode
  - Note: **NOT** 0xE7 — that is pypilot's RESET_CODE (clears OVERCURRENT_FAULT)

- `0xEA` WARNING
  - value subtypes:
    - 0 = WARN_NONE (clear all warnings)
    - 1 = WARN_AP_PRESSED (AP toggle rejected while remote in MANUAL — single 200ms beep + 5s OLED flash)
    - 2 = WARN_STEER_LOSS (TCP dropped, or REMOTE dead-man expired, in MANUAL mode — continuous rapid beeping, life-safety)
    - 3 = WARN_SILENCE (v3.3.0): remote STOP — silence every Nano alarm (steer loss, motor-link, hardware faults), the same flags as the Nano's own STOP button. Needed on units without helm buttons
- `0xE4` PILOT_RUDDER (v3.3.0 note): sent at 5 Hz only while pypilot's servo stream is alive (a frame within 1 s); the Nano's local-angle cross-check uses it only if it arrived within 0.6 s

### 2.4 Nano → Bridge: Buzzer state reporting (v0.2.0_B7)

- `0xEB` BUZZER_STATE
  - value: 1 = buzzer on, 0 = buzzer off
  - Sent on state change only (not polled)
  - Bridge relays to Remote via TCP text

### 2.5 Nano → Bridge: Comms-fault diagnostics (v0.2.0_B15–B16)

- `0xEC` COMMS_DIAG (added B15)
  - value: lo byte = `err_window_sum` (errors in 10s window), hi byte = `crit_consec_s`
  - Sent at 1 Hz (1000 ms period)

- `0xED` COMMS_ERR_DETAIL (added B16)
  - value: lo byte = CODE byte from the corrupt frame, hi byte = received CRC byte
  - Sent when a CRC error is detected on the Nano RX parser
  - Rate-limited to max 5 frames/second (200 ms minimum interval)
  - Latest-wins: if multiple errors occur within one 200 ms window, only the last is sent
  - Bridge logs these to `/tmp/inno_pilot_comms_diag.log` (volatile, does not survive reboot)

### 2.6 Nano → Bridge: Loop profiling (v1.3.3_B8–B9)

Diagnostic instrumentation for finding what blocks `loop()` long enough to overflow
the UART RX buffer. One metric per frame, emitted as a 13-frame burst every 5 s,
**one frame per loop pass** so a burst stays inside the Nano's 64-byte TX buffer
(sending all 78 bytes at once would block `loop()` waiting on the UART).

| Code | Metric | Unit |
|------|--------|------|
| `0xD0` | `oled_draw()` max | 4 µs |
| `0xD1` | `oled_draw()` mean | 4 µs |
| `0xD2` | `temp_service()` max | µs |
| `0xD3` | `service_rudder_adc()` max | µs |
| `0xD4` | `update_motor_from_command()` max | µs |
| `0xD5` | serial drain + parse max | µs |
| `0xD6` | whole `loop()` max | 4 µs |
| `0xD7` | whole `loop()` mean | 4 µs |
| `0xD8` | `Serial.available()` high-water at loop top | bytes |
| `0xD9` | times `available()` reached 63 | count |
| `0xDA` | compiled `SERIAL_RX_BUFFER_SIZE` | bytes |
| `0xDB` | `available()` immediately after `oled_draw()` | bytes |
| `0xDC` | `available()` immediately after `temp_service()` | bytes |

Notes:

- **`0xD0/D1/D6/D7` are in 4 µs units**, not µs. `micros()` quantises to 4 µs on AVR,
  so this costs no precision and raises the range from 65.5 ms to 262 ms. The bridge
  multiplies by 4 before logging, so the log reads in plain µs. A logged value of
  `262140 us` means the metric is still clamped and needs rescaling again.
- Plain-µs metrics clamp at `65535`. A reading of exactly 65535 means clamped.
- **`0xDA` is only what the *sketch* believes.** An `arduino-cli` build without
  `--clean` can link a core built without `-DSERIAL_RX_BUFFER_SIZE=128`, leaving the
  sketch's macro out of step with the real buffer. The trustworthy figure is the
  observed high-water (`0xD8`), which cannot exceed the real buffer minus one — if
  `0xDA` reports 128 while `0xD8` never passes 63, the flag did not reach the core.
- `0xD9` is deliberately fixed at 63 (the ceiling of a *default* 64-byte buffer)
  rather than derived from the macro, so a stale-cache mismatch cannot make it read
  a reassuring zero.
- Bridge logs these at **INFO** (not DEBUG) and mirrors them into
  `/tmp/inno_pilot_comms_diag.log`, so profiling and `ERR_DETAIL` share one timeline.
  DEBUG is deliberately avoided: it floods journald on a Pi Zero and perturbs bridge
  send pacing — the very timing being measured.

---

## 3) TCP protocol: Remote ↔ Bridge (v0.2.0_B7)

### Physical link
- TCP on port **8555**
- ASCII text, newline-terminated (`\n`)
- Remote connects to Bridge (Pi Zero IP, typically 192.168.6.13)

### Remote → Bridge (commands)

| Line | Description |
|------|-------------|
| `PING` | Keepalive (auto-sent by TCP client) |
| `ESTOP` | Emergency stop — disengage AP, clear MANUAL, end a jog, and (v3.3.0) silence every Nano alarm (`0xEA` value 3) — the web STOP behaves like the Nano's STOP button |
| `BTN TOGGLE` | Toggle `ap.enabled` |
| `BTN -10` / `BTN -1` / `BTN +1` / `BTN +10` | Adjust `ap.heading_command` |
| `MODE MANUAL` | Enter MANUAL steering mode |
| `MODE MANUAL ALIVE` | Enter MANUAL and arm the REMOTE dead-man at once (the web page; it promises `ALIVE` heartbeats) |
| `MODE AUTO` | Exit MANUAL, return to AP/IDLE |
| `RUD xx.x` | Rudder target 0.0–100.0% (only valid in MANUAL) |
| `ALIVE` | REMOTE dead-man heartbeat, sent by the web page every 0.4 s while in MANUAL. Opt-in: armed by `MODE MANUAL ALIVE` or by the first ALIVE of a MANUAL session; then no ALIVE for 1.5 s exits MANUAL with STEER LOSS (`WARN STEER_LOSS`). A remote that never sends ALIVE keeps the old behaviour (TCP drop only) |
| `NUDGE PORT` / `NUDGE STBD` | Start a hold-to-run motor jog (IDLE or AP only; 10 s cap) |
| `NUDGE HOLD` | Nudge dead-man heartbeat, every 0.2 s while the button is held; no HOLD for 0.8 s stops the jog. Never starts or resumes a jog |
| `NUDGE STOP` | Button released — stop the jog now |
| `RAM_SETUP <deg> [cycles]` | Arm the rudder sweep (Hardware Test 1). Disables AP, enters MANUAL on the Nano, mode `RAM_OFF`. Refused (reply `RAM_RESULT {"result":"REFUSED",...}`) if the sweep is too small for the rudder range/limits/deadband. Disarms after 60 s if never held (v3.0.3) |
| `RAM HOLD` | Dead-man heartbeat, sent every ~200 ms while the operator holds B3. First HOLD starts the sweep (`RAM_ON`); no HOLD for 1 s stops it (v3.0.3) |
| `RAM RELEASE` | Operator let go — stops the sweep immediately. Releasing ends the test; a later HOLD does not resume it (v3.0.3) |
| `LIMITS RANGE <deg>` | Dock-Side Wizard (v3.3.0): declared rudder angle at the stops → pypilot `rudder.range` (10–100). Clears captured points. IDLE only, like all `LIMITS` commands |
| `LIMITS CAPTURE CENTRE\|PORT\|STBD` | Sample the rudder for 1 s (skipper holds it still, clutch out); if it moved < 0.5° the point goes to pypilot's 3-point calibration (`rudder.calibration_state`). Never moves the rudder |
| `LIMITS SET <port_deg> <stbd_deg>` | Soft limits (magnitudes, ≥ 5°, ≥ 1° inside the stops). Needs all three points captured this session (or valid limits already) and a usable calibration; saves `/var/lib/inno-pilot/rudder_limits.json` |
| `TEST <id>` | Disabled since v3.0.3 (no Nano sketch implements it). Bridge replies `TEST_LINE Not available...` + `TEST_DONE` |

**No rudder motion before limits (v3.3.0):** while the rudder limits are not set, or no longer
match pypilot's calibration, the bridge refuses `BTN TOGGLE` (engage), `MODE MANUAL`,
`NUDGE PORT|STBD` and `RAM_SETUP` (and Nano B3 engage), drops pypilot COMMAND frames to the
Nano, and stops anything already moving. Each refusal raises `WARN LIMITS_UNSET`.

While the sweep is armed or running, any `ESTOP`, `BTN`, `MODE`, `NUDGE`, `RUD` or `TGT`
stops it first (single exit path `ram_test_abort()`); `BTN` is then swallowed, the others
proceed normally. Any Nano button and a remote disconnect also stop it.

### Bridge → Remote (telemetry, ~1Hz)

| Line | Description |
|------|-------------|
| `AP 0` / `AP 1` | Autopilot enabled state |
| `HDG xxx.x` | IMU heading (degrees) |
| `CMD xxx.x` | Heading command (degrees) |
| `RDR xxx.x` | Rudder angle (signed degrees) |
| `RDR_PCT xxx.x` | Rudder percentage (0–100) |
| `FLAGS xxxx` | Nano fault flags (bitmask, decimal) |
| `MODE IDLE` / `MODE AP` / `MODE MANUAL` / `MODE RAM_OFF` / `MODE RAM_ON` | Bridge mode state (`RAM_*` = rudder sweep armed / running) |
| `RAM_PROG <done> <total>` | Rudder sweep progress in full strokes (only while armed/running) |
| `RAM_RESULT <json>` | Rudder sweep outcome, once, at the end: `result` = PASS / CHECK / STOPPED / DISARMED / REFUSED, `reason`, per-direction `dirs.port`/`dirs.stbd` (speed_dps, hard_over_s, lag_ms, overshoot_max_deg, end_err_deg, settle_timeouts, i_peak_a, i_mean_a), `speed_asym_pct`, `nano_deadband_deg`, `slow_zone_deg`, `speed_power` (full / mixed / reduced) |
| `WARN` / `WARN <code>` | Bridge-raised skipper alert, every telemetry cycle; bare `WARN` = none. `STEER_LOSS` = REMOTE steering stopped (phone/link lost), `AP_LOST` = autopilot stopped because pypilot vanished, `LIMITS_UNSET` = motion refused or stopped because the rudder limits are not set/valid (v3.3.0). Clears 2 s after the skipper re-engages, or after 120 s. (Replaces the documented-but-never-sent `WARN AP_PRESSED`; AP-pressed is a Nano-only warning, `0xEA` value 1) |
| `LIMITS <json>` | Rudder limits status, every telemetry cycle (v3.3.0): `ok`, `reason`, `set`, `port_deg`, `stbd_deg`, `range`, `cal_problem`, `capturing`, `done` (points captured this session), `msg` (last `LIMITS` command result), `margin` |
| `PONG` | Keepalive reply |

---

## 4) Bridge behaviour (conceptual)
- Read from pypilot PTY → write to Nano
- Read from Nano → write to pypilot PTY
- Parse Nano button frames (`0xE0`) and call pypilot API via `pypilotClient.set(...)`
- Watch pypilot values and periodically inject state frames (`0xE1..0xE6`) to Nano
- Listen on TCP port 8555 for Remote connections
- Translate Remote TCP commands to Nano serial frames and pypilot API calls
- Send telemetry snapshots to Remote at ~1Hz
- Track mode state (`IDLE / AP / MANUAL`) and handle Remote TCP disconnects safely

Bridge should not block or “interpret” normal servo frames; it must behave like a transparent wire.

