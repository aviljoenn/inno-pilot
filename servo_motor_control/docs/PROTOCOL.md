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
- 6 = STOP (force AP off)

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
    - 2 = WARN_STEER_LOSS (TCP dropped in MANUAL mode — continuous rapid beeping, life-safety)

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
| `ESTOP` | Emergency stop — disengage AP, clear MANUAL |
| `BTN TOGGLE` | Toggle `ap.enabled` |
| `BTN -10` / `BTN -1` / `BTN +1` / `BTN +10` | Adjust `ap.heading_command` |
| `MODE MANUAL` | Enter MANUAL steering mode |
| `MODE AUTO` | Exit MANUAL, return to AP/IDLE |
| `RUD xx.x` | Rudder target 0.0–100.0% (only valid in MANUAL) |

### Bridge → Remote (telemetry, ~1Hz)

| Line | Description |
|------|-------------|
| `AP 0` / `AP 1` | Autopilot enabled state |
| `HDG xxx.x` | IMU heading (degrees) |
| `CMD xxx.x` | Heading command (degrees) |
| `RDR xxx.x` | Rudder angle (signed degrees) |
| `RDR_PCT xxx.x` | Rudder percentage (0–100) |
| `FLAGS xxxx` | Nano fault flags (bitmask, decimal) |
| `MODE IDLE` / `MODE AP` / `MODE MANUAL` | Bridge mode state |
| `WARN AP_PRESSED` | AP toggle was rejected (remote shows warning) |
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

