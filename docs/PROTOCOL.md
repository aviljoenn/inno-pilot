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

---

## 3) Bridge behaviour (conceptual)
- Read from pypilot PTY → write to Nano
- Read from Nano → write to pypilot PTY
- Parse Nano button frames (`0xE0`) and call pypilot API via `pypilotClient.set(...)`
- Watch pypilot values and periodically inject state frames (`0xE1..0xE6`) to Nano

Bridge should not block or “interpret” normal servo frames; it must behave like a transparent wire.

