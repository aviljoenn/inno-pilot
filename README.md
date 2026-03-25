# Inno-Pilot – Open Marine Autopilot System

Inno-Pilot is a modular, open autopilot system for sailboats and small craft.

It combines:

- A **servo motor controller** (e.g. Arduino Nano today, Pi Pico / ESP32 in future)
- A **compute module** (e.g. Raspberry Pi Zero or other SBC) running the autopilot logic
- A set of **hardware modules** (IMU, motor driver, clutch, rudder sensor, control head)
- A small amount of **“glue”** code on the compute module that connects everything cleanly

The core autopilot algorithms and network integration are based on
[Sean D’Epagnier’s pypilot project](https://github.com/pypilot/pypilot).  
Huge credit to Sean for the original work – Inno-Pilot builds on that foundation and
wraps it into a more opinionated hardware + software product.

---

## Repository layout

This repository is structured around the Inno-Pilot product, not just pypilot.

### `servo_motor_control/`

Everything related to the **servo/motor controller** firmware – i.e. the electronics
that physically drive the hydraulic ram or tiller pilot.

Current structure:

- `servo_motor_control/arduino/`  
  Arduino Nano implementation (e.g. `motor_simple.ino`) for the IBT-2 driver + clutch +
  rudder sensor.
- `servo_motor_control/docs/`  
  Documentation and artefacts about the physical system (modules, wiring, enclosure,
  datasheets).

_Example:_ `servo_motor_control/docs/HKD Dual 43A H-Bridge Motor Driver (IBT-2) – Overview & Arduino Control.docx`

Future expansions might include:

- `servo_motor_control/pico/` – Raspberry Pi Pico-based controller  
- `servo_motor_control/esp32/` – ESP32-based controller  

The goal is that Inno-Pilot defines the **protocol and behaviour**, and the builder
can choose the MCU platform that best fits their hardware.

### `compute_module/`

Everything that runs on the **compute module** (Pi Zero or other SBC).

Two main submodules:

#### `compute_module/pypilot/`

Vendored pypilot core and related code. This is essentially a fork of Sean’s
pypilot project and contains:

- `compute_module/pypilot/pypilot/` – the Python package
- `compute_module/pypilot/docs/`
- `compute_module/pypilot/hat/`
- `compute_module/pypilot/ui/`
- `compute_module/pypilot/web/`
- `compute_module/pypilot/scripts/`
- `compute_module/pypilot/dependencies.py`
- `compute_module/pypilot/setup.py`, `setup.cfg`, `pypilot.build`
- `compute_module/pypilot/README.md` – original pypilot README

Inno-Pilot treats this as the **autopilot engine**:

- Heading/track control
- Sensor fusion (IMU, GPS, wind, etc.)
- Network integration (Signal K, NMEA)

#### `compute_module/glue/`

“Inno-Pilot-Glue”: the integration layer that makes the Nano + pypilot + Pi Zero
behave as one coherent system.

Contents:

- `inno_pilot_bridge.py`  
  Python bridge between the Nano servo port and pypilot:
  - Forwards servo protocol traffic unchanged
  - Listens for button event frames from the Nano and converts them into pypilot
    API calls (`ap.enabled`, `ap.heading_command`)

- `inno_pilot_fix_symlink.sh`  
  Helper script that:
  - Moves the original Nano USB by-id symlink to a `.real` alias
  - Repoints the original by-id name at a PTY created by `socat`

- `inno-pilot-socat.service`  
  systemd unit that creates a PTY pair:
  - `/dev/ttyINNOPILOT`
  - `/dev/ttyINNOPILOT_BRIDGE`

- `inno-pilot-fixlink.service`  
  systemd oneshot that runs `inno_pilot_fix_symlink.sh` after socat has started.

- `inno-pilot-bridge.service`  
  systemd service that runs `inno_pilot_bridge.py` as a user-level daemon.

- `deploy_inno_pilot_glue.sh`  
  Deployment script that:
  - Copies the glue scripts to `/usr/local/bin` / `/usr/local/sbin`
  - Installs/enables the systemd units
  - Ensures `~/.pypilot/servodevice` uses the USB by-id name
  - Adds a systemd drop-in so `pypilot.service` starts **after** the glue services

- `README.md`, `CLAUDE.md`
  Documentation for how this glue works and how to maintain it.

---

### `inno-remote/`

ESP32-C3 handheld wireless remote for the autopilot (hardware + firmware).

- `inno-remote/firmware/inno_remote/` – ESP-IDF main application
- `inno-remote/docs/` – wiring diagrams, troubleshooting
- `inno-remote/README.md` – hardware design, pinout, build instructions

The remote connects to the bridge via Wi-Fi TCP (port 8555) and provides:
- AP engage/disengage, heading ±1°/±10° buttons
- Physical 3-position mode switch (AP / OFF / MANUAL)
- Manual rudder control via potentiometer in MANUAL mode
- Emergency STOP button
- OLED display with heading, rudder bar, mode, and warning overlays

---

## High-level architecture (Inno-Pilot V2)

At a high level, an Inno-Pilot installation looks like this:

```text
                                     +---------------------------+
[ Rudder Sensor ]---+                |  Compute Module           |
[ Limit Logic   ]   |       IMU --->|  (Pi Zero / SBC)          |
[ IBT-2 Driver  ]---+               |                           |
[ Clutch Relay  ]-------USB-------->|  pypilot core             |
                                     |  + Inno-Pilot Bridge     |
     Nano / MCU                      |    - PTY + symlink       |
     (Servo Ctrl)                    |    - TCP server :8555    |
     + OLED + Buttons                +----------+------+--------+
                                                |      |
                                     SignalK/NMEA   Wi-Fi TCP
                                                |      |
                                    [ Instruments ] [ ESP32-C3 Remote ]
                                                       + OLED
                                                       + Buttons/Pot
                                                       + Mode Switch
                                                       + ESTOP
```

### Key ideas

The servo motor controller (e.g. Nano) is responsible for:

- Safe motor control and rudder limits
- Reading rudder position, current, voltage
- Local control UI (OLED + buttons)
- Emitting a simple binary protocol understood by pypilot

The compute module (e.g. Pi Zero) runs:

- pypilot (heading, track, wind modes, etc.)
- The Inno-Pilot glue that connects the Nano’s control surface/buttons to pypilot’s
  `ap.*` interface
- A TCP server (port 8555) for the wireless remote

The wireless remote (ESP32-C3) provides:

- Handheld cockpit control with OLED display
- AP engage/disengage and heading adjustment buttons
- Physical mode switch (AP / OFF / MANUAL)
- Manual rudder potentiometer for direct steering
- Emergency STOP button (software command over Wi-Fi)
- Real-time telemetry display (heading, rudder, mode, warnings)

---

## Credits

**Autopilot core:**  
Inno-Pilot is based on pypilot by Sean D’Epagnier. The original project provides
the Python autopilot engine, sensor fusion, and network integration.

**Inno-Pilot integration:**  
This repository adds:

- A structured hardware + firmware + compute layout
- Custom servo motor controller firmware
- The Inno-Pilot glue layer that makes pypilot and the servo hardware work together
  cleanly

---

## Future enhancements

Feature research based on Garmin GHP 12, Raymarine Evolution, Simrad AP44, B&G Triton2/H5000,
and Furuno NAVpilot-300. Prioritised by sailing value and implementation feasibility on existing hardware.

### Tier 1 — High value, low complexity

| Feature | Description | Extra hardware? |
|---------|-------------|-----------------|
| **Wind Hold mode** | Steer to a fixed apparent/true wind angle instead of compass heading. When the wind shifts, the boat follows. Pypilot already implements this algorithm — just needs a wind sensor and a mode-select button on the remote. | Wind sensor |
| **Auto-Tack / Auto-Gybe** | Single button press commands a controlled tack through a configurable angle (90–110°) at a configurable rate. The motor controller already accepts heading commands — auto-tack is a scripted heading delta. | None |
| **Shadow Drive** | Detect unexpected rudder movement (helm turned by hand), auto-disengage the pilot, re-engage on new heading when helm is held steady. Uses the existing rudder pot — no new hardware. | None |
| **Watch / Deadman alarm** | Periodic countdown timer requiring a button press to confirm the watch. If expired: buzzer + ESTOP. Pure ESP32 remote firmware change. | None |
| **Sea State adaptive gain** | A 1–5 "sea state" slider on the remote that maps to pypilot PID gain presets. Calm-water settings won't hunt in chop; rough-water settings won't be sluggish in flat water. | None |

### Tier 2 — High value, medium complexity

| Feature | Description | Extra hardware? |
|---------|-------------|-----------------|
| **Multi-page OLED display** | Cycle through pages on the remote: Heading → Wind → Nav → Diagnostics. Exposes boat data (SOG, COG, wind angle, battery, motor temp) without needing a chartplotter. | None |
| **Heading error bar graph** | Horizontal pixel bar showing ±15° heading error — instant analog read of pilot performance. Reveals PID hunting that a number alone hides. | None |
| **No-Drift / GPS track mode** | Combine GPS cross-track error with heading hold so the boat maintains its actual GPS track, correcting for leeway and tidal set. Pypilot has the algorithm; needs a GPS dongle. | USB GPS |
| **Gybe guard alarm** | When in wind hold mode running downwind: alarm if apparent wind angle exceeds a configurable threshold (risk of accidental gybe). | Wind sensor |
| **MOB button mode** | Long-press STOP to mark GPS position, disengage AP, fire continuous buzzer, show return bearing on OLED. | None |
| **Motor thermal duty limiting** | Instead of just alarming on overtemp, back off PWM duty cycle as temperature rises. Prevents nuisance shutdowns in heavy weather. | None |

### Tier 3 — Medium value, higher complexity

| Feature | Description | Extra hardware? |
|---------|-------------|-----------------|
| **Auto-Tune PID** | Guided sea-trial sequence: command step inputs, measure heading response, calculate optimal PID gains. Removes guesswork from manual tuning. | None |
| **Speed-adaptive gain** | Automatically scale rudder gain with boat speed. At low speed more rudder is needed; at hull speed less. | GPS SOG |
| **Configurable tack angle/rate** | Store tack parameters in EEPROM, adjust from remote. A fin-keel racer tacks in 5s through 90°; a full-keel cruiser needs 15s through 110°. | None |
| **Route/waypoint following** | Receive waypoint data from OpenCPN and steer to each waypoint in sequence using cross-track error. | Chartplotter/OpenCPN |
| **Motor run-time / rudder activity log** | Track cumulative motor run time and direction reversals in EEPROM. Predicts maintenance intervals for hydraulic drive. | None |
| **Commissioning / self-test mode** | Structured startup test verifying rudder pot range, motor response, sensor readings, and link integrity. | None |
| **OLED night dimming** | SSD1306 contrast control via long-press. Preserves night vision on passage. | None |
| **NMEA/SignalK data logging** | Log all autopilot events (engage/disengage, heading errors, rudder, mode changes) with timestamps to SD card for post-passage analysis. | None |

---

## Getting started

If you want to:

- **Build the hardware:**
  Look under `servo_motor_control/docs/` for modules, wiring and BOM information.
- **Work on the servo motor controller:**
  Start in `servo_motor_control/arduino/`.
- **Work on the compute module (Pi Zero) side:**
  - `compute_module/pypilot/` – pypilot core and its original README
  - `compute_module/glue/` – bridge, systemd units, deploy script, docs
