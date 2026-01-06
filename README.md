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

### `hardware/`

Documentation and artefacts about the physical system:

- Modules and components (IMU, motor driver, clutch relay, Nano board, etc.)
- Wiring diagrams and connection notes
- Enclosure / mounting guidance
- Datasheets and application notes, e.g. IBT-2 H-Bridge

_Example:_ `hardware/HKD Dual 43A H-Bridge Motor Driver (IBT-2) – Overview & Arduino Control.docx`

### `servo_motor_control/`

Everything related to the **servo/motor controller** firmware – i.e. the electronics
that physically drive the hydraulic ram or tiller pilot.

Current structure:

- `servo_motor_control/arduino/`  
  Arduino Nano implementation (e.g. `motor_simple.ino`) for the IBT-2 driver + clutch +
  rudder sensor.

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

- `README.md`, `AGENTS.md`  
  Documentation for how this glue works and how to maintain it.

---

## High-level architecture (Inno-Pilot V2)

At a high level, an Inno-Pilot installation looks like this:

```text
[ Rudder Sensor ]---+
[ Limit Logic   ]   |       [ IMU / Sensors ]---+
[ IBT-2 Driver  ]---+                +----------+-----------+
[ Clutch Relay  ]------------------> |  Compute Module      |
                                     |  (Pi Zero / SBC)     |
                                     |                      |
                     Nano / MCU      |  pypilot core        |
                     (Servo Ctrl)    |  + Inno-Pilot Glue   |
                     + OLED +        |      - bridge        |
                     Buttons         |      - PTY + symlink |
                                     +----------+-----------+
                                                |
                                                | Network (Signal K, NMEA)
                                                |
                                           [ Instruments, plotter, etc. ]
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

## Getting started

If you want to:

- **Build the hardware:**
  Look under `hardware/` for modules, wiring and BOM information.
- **Work on the servo motor controller:**
  Start in `servo_motor_control/arduino/`.
- **Work on the compute module (Pi Zero) side:**
  - `compute_module/pypilot/` – pypilot core and its original README
  - `compute_module/glue/` – bridge, systemd units, deploy script, docs
