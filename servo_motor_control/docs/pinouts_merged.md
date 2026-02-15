# Inno-Pilot Pinouts (V2)

This document describes the **default V2** pinout. Some pins are optional (limit switches).


## Controller I/O overview

### Inputs and outputs
- 12V power input
- Rudder actuator (bi-directional 12V output)
- Clutch output (12V output)
- Rudder position feedback sensor (3-wire, 10k pot)
- Gyro / IMU sensor
- Emergency stop pushbutton (on control box)
- Over-travel limit switches (optional) (2x, 2-wire, NC)
- Display (on control box)
- Pi controller
- Arduino controller
- Power supply unit (PSU)

## Power supply

### 12V power input path (V2)
- 12V positive input → terminal block (pin 1) → 15A fuse holder (flat)
- 15A fuse holder (round) → current sensor (IP+) & voltage sensor (pin+)
- Current sensor (IP−) → 12V power bus+ (Vcc)
- 12V negative input → terminal block (pin 2) → 12V power bus− (GND)

## Arduino Nano pinout

### Motor driver (IBT-2 / BTS7960)
- **D2**  → IBT-2 **RPWM** (direction)
  - At Arduino: 10 kΩ pulldown **D2 → GND**
- **D3**  → IBT-2 **LPWM** (direction)
  - At Arduino: 10 kΩ pulldown **D3 → GND**
- **D9**  → IBT-2 **R_EN + L_EN bridged** (enable / “PWM gate”)
  - In V2 hydraulic pump mode this is effectively **ON/OFF** (often fixed at 255 in software)
  - At Arduino: 10 kΩ pulldown **D9 → GND**

IBT-2 power:
- IBT-2 V+ → 12V supply
- IBT-2 GND → 0V (must share common ground with Nano and sensors)
- IBT-2 motor outputs → hydraulic pump motor

### Clutch
- **D11** → clutch driver input
  - **HIGH = clutch engaged**
  - **LOW  = clutch disengaged**

#### Clutch MOSFET driver wiring notes (V2)
- `+CLUTCH` = the +12V that feeds the clutch coil at that end
- `GND_PWR` = the 0V at the MOSFET source end (**star/common ground**)
- Place the **2200 µF electrolytic** within **2–10 cm** of the MOSFET:
  - Cap **+** → `+CLUTCH`
  - Cap **−** → `GND_PWR`
- Add a **ceramic 100 nF** bypass: `+CLUTCH` → `GND_PWR`
- Gate pulldown (**10 kΩ**) physically at the MOSFET: **Gate (Pin 1) → Source (Pin 3)**
- Add a **10 nF** cap: **Gate (Pin 1) → Source (Pin 3)** physically at the MOSFET pins
- Flyback diode across the clutch coil: **STPS20H100CT**, reverse-biased during normal operation; conducts when the MOSFET turns off and the coil collapses.
  - STPS20H100CT pinout (TO-220AB, text facing you, legs down):
    - Pin 1 = Anode (A1)
    - Pin 2 = Cathode (K) (tab = Cathode)
    - Pin 3 = Anode (A2)
- Sanity check: on **IRLZ44N** (TO-220), **Pin 1=Gate, Pin 2=Drain (tab), Pin 3=Source** — verify your board wiring matches your intended low-side/high-side topology.


### Emergency stop (PTM)
- **D4** → PTM button (wired to GND)
  - Configure as `INPUT_PULLUP`
  - Press = LOW

### Buzzer (via transistor)
- **D10** → base resistor (e.g. 2.2k) → NPN transistor base
- Transistor emitter → GND
- Transistor collector → buzzer negative
- Buzzer positive → +12V
- Nano GND must be tied to 12V GND

### OLED (I2C)
- **A4 (SDA)** → OLED SDA
- **A5 (SCL)** → OLED SCL
- OLED VCC → +5V
- OLED GND → GND

### DS18B20 (1-Wire)
- **D12** → DS18B20 DQ
- DS18B20 VDD → +5V
- DS18B20 GND → GND
- 4.7k pull-up from DQ to +5V

### Rudder position pot
- **A2** → pot wiper
- pot ends → +5V and GND (ensure stable reference)

### Analog sensors (ADC)
- **A0** → main supply voltage sense (via divider)
- **A1** → current sensor output
- **A3** → Pi Zero 5V bus sense (via divider)
- **A6** → 5-button resistor ladder input (analog-only pin)

### Optional limit switches (disabled by default in V2)
- **D7** → Port limit switch (NC to GND recommended), `INPUT_PULLUP`
- **D8** → Stbd limit switch (NC to GND recommended), `INPUT_PULLUP`

V2 default: **D7/D8 left free**, `LIMIT_SWITCHES_ACTIVE=false`.

---

## Pi Zero pinout (compute module)

### IMU (LSM9DS1)
- Uses Pi I2C bus (commonly `/dev/i2c-1`)
- LSM9DS1 typically appears at:
  - **0x6B** (accel/gyro)
  - **0x1E** (mag)
- Wiring:
  - Pi SDA/SCL → IMU SDA/SCL
  - Pi 3V3 → IMU VCC (confirm board voltage requirements)
  - Pi GND → IMU GND

### Servo controller (Nano)
- Connected via USB serial (CH340), e.g. `/dev/ttyUSB0`
- Inno-Pilot glue creates PTYs:
  - `/dev/ttyINNOPILOT`
  - `/dev/ttyINNOPILOT_BRIDGE`

---

## Wiring cautions (hard learned)
- **Common ground** is mandatory between:
  - Nano, IBT-2, clutch driver, current sensor, voltage dividers, Pi
- Avoid backfeeding +5V between devices unless you *really* know the power path.
- If you see Pi reboots when plugging Nano, treat it as **power integrity / backfeed** first, not software.

