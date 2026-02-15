# Inno-Pilot Pinouts (V2)

This document describes the **default V2** pinout. Some pins are optional (limit switches).

## Description of components, inputs, and outputs of the controller

### Inputs and outputs
- 12V power input
- Rudder actuator (bi-directional 12V output)
- Clutch output (12V output)
- Rudder position feedback sensor (3-wire, 10k potentiometer)
- Gyro sensor
- Emergency stop push button (on control box)
- Over-travel limit switches (optional, 2x, 2-wire, NC)
- Display (on control box)

### Major modules
- Pi controller
- Arduino controller
- Power Supply Unit (PSU)

## Power supply

- **12V power input (4-way power terminal block)**
  - 12V positive input → Terminal Block (pin 1) → 15A Fuse Holder (flat)
  - 15A Fuse Holder (round) → Current Sensor (IP+) and Voltage Sensor (pin+)
  - Current Sensor (IP-) → 12V Power Bus+ (Vcc)
  - 12V negative input → Terminal Block (pin 2) → 12V Power Bus- (GND)

## Arduino Nano pinout

### Motor driver (IBT-2 / BTS7960)
- **D2** → IBT-2 **RPWM** (direction)
  - At Arduino: 10kΩ pulldown from D2 → GND
- **D3** → IBT-2 **LPWM** (direction)
  - At Arduino: 10kΩ pulldown from D3 → GND
- **D9** → IBT-2 **R_EN + L_EN bridged** (enable / “PWM gate”)
  - In V2 hydraulic pump mode this is effectively **ON/OFF** (often fixed at 255 in software)
  - At Arduino: 10kΩ pulldown from D9 → GND

IBT-2 power:
- IBT-2 V+ → 12V supply
- IBT-2 GND → 0V (must share common ground with Nano and sensors)
- IBT-2 motor outputs → hydraulic pump motor

### Clutch
- **D11** → clutch driver input
  - **HIGH = clutch engaged**
  - **LOW = clutch disengaged**
- **+CLUTCH** = the +12V that feeds the clutch coil at that end
- **GND_PWR** = the 0V at the MOSFET source end (star common ground)
- 2200 µF electrolytic: within 2–10 cm of MOSFET
  - Cap + → +CLUTCH
  - Cap - → GND_PWR
- Ceramic 100 nF: +CLUTCH → GND_PWR
- +CLUTCH → IRLZ44N Pin 3 (Source) / GND_PWR
- STPS20H100CT sits reverse-biased across the clutch coil and only conducts when the MOSFET turns off and the coil collapses
- Gate pulldown (10kΩ): at the MOSFET between MOSFET Pin 1 (Gate) and Pin 3 (Source)
- 10 nF capacitor: MOSFET Gate Pin 1 → MOSFET Source Pin 3 (physically at MOSFET pins)
- STPS20H100CT (TO-220AB, text facing you, legs down)
  - Pin 1 = Anode (A1)
  - Pin 2 = Cathode (K), tab = Cathode
  - Pin 3 = Anode (A2)

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
- Pot ends → +5V and GND (ensure stable reference)

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
