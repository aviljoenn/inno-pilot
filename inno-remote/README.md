# inno-remote (ESP32-C3 Autopilot Wireless Remote)

**TL;DR:** `inno-remote` is a handheld, battery-powered ESP32-C3 remote for the inno-pilot autopilot. It provides a small SPI OLED UI, 5 autopilot command buttons via an ADC resistor ladder, a manual rudder potentiometer, a 3-position AP/OFF/MANUAL switch, and a dedicated “Emergency Stop” button. Firmware is built with **ESP-IDF** for long-term maintainability (Wi‑Fi control + OTA update capability).

---

## 1) What this is

A small handheld enclosure containing:

- **ESP32‑C3 Super Mini (V2)** as the controller + Wi‑Fi
- **SPI OLED** display for status/menus
- **5 PTM buttons** for autopilot step commands: `-10, -1, Enter, +1, +10` (single ADC via resistor ladder)
- **Large red PTM** button intended as an emergency stop trigger (software command)
- **5k potentiometer** for manual rudder command
- **3‑position toggle switch**: **AP / OFF / MANUAL**
- **9V battery** and **9V→5V buck** powering the ESP32 (and 3V3 rail for logic/display)

Designed to be installed where USB access is difficult later, so the software plan includes **Wi‑Fi control** and **OTA firmware updates**.

---

## 2) Safety and “Emergency Stop” note (important)

The large red button on the remote can **request** an emergency stop over Wi‑Fi, but a wireless remote cannot be made truly fail-safe on its own.

**Fail-safe emergency stop requires a hard power cut near the actuator/motor driver** (or a local safety controller) that does not depend on RF, firmware, or battery state.

This remote’s E‑stop is therefore treated as:
- A **high-priority software command**, and
- A trigger for the autopilot side to immediately disengage output / clutch / drive.

---

## 3) Hardware block diagram (high level)

9V Battery
→ DPDT ON‑OFF‑ON toggle (**Power + Mode sense**)
→ Buck converter (5V)
→ ESP32‑C3 board (5V input)
→ 3V3 rail feeds OLED + analog reference for ladder/pot
→ Inputs: E‑stop, mode sense, button ladder ADC, pot ADC
→ Output: Wi‑Fi messaging to inno-pilot (autopilot controller)

---

## 4) Power design

- **9V battery** feeds a **9V→5V buck**
- Buck 5V goes to ESP32‑C3 **5V/VBUS input**
- OLED and all IO run at **3.3V logic**

Recommended decoupling:
- Near ESP32 5V input: **100µF** bulk + **0.1µF** ceramic
- Near OLED VCC: **0.1µF** ceramic

> Note: Rectangular 9V batteries are poor at sustained high current. If you see brownouts during Wi‑Fi transmissions, upgrade the supply (Li‑ion + boost/buck or AA pack) or increase bulk capacitance and ensure the buck is high quality.

---

## 5) Pin assignment (default wiring used in this project)

> These are the default pins used during construction in this chat. If you change wiring later, update firmware pin definitions accordingly.

### OLED (SPI)
- OLED **CLK/SCK** → **GPIO4**
- OLED **MOSI** → **GPIO5**
- OLED **CS** → **GPIO6**
- OLED **DC** → **GPIO7**
- OLED **RES** → **GPIO10**
- OLED **VCC** → **3V3**
- OLED **GND** → **GND**

### Button ladder (5 PTM autopilot buttons) on one ADC
- Ladder sense node → **GPIO0 (ADC)**
- Pull-up: **10k** from sense node to **3V3**
- Each button pulls the sense node to **GND** through its resistor:
  - `-10` → **22k**
  - `-1` → **10k**
  - `Enter` → **4.7k**
  - `+1` → **2.2k**
  - `+10` → **1k**

Filtering / jitter control (hardware):
- **1k series** into ADC pin (placed at ESP32 end)
- **100nF** from ADC pin to GND (placed at ESP32 end)

### Manual rudder pot (5k) on one ADC
- Pot end A → **3V3**
- Pot end B → **GND**
- Pot wiper → **GPIO1 (ADC)**

Filtering / jitter control (hardware):
- **1k series** into ADC pin (placed at ESP32 end)
- **1µF** from ADC pin to GND (placed at ESP32 end)
- Optional at pot end: **100nF wiper→GND** and **100nF 3V3→GND** if wiring is long/noisy

### Emergency Stop button (digital)
- One side → **GND**
- Other side → **GPIO3** (input with internal pull-up)

### Mode switch (DPDT ON‑OFF‑ON recommended)
Pole A (power):
- 9V+ → common
- top/bottom throws → buck IN+
- middle → open (OFF)

Pole B (mode sense):
- GND → common
- top throw → **GPIO20** (AP mode sense)
- bottom throw → **GPIO21** (MANUAL mode sense)
- middle → open (OFF)

Firmware reads:
- GPIO20 low → AP mode selected
- GPIO21 low → Manual mode selected
- Neither (because power off) → device off

### Buzzer (piezo, via NPN transistor)
- **GPIO8** → **1k resistor** → base (BC547 / 2N2222)
- Collector → buzzer (+)
- Emitter → GND
- Buzzer (−) → 3V3 (or 5V if buzzer requires it)
- Optional: **10k base-to-emitter resistor** to suppress brief boot chirp (or omit to use chirp as power-on indicator)

> **Note:** GPIO8 also drives the onboard RGB LED on the ESP32‑C3 Super Mini V2. The LED may flicker when the buzzer is active — this is harmless and expected since the RGB LED is not used by this project.

---

## 6) Firmware approach (ESP-IDF)

### Why ESP-IDF
- Best long-term support for ESP32 networking features
- Clean pathway for **OTA** updates (important for inaccessible installs)
- More control over timing, tasks, power saving, and robustness

### Planned software modules (high level)
- **Display/UI**: OLED driver + simple pages/menus
- **Input**:
  - ADC ladder decoding (5 buttons) with debounce and threshold bands
  - Pot sampling with filtering + deadband
  - E-stop GPIO interrupt / immediate command path
  - Mode sense GPIOs
- **Comms**:
  - Wi‑Fi connect
  - Messaging protocol to inno-pilot (e.g., UDP/TCP/MQTT — TBD)
- **Reliability**:
  - watchdog
  - reconnect logic
  - OTA with rollback strategy (future)

---

## 7) Build / flash (developer workflow)

Assumes:
- ESP-IDF installed and working
- Board on a COM port (example uses COM9)
- Project target is ESP32‑C3

Secrets (not committed):
```bash
cp wifi_secrets.example.h wifi_secrets.h
# Fill in Wi‑Fi credentials in wifi_secrets.h (do not commit).
```

Build:
```bash
idf.py set-target esp32c3  # Only needed the first time.
idf.py build
idf.py -p COMx flash
```

Typical flow:
```bash
idf.py set-target esp32c3
idf.py -p COM9 flash monitor
```

Faster iterative flashing (app only):
```bash
idf.py -p COM9 app-flash monitor
```

If flashing fails to connect:
- Hold **BOOT**
- Tap **RESET**
- Release **BOOT**
- Retry flash

---

## 8) Current status (v0.2.0_B7)

- Hardware assembled and powered
- ESP-IDF v5.5.2 toolchain verified
- All hardware inputs working: OLED, button ladder, pot, mode switch, ESTOP, buzzer
- Wi-Fi STA with auto-reconnect to boat network
- TCP client connecting to bridge on port 8555
- Full bidirectional integration with bridge:
  - Receives and displays: AP state, heading, command, rudder, flags, mode, warnings
  - Sends: BTN TOGGLE/±1/±10, ESTOP, MODE MANUAL/AUTO, RUD 0-100%
- OLED display: heading, rudder bar, mode, AP state, warning overlays
- Demo mode (no Wi-Fi) for bench testing with simulated data
- Log view (5-tap STOP) for Wi-Fi diagnostics

### What works end-to-end
- AUTO mode: buttons control AP via bridge → pypilot → Nano motor
- MANUAL mode: pot drives rudder directly via bridge → Nano IBT-2 motor
- ESTOP: immediate AP disengage and motor stop
- Steer-loss detection: if TCP drops in MANUAL, Nano buzzer + OLED alarm
- AP-rejected warning: if AP toggle pressed while in MANUAL, OLED shows `?AP Rejected?`

### Not yet implemented
- OTA firmware updates
- Per-boat provisioning (SSID/IP config via menu)

---

## 9) Folder structure (suggested)

```
inno-remote/
  README.md
  firmware/
    hello_world/          # initial bring-up
    inno_remote_app/      # main application
  docs/
    wiring/               # wiring diagrams, photos
    ui/                   # UI screenshots / layout
```

---

## 10) License / attribution

TBD.

If you plan to ship this publicly, add:
- License file (MIT/BSD/Apache-2.0 etc.)
- Any third-party font/driver attributions if used
