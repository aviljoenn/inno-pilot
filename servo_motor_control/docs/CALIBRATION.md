# Inno-Pilot Calibration

This is a practical “do this, then that” calibration sequence.

## 0) Baseline checks (before calibration)
- Confirm services are running:
  - `inno-pilot-socat`
  - `inno-pilot-fixlink`
  - `inno-pilot-bridge`
  - `pypilot`
  - `pypilot_web`
- Confirm the servo device used by pypilot points to the by-id path:
  - `~/.pypilot/servodevice`
- Confirm Nano is reachable through the glue path:
  - pypilot should probe and find the servo device
- Confirm OLED is stable and readings update (Vin/I/Temp always bottom line)

---

## 1) IMU calibration (pypilot truth)
IMU: LSM9DS1 on I2C.

### Goals
- stable heading with low drift
- compass distortions resolved
- correct axis alignment

### Steps (high level)
- Use pypilot web UI calibration pages:
  - accelerate/gyro checks
  - compass calibration (full coverage)
  - alignment adjustments (imu.alignmentQ)
- Keep IMU away from magnetic sources as much as practical.

---

## 2) Rudder calibration (pypilot truth)
**Principle:** Inno-Pilot sails by pypilot’s calibrated rudder model. Nano does not “invent” rudder angles.

### Goals
- pypilot knows true rudder angle
- pypilot knows min/max angle (port/stbd limits)
- Nano displays pypilot port|actual|stbd (all from pypilot)

### Steps
1. Ensure rudder pot is wired correctly to Nano (A2) and stable.
2. In pypilot web UI:
   - run rudder calibration procedure (move rudder through range)
   - set/confirm `rudder.range` (e.g. 40°)
   - verify `rudder.offset`, `rudder.scale`, `rudder.nonlinearity`
3. Confirm the UI rudder angle matches physical reality.

### Nano “last-ditch” safety
The Nano should only hard-stop if:
- raw ADC approaches 0 or 1023 (pot wiring/calibration failure)
- pypilot-reported rudder exceeds pypilot-reported limits beyond tolerance

When tripped:
- stop motor + clutch
- raise fault
- show “Calibration required” message

---

## 2b) Rudder sweep — commissioning measurement (v3.0.3+)
Web remote: switch to **OFF** → Settings → Hardware Tests → **1. Rudder Sweep**.

**Prerequisites (do these first):** rudder calibration (section 2) and the port/stbd
rudder limits in Settings. The sweep uses pypilot's calibrated angle and stays inside
the limits; with a wrong calibration it measures nonsense.

**Purpose:** confirm the steering is fit for autopilot use and measure it. Not an
endurance test — it cannot run unattended.

**Running it**
1. Choose amplitude (± degrees each side of centre) and cycles (1–10), press PROCEED.
   The OLED shows `HOLD B3`.
2. Press and **hold B3** for the whole run (dead-man). Letting go, locking/dropping the
   phone, switching apps, closing the page or losing WiFi stops it where it is (it never
   drives back to centre by itself). The Nano's STOP or any helm button also stops it.
3. The results screen appears at the end; the same summary is in the bridge journal
   (`journalctl -u inno-pilot-bridge | grep "RAM sweep"`).

**Minimum amplitude depends on the deadband setting.** The Nano stops anywhere within
`deadband_pct` × full span of a target (10% on a ±30° rudder = ±6°), so the sweep needs
at least deadband + 5° and refuses smaller sweeps with that explanation.

**Reading the results** (per direction, to port / to stbd)
| Field | Meaning | Feeds |
|---|---|---|
| Speed °/s | timed over the central 60% of the travel actually achieved | autopilot expectations; port/stbd balance |
| Hard-over s | estimated full lock-to-lock time at that speed | compare with your drive's spec |
| Lag ms | target sent → rudder starts moving | play / air in the hydraulics |
| Overshoot ° | worst travel past the target | braking, position tolerance |
| End error ° | average final position vs target (− = short) | position tolerance (`deadband_pct`) |
| Settle t/o | strokes that never came to rest | hunting — tolerance too small |
| Current A | peak / mean while driving (only with a current sensor) | future max-current setting |
| Speed difference % | port vs stbd speed | unbalanced ram, leak, air |

If the results say speed was timed **partly/entirely at reduced power**, the Nano was
inside its slow zone (4 × deadband of the target) — speed and hard-over then understate
full-power performance. A smaller deadband or wider sweep fixes that.

PASS = all cycles completed and every stroke settled; CHECK = completed but some
strokes did not settle; STOPPED = ended early (reason shown).

Hardware Tests 2–9 are greyed out: they need the bench firmware `pwm_test.ino` and a
protocol it does not have yet (see `TODO.md`).

---

## 3) Voltage calibration
### A0: main supply voltage
- adjust `VOLTAGE_SCALE` so OLED Vin matches multimeter at the supply input

### A3: Pi Zero 5V bus
- adjust `PI_VSENSE_SCALE` so OLED 5V bus matches multimeter on the Pi 5V rail

Fault thresholds:
- set `PI_VOLT_HIGH_FAULT` / `PI_VOLT_LOW_FAULT` to your real acceptable range
- verify fault alarm behaviour + PTM acknowledgement

---

## 4) Current sensor calibration
Your current sensor ADC response is not perfectly linear, so Inno-Pilot uses a calibration table (piecewise linear interpolation).

### Steps
1. Measure current with a multimeter at several loads.
2. Record corresponding ADC values on A1.
3. Update `CURR_ADC[]` and `CURR_A[]` arrays.
4. Verify the OLED current matches reality across low and higher currents.

---

## 5) Temperature calibration
- DS18B20 on D12
- confirm readings are plausible
- set `MAX_CONTROLLER_TEMP_C` to a conservative safe value (e.g. 50°C)
- verify overtemp fault stops motor/clutch and signals alarm

---

## 6) Button ladder calibration (A6)
- display raw ADC values for each button
- choose thresholds at midpoints between observed values
- confirm:
  - each button is stable (debounced)
  - no “ghost presses”

---

## 7) Save points (where “truth” lives)
- pypilot stores calibration in:
  - `~/.pypilot/pypilot.conf`
  - and `~/.pypilot/pypilot.conf.bak`
- Treat that file as a calibrated artifact (backup it).

