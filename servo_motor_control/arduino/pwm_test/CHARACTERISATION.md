# Hydraulic Rudder Drive — PWM Characterisation

**Hardware:** Arduino Nano + IBT-2 (BTS7960) H-bridge + hydraulic pump + ram
**Date:** 2026-03-28
**Branch:** `feature/inno-remote-integration`
**Installation:** Pi Zero at 192.168.6.13 (dev boat, `.13`)

---

## 1. Purpose

Characterise the specific hydraulic installation so that `motor_simple.ino` can be
tuned for accurate, overshoot-free rudder positioning. Four questions:

1. What is the minimum PWM that reliably starts the pump from standstill?
2. How does rudder speed (counts/s) vary across the PWM range?
3. After a hard stop command, how far does the rudder coast?
4. Can full-reverse braking eliminate that overshoot, and if so, for how long?

---

## 2. Hardware wiring (IBT-2 non-standard)

| Signal | Pin | Role |
|--------|-----|------|
| RPWM   | D2  | Direction (digital) |
| LPWM   | D3  | Direction (digital) |
| EN (R_EN + L_EN tied) | D9 | PWM speed |
| Rudder pot | A2 | Position feedback |
| Current sensor | A1 | IBT-2 analog output |
| Clutch | D11 | HIGH = engaged |

**Direction convention** (matches `motor_simple.ino`):

- STBD: RPWM=LOW, LPWM=HIGH → inverted ADC increases
- PORT: LPWM=LOW, RPWM=HIGH → inverted ADC decreases
- ADC returned by `read_rudder()` = `1023 − raw` so STBD = increasing value.

---

## 3. Test sketch: `pwm_test.ino`

Located at `servo_motor_control/arduino/pwm_test/pwm_test.ino`.

### Key constants

```cpp
const int  COAST_TRAVEL    = 150;   // counts driven before cut/brake trigger
const uint16_t COAST_SETTLE_MS = 350; // settle time after stop (ms)
const uint16_t SPEED_DWELL_MS  = 600; // drive time for speed measurement (ms)
const uint8_t  BRAKE_THRESHOLD = 5;   // "dead stop" tolerance (±counts)
const uint16_t BRAKE_MAX_MS    = 250; // max reverse braking time tried (ms)
const int      PULSE_MOVE      = 3;   // min counts to count as "rudder moved"
const uint8_t  RETURN_DUTY     = 220; // duty used to return rudder to centre
const int      CENTRE_ADC      = 512; // auto-centre target before each test
```

### Functions

| Function | What it does |
|----------|-------------|
| `read_rudder()` | 2 dummy + 6 real ADC reads, trim min/max, avg 4, invert to match `motor_simple.ino` |
| `return_to_start(target)` | Bidirectional return — chases target from either side (handles hydraulic drift) |
| `centre_rudder()` | Drives to ADC=512 before each test |
| `run_direction_test(dir)` | Binary search for minimum starting PWM (0–255) |
| `run_speed_sweep(dir)` | Fixed dwell at 11 PWM steps, reports counts/s |
| `run_coast_test()` | Hard-cut + proportional ramp at 3 PWM values |
| `measure_coast(pwm)` | Drive 150 counts STBD, hard cut, return overshoot counts |
| `measure_min_pulse(pwm, &counts)` | Binary search (5–300 ms) for shortest pulse inducing ≥3 counts |
| `measure_speed(pwm)` | Drive 600 ms, return counts/s |
| `measure_brake_overshoot(pwm, brake_ms)` | Drive 150 counts, cut, apply full reverse for `brake_ms`, return net overshoot |
| `find_brake_ms(pwm)` | Binary search (0–250 ms) for minimum reverse time achieving ≤5 count stop |
| `run_comprehensive_table()` | All five columns for 15 PWM values (120–255 in steps of 10, plus 255) |

### How to run

```bash
# On the Pi — stop services first
sudo systemctl stop pypilot inno-pilot-bridge inno-pilot-socat inno-pilot-fixlink

# Compile and flash
cd ~/inno-pilot/servo_motor_control/arduino/pwm_test
arduino-cli compile --fqbn arduino:avr:nano \
  --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .

# Capture output (Python approach — mirrors bridge HUPCL handling)
python3 /tmp/capture_pwm.py          # see section 4 for script

# When done — reflash production sketch
cd ~/inno-pilot/servo_motor_control/arduino/motor_simple
arduino-cli compile --fqbn arduino:avr:nano \
  --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .

# Restart services
sudo systemctl start inno-pilot-socat
sleep 2
sudo systemctl start inno-pilot-fixlink
sleep 1
sudo systemctl start inno-pilot-bridge
sleep 2
sudo systemctl start pypilot
```

### Capture script (`/tmp/capture_pwm.py`)

```python
import serial, termios, time

s = serial.Serial()
s.port = '/dev/ttyUSB0'
s.baudrate = 115200
s.timeout = 0.2
s.open()

# Clear HUPCL so closing won't reset the Nano (mirrors bridge approach)
attrs = termios.tcgetattr(s.fileno())
attrs[2] &= ~termios.HUPCL
termios.tcsetattr(s.fileno(), termios.TCSANOW, attrs)

# Force DTR LOW→HIGH to guarantee Nano reset via 100nF cap differentiator
s.setDTR(False)
time.sleep(0.2)
s.setDTR(True)
time.sleep(0.1)

deadline = time.time() + 2400   # 40 min safety timeout
buf = b''
with open('/tmp/pwm_result.txt', 'wb') as f:
    while time.time() < deadline:
        data = s.read(256)
        if data:
            f.write(data)
            f.flush()
            buf += data
            if b'Clutch disengaged' in buf:
                break
            if len(buf) > 4096:
                buf = buf[-2048:]
s.close()
```

---

## 4. Results

All measurements: STBD direction from ADC≈512 (centre), single run per cell.

### Full characterisation table

```
PWM   coast   min_pulse_ms   pulse_counts   speed_cps   brake_ms
---   -----   ------------   ------------   ---------   --------
120    25         166             16             86          114
130    27         166             17            100          128
140    29         166             18            116          131
150    33         156             18            126          129
160    36         153             20            180          141
170    39          55              6            138          144
180    36          55              8            138          166
190    42          55             11            151          153
200    39          55              9            155          141
210    42          55             11            150          161
220    46          55             10            163          162
230    46          55             12            158          162
240    46          55             12            165          163
250    48          55             15            173          170
255    51          55             13            185          182
```

**Column definitions:**

- `coast` — ADC counts the rudder travels past the cut point before stopping (hard stop, no braking)
- `min_pulse_ms` — shortest motor-on burst (ms) that induces any rudder movement (≥3 counts)
- `pulse_counts` — counts moved by that minimum-duration pulse (average of 2 runs)
- `speed_cps` — rudder speed in counts/second during sustained drive at this PWM
- `brake_ms` — minimum full-reverse duration (ms at PWM 255) that stops the rudder within ±5 counts of the cut point

---

## 5. Key findings

### 5.1 Two distinct hydraulic regimes

| Range | Cracking lag | Behaviour |
|-------|-------------|-----------|
| PWM 120–155 | 156–166 ms | Long pressure build-up before ram moves; speed ramps gradually |
| PWM 160–255 | 45–55 ms   | Flat hydraulic cracking lag; speed plateaus with high scatter |

**55 ms is the hydraulic cracking constant** — the time to build enough line pressure to crack the ram valve and start movement, independent of pump speed above ~160 PWM.

### 5.2 Speed is largely flat above 160 PWM

Speed range: 86 cps (PWM 120) to ~185 cps (PWM 255). Above 160 PWM the speed is noisy and flat (138–185 cps) — the pump is cavitating or the ram valve is the bottleneck. There is no useful proportional speed control above 160 PWM.

### 5.3 Coast (hard-stop overshoot)

Coast grows from 25 counts (PWM 120) to 51 counts (PWM 255). This is **hydraulic line-pressure tail** — the pump stops immediately but residual pressure continues driving the ram for 25–51 counts (≈2–4 degrees of rudder). It is not motor inertia.

### 5.4 Proportional ramp-down (tested separately)

A proportional ramp `duty = pwm × (target − cur) / coast_counts` starting `coast_counts` before the target reduces overshoot from 33–51 counts to ~20–25 counts. **It cannot reach zero** because residual pressure at the end of the ramp still drives the ram a further 20–25 counts.

### 5.5 Full-reverse braking — highly effective

Applying full PWM (255) in reverse for `brake_ms` milliseconds immediately at the cut point achieves **≤5 count dead stop** at all tested PWM values. The brake duration required:

- Scales roughly linearly with PWM: ~114 ms at PWM 120, ~182 ms at PWM 255
- Linear fit: `brake_ms ≈ 0.47 × PWM + 58` (approximate)
- Is always longer than the hydraulic cracking lag (55 ms) — makes sense: braking requires pressure build-up in the reverse direction plus time to decelerate the ram

**This is the recommended stopping strategy** for precise rudder positioning.

### 5.6 Current draw

Flat at ~1.53 A across all PWM values and directions. No load-current signature at different speeds — the hydraulic pump is the constant load, not the motor bearing.

### 5.7 Minimum useful PWM

Binary search (separate test) found minimum starting PWM ≈ 110–120. Below 110 the pump stalls. `motor_simple.ino` currently uses `MIN_DUTY = MAX_DUTY = 255` (bang-bang only). A practical minimum for sustained drive is 120 PWM.

---

## 6. Run-on-demand calibration plan (NOT YET IMPLEMENTED)

The goal is a field calibration script that can be run on any installation to populate
a JSON file used by the bridge to adapt to site-specific hydraulics.

### 6.1 Script: `inno_pilot_pwm_calibrate.sh`

**Location in repo:** `compute_module/glue/inno_pilot_pwm_calibrate.sh`
**Deployed to:** `/usr/local/sbin/inno_pilot_pwm_calibrate.sh`
**Invocation:** `sudo inno_pilot_pwm_calibrate.sh`

**What it does:**

1. Stops `pypilot`, `inno-pilot-bridge`, `inno-pilot-socat`, `inno-pilot-fixlink`
2. Compiles and flashes `pwm_test.ino` onto the Nano
3. Captures serial output using the Python capture approach above
4. Parses the tab-separated table from the output
5. Writes results to `/var/lib/inno-pilot/pwm_cal.json`
6. Archives a timestamped copy to `/var/lib/inno-pilot/pwm_cal_YYYYMMDD_HHMMSS.json`
7. Recompiles and reflashes `motor_simple.ino`
8. Restarts all services in correct order

### 6.2 JSON output format

```json
{
  "timestamp": "2026-03-28T12:39:00",
  "installation": "autopilotzero",
  "coast_travel": 150,
  "brake_threshold_counts": 5,
  "entries": [
    {
      "pwm": 120,
      "coast_counts": 25,
      "min_pulse_ms": 166,
      "pulse_counts": 16,
      "speed_cps": 86,
      "brake_ms": 114
    }
  ],
  "regime_break_pwm": 160,
  "hydraulic_crack_ms": 55,
  "min_reliable_pwm": 120,
  "brake_fit": { "slope": 0.47, "intercept": 58 }
}
```

### 6.3 Bridge integration

On startup, `inno_pilot_bridge.py` reads `/var/lib/inno-pilot/pwm_cal.json` (if present)
and logs the calibration timestamp and key parameters. Future motor control logic can
use the `brake_ms` lookup to apply timed reverse braking instead of hard stops.

---

## 7. Pending implementation in `motor_simple.ino`

The characterisation data supports the following changes, **not yet implemented**:

1. **Reverse-brake stop**: when a stop command is issued during motion, apply full reverse
   for `brake_ms` (looked up from a small table or linear formula) then cut to zero.
   Expected result: rudder stops dead within ±5 counts of the commanded position.

2. **`MIN_DUTY` update**: raise from current 255 (bang-bang) to 120 for any future
   proportional control implementation.

3. **Regime-aware control**: avoid PWM 120–155 range for precision moves due to
   unpredictable cracking lag; use ≥160 PWM with reverse-brake stop.

These changes should be implemented and tested as a dedicated PR once the
run-on-demand calibration script (section 6) is in place, so any installation can
regenerate the lookup table after the implementation.
