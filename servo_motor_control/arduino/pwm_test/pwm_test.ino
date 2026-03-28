// pwm_test.ino
// Binary-search for the minimum PWM duty that reliably starts the motor
// from standstill on the IBT-2 H-bridge.
//
// Usage:
//   1. Stop inno-pilot-bridge, pypilot services on the Pi.
//   2. Upload this sketch (arduino-cli compile/upload with arduino:avr:nano).
//   3. Open Serial monitor at 115200 baud — results print automatically.
//   4. Rudder should be near mid-travel before starting (see [INIT] message).
//   5. When done, re-flash motor_simple.ino.
//
// Pin wiring (same as motor_simple.ino):
//   A2  = rudder pot (position feedback)
//   A1  = current sensor (IBT-2 analog output)
//   D2  = HBRIDGE_RPWM (direction)
//   D3  = HBRIDGE_LPWM (direction)
//   D9  = HBRIDGE_EN   (R_EN + L_EN tied — PWM speed)
//   D11 = Clutch (HIGH = engaged)
//
// Algorithm:
//   Binary search over 0..255 in each direction (STBD then PORT).
//   Each step: apply PWM for DWELL_MS, measure ADC delta, return to start.
//   Convergence: 8 iterations per direction, ~3 s each = ~30 s total.

#include <Arduino.h>

// ---- Pin map (must match motor_simple.ino exactly) ----
const uint8_t RUDDER_PIN       = A2;
const uint8_t PIN_CURRENT      = A1;
const uint8_t HBRIDGE_RPWM_PIN = 2;
const uint8_t HBRIDGE_LPWM_PIN = 3;
const uint8_t HBRIDGE_PWM_PIN  = 9;   // EN: R_EN + L_EN tied together
const uint8_t CLUTCH_PIN       = 11;

// ---- Hard rudder limits (from motor_simple.ino) ----
const int RUDDER_PORT_END  = 1;
const int RUDDER_STBD_END  = 1022;

// ---- Test tuning ----
// Increase DWELL_MS if the motor is slow to start or load is heavy.
// Decrease MOVE_THRESHOLD if the pot moves very little even at full power.
const uint16_t DWELL_MS           = 1000;  // motor-on time per binary step (ms)
const uint16_t RETURN_TIMEOUT_MS  = 5000;  // max time to return to start position (ms)
const int      MOVE_THRESHOLD     = 8;     // ADC delta (counts) to count as "moved"
const int      LIMIT_MARGIN       = 60;    // safety margin from hard ends (ADC counts)
const int      MIN_TRAVEL_NEEDED  = 250;   // minimum counts of free travel before testing

// ---- Current sensor ----
// From motor_simple.ino calibration: ADC=430 @ 0 A, decreases ~1.3 counts/A.
const int   CURR_ZERO_ADC = 430;
const float CURR_SLOPE    = 1.31f;  // ADC counts per amp (approx)

// ====================================================================
// Low-level helpers
// ====================================================================

// Rudder ADC burst read: 2 dummy reads (S/H channel-switch settle) +
// 6 real reads → trim min/max → average 4.  Same Stage-1 as motor_simple.ino.
int read_rudder() {
  analogRead(RUDDER_PIN);  // dummy 1
  analogRead(RUDDER_PIN);  // dummy 2
  uint16_t mn = 1023, mx = 0, sum = 0;
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t r = (uint16_t)analogRead(RUDDER_PIN);
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    sum += r;
  }
  return (int)(1023 - (sum - mn - mx) / 4);  // invert: match motor_simple.ino convention (STBD=high)
}

// Current ADC: average 16 samples (all on A1 after channel settle).
// Call only when not mid-dwell to avoid mixing A1/A2 reads excessively.
int read_current_adc() {
  analogRead(PIN_CURRENT);  // settle after possible A2 access
  uint32_t s = 0;
  for (uint8_t i = 0; i < 16; i++) s += (uint32_t)analogRead(PIN_CURRENT);
  return (int)(s >> 4);
}

float adc_to_amps(int adc) {
  float delta = (float)(CURR_ZERO_ADC - adc);
  return (delta < 0.0f) ? 0.0f : delta / CURR_SLOPE;
}

void motor_stop() {
  analogWrite(HBRIDGE_PWM_PIN, 0);
  digitalWrite(HBRIDGE_RPWM_PIN, LOW);
  digitalWrite(HBRIDGE_LPWM_PIN, LOW);
}

// Drive H-bridge.  dir: +1 = STBD (ADC increases), -1 = PORT (ADC decreases).
void motor_drive(int8_t dir, uint8_t duty) {
  if (dir > 0) {
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
  } else {
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
  }
  analogWrite(HBRIDGE_PWM_PIN, duty);
}

// Drive toward target_adc at up to RETURN_DUTY, stopping when within
// MOVE_THRESHOLD counts.  Direction is determined each iteration from the
// current vs target position, so it chases the target even if the rudder
// drifted the "wrong" way while the pump was stopped.
// Guards both hard limits.  Returns true on success, false on timeout/limit.
const uint8_t RETURN_DUTY = 220;  // slightly below max to reduce overshoot

bool return_to_start(int target_adc) {
  unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < RETURN_TIMEOUT_MS) {
    int cur = read_rudder();
    // Reached target?
    if (abs(cur - target_adc) <= MOVE_THRESHOLD) {
      motor_stop();
      delay(200);
      return true;
    }
    // Figure out which way to drive from current position, not assumed direction.
    int8_t dir_needed = (cur < target_adc) ? +1 : -1;

    // Limit guards — don't drive past either hard end.
    if (dir_needed > 0 && cur >= (RUDDER_STBD_END - LIMIT_MARGIN)) {
      motor_stop();
      Serial.println(F("[WARN] Return hit STBD limit guard."));
      return false;
    }
    if (dir_needed < 0 && cur <= (RUDDER_PORT_END + LIMIT_MARGIN)) {
      motor_stop();
      Serial.println(F("[WARN] Return hit PORT limit guard."));
      return false;
    }
    motor_drive(dir_needed, RETURN_DUTY);
  }
  motor_stop();
  return false;
}

// ====================================================================
// Auto-centre: drive rudder to CENTRE_ADC before each direction test.
// Uses return_to_start() at RETURN_DUTY.  Fails gracefully if centre
// is unreachable (limit guard or timeout).
// ====================================================================
const int CENTRE_ADC = 512;

bool centre_rudder() {
  Serial.print(F("[CENTRE] Driving to ADC="));
  Serial.print(CENTRE_ADC);
  Serial.print(F(" from ADC="));
  Serial.println(read_rudder());
  if (return_to_start(CENTRE_ADC)) {
    Serial.print(F("[CENTRE] Reached ADC="));
    Serial.println(read_rudder());
    delay(500);   // let hydraulics settle at centre
    return true;
  }
  Serial.println(F("[CENTRE] Failed to reach centre — aborting test."));
  return false;
}

// ====================================================================
// Binary search for minimum starting PWM in one direction
// ====================================================================

void run_direction_test(int8_t dir) {
  const char* label = (dir > 0) ? "STBD" : "PORT";

  Serial.print(F("\n--- Direction: "));
  Serial.print(label);
  Serial.println(F(" ---"));

  // --- Auto-centre before each test ---
  if (!centre_rudder()) return;

  // --- Pre-flight: verify we have enough room ---
  int start_adc = read_rudder();
  int room_fwd  = (dir > 0) ? (RUDDER_STBD_END - start_adc)
                             : (start_adc - RUDDER_PORT_END);
  int room_rev  = (dir > 0) ? (start_adc - RUDDER_PORT_END)
                             : (RUDDER_STBD_END - start_adc);

  Serial.print(F("[INIT] Start ADC="));
  Serial.print(start_adc);
  Serial.print(F("  room_fwd="));
  Serial.print(room_fwd);
  Serial.print(F("  room_rev="));
  Serial.println(room_rev);

  if (room_fwd < MIN_TRAVEL_NEEDED || room_rev < (LIMIT_MARGIN * 2)) {
    Serial.println(F("[SKIP] Not enough mid-travel room after centering."));
    return;
  }

  // --- Binary search ---
  int lo = 0, hi = 255;
  int result = -1;         // -1 = not found yet

  while (lo < hi) {
    int mid = lo + (hi - lo) / 2;

    // Safety: re-check position before each step
    int pre_adc = read_rudder();
    int dist_fwd = (dir > 0) ? (RUDDER_STBD_END - pre_adc)
                              : (pre_adc - RUDDER_PORT_END);
    if (dist_fwd < LIMIT_MARGIN) {
      Serial.println(F("[WARN] Too close to limit before step — aborting."));
      motor_stop();
      break;
    }

    // Baseline current (motor off)
    int curr_idle = read_current_adc();

    // Apply PWM and dwell
    motor_drive(dir, (uint8_t)mid);

    unsigned long t0 = millis();
    uint32_t curr_sum = 0;
    uint16_t curr_cnt = 0;
    bool limit_tripped = false;

    // Check limit on EVERY iteration (~416 µs/loop) so a fast motor cannot
    // overshoot between polls.  Per-iteration pattern:
    //   1. dummy analogRead(A2)  — settle S/H from last A1 read
    //   2. real  analogRead(A2)  — quick limit guard (±a few counts OK here)
    //   3. dummy analogRead(A1)  — settle S/H from A2
    //   4. real  analogRead(A1)  — current sample
    // Accepts minor channel-switch noise; limit margin >> noise floor.
    while ((unsigned long)(millis() - t0) < DWELL_MS) {
      // Quick rudder position check (A2)
      (void)analogRead(RUDDER_PIN);                      // dummy: settle from A1
      int a_now = (int)analogRead(RUDDER_PIN);           // real limit-guard read
      int d_fwd = (dir > 0) ? (RUDDER_STBD_END - a_now)
                             : (a_now - RUDDER_PORT_END);
      if (d_fwd < LIMIT_MARGIN) {
        limit_tripped = true;
        break;
      }

      // Current sample (A1)
      (void)analogRead(PIN_CURRENT);                     // dummy: settle from A2
      curr_sum += (uint32_t)analogRead(PIN_CURRENT);     // real current read
      curr_cnt++;
    }

    motor_stop();
    delay(150);  // brief mechanical settle

    int post_adc  = read_rudder();
    int delta     = (dir > 0) ? (post_adc - pre_adc) : (pre_adc - post_adc);
    int curr_avg  = (curr_cnt > 0) ? (int)(curr_sum / curr_cnt) : curr_idle;
    float amps    = adc_to_amps(curr_avg);
    bool  moved   = (delta > MOVE_THRESHOLD);

    // --- Print step result ---
    Serial.print(F("[TRY ] PWM="));
    if (mid < 100) Serial.print(' ');
    if (mid <  10) Serial.print(' ');
    Serial.print(mid);
    Serial.print(F("  delta="));
    if (delta >= 0 && delta < 100) Serial.print(' ');
    if (delta >= 0 && delta <  10) Serial.print(' ');
    Serial.print(delta);
    Serial.print(F("  ~"));
    // Print amps with 2 decimal places manually (avoids float formatting issues)
    int amps_int  = (int)amps;
    int amps_frac = (int)((amps - (float)amps_int) * 100.0f + 0.5f);
    Serial.print(amps_int);
    Serial.print('.');
    if (amps_frac < 10) Serial.print('0');
    Serial.print(amps_frac);
    Serial.print(F("A"));
    if (limit_tripped) Serial.print(F("  [LIMIT TRIP]"));
    Serial.print(F("  => "));
    Serial.println(moved ? F("MOVED") : F("no move"));

    // Binary search bookkeeping
    if (moved) {
      result = mid;
      hi = mid;       // mid worked — search lower half for smaller starting PWM
    } else {
      lo = mid + 1;   // mid too low — try higher
    }

    // Return to start position
    if (!return_to_start(start_adc)) {
      Serial.println(F("[WARN] Return-to-start timed out. Position may have drifted."));
    }
    delay(300);  // settle before next iteration
  }

  // --- Final result ---
  Serial.print(F("[DONE] "));
  Serial.print(label);
  Serial.print(F(": "));
  if (result >= 0) {
    Serial.print(F("min starting PWM = "));
    Serial.println(result);
  } else {
    Serial.println(F("motor did NOT move at any PWM (0-255). Check wiring/clutch."));
  }
}

// ====================================================================
// Speed sweep: drive at fixed PWM steps and measure counts/second.
// One direction only (STBD) is enough to characterise the motor;
// run with dir=+1 for STBD or dir=-1 for PORT.
// ====================================================================

// PWM values to sweep — starts above threshold, ends at max.
const uint8_t SWEEP_PWM[]  = {100, 110, 120, 130, 140, 150, 170, 190, 210, 230, 255};
const uint8_t SWEEP_N      = sizeof(SWEEP_PWM) / sizeof(SWEEP_PWM[0]);

void run_speed_sweep(int8_t dir) {
  const char* label = (dir > 0) ? "STBD" : "PORT";
  Serial.print(F("\n=== Speed sweep: "));
  Serial.print(label);
  Serial.println(F(" ==="));
  Serial.println(F("  PWM  counts/s  amps"));
  Serial.println(F("  ---  --------  ----"));

  for (uint8_t i = 0; i < SWEEP_N; i++) {
    uint8_t pwm = SWEEP_PWM[i];

    // Centre before each step
    if (!return_to_start(CENTRE_ADC)) {
      Serial.println(F("[WARN] Could not centre — aborting sweep."));
      return;
    }
    delay(400);

    int pre_adc    = read_rudder();
    int curr_idle  = read_current_adc();

    // Dwell
    motor_drive(dir, pwm);
    unsigned long t0 = millis();
    uint32_t curr_sum = 0;
    uint16_t curr_cnt = 0;
    bool limit_hit    = false;

    while ((unsigned long)(millis() - t0) < DWELL_MS) {
      (void)analogRead(RUDDER_PIN);
      int a_now = (int)analogRead(RUDDER_PIN);
      int d_fwd = (dir > 0) ? (RUDDER_STBD_END - a_now) : (a_now - RUDDER_PORT_END);
      if (d_fwd < LIMIT_MARGIN) { limit_hit = true; break; }
      (void)analogRead(PIN_CURRENT);
      curr_sum += (uint32_t)analogRead(PIN_CURRENT);
      curr_cnt++;
    }
    motor_stop();
    delay(150);

    int post_adc  = read_rudder();
    int delta     = (dir > 0) ? (post_adc - pre_adc) : (pre_adc - post_adc);
    int curr_avg  = (curr_cnt > 0) ? (int)(curr_sum / curr_cnt) : curr_idle;
    float amps    = adc_to_amps(curr_avg);

    // Print row: PWM, counts/s (= delta over DWELL_MS seconds), amps
    Serial.print(F("  "));
    if (pwm < 100) Serial.print(' ');
    Serial.print(pwm);
    Serial.print(F("  "));
    if (delta >= 0 && delta < 100) Serial.print(' ');
    if (delta >= 0 && delta <  10) Serial.print(' ');
    // delta is already counts over DWELL_MS ms; convert to counts/s
    int cps = (int)((long)delta * 1000 / DWELL_MS);
    if (cps >= 0 && cps < 100) Serial.print(' ');
    if (cps >= 0 && cps <  10) Serial.print(' ');
    Serial.print(cps);
    Serial.print(F("       "));
    int a_int  = (int)amps;
    int a_frac = (int)((amps - (float)a_int) * 100.0f + 0.5f);
    Serial.print(a_int); Serial.print('.');
    if (a_frac < 10) Serial.print('0');
    Serial.print(a_frac);
    if (limit_hit) Serial.print(F("  [LIMIT]"));
    Serial.println();
  }
  Serial.println(F("=== Sweep done ==="));
}

// ====================================================================
// Coasting / ramp-down test
//
// For each speed (150, 190, 255 PWM), two phases:
//   Phase A — hard cut at trigger point: measures coast distance.
//   Phase B — proportional ramp starting coast_counts before trigger:
//             duty = entry_pwm × (counts_to_target / coast_counts).
//             Measures stopping error vs target.
//
// Reports: PWM | coast | hard_overshoot | ramp_error | improvement
// Positive error = overshoot, negative = undershoot.
// ====================================================================

const int  COAST_TRAVEL    = 150;  // counts to drive before cut/ramp trigger
const uint16_t COAST_SETTLE_MS = 350;  // position stable for this long = stopped

// Poll every 30 ms until position stable for settle_ms or timeout.
// Returns the final stable position.
// settle_ms defaults to COAST_SETTLE_MS; pass a shorter value for fast pulse checks.
int wait_for_stop(unsigned long timeout_ms, uint16_t settle_ms = COAST_SETTLE_MS) {
  int  last     = read_rudder();
  unsigned long t_stable = millis();
  unsigned long t_start  = millis();
  while ((unsigned long)(millis() - t_start) < timeout_ms) {
    delay(30);
    int cur = read_rudder();
    if (abs(cur - last) > 1) {
      t_stable = millis();
      last = cur;
    } else if ((unsigned long)(millis() - t_stable) >= settle_ms) {
      break;
    }
  }
  return read_rudder();
}

void run_coast_test() {
  const uint8_t TEST_PWMS[] = {150, 190, 255};
  const uint8_t N  = sizeof(TEST_PWMS) / sizeof(TEST_PWMS[0]);
  const int8_t  dir = +1;  // STBD — symmetric so one direction is enough

  Serial.println(F("\n=== Coasting / ramp-down test (STBD) ==="));
  Serial.println(F("Travel per step: " )); Serial.print(COAST_TRAVEL); Serial.println(F(" counts"));
  Serial.println(F("  PWM  coast  hard_err  ramp_err  improvement"));
  Serial.println(F("  ---  -----  --------  --------  -----------"));

  for (uint8_t i = 0; i < N; i++) {
    uint8_t pwm = TEST_PWMS[i];

    // ------------------------------------------------------------------
    // Phase A: hard cut — drive COAST_TRAVEL counts then cut to 0.
    //          Measure how far past the trigger the rudder coasts.
    // ------------------------------------------------------------------
    if (!return_to_start(CENTRE_ADC)) {
      Serial.println(F("[ERR] Centre failed, skipping."));
      continue;
    }
    delay(500);
    int start  = read_rudder();
    int target = start + COAST_TRAVEL;   // trigger / ideal stop point

    motor_drive(dir, pwm);
    unsigned long t0 = millis();
    while ((unsigned long)(millis() - t0) < 6000UL) {
      int cur = read_rudder();
      if (cur >= target)                           break;  // reached trigger
      if (cur >= (RUDDER_STBD_END - LIMIT_MARGIN)) break;  // safety
    }
    motor_stop();
    int stop_hard   = wait_for_stop(2000);
    int coast_counts = stop_hard - target;
    if (coast_counts < 0) coast_counts = 0;
    int hard_err    = stop_hard - target;  // >0 = overshoot

    // ------------------------------------------------------------------
    // Phase B: proportional ramp.
    //   Ramp starts coast_counts before target.
    //   duty = entry_pwm × (target − cur) / coast_counts.
    //   Motor turns off when duty reaches 0 or rudder passes target.
    // ------------------------------------------------------------------
    int ramp_err = 9999;
    if (coast_counts > 0) {
      if (!return_to_start(CENTRE_ADC)) {
        Serial.println(F("[ERR] Centre failed, skipping."));
        continue;
      }
      delay(500);
      start         = read_rudder();
      target        = start + COAST_TRAVEL;
      int ramp_start = target - coast_counts;
      if (ramp_start <= start) ramp_start = start + 1;

      motor_drive(dir, pwm);   // set direction and full duty
      t0 = millis();
      bool ramping = false;
      while ((unsigned long)(millis() - t0) < 6000UL) {
        int cur = read_rudder();
        if (cur >= (RUDDER_STBD_END - LIMIT_MARGIN)) { motor_stop(); break; }
        if (cur >= target)                             { motor_stop(); break; }

        if (!ramping && cur >= ramp_start) ramping = true;

        if (ramping) {
          int  dist = target - cur;
          int  duty = (dist <= 0) ? 0 : (int)((long)pwm * dist / coast_counts);
          if (duty < 0)   duty = 0;
          if (duty > pwm) duty = (int)pwm;
          analogWrite(HBRIDGE_PWM_PIN, (uint8_t)duty);
          if (duty == 0) break;
        }
      }
      motor_stop();
      int stop_ramp = wait_for_stop(2000);
      ramp_err = stop_ramp - target;
    }

    // ------------------------------------------------------------------
    // Print result row
    // ------------------------------------------------------------------
    Serial.print(F("  "));
    Serial.print(pwm);
    Serial.print(F("  "));
    if (coast_counts < 100) Serial.print(' ');
    if (coast_counts <  10) Serial.print(' ');
    Serial.print(coast_counts);
    Serial.print(F("     "));
    if (hard_err >= 0) Serial.print('+'); else Serial.print('-');
    int ah = (hard_err >= 0) ? hard_err : -hard_err;
    if (ah < 10) Serial.print('0');
    Serial.print(ah);
    Serial.print(F("       "));
    if (ramp_err == 9999) {
      Serial.print(F("  N/A        N/A"));
    } else {
      if (ramp_err >= 0) Serial.print('+'); else Serial.print('-');
      int ar = (ramp_err >= 0) ? ramp_err : -ramp_err;
      if (ar < 10) Serial.print('0');
      Serial.print(ar);
      Serial.print(F("       "));
      int improvement = abs(hard_err) - abs(ramp_err);
      if (improvement > 0) {
        Serial.print('+'); Serial.print(improvement); Serial.print(F(" better"));
      } else if (improvement < 0) {
        Serial.print(improvement); Serial.print(F(" worse"));
      } else {
        Serial.print(F("no change"));
      }
    }
    Serial.println();
    delay(300);
  }

  Serial.println(F("=== Coast test done ==="));
  Serial.println(F("Errors in ADC counts (+overshoot / -undershoot)"));
  return_to_start(CENTRE_ADC);
}

// ====================================================================
// Comprehensive characterisation table
//
// For each PWM from 120 to 255 in steps of 10 (plus 255), STBD direction:
//   Col 1: coast counts  — hard cut after COAST_TRAVEL, measure overshoot
//   Col 2: min pulse ms  — shortest burst that produces >= PULSE_MOVE counts
//   Col 3: pulse counts  — counts induced by that minimum-duration pulse
//
// Output is tab-friendly for pasting into a spreadsheet.
// ====================================================================

const int PULSE_MOVE = 3;   // counts: threshold for "rudder moved" in pulse test

// ---- Coast measurement (one shot at a fixed travel distance) --------
int measure_coast(uint8_t pwm) {
  if (!return_to_start(CENTRE_ADC)) return -1;
  delay(400);
  int start  = read_rudder();
  int target = start + COAST_TRAVEL;

  motor_drive(+1, pwm);
  unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < 6000UL) {
    int cur = read_rudder();
    if (cur >= target)                            break;
    if (cur >= (RUDDER_STBD_END - LIMIT_MARGIN))  break;
  }
  motor_stop();
  int final_pos   = wait_for_stop(2000);          // long settle: catch full coast
  return final_pos - target;                       // >0 overshoot, <0 undershoot
}

// ---- Minimum pulse: binary search on duration (ms) ------------------
// Returns min duration in ms; sets out_counts to delta at that duration.
// Averages two confirmatory runs at the found minimum for stability.
int measure_min_pulse(uint8_t pwm, int &out_counts) {
  int lo = 5, hi = 300;

  while (lo < hi) {
    int mid = lo + (hi - lo) / 2;

    if (!return_to_start(CENTRE_ADC)) { out_counts = -1; return -1; }
    delay(250);
    int start = read_rudder();

    motor_drive(+1, pwm);
    delay(mid);
    motor_stop();

    int final_pos = wait_for_stop(700, 180);  // short settle: speed up iterations
    int delta     = final_pos - start;

    if (delta >= PULSE_MOVE) hi = mid;
    else                     lo = mid + 1;
  }

  // Confirmatory: run the found minimum twice and average the delta
  int total = 0;
  for (uint8_t k = 0; k < 2; k++) {
    if (!return_to_start(CENTRE_ADC)) break;
    delay(250);
    int start = read_rudder();
    motor_drive(+1, pwm);
    delay(lo);
    motor_stop();
    int fp = wait_for_stop(700, 180);
    int d  = fp - start;
    if (d < 0) d = 0;
    total += d;
  }
  out_counts = total / 2;
  return lo;
}

// ---- Table runner ---------------------------------------------------
void run_comprehensive_table() {
  // 120, 130, ... 250, 255
  const uint8_t PWMS[] = {
    120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 255
  };
  const uint8_t N = sizeof(PWMS) / sizeof(PWMS[0]);

  Serial.println(F("\n=== Comprehensive PWM table (STBD, hard cut) ==="));
  Serial.println(F("Coast travel = " ));
  Serial.print  (COAST_TRAVEL);
  Serial.println(F(" counts before cut"));
  Serial.println(F("Pulse move threshold = " ));
  Serial.print  (PULSE_MOVE);
  Serial.println(F(" counts"));
  Serial.println(F(""));
  Serial.println(F("PWM\tcoast\tmin_pulse_ms\tpulse_counts"));
  Serial.println(F("---\t-----\t------------\t------------"));

  for (uint8_t i = 0; i < N; i++) {
    uint8_t pwm = PWMS[i];

    int coast = measure_coast(pwm);
    if (coast < 0) coast = 0;

    int pulse_counts = 0;
    int min_pulse_ms = measure_min_pulse(pwm, pulse_counts);

    Serial.print(pwm);
    Serial.print(F("\t"));
    Serial.print(coast);
    Serial.print(F("\t"));
    if (min_pulse_ms < 0) Serial.print(F("ERR"));
    else                  Serial.print(min_pulse_ms);
    Serial.print(F("\t\t"));
    if (pulse_counts < 0) Serial.print(F("ERR"));
    else                  Serial.print(pulse_counts);
    Serial.println();

    delay(200);
  }

  Serial.println(F("\n=== Table complete ==="));
  return_to_start(CENTRE_ADC);
}

// ====================================================================
// Arduino entry points
// ====================================================================

void setup() {
  Serial.begin(115200);
  delay(800);  // allow USB serial to connect

  // H-bridge
  pinMode(HBRIDGE_RPWM_PIN, OUTPUT);
  pinMode(HBRIDGE_LPWM_PIN, OUTPUT);
  pinMode(HBRIDGE_PWM_PIN,  OUTPUT);
  motor_stop();

  // Clutch: engage
  pinMode(CLUTCH_PIN, OUTPUT);
  digitalWrite(CLUTCH_PIN, HIGH);
  delay(600);  // allow clutch to seat fully

  // ADC pins
  pinMode(RUDDER_PIN,  INPUT);
  pinMode(PIN_CURRENT, INPUT);

  Serial.println(F("========================================"));
  Serial.println(F("  PWM Motor-Start Threshold Test"));
  Serial.println(F("========================================"));
  Serial.print  (F("  DWELL_MS          = ")); Serial.println(DWELL_MS);
  Serial.print  (F("  MOVE_THRESHOLD    = ")); Serial.println(MOVE_THRESHOLD);
  Serial.print  (F("  RETURN_TIMEOUT_MS = ")); Serial.println(RETURN_TIMEOUT_MS);
  Serial.print  (F("  LIMIT_MARGIN      = ")); Serial.println(LIMIT_MARGIN);
  Serial.print  (F("  MIN_TRAVEL_NEEDED = ")); Serial.println(MIN_TRAVEL_NEEDED);
  Serial.println(F(""));
  Serial.println(F("Ensure rudder is near mid-travel before this runs."));
  Serial.println(F("Starting in 2 s..."));
  delay(2000);

  // Previous results (threshold ~91-105 PWM, speed profile) are already known.
  // This run collects the comprehensive coast + minimum-pulse table only.
  run_comprehensive_table();

  // Safe state
  motor_stop();
  digitalWrite(CLUTCH_PIN, LOW);  // disengage clutch

  Serial.println(F("\n========================================"));
  Serial.println(F("  Test complete. Clutch disengaged."));
  Serial.println(F("  Re-flash motor_simple.ino when ready."));
  Serial.println(F("========================================"));
}

void loop() {
  // Nothing — all work done in setup()
}
