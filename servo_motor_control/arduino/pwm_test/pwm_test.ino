// pwm_test.ino
// Binary-search for the minimum PWM duty that reliably starts the motor
// from standstill on the IBT-2 H-bridge.
//
// ---- Test mode ----
// RUN_INTERACTIVE_TEST (default): manual remote-target test with OLED + B3 button.
//   - Centre rudder, display a pre-set ADC target on OLED, wait for B3 (GO).
//   - Drive at 255 PWM, hard-cut at target, measure coast overshoot.
//   - Repeat for 3 targets × 3 reps × 2 directions = 18 runs.
// Comment out RUN_INTERACTIVE_TEST to run the automated comprehensive table instead.
//
// Usage (either mode):
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
//   A6  = Button ladder (B3 = GO / next, same resistor divider as motor_simple.ino)

// ---- Test mode selector ----
// Define exactly one of the modes:
//   RUN_FINE_BURST_TEST   — 10–30 ms × 1 ms steps, 3 reps each, both dirs (current)
//   RUN_BURST_SWEEP_TEST  — 24-step coarse burst-duration sweep
//   RUN_INTERACTIVE_TEST  — manual target test with B3 GO button
//   (neither)             — automated comprehensive table
#define RUN_FINE_BURST_TEST
//#define RUN_BURST_SWEEP_TEST
//#define RUN_INTERACTIVE_TEST

#include <Arduino.h>

// OLED + button hardware used by all interactive modes
#if defined(RUN_INTERACTIVE_TEST) || defined(RUN_BURST_SWEEP_TEST) || defined(RUN_FINE_BURST_TEST)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

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

// ---- Coasting / ramp-down test constants ----
const int  COAST_TRAVEL    = 150;  // counts to drive before cut/ramp trigger
const uint16_t COAST_SETTLE_MS = 350;  // position stable for this long = stopped

// ---- Speed measurement ----
const uint16_t SPEED_DWELL_MS  = 600;  // ms to drive while measuring counts/s

// ---- Reverse-braking test ----
const uint8_t  BRAKE_THRESHOLD = 5;    // counts: "dead stop" tolerance (±5 counts)
const uint16_t BRAKE_MAX_MS    = 250;  // max reverse braking duration to try (ms)

const int PULSE_MOVE = 3;   // counts: threshold for "rudder moved" in pulse test

// ---- Per-run result for interactive test (defined unconditionally so the
//      Arduino preprocessor can resolve it when generating function prototypes)
struct TargetRunResult {
  int           start_adc;
  int           target_adc;
  int           final_adc;
  int           overshoot;       // +ve = overshot past target, -ve = undershot
  unsigned long drive_ms;        // time from motor-on to settled/cut
  int16_t       speed_at_brake;  // cps when brake was triggered (0 = no brake / hard-cut)
  unsigned long brake_ms_used;   // ms of reverse brake applied  (0 = no brake / hard-cut)
};

// Per-burst result for RUN_BURST_SWEEP_TEST (defined unconditionally so Arduino
// preprocessor can resolve it when generating function prototypes)
struct BurstResult {
  uint16_t burst_ms;    // programmed burst duration (ms)
  int8_t   dir;         // +1 = STBD, -1 = PORT
  int      start_adc;   // ADC at burst start
  int      final_adc;   // ADC after hard-cut + hydraulic settle
  int      counts_moved;// signed net displacement (final - start); includes coast
};

// ====================================================================
// OLED + button — shared by all interactive test modes
// ====================================================================
#if defined(RUN_INTERACTIVE_TEST) || defined(RUN_BURST_SWEEP_TEST) || defined(RUN_FINE_BURST_TEST)

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDR    0x3C
static Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool oled_ok = false;

// ---- Button (A6 resistor ladder, same thresholds as motor_simple.ino) ----
const uint8_t  BUTTON_ADC_PIN  = A6;
const unsigned long BTN_DEBOUNCE_MS = 60UL;

// B3 ADC window: > 257 and <= 419 (from motor_simple.ino decode_button_from_adc)
static bool is_btn3() {
  int v = analogRead(BUTTON_ADC_PIN);
  return (v > 257 && v <= 419);
}

// Block until B3 is pressed and released (debounced).
static void wait_for_btn3() {
  // Wait for press
  while (true) {
    if (is_btn3()) {
      delay(BTN_DEBOUNCE_MS);
      if (is_btn3()) break;
    }
    delay(20);
  }
  // Wait for release
  while (is_btn3()) { delay(20); }
  delay(BTN_DEBOUNCE_MS);
}

#ifdef RUN_INTERACTIVE_TEST
// ---- Algorithm selector ----
// 0 = hard-cut: motor off the instant target ADC is crossed (worst-case baseline)
// 1 = speed-aware braking: motor_simple.ino B5 state machine (RM_DRIVING/BRAKING/SETTLED)
#define TEST_ALGORITHM  1

// ---- Interactive test parameters ----
// 6 runs total (3 STBD + 3 PORT), direction toggles after each run.
// Each run starts from wherever the rudder stopped after the previous run.
// Travel distance is fixed at TRAVEL_COUNTS; must exceed MIN_TRAVEL_COUNTS
// (40% of usable ADC range 61..962 = 901 counts → 360 counts minimum).
#define N_RUNS  6
const int TRAVEL_COUNTS     = 450;  // 50% of usable range per run (~enough to show coast clearly)
const int MIN_TRAVEL_COUNTS = 360;  // 40% floor — warn user if less is available
#endif  // RUN_INTERACTIVE_TEST (constants only)

#endif  // RUN_INTERACTIVE_TEST || RUN_BURST_SWEEP_TEST

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

// ---- Speed: drive from centre for SPEED_DWELL_MS, return counts/s -----
int measure_speed(uint8_t pwm) {
  if (!return_to_start(CENTRE_ADC)) return -1;
  delay(400);
  int start = read_rudder();
  motor_drive(+1, pwm);
  delay(SPEED_DWELL_MS);
  motor_stop();
  delay(150);
  int end_pos = read_rudder();
  int delta   = end_pos - start;
  return (int)((long)delta * 1000L / SPEED_DWELL_MS);
}

// ---- Reverse-brake overshoot: drive COAST_TRAVEL counts STBD, then apply
//      full reverse for brake_ms ms immediately at the cut point.
//      Returns (final_pos - cut_pos): positive = still overshot forward,
//      negative = overbraked past cut point.
int measure_brake_overshoot(uint8_t pwm, uint16_t brake_ms) {
  if (!return_to_start(CENTRE_ADC)) return 9999;
  delay(400);
  int start   = read_rudder();
  int trigger = start + COAST_TRAVEL;
  int cut_pos = start;

  motor_drive(+1, pwm);
  unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < 6000UL) {
    cut_pos = read_rudder();
    if (cut_pos >= trigger)                           break;
    if (cut_pos >= (RUDDER_STBD_END - LIMIT_MARGIN)) break;
  }

  // Apply reverse immediately — no delay between cut and reverse
  if (brake_ms > 0) {
    motor_drive(-1, 255);
    delay(brake_ms);
  }
  motor_stop();

  int final_pos = wait_for_stop(2000);
  return final_pos - cut_pos;
}

// ---- Binary search for minimum reverse braking time that dead-stops the
//      rudder within BRAKE_THRESHOLD counts of the cut point.
//      Returns ms found; -1 if BRAKE_MAX_MS is still insufficient.
int find_brake_ms(uint8_t pwm) {
  // Precondition check: BRAKE_MAX_MS must overbrake (go negative)
  int ov_hi = measure_brake_overshoot(pwm, BRAKE_MAX_MS);
  if (ov_hi > (int)BRAKE_THRESHOLD) {
    return -1;  // even max reverse not enough — report as error
  }

  uint16_t lo = 0, hi = BRAKE_MAX_MS;
  for (uint8_t iter = 0; iter < 8; iter++) {
    uint16_t mid = lo + (hi - lo) / 2;
    int ov = measure_brake_overshoot(pwm, mid);
    if (ov > (int)BRAKE_THRESHOLD) {
      lo = mid + 1;  // still overshot — need more reverse
    } else {
      hi = mid;      // dead-stopped or overbraked — try less
    }
  }
  return (int)hi;
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
  Serial.println(F("PWM\tcoast\tmin_pulse_ms\tpulse_counts\tspeed_cps\tbrake_ms"));
  Serial.println(F("---\t-----\t------------\t------------\t---------\t--------"));

  for (uint8_t i = 0; i < N; i++) {
    uint8_t pwm = PWMS[i];

    int coast = measure_coast(pwm);
    if (coast < 0) coast = 0;

    int pulse_counts = 0;
    int min_pulse_ms = measure_min_pulse(pwm, pulse_counts);

    int spd_cps = measure_speed(pwm);

    int brk_ms  = find_brake_ms(pwm);

    Serial.print(pwm);
    Serial.print(F("\t"));
    Serial.print(coast);
    Serial.print(F("\t"));
    if (min_pulse_ms < 0) Serial.print(F("ERR"));
    else                  Serial.print(min_pulse_ms);
    Serial.print(F("\t\t"));
    if (pulse_counts < 0) Serial.print(F("ERR"));
    else                  Serial.print(pulse_counts);
    Serial.print(F("\t\t"));
    if (spd_cps < 0)      Serial.print(F("ERR"));
    else                  Serial.print(spd_cps);
    Serial.print(F("\t\t"));
    if (brk_ms < 0)       Serial.print(F("ERR"));
    else                  Serial.print(brk_ms);
    Serial.println();

    delay(200);
  }

  Serial.println(F("\n=== Table complete ==="));
  return_to_start(CENTRE_ADC);
}

// ====================================================================
// Interactive target test
//
// Algorithm: hard-cut (worst-case baseline)
//   - Drive at 255 PWM from current rudder position.
//   - Hard cut (motor_stop) the instant the rudder ADC crosses the target.
//   - Wait for rudder to coast to a stop; record final position.
//   - No auto-centring: each run starts where the previous one ended.
//
// Sequence: 6 runs, direction toggles STBD/PORT/STBD/PORT/STBD/PORT.
//   Target = current_pos + dir × 450 cts (clamped to safety limits).
//   Travel ≥ 40% of usable range (360 of 901 cts); OLED warns if short.
// OLED shows: waiting screen (from/to/dist) → driving (static) → result.
// B3 button (A6 ladder) confirms GO and advances to next run.
//
// Serial output: TSV header + one row per run (copy-paste into spreadsheet).
// ====================================================================
#if defined(RUN_INTERACTIVE_TEST) || defined(RUN_BURST_SWEEP_TEST) || defined(RUN_FINE_BURST_TEST)

// ---- OLED screens (shared) ----

static void oled_show_centering_screen(int current_adc) {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.print(F("Centering..."));
  oled.setCursor(0, 14); oled.print(F("ADC: ")); oled.print(current_adc);
  oled.print(F(" -> ")); oled.print(CENTRE_ADC);
  oled.display();
}

#endif  // shared OLED + button block

// ====================================================================
// Fine burst test — 10–30 ms × 1 ms steps, 3 reps each direction
// STBD: starts from PORT limit.  PORT: starts from STBD limit.
// Single B3 press, then fully automatic.
// ====================================================================
#ifdef RUN_FINE_BURST_TEST

static const uint8_t  FINE_REPS       = 10;
static const uint16_t FINE_SETTLE_MS  = 800;   // extra settle after return, before each burst

static void oled_show_fine_wait() {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,  0); oled.print(F("Fine Burst Test"));
  oled.setCursor(0, 12); oled.print(F("15-25ms, 1ms steps"));
  oled.setCursor(0, 24); oled.print(F("10 reps x 2 dirs"));
  oled.setCursor(0, 36); oled.print(F("PWM=255, hard-cut"));
  oled.setCursor(0, 54); oled.print(F("B3 = START"));
  oled.display();
}

static void oled_show_fine_run(uint16_t bms, int8_t dir, uint8_t rep) {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,  0); oled.print(F("Fine Burst Test"));
  oled.setCursor(0, 14); oled.print(dir > 0 ? F(">> STBD") : F("<< PORT"));
  oled.setCursor(0, 28); oled.print(bms); oled.print(F(" ms  rep "));
  oled.print(rep); oled.print(F("/")); oled.print(FINE_REPS);
  oled.setCursor(0, 42); oled.print(F("Running..."));
  oled.display();
}

// Run one direction's fine sweep.
// Before each burst_ms group: return to start_pos (hydraulics reset for that group).
// Within each group: 10 consecutive bursts, max 100ms between reps, no repositioning.
// After the 10 reps: wait for full settle, then return to start for the next group.
static void run_fine_direction(int start_pos, int8_t dir) {
  const char* label = (dir > 0) ? "STBD" : "PORT";
  Serial.print(F("[INFO] "));
  Serial.print(label);
  Serial.print(F(" direction: start ADC="));
  Serial.println(start_pos);

  for (uint16_t bms = 15; bms <= 25; bms++) {
    // Return to start once before this group's 10 reps
    oled_show_centering_screen(read_rudder());
    return_to_start(start_pos);
    wait_for_stop(2000);
    delay(FINE_SETTLE_MS);

    int counts[FINE_REPS];

    for (uint8_t rep = 0; rep < FINE_REPS; rep++) {
      // OLED updates each rep: shows ms value, direction, rep number
      oled_show_fine_run(bms, dir, rep + 1);

      int s = read_rudder();
      motor_drive(dir, 255);
      delay(bms);
      motor_stop();
      delay(250);          // 250ms between reps — keeps hydraulics primed
      int f = read_rudder();
      counts[rep] = f - s;
    }

    // Full settle before the return drive
    wait_for_stop(2000);

    // Print row: burst_ms, dir, c1..cN, avg (1 decimal place)
    int sum = 0;
    for (uint8_t r = 0; r < FINE_REPS; r++) sum += counts[r];
    int avg_x10 = (sum * 10) / (int)FINE_REPS;
    Serial.print(bms);
    Serial.print('\t');
    Serial.print(label);
    for (uint8_t r = 0; r < FINE_REPS; r++) {
      Serial.print('\t');
      if (counts[r] >= 0) Serial.print('+');
      Serial.print(counts[r]);
    }
    Serial.print('\t');
    if (avg_x10 >= 0) Serial.print('+');
    Serial.print(avg_x10 / 10);
    Serial.print('.');
    int frac = avg_x10 % 10;
    if (frac < 0) frac = -frac;
    Serial.println(frac);
  }
}

void run_fine_burst_test() {
  Serial.println(F("\n=== Fine Burst Test (15–25 ms, 1 ms steps, 10 reps) ==="));
  Serial.println(F("PWM=255, hard-cut, 10 reps per step, 250ms between reps, reposition between groups"));
  Serial.println(F("STBD: PORT limit start per group.  PORT: STBD limit start per group."));

  oled_show_fine_wait();
  Serial.println(F("Press B3 to start..."));
  wait_for_btn3();

  Serial.print(F("\nburst_ms\tdir"));
  for (uint8_t i = 1; i <= FINE_REPS; i++) {
    Serial.print('\t'); Serial.print('c'); Serial.print(i);
  }
  Serial.println(F("\tavg"));
  Serial.print(F("--------\t---"));
  for (uint8_t i = 0; i < FINE_REPS; i++) { Serial.print(F("\t--")); }
  Serial.println(F("\t---"));

  // STBD direction: start at PORT limit
  run_fine_direction(RUDDER_PORT_END + LIMIT_MARGIN, +1);

  // PORT direction: start at STBD limit
  run_fine_direction(RUDDER_STBD_END - LIMIT_MARGIN, -1);

  // Park at centre
  oled_show_centering_screen(read_rudder());
  return_to_start(CENTRE_ADC);

  Serial.println(F("\n=== Fine burst test complete ==="));
}

#endif  // RUN_FINE_BURST_TEST

// ====================================================================
// Burst sweep test — 24 burst durations x 2 directions, fully automatic
// ====================================================================
#ifdef RUN_BURST_SWEEP_TEST

// Burst duration sequence (ms).  Fine steps around the hydraulic cracking
// threshold (~55 ms at PWM 255); coarser steps below and above.
static const uint16_t burst_seq[] = {
   10,  20,  30,  40,
   45,  50,  55,  60,  65,  70,  75,  80,  85,  90,
  100, 125, 150, 175, 200, 250, 300, 400, 500, 600
};
static const uint8_t N_BURST_STEPS = (uint8_t)(sizeof(burst_seq) / sizeof(burst_seq[0]));

static void oled_show_burst_wait() {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,  0); oled.print(F("Burst Sweep Test"));
  oled.setCursor(0, 12); oled.print(F("PWM=255, hard-cut"));
  oled.setCursor(0, 24); oled.print(N_BURST_STEPS * 2);
  oled.print(F(" runs, auto"));
  oled.setCursor(0, 36); oled.print(F("Centre rudder 1st"));
  oled.setCursor(0, 54); oled.print(F("B3 = START"));
  oled.display();
}

static void oled_show_burst_run(uint16_t bms, int8_t dir, uint8_t run_num) {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,  0); oled.print(F("Run ")); oled.print(run_num);
  oled.print(F("/")); oled.print(N_BURST_STEPS * 2);
  oled.setCursor(0, 14); oled.print(dir > 0 ? F(">> STBD") : F("<< PORT"));
  oled.setCursor(0, 28); oled.print(bms); oled.print(F(" ms  PWM=255"));
  oled.setCursor(0, 42); oled.print(F("Running..."));
  oled.display();
}

// Execute one hard-cut burst: motor on for burst_ms, hard cut, wait for settle.
// Returns a BurstResult with net counts_moved (includes hydraulic coast).
static BurstResult run_single_burst(int8_t dir, uint16_t bms) {
  BurstResult r;
  r.burst_ms    = bms;
  r.dir         = dir;
  r.start_adc   = read_rudder();
  motor_drive(dir, 255);
  delay(bms);
  motor_stop();
  r.final_adc   = wait_for_stop(2000);
  r.counts_moved = r.final_adc - r.start_adc;
  return r;
}

static void print_burst_row(const BurstResult& r) {
  Serial.print(r.burst_ms);
  Serial.print('\t');
  Serial.print(r.dir > 0 ? F("STBD") : F("PORT"));
  Serial.print('\t');
  Serial.print(r.start_adc);
  Serial.print('\t');
  Serial.print(r.final_adc);
  Serial.print('\t');
  if (r.counts_moved >= 0) Serial.print('+');
  Serial.println(r.counts_moved);
}

void run_burst_sweep_test() {
  Serial.println(F("\n=== Burst Sweep Test ==="));
  Serial.println(F("PWM=255, hard-cut, no braking"));
  Serial.println(F("Returns to ADC=512 between each burst"));
  Serial.print  (F("Steps: ")); Serial.print(N_BURST_STEPS);
  Serial.println(F(" burst durations x 2 directions (STBD then PORT)"));

  // Centre rudder before we start
  oled_show_centering_screen(read_rudder());
  if (!centre_rudder()) return;

  // Single GO press
  oled_show_burst_wait();
  Serial.println(F("Press B3 to start — then hands off..."));
  wait_for_btn3();

  Serial.println(F("\nburst_ms\tdir\tstart\tfinal\tcounts_moved"));
  Serial.println(F("--------\t---\t-----\t-----\t------------"));

  uint8_t run_num = 0;
  for (uint8_t i = 0; i < N_BURST_STEPS; i++) {
    uint16_t bms = burst_seq[i];

    // STBD burst
    return_to_start(CENTRE_ADC);
    delay(300);
    oled_show_burst_run(bms, +1, ++run_num);
    BurstResult rs = run_single_burst(+1, bms);
    print_burst_row(rs);

    // PORT burst
    return_to_start(CENTRE_ADC);
    delay(300);
    oled_show_burst_run(bms, -1, ++run_num);
    BurstResult rp = run_single_burst(-1, bms);
    print_burst_row(rp);
  }

  // Return to centre for safety
  return_to_start(CENTRE_ADC);

  Serial.println(F("\n=== Burst sweep complete ==="));
}

#endif  // RUN_BURST_SWEEP_TEST

// ====================================================================
// Interactive target test — OLED screens (RUN_INTERACTIVE_TEST only)
// ====================================================================
#ifdef RUN_INTERACTIVE_TEST

// ---- OLED screens ----

static void oled_show_waiting(int start_adc, int target_adc, int8_t dir, int run_num, int total_runs) {
  if (!oled_ok) return;
  int dist = abs(target_adc - start_adc);
  bool too_short = (dist < MIN_TRAVEL_COUNTS);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 0);
  oled.print(F("Run "));
  oled.print(run_num);
  oled.print(F("/"));
  oled.print(total_runs);
  oled.print(dir > 0 ? F("  >>STBD") : F("  <<PORT"));

  oled.setCursor(0, 12);
  oled.print(F("From: "));
  oled.print(start_adc);

  oled.setCursor(0, 22);
  oled.print(F("To:   "));
  oled.print(target_adc);

  oled.setCursor(0, 32);
  oled.print(dist);
  oled.print(F(" cts"));
  if (too_short) oled.print(F(" !SHORT"));

  oled.setCursor(0, 54);
  if (too_short) oled.print(F("REPOSITION/B3 go"));
  else           oled.print(F("B3 = GO"));

  oled.display();
}


static void oled_show_driving(int start_adc, int target_adc, int8_t dir) {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  // Direction indicator in large text
  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.print(dir > 0 ? F(">> STBD") : F("<< PORT"));
  oled.setTextSize(1);
  oled.setCursor(0, 20);
  oled.print(F("From: "));
  oled.print(start_adc);
  oled.setCursor(0, 30);
  oled.print(F("To:   "));
  oled.print(target_adc);
  oled.display();
}

static void oled_show_result(const TargetRunResult& r, int run_num, int total_runs) {
  if (!oled_ok) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 0);
  oled.print(F("Run "));
  oled.print(run_num);
  oled.print(F("/"));
  oled.print(total_runs);

  oled.setCursor(0, 12);
  oled.print(F("Tgt:"));
  oled.print(r.target_adc);
  oled.print(F("  Got:"));
  oled.print(r.final_adc);

  oled.setCursor(0, 22);
  oled.print(F("Err: "));
  if (r.overshoot >= 0) oled.print('+');
  oled.print(r.overshoot);
  oled.print(F(" cts"));

  oled.setCursor(0, 32);
  if (r.brake_ms_used > 0) {
    // Speed-brake algo: show brake telemetry
    oled.print(r.speed_at_brake);
    oled.print(F("cps "));
    oled.print(r.brake_ms_used);
    oled.print(F("ms brk"));
  } else {
    // Hard-cut: show drive time
    oled.print(r.drive_ms);
    oled.print(F("ms (hard cut)"));
  }

  oled.setCursor(0, 54);
  oled.print(F("B3 = next"));

  oled.display();
}

// ---- Drive to ADC target, hard cut at trigger point ----
//
// Starts from the current rudder position — no auto-centring.
// The OLED driving screen is shown ONCE before the drive loop starts
// so the I2C update does not delay the cut-point poll.
static TargetRunResult drive_to_target_hard_cut(int target_adc) {
  TargetRunResult r;
  r.target_adc = target_adc;
  r.start_adc  = read_rudder();  // start from wherever the rudder currently is

  int8_t dir  = (target_adc > r.start_adc) ? +1 : -1;

  // Show static driving screen before starting — no OLED updates inside the
  // drive loop so nothing delays the cut-point poll.
  oled_show_driving(r.start_adc, target_adc, dir);

  unsigned long t_start = millis();
  motor_drive(dir, 255);

  while (true) {
    int cur = read_rudder();

    // Hard cut: stop the instant we cross the target ADC
    if (dir > 0 && cur >= target_adc) break;
    if (dir < 0 && cur <= target_adc) break;

    // Safety: hard limits
    if (cur >= (RUDDER_STBD_END - LIMIT_MARGIN)) {
      Serial.println(F("[WARN] STBD limit guard hit"));
      break;
    }
    if (cur <= (RUDDER_PORT_END + LIMIT_MARGIN)) {
      Serial.println(F("[WARN] PORT limit guard hit"));
      break;
    }
    // Safety: timeout
    if ((unsigned long)(millis() - t_start) > 10000UL) {
      Serial.println(F("[WARN] drive timeout"));
      break;
    }
  }
  motor_stop();
  r.drive_ms = millis() - t_start;

  // Wait for rudder to coast to a complete stop
  r.final_adc = wait_for_stop(2000);
  r.overshoot = r.final_adc - target_adc;  // +ve = overshot, -ve = undershot

  return r;
}

// ---- Drive to ADC target: speed-aware braking (motor_simple.ino B5 algorithm) ----
//
// Faithfully replicates the RM_DRIVING / RM_BRAKING / RM_SETTLED state machine
// from motor_simple.ino B5.  Runs in a blocking loop; same constants throughout.
//
// Constants (copied verbatim from motor_simple.ino B5):
//   REMOTE_DEADBAND   = 21 ADC counts
//   BRAKE_MIN_SPEED   = 60 cps
//   SPEED_WINDOW_MS   = 50 ms
//   brake_ms formula  = 0.54 × speed_cps + 72  (integer: (54*s+50)/100 + 72)
//   brake_dist formula = speed_cps × brake_ms / 2000
static TargetRunResult drive_to_target_speed_brake(int target_adc) {
  TargetRunResult r = {0, 0, 0, 0, 0, 0, 0};
  r.target_adc = target_adc;
  r.start_adc  = read_rudder();

  const int     REMOTE_DEADBAND  = 21;
  const int16_t BRAKE_MIN_SPEED  = 60;
  const unsigned long SPEED_WIN  = 50;
  const unsigned long TIMEOUT    = 12000UL;

  // State machine (mirrors motor_simple.ino globals, but local here)
  uint8_t       rm_state          = 0;  // 0=DRIVING 1=BRAKING 2=SETTLED
  int8_t        rm_brake_dir      = 0;
  unsigned long rm_brake_start_ms = 0;
  unsigned long rm_brake_dur_ms   = 0;

  int           speed_prev_adc    = r.start_adc;
  unsigned long speed_prev_ms     = millis();
  int16_t       rudder_speed_cps  = 0;

  int8_t initial_dir = (target_adc > r.start_adc) ? +1 : -1;
  oled_show_driving(r.start_adc, target_adc, initial_dir);

  unsigned long t_start = millis();

  while (true) {
    unsigned long now = millis();
    int a = read_rudder();

    // Safety hard limits
    if (a >= (RUDDER_STBD_END - LIMIT_MARGIN) && initial_dir > 0) {
      motor_stop();
      Serial.println(F("[WARN] STBD limit guard hit"));
      break;
    }
    if (a <= (RUDDER_PORT_END + LIMIT_MARGIN) && initial_dir < 0) {
      motor_stop();
      Serial.println(F("[WARN] PORT limit guard hit"));
      break;
    }
    if ((unsigned long)(now - t_start) > TIMEOUT) {
      motor_stop();
      Serial.println(F("[WARN] drive timeout"));
      break;
    }

    // --- Speed estimate (50 ms sliding window) ---
    if (now - speed_prev_ms >= SPEED_WIN) {
      int delta_adc        = a - speed_prev_adc;
      unsigned long dt     = now - speed_prev_ms;
      rudder_speed_cps     = (int16_t)((long)delta_adc * 1000L / (long)dt);
      speed_prev_adc       = a;
      speed_prev_ms        = now;
    }

    int16_t abs_speed = (rudder_speed_cps >= 0) ? rudder_speed_cps : -rudder_speed_cps;
    int error         = target_adc - a;
    int abs_error     = (error >= 0) ? error : -error;

    // Brake parameters from current speed
    unsigned long est_brake_ms   = (unsigned long)((54L * abs_speed + 50) / 100) + 72UL;
    int           est_brake_dist = (int)((long)abs_speed * (long)est_brake_ms / 2000L);
    if (est_brake_dist < 1) est_brake_dist = 1;

    // --- State machine ---
    if (rm_state == 1) {  // RM_BRAKING
      if (now - rm_brake_start_ms >= rm_brake_dur_ms) {
        motor_stop();
        rm_state = (abs_error <= REMOTE_DEADBAND) ? 2 : 0;
      }
      // else: brake pulse still running — do nothing, pins already set

    } else if (rm_state == 2) {  // RM_SETTLED
      if (abs_error > REMOTE_DEADBAND) {
        rm_state = 0;  // target moved outside deadband — re-drive
        // fall through to DRIVING below
      } else {
        motor_stop();
        break;  // within deadband, we're done
      }
    }

    if (rm_state == 0) {  // RM_DRIVING (also handles fall-through from SETTLED)
      int8_t dir = (error > 0) ? +1 : (error < 0) ? -1 : 0;

      if (dir == 0) {
        motor_stop();
        rm_state = 2;
        break;
      }

      bool moving_toward = (dir > 0 && rudder_speed_cps > 0) ||
                           (dir < 0 && rudder_speed_cps < 0);

      if (moving_toward && abs_speed >= BRAKE_MIN_SPEED && abs_error <= est_brake_dist) {
        // Initiate reverse brake pulse
        rm_brake_dir      = -dir;
        rm_brake_dur_ms   = est_brake_ms;
        rm_brake_start_ms = now;
        rm_state          = 1;
        r.speed_at_brake  = abs_speed;   // record for analysis
        r.brake_ms_used   = est_brake_ms;
        motor_drive(rm_brake_dir, 255);
      } else {
        motor_drive(dir, 255);
      }
    }
  }

  r.drive_ms  = millis() - t_start;
  r.final_adc = wait_for_stop(2000);
  r.overshoot = r.final_adc - target_adc;
  return r;
}

// ---- Print one TSV result row to serial ----
static void print_result_row(int run_num, const char* dir_label, const TargetRunResult& r) {
  Serial.print(run_num);
  Serial.print('\t');
  Serial.print(dir_label);
  Serial.print('\t');
  Serial.print(r.start_adc);
  Serial.print('\t');
  Serial.print(r.target_adc);
  Serial.print('\t');
  Serial.print(r.final_adc);
  Serial.print('\t');
  if (r.overshoot >= 0) Serial.print('+');
  Serial.print(r.overshoot);
  Serial.print('\t');
  Serial.print(r.drive_ms);
  Serial.print('\t');
  Serial.print(r.speed_at_brake);
  Serial.print('\t');
  Serial.print(r.brake_ms_used);
  Serial.println();
}

// ---- Main interactive test loop ----
//
// 6 runs total; direction toggles after each run (STBD/PORT/STBD/PORT/STBD/PORT).
// Each run starts from wherever the rudder stopped at the end of the previous run.
// Target = current_pos + dir × TRAVEL_COUNTS, clamped to safety limits.
// If available travel < MIN_TRAVEL_COUNTS the OLED warns; B3 still proceeds.
void run_remote_target_test() {
  const int total_runs = N_RUNS;
  const int safe_lo    = RUDDER_PORT_END + LIMIT_MARGIN;
  const int safe_hi    = RUDDER_STBD_END - LIMIT_MARGIN;

  Serial.println(F("\n=== Interactive Target Test ==="));
#if TEST_ALGORITHM == 1
  Serial.println(F("Algorithm: speed-aware braking (motor_simple.ino B5)"));
  Serial.println(F("  brake_ms = 0.54*speed_cps + 72,  deadband = 21 cts,  min_speed = 60 cps"));
#else
  Serial.println(F("Algorithm: hard-cut baseline (255 PWM, cut at target crossing)"));
#endif
  Serial.println(F("Direction: alternates STBD/PORT each run"));
  Serial.print  (F("Travel per run: ")); Serial.print(TRAVEL_COUNTS); Serial.println(F(" cts from current stop"));
  Serial.println(F(""));
  Serial.println(F("run\tdir\tstart\ttarget\tfinal\tovershoot\tdrive_ms\tspeed_cps\tbrake_ms"));
  Serial.println(F("---\t---\t-----\t------\t-----\t---------\t--------\t---------\t--------"));

  for (int run_num = 1; run_num <= total_runs; run_num++) {
    // Odd runs = STBD (+1), even runs = PORT (-1)
    int8_t dir = ((run_num % 2) == 1) ? +1 : -1;

    // Compute target from current position; clamp to safe travel range
    int start   = read_rudder();
    int raw_tgt = start + (int)dir * TRAVEL_COUNTS;
    int target  = (raw_tgt < safe_lo) ? safe_lo :
                  (raw_tgt > safe_hi) ? safe_hi : raw_tgt;

    oled_show_waiting(start, target, dir, run_num, total_runs);

    Serial.print(F("[WAIT] Run ")); Serial.print(run_num);
    Serial.print(dir > 0 ? F(" STBD") : F(" PORT"));
    Serial.print(F("  from=")); Serial.print(start);
    Serial.print(F("  to=")); Serial.print(target);
    Serial.print(F("  dist=")); Serial.println(abs(target - start));

    wait_for_btn3();

    // Re-read at the instant of GO in case of hydraulic drift since the
    // waiting screen was drawn, then recompute target from new start.
    start   = read_rudder();
    raw_tgt = start + (int)dir * TRAVEL_COUNTS;
    target  = (raw_tgt < safe_lo) ? safe_lo :
              (raw_tgt > safe_hi) ? safe_hi : raw_tgt;

#if TEST_ALGORITHM == 1
    TargetRunResult r = drive_to_target_speed_brake(target);
#else
    TargetRunResult r = drive_to_target_hard_cut(target);
#endif

    oled_show_result(r, run_num, total_runs);
    print_result_row(run_num, (dir > 0) ? "STBD" : "PORT", r);

    wait_for_btn3();
  }

  Serial.println(F("\n=== Interactive test complete ==="));
  Serial.println(F("Re-flash motor_simple.ino when ready."));

  if (oled_ok) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);  oled.println(F("Test complete!"));
    oled.setCursor(0, 16); oled.println(F("Re-flash"));
    oled.setCursor(0, 24); oled.println(F("motor_simple.ino"));
    oled.display();
  }
}

#endif  // RUN_INTERACTIVE_TEST

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

#if defined(RUN_INTERACTIVE_TEST) || defined(RUN_BURST_SWEEP_TEST) || defined(RUN_FINE_BURST_TEST)
  // OLED init (non-blocking: if absent or dead, oled_ok stays false)
  oled_ok = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oled_ok) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
#ifdef RUN_FINE_BURST_TEST
    oled.setCursor(0, 0);  oled.println(F("Fine Burst Test"));
    oled.setCursor(0, 12); oled.println(F("15-25ms  1ms steps"));
    oled.setCursor(0, 30); oled.println(F("Centre rudder then"));
    oled.setCursor(0, 40); oled.println(F("press B3 to begin"));
#elif defined(RUN_BURST_SWEEP_TEST)
    oled.setCursor(0, 0);  oled.println(F("Burst Sweep Test"));
    oled.setCursor(0, 12); oled.println(F("PWM=255 hard-cut"));
    oled.setCursor(0, 30); oled.println(F("Centre rudder then"));
    oled.setCursor(0, 40); oled.println(F("press B3 to begin"));
#else
    oled.setCursor(0, 0);  oled.println(F("PWM Target Test"));
    oled.setCursor(0, 12); oled.println(F("Hard-cut baseline"));
    oled.setCursor(0, 30); oled.println(F("Centre rudder then"));
    oled.setCursor(0, 40); oled.println(F("press B3 to begin"));
#endif
    oled.display();
  }
#endif

  Serial.println(F("========================================"));
#ifdef RUN_FINE_BURST_TEST
  Serial.println(F("  PWM Fine Burst Test (15-25 ms, 10 reps)"));
#elif defined(RUN_BURST_SWEEP_TEST)
  Serial.println(F("  PWM Burst Sweep Test"));
#elif defined(RUN_INTERACTIVE_TEST)
  Serial.println(F("  PWM Target Test — Hard-Cut Baseline"));
#else
  Serial.println(F("  PWM Motor-Start Threshold Test"));
#endif
  Serial.println(F("========================================"));
  Serial.print  (F("  LIMIT_MARGIN      = ")); Serial.println(LIMIT_MARGIN);
  Serial.print  (F("  RETURN_TIMEOUT_MS = ")); Serial.println(RETURN_TIMEOUT_MS);
  Serial.println(F(""));
  Serial.println(F("Ensure rudder is near mid-travel before this runs."));

#ifdef RUN_FINE_BURST_TEST
  run_fine_burst_test();
#elif defined(RUN_BURST_SWEEP_TEST)
  run_burst_sweep_test();
#elif defined(RUN_INTERACTIVE_TEST)
  Serial.println(F("Waiting for B3 (GO) to start first run..."));
  run_remote_target_test();
#else
  Serial.println(F("Starting in 2 s..."));
  delay(2000);
  // Runs the extended characterisation table:
  //   coast counts, min pulse ms, pulse counts, speed (counts/s), brake ms
  run_comprehensive_table();
#endif

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
