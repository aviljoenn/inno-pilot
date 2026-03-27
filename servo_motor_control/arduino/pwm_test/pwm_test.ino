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

// Drive opposite direction at full speed until ADC is back within
// MOVE_THRESHOLD of target, or RETURN_TIMEOUT_MS expires.
// Also guards the hard limit in the return direction (prevents overshoot
// past the far end if start_adc was near that end).
// Returns true on success, false on timeout or limit guard.
bool return_to_start(int target_adc, int8_t return_dir) {
  // Hard limit for the return direction, with same LIMIT_MARGIN clearance.
  int return_hard_limit = (return_dir > 0) ? (RUDDER_STBD_END - LIMIT_MARGIN)
                                           : (RUDDER_PORT_END  + LIMIT_MARGIN);

  unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < RETURN_TIMEOUT_MS) {
    int cur = read_rudder();
    // Reached target?
    if (abs(cur - target_adc) <= MOVE_THRESHOLD) {
      motor_stop();
      delay(200);
      return true;
    }
    // Limit guard in the return direction — stop rather than crash into end.
    bool at_return_limit = (return_dir > 0) ? (cur >= return_hard_limit)
                                            : (cur <= return_hard_limit);
    if (at_return_limit) {
      motor_stop();
      Serial.println(F("[WARN] Return hit limit guard — stopping short of target."));
      return false;
    }
    motor_drive(return_dir, 255);
  }
  motor_stop();
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

  // --- Pre-flight: check mid-travel ---
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
    Serial.println(F("[SKIP] Not enough mid-travel room. Move rudder to centre and retry."));
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
    if (!return_to_start(start_adc, -dir)) {
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

  run_direction_test(+1);  // STBD
  delay(1000);
  run_direction_test(-1);  // PORT

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
