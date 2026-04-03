// motor_simple.ino
// Minimal pypilot-compatible motor controller core
// - 16 MHz Nano, Serial @ 38400
// - Uses crc.h (CRC-8 poly 0x31, init 0xFF)
// - Receives framed 4-byte packets: [MAGIC1, MAGIC2, code, value_lo, value_hi, crc]
// - Sends periodic FLAGS and RUDDER frames with the same framing
// - Drives IBT-2 H-bridge + clutch based on COMMAND / DISENGAGE
// - Obeys limit switches + rudder pot min/max

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "crc.h"    // your existing CRC-8 table + crc8()

enum ButtonID : uint8_t;

// ---- Inno-Pilot version (must match bridge + remote) ----
const char INNOPILOT_VERSION[] = "v1.2.0_B21";
const uint16_t INNOPILOT_BUILD_NUM = 21;  // increment with each push during development

// Boot / online timing (user-tweakable)
bool ap_enabled_remote = false;        // true when AP engaged (set by COMMAND_CODE, cleared by DISENGAGE_CODE)
bool ap_display        = false;        // what OLED shows (can be briefly “optimistic”)
unsigned long ap_display_override_until_ms = 0;
const unsigned long PI_BOOT_EST_MS    = 98000UL;  // 60s estimate, tweak later
const unsigned long ONLINE_SPLASH_MS  = 3000UL;   // 3s "On-line" splash
bool pi_online_at_boot = false;

// Bridge->Nano: extra telemetry (tenths of degrees)
const uint8_t PILOT_HEADING_CODE = 0xE2; // ap.heading * 10 (uint16)
const uint8_t PILOT_COMMAND_CODE = 0xE3; // ap.heading_command * 10 (uint16)
const uint8_t PILOT_RUDDER_CODE  = 0xE4; // rudder.angle * 10 (int16, two's complement)
// Bridge framing + handshake (prevents direct pypilot probe binding to Nano)
const uint8_t BRIDGE_MAGIC1 = 0xA5;
const uint8_t BRIDGE_MAGIC2 = 0x5A;
const uint8_t BRIDGE_HELLO_CODE = 0xF0;
const uint8_t BRIDGE_HELLO_ACK_CODE = 0xF1;
const uint8_t BRIDGE_VERSION_CODE = 0xF2;  // Bridge -> Nano: build number
// Bridge->Nano: pypilot rudder limits (tenths of degrees)
const uint8_t PILOT_RUDDER_PORT_LIM_CODE = 0xE5; // port limit * 10 (int16)
const uint8_t PILOT_RUDDER_STBD_LIM_CODE = 0xE6; // stbd limit * 10 (int16)
// Remote manual control (Bridge->Nano)
const uint8_t MANUAL_RUD_TARGET_CODE = 0xE8; // remote rudder target 0..1000
const uint8_t MANUAL_MODE_CODE       = 0xE9; // 1=enter MANUAL, 0=exit MANUAL
// Warnings from bridge (Bridge->Nano)
const uint8_t WARNING_CODE           = 0xEA; // warning subtype
const uint8_t WARN_NONE              = 0;
const uint8_t WARN_AP_PRESSED        = 1;    // AP button pressed while remote not in MANUAL
const uint8_t WARN_STEER_LOSS        = 2;    // TCP dropped in MANUAL mode
// Nano->Bridge: buzzer state reporting
const uint8_t BUZZER_STATE_CODE      = 0xEB; // 1=buzzer on, 0=off

// Cached telemetry from pypilot (for OLED)
bool     pilot_heading_valid = false;
uint16_t pilot_heading_deg10 = 0;

bool     pilot_command_valid = false;
uint16_t pilot_command_deg10 = 0;

bool     pilot_rudder_valid  = false;
int16_t  pilot_rudder_deg10  = 0;

// Rudder limits from pypilot (tenths of degrees)
bool     pilot_port_lim_valid = false;
int16_t  pilot_port_lim_deg10 = 0;

bool     pilot_stbd_lim_valid = false;
int16_t  pilot_stbd_lim_deg10 = 0;

bool any_serial_rx = false;
unsigned long last_serial_rx_ms = 0;
bool pi_online           = false;    // true once we see first valid frame
unsigned long boot_start_ms    = 0;  // reference after splash
unsigned long splash_until_ms  = 0;  // non-blocking 3 s boot splash timer (0 = not active)
unsigned long pi_online_time_ms = 0; // when we first saw Pi online
// NEW: track if Pi was ever online and when we last heard from it
bool pi_ever_online       = false;
unsigned long last_pi_frame_ms = 0;
const unsigned long PI_OFFLINE_TIMEOUT_MS = 5000UL;  // 5s no frames => offline
uint16_t bridge_build_num = 0;     // received from bridge via BRIDGE_VERSION_CODE
bool     bridge_build_valid = false;

// ---- Pins ----
const uint8_t LED_PIN          = 13;
const uint8_t RUDDER_PIN       = A2;

const uint8_t PTM_PIN          = 4;
const unsigned long PTM_DEBOUNCE_MS = 30;
const uint8_t BUZZER_PIN       = 10;

const uint8_t PIN_DS18B20      = 12;
const uint8_t PIN_PI_VSENSE    = A3;
const uint8_t PIN_VOLTAGE      = A0;
const uint8_t PIN_CURRENT      = A1;
const uint8_t BUTTON_ADC_PIN   = A6;   // analog-only pin for button ladder

// IBT-2 (BTS7960) pins
const uint8_t HBRIDGE_RPWM_PIN = 2;   // RPWM
const uint8_t HBRIDGE_LPWM_PIN = 3;   // LPWM
const uint8_t HBRIDGE_PWM_PIN  = 9;   // EN (R_EN + L_EN tied together)

// Clutch pin (active-HIGH: HIGH = engaged)
const uint8_t CLUTCH_PIN       = 11;

// Limit switches (NC -> GND, HIGH = tripped / broken)
const uint8_t PORT_LIMIT_PIN   = 7;
const uint8_t STBD_LIMIT_PIN   = 8;

const float ADC_VREF              = 5.00f;
const float VOLTAGE_SCALE         = 5.156f;

// New sensor calibration from ADC readings:
// @ 0 A  : ADC = 430 -> V0 ≈ 2.103 V
// @ 10 A : ADC = 417 -> V10 ≈ 2.039 V
// drop ≈ 0.064 V over 10 A => ~6.4 mV/A

const uint8_t ADC_SAMPLES         = 16;

// Rudder ADC two-stage filter:
//   Stage 1 (per loop): 8 reads — 2 dummy (S/H settle) + 6 real.
//                       Drop highest and lowest of the 6; average remaining 4.
//   Stage 2 (FIFO MA): sliding window of RUDDER_FIFO_SIZE Stage-1 values.
//                       Running sum keeps the average O(1) per update.
// Combined noise reduction: σ / (√4 × √RUDDER_FIFO_SIZE) = σ / 8.
// Tune RUDDER_FIFO_SIZE to trade smoothness vs step-response lag.
const uint8_t RUDDER_FIFO_SIZE    = 16;

const float PI_VSENSE_SCALE = 5.25f;
const float PI_VOLT_HIGH_FAULT = 5.40f;
const float PI_VOLT_LOW_FAULT  = 4.80f;
const float MAX_CONTROLLER_TEMP_C = 50.0f;

// Manual jog state (used when AP is disengaged)
bool  manual_override = false;  // true while a manual jog is active
int8_t manual_dir     = 0;      // -1 = port, +1 = starboard, 0 = none

// Remote manual mode state (from Bridge via TCP)
bool     remote_manual_active     = false;  // true when Bridge is in MANUAL mode
uint16_t manual_rud_target_0_1000 = 500;    // remote rudder target 0=full port, 1000=full stbd

// ---- Remote-manual brake state machine ----
// States: 0 = DRIVING (toward target), 1 = BRAKING (timed reverse pulse), 2 = SETTLED (in deadband)
enum RemoteManualState : uint8_t { RM_DRIVING = 0, RM_BRAKING = 1, RM_SETTLED = 2 };
RemoteManualState rm_state         = RM_SETTLED;
int8_t            rm_brake_dir     = 0;       // direction of brake pulse (-1 or +1)
unsigned long     rm_brake_start_ms = 0;      // millis() when brake pulse began
unsigned long     rm_brake_dur_ms  = 0;       // computed brake duration for this pulse

// ---- Rudder speed estimation ----
// Differentiates smoothed ADC over a ~50 ms window.
const unsigned long SPEED_WINDOW_MS = 50;
int           speed_prev_adc  = 511;          // ADC snapshot at start of window
unsigned long speed_prev_ms   = 0;            // millis() at snapshot
int16_t       rudder_speed_cps = 0;           // signed: +ve = increasing ADC (stbd)

// Minimum speed (cps) below which we just coast instead of active braking.
// Below ~60 cps coast distance is within deadband.
const int16_t BRAKE_MIN_SPEED_CPS = 60;
// AP-pressed warning (Bridge rejected AP toggle in MANUAL mode)
bool          ap_pressed_warn_active = false;
unsigned long ap_pressed_warn_ms     = 0;
const unsigned long AP_PRESSED_WARN_MS = 5000UL; // 5s OLED flash duration
unsigned long ap_warn_beep_end_ms    = 0;         // when single beep ends
// Steer-loss warning (remote TCP dropped while in MANUAL mode)
bool steer_loss_active   = false;
bool steer_loss_silenced = false;
// Buzzer state tracking for change reporting to bridge
bool last_buzzer_state = false;

// Current calibration points: ADC reading vs measured amps
const uint8_t  CURR_N = 8;
const uint16_t CURR_ADC[CURR_N] PROGMEM = {430, 427, 426, 424, 423, 422, 420, 417};
const float    CURR_A[CURR_N]   PROGMEM = {0.0f, 0.11f, 0.65f, 1.70f, 2.23f, 2.33f, 4.00f, 5.30f};

static inline uint16_t curr_adc_at(uint8_t idx) {
  return pgm_read_word(&CURR_ADC[idx]);
}

static inline float curr_a_at(uint8_t idx) {
  return pgm_read_float(&CURR_A[idx]);
}

// ---- DS18B20 scheduling ----
const unsigned long TEMP_PERIOD_MS = 1000;
const unsigned long TEMP_CONV_MS = 200;

// ---- OLED ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR  0x3C

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);
SSD1306AsciiWire display;
bool oled_ok = false;
unsigned long oled_last_init_ms = 0;
const unsigned long OLED_INIT_RETRY_MS = 1000UL;

// ---- Rudder calibration (from motor_limit_test) ----
// Raw ADC counts (0..1023)
// Hard safety endpoints for the rudder pot. These should be near the ADC rails so
// manual jog only stops at the mechanical ends if everything else fails.
const int RUDDER_ADC_PORT_END  = 1;     // ADC at port end (lower = port)
const int RUDDER_ADC_STBD_END  = 1022;  // ADC at starboard end (higher = stbd)
const int RUDDER_ADC_MARGIN    = 10;    // safety margin on each end
const int RUDDER_ADC_END_HYST  = 8;    // extra counts to CLEAR end-latch (prevents flicker)

// Tolerance for centring (not used here but handy later)
const int RUDDER_CENTRE_ADC    = (RUDDER_ADC_PORT_END + RUDDER_ADC_STBD_END) / 2;
const int RUDDER_CENTRE_TOL    = 5;

// Rudder angle display range (approx ±40°)
const float RUDDER_RANGE_DEG = 40.0f;

// Enable/disable physical limit switches on D7/D8. (Optional)
// true  -> use NC limit switches on PORT_LIMIT_PIN / STBD_LIMIT_PIN.
// false -> ignore switch pins, use only pot-based soft limits.
const bool LIMIT_SWITCHES_ACTIVE = false;

// ---- Command codes (same as motor.ino) ----
enum commands {
  COMMAND_CODE              = 0xC7,
  RESET_CODE                = 0xE7,
  MAX_CURRENT_CODE          = 0x1E,
  MAX_CONTROLLER_TEMP_CODE  = 0xA4,
  MAX_MOTOR_TEMP_CODE       = 0x5A,
  RUDDER_RANGE_CODE         = 0xB6,
  RUDDER_MIN_CODE           = 0x2B,
  RUDDER_MAX_CODE           = 0x4D,
  REPROGRAM_CODE            = 0x19,
  DISENGAGE_CODE            = 0x68,
  MAX_SLEW_CODE             = 0x71,
  EEPROM_READ_CODE          = 0x91,
  EEPROM_WRITE_CODE         = 0x53,
  CLUTCH_PWM_AND_BRAKE_CODE = 0x36
};

// ---- Forward declarations ----
uint16_t read_rudder_scaled();
int read_rudder_adc();
void service_rudder_adc();
bool port_limit_switch_hit();
bool stbd_limit_switch_hit();
bool oled_try_init(bool allow_blocking_splash);

// ---- Result codes ----
enum results {
  CURRENT_CODE         = 0x1C,
  VOLTAGE_CODE         = 0xB3,
  CONTROLLER_TEMP_CODE = 0xF9,
  MOTOR_TEMP_CODE      = 0x48,
  RUDDER_SENSE_CODE    = 0xA7,
  FLAGS_CODE           = 0x8F,
  EEPROM_VALUE_CODE    = 0x9A
};

// ---- Flags (subset, matching your Python decode) ----
enum {
  SYNC                = 0x0001,
  OVERTEMP_FAULT      = 0x0002,
  OVERCURRENT_FAULT   = 0x0004,
  ENGAGED             = 0x0008,
  INVALID             = 0x0010,
  PORT_PIN_FAULT      = 0x0020,
  STARBOARD_PIN_FAULT = 0x0040,
  BADVOLTAGE_FAULT    = 0x0080,
  MIN_RUDDER_FAULT    = 0x0100,   // starboard end
  MAX_RUDDER_FAULT    = 0x0200,   // port end
  CURRENT_RANGE       = 0x0400,
  BAD_FUSES           = 0x0800,
  COMMS_WARN_FAULT    = 0x1000,  // error rate elevated (>= 5 errors in 10-second window)
  COMMS_CRIT_FAULT    = 0x2000,  // error rate unsafe (>= 15/10s held 3s) — AP auto-disengaged
  REBOOTED            = 0x8000
};

// Button IDs from the A6 resistor ladder
enum ButtonID : uint8_t {
  BTN_NONE = 0,
  BTN_B1,  // -10 deg
  BTN_B2,  // -1  deg
  BTN_B3,  // AP toggle
  BTN_B4,  // +1 deg
  BTN_B5   // +10  deg
};

// Optional: button event codes to send to Pi via servo protocol
const uint8_t BUTTON_EVENT_CODE = 0xE0;  // new result code, must be handled by Pi-side helper

enum ButtonEvent {
  BTN_EVT_NONE    = 0,
  BTN_EVT_MINUS10 = 1,
  BTN_EVT_MINUS1  = 2,
  BTN_EVT_TOGGLE  = 3,
  BTN_EVT_PLUS10  = 4,
  BTN_EVT_PLUS1   = 5,
  BTN_EVT_STOP    = 6
};

// Overlay for big transient messages (e.g. -10, +1, AP: ON)
bool overlay_active = false;
char overlay_text[8] = "";
unsigned long overlay_start_ms = 0;
const unsigned long OVERLAY_DURATION_MS = 600UL;  // 0.6s display

// ---- RX state ----
uint8_t  in_bytes[3];
uint8_t  sync_b        = 0;
uint8_t  in_sync_count = 0;
uint8_t  bridge_magic_state = 0;

// ---- Diagnostic counters (shown on OLED row 2) ----
uint16_t rx_good_count   = 0;  // CRC-valid frames processed
uint16_t rx_crc_err_count = 0;  // CRC mismatches
uint16_t rx_sync_count   = 0;  // frames discarded during initial sync

// ---- Comms-fault rate detection (sliding 10-second window) ----
const uint8_t  COMMS_DIAG_CODE         = 0xEC;    // Nano->Bridge: lo=err_window_sum, hi=crit_consec_s
const uint8_t  COMMS_ERR_DETAIL_CODE   = 0xED;    // Nano->Bridge: lo=corrupt_code, hi=rx_crc
const unsigned long ERR_DETAIL_MIN_MS  = 200UL;   // rate limit: max 5 detail frames/second
const uint8_t  COMMS_ERR_BUCKETS    = 10;      // 10 x 1-second buckets = 10-second window
const uint8_t  COMMS_WARN_THRESH    = 5;       // errors in window to enter WARN state
const uint8_t  COMMS_CRIT_THRESH    = 15;      // errors in window to enter CRITICAL state
const uint8_t  COMMS_CRIT_HOLD_S    = 3;       // consecutive seconds above CRIT before disengage
const unsigned long COMMS_BUCKET_MS = 1000UL;

uint8_t       err_buckets[COMMS_ERR_BUCKETS];  // per-second error count (circular buffer)
uint8_t       err_bucket_idx    = 0;           // index of current bucket
unsigned long err_bucket_ms     = 0;           // start time of current bucket
uint8_t       err_window_sum    = 0;           // cached sum of all 10 buckets
uint8_t       crit_consec_s     = 0;           // consecutive seconds above CRIT threshold

bool          comms_warn_active      = false;
bool          comms_crit_active      = false;
bool          comms_fault_disengaged = false;  // true after autonomous AP disengage on CRIT
bool          comms_fault_silenced   = false;  // true after user silences comms buzzer via PTM

// ---- Error detail forwarding (1-slot, latest-wins, rate-limited to bridge) ----
bool          err_detail_pending     = false;   // true when an unsent error detail is queued
uint16_t      err_detail_value       = 0;       // packed: lo=code byte, hi=received CRC
unsigned long err_detail_last_ms     = 0;       // last time an error detail frame was sent

// ---- State / telemetry ----
uint16_t flags            = REBOOTED;   // reported once then cleared
uint16_t last_command_val = 1000;       // 0..2000, 1000 = neutral
unsigned long last_command_ms = 0;
uint16_t rudder_raw       = 0;          // 0..65535 (scaled)
int      rudder_adc_last  = 0;          // 0..1023, inverted (higher = stbd)

// ---- Rudder ADC FIFO moving-average state ----
// service_rudder_adc() writes here; all other code reads rudder_adc_smoothed.
uint16_t rdr_fifo[16];                 // ring buffer — size matches RUDDER_FIFO_SIZE
uint8_t  rdr_fifo_idx         = 0;    // next write position
uint32_t rdr_fifo_sum         = 0;    // running sum for O(1) average
bool     rdr_fifo_ready       = false; // false until first Stage-1 pre-fills the FIFO
int      rudder_adc_smoothed  = 511;  // published value, inverted (higher = stbd)
int      rudder_adc_raw_disp  = 511;  // non-inverted, for OLED debug row

float pi_voltage_v = 0.0f;
bool  pi_overvolt_fault = false;
bool  pi_undervolt_fault = false;
bool  pi_fault = false;
bool  pi_fault_alarm_silenced = false;

// Main supply (Vin) voltage faults
const float VIN_LOW_FAULT_V  = 10.9f;   // below this: undercharge / brown-out
const float VIN_HIGH_FAULT_V = 15.1f;   // above this: overcharge risk
float vin_v = 0.0f;
bool  vin_low_fault  = false;
bool  vin_high_fault = false;

float temp_c = NAN;
bool temp_pending = false;
unsigned long temp_cycle_ms = 0;
unsigned long temp_req_ms = 0;

float current_ema_a = 0.0f;
bool  current_ema_init = false;
const float CURRENT_EMA_ALPHA = 0.20f;
// ---- Current sensor debug (for calibration) ----
uint16_t current_debug_adc = 0;
float    current_debug_v   = 0.0f;

// ---- Telemetry timing ----
unsigned long last_flags_ms  = 0;
unsigned long last_rudder_ms = 0;
unsigned long last_current_ms = 0;
unsigned long last_voltage_ms = 0;
unsigned long last_temp_ms = 0;
const unsigned long FLAGS_PERIOD_MS  = 200; // 5 Hz
const unsigned long RUDDER_PERIOD_MS = 200; // 5 Hz
const unsigned long CURRENT_PERIOD_MS = 200; // 5 Hz
const unsigned long VOLTAGE_PERIOD_MS = 200; // 5 Hz
const unsigned long TEMP_PERIOD_SEND_MS = 500; // 2 Hz

void show_overlay(const char *text) {
  strncpy(overlay_text, text, sizeof(overlay_text));
  overlay_text[sizeof(overlay_text) - 1] = '\0';
  overlay_active = true;
  overlay_start_ms = millis();
}

ButtonID decode_button_from_adc(int adc) {
  // These thresholds are starting guesses. You'll tune them based on real ADC values.
  // NONE: high ADC, buttons bring it down in steps.
  if (adc > 865) return BTN_NONE;
  if (adc > 608) return BTN_B5;  // +10 deg
  if (adc > 419) return BTN_B4;  // +1 deg
  if (adc > 257) return BTN_B3;  // AP toggle
  if (adc > 139) return BTN_B2;  // -1 deg
  return BTN_B1;                 // -10 deg
}

void send_button_event(uint16_t ev) {
  // Reuse send_frame(): code=BUTTON_EVENT_CODE, 16-bit value = ev
  send_frame(BUTTON_EVENT_CODE, ev);
}

void handle_button(ButtonID b) {
  if (b == BTN_NONE) return;

  switch (b) {
    case BTN_B1:  // -10
      show_overlay("-10");
      send_button_event(BTN_EVT_MINUS10);
      break;

    case BTN_B2:  // -1
      show_overlay("-1");
      send_button_event(BTN_EVT_MINUS1);
      break;

    case BTN_B3: { // AP On or Off (optimistic local UI, remote truth follows)
      ap_display = !ap_display;
      ap_display_override_until_ms = millis() + 2000UL; // 2s grace for remote confirm
      show_overlay(ap_display ? "AP: ON" : "AP: OFF");
      send_button_event(BTN_EVT_TOGGLE);
      break;
    }

    case BTN_B4:  // +1
      show_overlay("+1");
      send_button_event(BTN_EVT_PLUS1);
      break;

    case BTN_B5:  // +10
      show_overlay("+10");
      send_button_event(BTN_EVT_PLUS10);
      break;

    default:
      break;
  }
}

// Helper: send a framed 4-byte packet [MAGIC1, MAGIC2, code, value_lo, value_hi, crc]
void send_frame(uint8_t code, uint16_t value) {
  uint8_t body[3];
  body[0] = code;
  body[1] = value & 0xFF;
  body[2] = (value >> 8) & 0xFF;
  uint8_t c = crc8(body, 3);
  Serial.write(BRIDGE_MAGIC1);
  Serial.write(BRIDGE_MAGIC2);
  Serial.write(body, 3);
  Serial.write(c);
}

// ---- Comms-fault sliding-window helpers ----

// Record one CRC error into the current 1-second bucket.
void comms_err_record() {
  if (err_buckets[err_bucket_idx] < 255) {
    err_buckets[err_bucket_idx]++;
    if (err_window_sum < 255) err_window_sum++;
  }
}

// Advance the circular bucket window every 1 second.
// Also maintains crit_consec_s (consecutive seconds above CRIT threshold).
void comms_err_bucket_tick(unsigned long now) {
  if (err_bucket_ms == 0) {
    // First call: initialise (global array is already zero-initialised)
    err_bucket_ms = now;
    return;
  }
  while (now - err_bucket_ms >= COMMS_BUCKET_MS) {
    err_bucket_ms += COMMS_BUCKET_MS;
    // Move to next slot: subtract outgoing bucket then zero it
    err_bucket_idx = (err_bucket_idx + 1) % COMMS_ERR_BUCKETS;
    if (err_window_sum >= err_buckets[err_bucket_idx]) {
      err_window_sum -= err_buckets[err_bucket_idx];
    } else {
      err_window_sum = 0;
    }
    err_buckets[err_bucket_idx] = 0;
    // Evaluate CRIT hold counter once per second
    if (err_window_sum >= COMMS_CRIT_THRESH) {
      if (crit_consec_s < 255) crit_consec_s++;
    } else {
      crit_consec_s = 0;
    }
  }
}

uint16_t read_adc_avg(uint8_t pin, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += (uint16_t)analogRead(pin);
    delayMicroseconds(200);
  }
  return (uint16_t)(sum / samples);
}

float adc_to_volts(uint16_t adc) {
  return (adc * ADC_VREF) / 1023.0f;
}

float read_voltage_v() {
  uint16_t adc = read_adc_avg(PIN_VOLTAGE, ADC_SAMPLES);
  float v_adc = adc_to_volts(adc);
  return v_adc * VOLTAGE_SCALE;
}

float read_current_a() {
  uint16_t adc = read_adc_avg(PIN_CURRENT, ADC_SAMPLES);

    // debug tracking
  current_debug_adc = adc;
  current_debug_v   = adc_to_volts(adc);

  // Above "zero" point → treat as 0 A
  if (adc >= curr_adc_at(0)) {
    return curr_a_at(0);
  }
  // Below or equal to lowest calibration → clamp to max
  if (adc <= curr_adc_at(CURR_N - 1)) {
    return curr_a_at(CURR_N - 1);
  }

  // Find the segment CURR_ADC[i] >= adc >= CURR_ADC[i+1]
  for (uint8_t i = 0; i < CURR_N - 1; i++) {
    uint16_t a0 = curr_adc_at(i);
    uint16_t a1 = curr_adc_at(i + 1);

    if (adc <= a0 && adc >= a1) {
      
      float x0 = (float)a0;
      float x1 = (float)a1;
      float y0 = curr_a_at(i);
      float y1 = curr_a_at(i + 1);

      // Since ADC decreases with current, invert the fraction:
      float t = (x0 - (float)adc) / (x0 - x1);   // 0 at x0, 1 at x1
      float I = y0 + t * (y1 - y0);
      if (I < 0.0f) I = 0.0f;
      return I;
    }
  }

  // Fallback (shouldn't hit if table covers full range)
  return 0.0f;
}

float smooth_current_for_display(float a_instant) {
  if (!current_ema_init) {
    current_ema_a = a_instant;
    current_ema_init = true;
    return current_ema_a;
  }

  current_ema_a = (CURRENT_EMA_ALPHA * a_instant) +
                  ((1.0f - CURRENT_EMA_ALPHA) * current_ema_a);
  return current_ema_a;
}

float read_pi_voltage_v() {
  uint16_t adc = read_adc_avg(PIN_PI_VSENSE, ADC_SAMPLES);
  float v_adc = adc_to_volts(adc);
  return v_adc * PI_VSENSE_SCALE;
}

void temp_service(unsigned long now) {
  if (!temp_pending) {
    if (temp_cycle_ms == 0 || (now - temp_cycle_ms >= TEMP_PERIOD_MS)) {
      tempSensors.requestTemperatures();
      temp_req_ms = now;
      temp_cycle_ms = now;
      temp_pending = true;
    }
  }

  if (temp_pending && (now - temp_req_ms >= TEMP_CONV_MS)) {
    float t = tempSensors.getTempCByIndex(0);
    if (t > -55.0f && t < 125.0f) {
      temp_c = t;
    }
    temp_pending = false;
  }
}



void oled_draw() {
  if (!oled_ok) {
    return;
  }

  const unsigned long now = millis();

  // Non-blocking boot splash: hold the display on the splash screen until timer expires
  if (splash_until_ms) {
    if (now < splash_until_ms) return;
    splash_until_ms = 0;   // expired — fall through to first normal draw
  }

  // ----------------------------
  // Measurements used everywhere
  // ----------------------------
  float vin        = read_voltage_v();
  float ia_instant = read_current_a();
  float ia         = smooth_current_for_display(ia_instant);

  bool temp_valid  = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);

  // Format bottom line: Vin / I / Temp (ALWAYS at y=52)
  char vnum[10], inum[10], tnum[10];
  char vbuf[12], ibuf[12], tbuf[12];

  // Vin
  dtostrf(vin, 0, 1, vnum);
  strncpy(vbuf, vnum, sizeof(vbuf));
  vbuf[sizeof(vbuf) - 1] = '\0';
  strncat(vbuf, "V", sizeof(vbuf) - strlen(vbuf) - 1);

  // Current
  dtostrf(ia, 0, 2, inum);
  strncpy(ibuf, inum, sizeof(ibuf));
  ibuf[sizeof(ibuf) - 1] = '\0';
  strncat(ibuf, "A", sizeof(ibuf) - strlen(ibuf) - 1);

  // Temp
  if (!temp_valid) {
    strncpy(tbuf, "--.-C", sizeof(tbuf));
    tbuf[sizeof(tbuf) - 1] = '\0';
  } else {
    dtostrf(temp_c, 0, 1, tnum);
    strncpy(tbuf, tnum, sizeof(tbuf));
    tbuf[sizeof(tbuf) - 1] = '\0';
    strncat(tbuf, "C", sizeof(tbuf) - strlen(tbuf) - 1);
  }

  // ----------------------------
  // Controller status state
  // ----------------------------
  unsigned long elapsed_boot  = now - boot_start_ms;
  unsigned long since_last_pi = pi_ever_online ? (now - last_pi_frame_ms) : 0;

  bool pi_timed_out       = pi_ever_online && (since_last_pi > PI_OFFLINE_TIMEOUT_MS);
  bool pi_never_seen      = !pi_ever_online;
  bool within_boot_window = pi_never_seen && (elapsed_boot < PI_BOOT_EST_MS);
  bool boot_offline       = pi_never_seen && (elapsed_boot >= PI_BOOT_EST_MS);

  // Full clear only on display state transition (avoids per-frame flicker)
  // States: 0=boot, 1=offline, 2=online, 3=overlay
  static uint8_t prev_state = 0xFF;
  uint8_t cur_state;
  if (overlay_active && (now - overlay_start_ms < OVERLAY_DURATION_MS)) {
    cur_state = 3;
  } else if (within_boot_window) {
    cur_state = 0;
  } else if (boot_offline || pi_timed_out) {
    cur_state = 1;
  } else {
    cur_state = 2;
  }
  if (cur_state != prev_state) {
    display.clear();
    prev_state = cur_state;
  }

  // ----------------------------
  // Partial-update: skip rows 0-1 when their content hasn't changed.
  // Saves ~25% of I2C time (two of eight rows).
  // ----------------------------
  // Rows 0-1 content now depends only on cur_state (Row 1 is blank)
  uint8_t top_ctx = cur_state;
  static uint8_t prev_top_ctx = 0xFF;
  bool top_changed = (top_ctx != prev_top_ctx);
  prev_top_ctx = top_ctx;

  // ----------------------------
  // Draw
  // ----------------------------
  display.setFont(System5x7);
  display.set1X();

  // --- Rows 0-1: only redraw when context changed (or during boot — progress %) ---
  if (top_changed || within_boot_window) {
    // --- Row 0: heading (always name + version; boot shows progress instead) ---
    display.setCursor(0, 0);
    if (within_boot_window) {
      uint8_t pct = (uint8_t)((elapsed_boot * 100UL) / PI_BOOT_EST_MS);
      if (pct > 100) pct = 100;
      display.print(F("Inno-Cntl:Boot "));
      display.print(pct);
      display.print(F("%"));
    } else {
      display.print(F("Inno-Ctrl "));
      display.print(INNOPILOT_VERSION);
    }
    display.clearToEOL();

    // --- Row 1: fault/warning area — cleared on state change; fault section overwrites each frame ---
    display.setCursor(0, 1);
    display.clearToEOL();
  }

  // --- Pre-calculate rudder overshoot (used in rows 2-3 warning and row 6 bar) ---
  bool rudder_overshoot_active = false;
  bool rudder_over_port        = false;
  bool rudder_over_stbd        = false;
  {
    bool lims_ok = pilot_rudder_valid && pilot_port_lim_valid && pilot_stbd_lim_valid
                   && (pilot_stbd_lim_deg10 > pilot_port_lim_deg10);
    if (lims_ok) {
      if (pilot_rudder_deg10 < pilot_port_lim_deg10) {
        rudder_overshoot_active = true;
        rudder_over_port        = true;
      } else if (pilot_rudder_deg10 > pilot_stbd_lim_deg10) {
        rudder_overshoot_active = true;
        rudder_over_stbd        = true;
      }
    }
  }

  // --- Rows 1-2: fault/warning display ---
  // Priority: steer_loss > hw_fault > ver_mismatch > comms_crit > comms_warn > rud_overshoot > ap_warn > overlay
  // 2X messages span rows 1-2; 1X messages use row 1 only. Row 3 is always free for Helm.
  {
    bool any_hw_fault = pi_fault || (flags & OVERTEMP_FAULT) || vin_low_fault || vin_high_fault;
    bool hw_fault_shown = any_hw_fault && !pi_fault_alarm_silenced;

    // Expire ap_pressed_warn after 5s
    if (ap_pressed_warn_active && (now - ap_pressed_warn_ms >= AP_PRESSED_WARN_MS)) {
      ap_pressed_warn_active = false;
    }

    if (steer_loss_active && !steer_loss_silenced) {
      // STEER LOSS: flash 2X "!STEER LOSS" every 400ms
      static bool sl_vis = true;
      static unsigned long sl_flash_ms = 0;
      if (now - sl_flash_ms >= 400UL) {
        sl_flash_ms = now;
        sl_vis = !sl_vis;
      }
      if (sl_vis) {
        display.set2X();
        const char *msg = "!STEER LOSS";
        int16_t x = (SCREEN_WIDTH - (int16_t)strlen(msg) * 12) / 2;
        if (x < 0) x = 0;
        display.setCursor(x, 1);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (hw_fault_shown) {
      // Hardware fault: flash 2X message every 400ms
      static bool fault_vis = true;
      static unsigned long fault_flash_ms = 0;
      if (now - fault_flash_ms >= 400UL) {
        fault_flash_ms = now;
        fault_vis = !fault_vis;
      }
      if (fault_vis) {
        // Priority: OVERTEMP > PiV > Vin
        const char *msg;
        if      (flags & OVERTEMP_FAULT) msg = "!OVERTEMP!";
        else if (pi_overvolt_fault)      msg = "!PiV HIGH!";
        else if (pi_undervolt_fault)     msg = "!PiV LOW!";
        else if (vin_high_fault)         msg = "!Vin HIGH!";
        else                             msg = "!Vin LOW!";
        display.set2X();
        int16_t x = (SCREEN_WIDTH - (int16_t)strlen(msg) * 12) / 2;
        if (x < 0) x = 0;
        display.setCursor(x, 1);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (bridge_build_valid && bridge_build_num != INNOPILOT_BUILD_NUM) {
      // Version mismatch: flash 1X warning — all components must run the same build
      static bool vm_vis = true;
      static unsigned long vm_flash_ms = 0;
      if (now - vm_flash_ms >= 500UL) { vm_flash_ms = now; vm_vis = !vm_vis; }
      if (vm_vis) {
        display.setCursor(0, 1);
        display.print(F("!VER MISMATCH!"));
        display.clearToEOL();
        char vmbuf[22];
        snprintf(vmbuf, sizeof(vmbuf), "Pi:B%u Nano:B%u", bridge_build_num, INNOPILOT_BUILD_NUM);
        display.setCursor(0, 2);
        display.print(vmbuf);
        display.clearToEOL();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (comms_crit_active && !comms_fault_silenced) {
      // COMMS CRITICAL: flash 2X "!COMMS ERR!" every 400ms
      static bool cf_vis = true;
      static unsigned long cf_flash_ms = 0;
      if (now - cf_flash_ms >= 400UL) {
        cf_flash_ms = now;
        cf_vis = !cf_vis;
      }
      if (cf_vis) {
        display.set2X();
        const char *msg = "!COMMS ERR!";
        int16_t x = (SCREEN_WIDTH - (int16_t)strlen(msg) * 12) / 2;
        if (x < 0) x = 0;
        display.setCursor(x, 1);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (comms_warn_active) {
      // COMMS WARN: static 1X error rate summary
      display.setCursor(0, 1);
      char wbuf[22];
      snprintf(wbuf, sizeof(wbuf), "?Comms:%u err/10s", err_window_sum);
      display.print(wbuf);
      display.clearToEOL();
      display.setCursor(0, 2); display.clearToEOL();
    } else if (rudder_overshoot_active) {
      // Rudder exceeds soft limits: static 1X warning
      display.setCursor(0, 1);
      display.print(F("?Rud>Limit"));
      display.clearToEOL();
      display.setCursor(0, 2); display.clearToEOL();
    } else if (ap_pressed_warn_active) {
      // AP-pressed warning: 1X "?AP Pressed?" (no flashing)
      display.setCursor(0, 1);
      display.print(F("?AP Pressed?"));
      display.clearToEOL();
      display.setCursor(0, 2); display.clearToEOL();
    } else if (overlay_active && (now - overlay_start_ms < OVERLAY_DURATION_MS)) {
      // Overlay (big transient button feedback) on rows 1-2
      display.set2X();
      uint8_t len = strlen(overlay_text);
      int16_t x = (SCREEN_WIDTH - (int16_t)len * 12) / 2;
      if (x < 0) x = 0;
      display.setCursor(x, 1);
      display.print(overlay_text);
      display.clearToEOL();
      display.set1X();
    } else {
      if (!overlay_active || (now - overlay_start_ms >= OVERLAY_DURATION_MS)) {
        overlay_active = false;
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    }
  }

  // --- Row 7: V/A/T (always shown) ---
  display.setCursor(0, 7);
  display.print(vbuf);
  display.print(F("  "));
  display.print(ibuf);
  display.print(F("  "));
  display.print(tbuf);
  display.clearToEOL();

  // --- Boot window: show Nano + Bridge versions in the middle rows, then done ---
  if (within_boot_window) {
    // Row 4: Nano version centred
    {
      char buf[22];
      snprintf(buf, sizeof(buf), "Nano: %s", INNOPILOT_VERSION);
      int16_t tw = strlen(buf) * 6;
      display.setCursor((SCREEN_WIDTH - tw) / 2, 4);
      display.print(buf);
      display.clearToEOL();
    }
    // Row 5: Bridge version centred
    {
      char buf[22];
      if (bridge_build_valid) {
        snprintf(buf, sizeof(buf), "Bridge: v0.2.0_B%u", bridge_build_num);
      } else {
        snprintf(buf, sizeof(buf), "Bridge: waiting...");
      }
      int16_t tw = strlen(buf) * 6;
      display.setCursor((SCREEN_WIDTH - tw) / 2, 5);
      display.print(buf);
      display.clearToEOL();
    }
    return;
  }

  // Offline: clear middle rows and return
  if (boot_offline || pi_timed_out) {
    display.setCursor(0, 3); display.clearToEOL();
    display.setCursor(0, 4); display.clearToEOL();
    display.setCursor(0, 5); display.clearToEOL();
    display.setCursor(0, 6); display.clearToEOL();
    return;
  }

  // --- Online-only rows ---

  // Row 3: Helm mode — HAND (manual jog), AUTO (AP engaged), REMOTE (TCP remote manual)
  display.setCursor(0, 3);
  if (remote_manual_active) {
    display.print(F("Helm: REMOTE"));
  } else if (ap_display) {
    display.print(F("Helm: AUTO"));
  } else {
    display.print(F("Helm: HAND"));
  }
  display.clearToEOL();

  // Row 4: Cmd (left, 3-digit leading zeros) | HDG right-justified (3-digit leading zeros)
  {
    char row5[22];
    char tmp[8];
    memset(row5, ' ', 21);
    row5[21] = '\0';

    // Left: "Cmd:NNN"
    if (pilot_command_valid) {
      uint16_t deg = ((pilot_command_deg10 + 5) / 10) % 360;
      snprintf(tmp, sizeof(tmp), "Cmd:%03u", (unsigned)deg);
    } else {
      strncpy(tmp, "Cmd:---", sizeof(tmp));
    }
    memcpy(row5, tmp, 7);

    // Right: "HDG:NNN" — right-justified (cols 14-20 of the 21-char row)
    if (pilot_heading_valid) {
      uint16_t deg = ((pilot_heading_deg10 + 5) / 10) % 360;
      snprintf(tmp, sizeof(tmp), "HDG:%03u", (unsigned)deg);
    } else {
      strncpy(tmp, "HDG:---", sizeof(tmp));
    }
    memcpy(row5 + 14, tmp, 7);

    display.setCursor(0, 4);
    display.print(row5);
    display.clearToEOL();
  }

  // Row 5: Graphical rudder bar — P---I---S
  // 'P' = port end (col 0), 'S' = stbd end (col 20), 'I' = indicator at proportional position.
  // Track filled with '-'. When rudder exceeds a limit the 'I' blinks over 'P' or 'S'.
  {
    char rudbar[22];
    memset(rudbar, '-', 21);
    rudbar[0]  = 'P';
    rudbar[20] = 'S';
    rudbar[21] = '\0';

    bool lims_ok = pilot_rudder_valid && pilot_port_lim_valid && pilot_stbd_lim_valid
                   && (pilot_stbd_lim_deg10 > pilot_port_lim_deg10);

    if (lims_ok) {
      if (rudder_over_port) {
        // Blink 'I' over 'P' (col 0)
        static bool blink_p = true;
        static unsigned long blink_p_ms = 0;
        if (now - blink_p_ms >= 400UL) { blink_p_ms = now; blink_p = !blink_p; }
        if (blink_p) rudbar[0] = 'I';
      } else if (rudder_over_stbd) {
        // Blink 'I' over 'S' (col 20)
        static bool blink_s = true;
        static unsigned long blink_s_ms = 0;
        if (now - blink_s_ms >= 400UL) { blink_s_ms = now; blink_s = !blink_s; }
        if (blink_s) rudbar[20] = 'I';
      } else {
        // Map pilot_rudder_deg10 in [port_lim, stbd_lim] → columns 1..19
        int32_t num = (int32_t)(pilot_rudder_deg10 - pilot_port_lim_deg10) * 18;
        int32_t den = pilot_stbd_lim_deg10 - pilot_port_lim_deg10;
        int16_t col = (int16_t)(num / den) + 1;
        if (col < 1)  col = 1;
        if (col > 19) col = 19;
        rudbar[col] = 'I';
      }
    } else if (pilot_rudder_valid) {
      // No limit data: map pilot rudder angle over ±RUDDER_RANGE_DEG assumption
      float rdeg = pilot_rudder_deg10 / 10.0f;
      int16_t col = 10 + (int16_t)(rdeg * 9.0f / RUDDER_RANGE_DEG + 0.5f);
      if (col < 1)  col = 1;
      if (col > 19) col = 19;
      rudbar[col] = 'I';
    }
    // else: no valid rudder data — show P and S only, no indicator

    display.setCursor(0, 5);
    display.print(rudbar);
    display.clearToEOL();
  }
  // --- Row 6: averaged A2 ADC value (rudder pot), centre-justified ---
  {
    int a2_raw = rudder_adc_raw_disp;  // use pre-averaged value; no extra analogRead
    char buf[8];
    snprintf(buf, sizeof(buf), "A2:%d", a2_raw);
    int16_t tw = (int16_t)strlen(buf) * 6;
    display.setCursor((SCREEN_WIDTH - tw) / 2, 6);
    display.print(buf);
    display.clearToEOL();
  }
}

bool oled_try_init(bool allow_blocking_splash) {
  // Probe I2C to check if OLED is present
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    oled_ok = false;
    return false;
  }

  display.begin(&Adafruit128x64, OLED_ADDR);
  display.setFont(System5x7);
  oled_ok = true;

  display.clear();

  // Splash: big Inno-Pilot
  display.set2X();
  display.setCursor(0, 1);  // row 1 (y=8)
  display.println(F("Inno-Pilot"));

  display.set1X();
  display.setCursor(6, 4);  // row 4 (y=32), shifted left to fit on screen
  display.print(F("Version "));
  display.println(INNOPILOT_VERSION);

  // Non-blocking 3 s splash: oled_draw() will return early until the timer expires
  if (allow_blocking_splash && !pi_online_at_boot) {
    splash_until_ms = millis() + 3000UL;
  }

  return true;
}

// Read rudder pot and scale to 0..65535 for telemetry
uint16_t read_rudder_scaled() {
  int a = read_rudder_adc();   // 0..1023 (inverted so higher = starboard)
  rudder_adc_last = a;              // save raw for limit logic
  return (uint16_t)a * 64;          // 0..~65472
}

int read_rudder_adc() {
  // Return the pre-averaged, channel-settled value maintained by
  // service_rudder_adc(). Already inverted: higher counts = starboard.
  return rudder_adc_smoothed;
}

// Called once per main-loop iteration (end of loop, after all other ADC work).
//
// Stage 1 — synchronous burst of 8 reads, every loop:
//   2 dummy reads  : settle S/H cap after any preceding channel (A6/A0/A1/A3)
//   6 real reads   : take min/max on the fly (no array)
//   drop min + max : trimmed mean of 4 remaining values → stage1 (0..1023)
//   Cost: 8 × ~104 µs = ~832 µs per loop.
//
// Stage 2 — 16-slot FIFO moving average (O(1) running sum):
//   Insert stage1, evict oldest; published value = running_sum / RUDDER_FIFO_SIZE.
//   On first call, pre-fill all slots so output is immediately valid.
//   Combined noise floor: σ / (√4 × √16) = σ / 8.
void service_rudder_adc() {
  // --- Stage 1: trimmed burst ---
  (void)analogRead(RUDDER_PIN);   // dummy 1 — S/H cap settle
  (void)analogRead(RUDDER_PIN);   // dummy 2 — belt-and-braces
  uint16_t mn = 1023, mx = 0, sum6 = 0;
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t r = (uint16_t)analogRead(RUDDER_PIN);
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    sum6 += r;
  }
  uint16_t stage1 = (sum6 - mn - mx) / 4;  // trimmed mean of 4 middle values

  // --- Stage 2: FIFO moving average ---
  if (!rdr_fifo_ready) {
    // Pre-fill all slots with the first valid reading for instant stable output.
    for (uint8_t i = 0; i < RUDDER_FIFO_SIZE; i++) rdr_fifo[i] = stage1;
    rdr_fifo_sum   = (uint32_t)stage1 * RUDDER_FIFO_SIZE;
    rdr_fifo_idx   = 0;
    rdr_fifo_ready = true;
  } else {
    rdr_fifo_sum            -= rdr_fifo[rdr_fifo_idx];  // evict oldest
    rdr_fifo[rdr_fifo_idx]   = stage1;                  // insert newest
    rdr_fifo_sum            += stage1;
    rdr_fifo_idx = (rdr_fifo_idx + 1) % RUDDER_FIFO_SIZE;
  }

  // Publish: FIFO average, inverted so higher = starboard.
  uint16_t avg_raw    = (uint16_t)(rdr_fifo_sum / RUDDER_FIFO_SIZE);
  rudder_adc_smoothed = 1023 - (int)avg_raw;  // inverted for limit/motor logic
  rudder_adc_raw_disp = (int)avg_raw;          // non-inverted for OLED debug row
}

// ---- Limit logic helpers ----
// V2: if LIMIT_SWITCHES_ACTIVE is false, ignore the switch pins completely.
bool port_limit_switch_hit() {
  if (!LIMIT_SWITCHES_ACTIVE) {
    return false;
  }
  // NC -> GND, so HIGH = open/tripped/broken
  return digitalRead(PORT_LIMIT_PIN) == HIGH;
}

bool stbd_limit_switch_hit() {
  if (!LIMIT_SWITCHES_ACTIVE) {
    return false;
  }
  return digitalRead(STBD_LIMIT_PIN) == HIGH;
}

// ---- Motor + clutch drive based on last_command_val & flags ----
void update_motor_from_command() {
  // Always update rudder ADC for limit logic
  int a = read_rudder_adc();   // 0..1023 (inverted so higher = starboard)
  rudder_adc_last = a;
  
  unsigned long now = millis();
  bool pi_alive = pi_ever_online && (now - last_pi_frame_ms <= PI_OFFLINE_TIMEOUT_MS);
  bool ap_active = ap_enabled_remote && pi_alive;
  bool command_recent = (now - last_command_ms <= PI_OFFLINE_TIMEOUT_MS);
  bool pilot_limits_ok = pi_alive &&
                         pilot_rudder_valid &&
                         pilot_port_lim_valid &&
                         pilot_stbd_lim_valid &&
                         (pilot_port_lim_deg10 < pilot_stbd_lim_deg10);
  const int16_t PILOT_LIMIT_HYST_DEG10 = 5;  // 0.5 deg hysteresis to avoid jitter
  bool at_port_pilot_enter = pilot_limits_ok && (pilot_rudder_deg10 <= pilot_port_lim_deg10);
  bool at_stbd_pilot_enter = pilot_limits_ok && (pilot_rudder_deg10 >= pilot_stbd_lim_deg10);
  int16_t port_pilot_exit = pilot_port_lim_deg10 + PILOT_LIMIT_HYST_DEG10;
  int16_t stbd_pilot_exit = pilot_stbd_lim_deg10 - PILOT_LIMIT_HYST_DEG10;
  bool at_port_pilot_hold = pilot_limits_ok && (pilot_rudder_deg10 <= port_pilot_exit);
  bool at_stbd_pilot_hold = pilot_limits_ok && (pilot_rudder_deg10 >= stbd_pilot_exit);

  int16_t delta = (int16_t)last_command_val - 1000;   // -1000..+1000
  const int16_t DEADBAND = 20;

  bool manual_jog_active = manual_override;
  int8_t manual_jog_dir = manual_dir;
  if (!manual_override && !ap_active && !remote_manual_active && pi_alive && command_recent) {
    if (delta > DEADBAND) {
      manual_jog_active = true;
      manual_jog_dir = +1;
    } else if (delta < -DEADBAND) {
      manual_jog_active = true;
      manual_jog_dir = -1;
    }
  }

  // Clutch engages when AP is enabled remotely OR manual jog is active OR remote MANUAL mode.
  // Safety: clutch forced OFF during pi_fault or overtemp.
  bool clutch_should = (manual_jog_active || ap_active || remote_manual_active) &&
                       !pi_fault &&
                       !(flags & OVERTEMP_FAULT);

  digitalWrite(CLUTCH_PIN, clutch_should ? HIGH : LOW);

  // If in a fault state, don't drive the motor at all
  if (pi_fault || (flags & OVERTEMP_FAULT)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Enter thresholds (near the ends)
  bool at_port_enter = port_limit_switch_hit() ||
                       (a <= (RUDDER_ADC_PORT_END + RUDDER_ADC_MARGIN));

  bool at_stbd_enter = stbd_limit_switch_hit() ||
                       (a >= (RUDDER_ADC_STBD_END - RUDDER_ADC_MARGIN));

  // Exit thresholds (must move further away before we clear the latch)
  // PORT end is low  ADC: we "hold" port-end while ADC <= port_exit
  // STBD end is high ADC: we "hold" stbd-end while ADC >= stbd_exit
  int port_exit = RUDDER_ADC_PORT_END + (RUDDER_ADC_MARGIN + RUDDER_ADC_END_HYST);
  int stbd_exit = RUDDER_ADC_STBD_END - (RUDDER_ADC_MARGIN + RUDDER_ADC_END_HYST);

  bool at_port_hold = port_limit_switch_hit() || (a <= port_exit);
  bool at_stbd_hold = stbd_limit_switch_hit() || (a >= stbd_exit);

  // Latch ensures only one end is active; hysteresis prevents flicker/reset on jitter.
  static uint8_t end_latch = 0;  // 0 none, 1 port, 2 stbd

  switch (end_latch) {
    case 0:
      if (at_port_enter && !at_stbd_enter) {
        end_latch = 1;
      } else if (at_stbd_enter && !at_port_enter) {
        end_latch = 2;
      } else if (at_port_enter && at_stbd_enter) {
        // If both appear "true" (noise / overlap), pick the closer end.
        int dist_port = abs(a - RUDDER_ADC_PORT_END);
        int dist_stbd = abs(a - RUDDER_ADC_STBD_END);
        end_latch = (dist_port <= dist_stbd) ? 1 : 2;
      }
      break;

    case 1:
      // Stay latched until we've moved away past the EXIT threshold
      if (!at_port_hold) end_latch = 0;
      break;

    case 2:
      if (!at_stbd_hold) end_latch = 0;
      break;
  }

  bool at_port_end = (end_latch == 1) && at_port_hold;
  bool at_stbd_end = (end_latch == 2) && at_stbd_hold;

  // Pilot limit latch mirrors the ADC end latch to avoid jitter around calibrated limits.
  static uint8_t pilot_latch = 0;  // 0 none, 1 port, 2 stbd

  if (!pilot_limits_ok) {
    pilot_latch = 0;
  } else {
    switch (pilot_latch) {
      case 0:
        if (at_port_pilot_enter && !at_stbd_pilot_enter) {
          pilot_latch = 1;
        } else if (at_stbd_pilot_enter && !at_port_pilot_enter) {
          pilot_latch = 2;
        } else if (at_port_pilot_enter && at_stbd_pilot_enter) {
          int dist_port = abs(pilot_rudder_deg10 - pilot_port_lim_deg10);
          int dist_stbd = abs(pilot_rudder_deg10 - pilot_stbd_lim_deg10);
          pilot_latch = (dist_port <= dist_stbd) ? 1 : 2;
        }
        break;

      case 1:
        if (!at_port_pilot_hold) pilot_latch = 0;
        break;

      case 2:
        if (!at_stbd_pilot_hold) pilot_latch = 0;
        break;
    }
  }

  bool at_port_pilot = (pilot_latch == 1) && at_port_pilot_hold;
  bool at_stbd_pilot = (pilot_latch == 2) && at_stbd_pilot_hold;

  // Update rudder fault flags (as before)
  if (at_port_end) {
    flags |= MAX_RUDDER_FAULT;
  } else {
    flags &= ~MAX_RUDDER_FAULT;
  }

  if (at_stbd_end) {
    flags |= MIN_RUDDER_FAULT;
  } else {
    flags &= ~MIN_RUDDER_FAULT;
  }

  // ---- Remote MANUAL mode: drive at 255 PWM with active reverse-braking ----
  //
  // Speed-aware braking eliminates the coast overshoot that plagued the old
  // bang-bang controller.  The motor always runs at full duty (255) but a
  // timed reverse pulse decelerates the pump so it stops near the target
  // centre.  The 21-count deadband absorbs residual error.
  //
  // Brake formula (linear fit to measured coast/speed/brake_ms table):
  //   brake_ms  = 0.54 * speed_cps + 72
  //   brake_dist ≈ speed_cps * brake_ms / 2000  (half-speed assumption)
  //
  // State machine: RM_DRIVING → RM_BRAKING → RM_SETTLED
  //   RM_DRIVING : full 255 PWM toward target
  //   RM_BRAKING : timed reverse pulse running; no re-evaluation until done
  //   RM_SETTLED : inside deadband, motor off
  //
  if (remote_manual_active) {
    // Map 0..1000 target to port-end..stbd-end ADC range
    int target_adc = RUDDER_ADC_PORT_END +
      (int)((long)(RUDDER_ADC_STBD_END - RUDDER_ADC_PORT_END) * manual_rud_target_0_1000 / 1000);
    const int REMOTE_DEADBAND = 21;  // ADC counts — sized to absorb braking residual

    int error = target_adc - a;      // positive = need to go stbd (increase ADC)
    int abs_error = (error >= 0) ? error : -error;

    // --- Update rudder speed estimate ---
    if (now - speed_prev_ms >= SPEED_WINDOW_MS) {
      int delta_adc = a - speed_prev_adc;
      unsigned long dt = now - speed_prev_ms;
      // cps = delta_adc * 1000 / dt  (signed, +ve = moving stbd)
      rudder_speed_cps = (int16_t)((long)delta_adc * 1000L / (long)dt);
      speed_prev_adc = a;
      speed_prev_ms  = now;
    }

    // Absolute speed for brake calculations
    int16_t abs_speed = (rudder_speed_cps >= 0) ? rudder_speed_cps : -rudder_speed_cps;

    // --- Compute brake parameters from current speed ---
    // brake_ms = (54 * speed + 50) / 100 + 72   (integer-friendly 0.54*speed + 72)
    unsigned long est_brake_ms = (unsigned long)((54L * abs_speed + 50) / 100) + 72UL;
    // brake distance ≈ speed * brake_ms / 2000 counts
    int est_brake_dist = (int)((long)abs_speed * (long)est_brake_ms / 2000L);
    if (est_brake_dist < 1) est_brake_dist = 1;

    // --- State machine ---
    switch (rm_state) {

      case RM_BRAKING: {
        // Timed reverse pulse in progress — hold until duration expires
        if (now - rm_brake_start_ms >= rm_brake_dur_ms) {
          // Brake pulse complete — cut motor
          analogWrite(HBRIDGE_PWM_PIN, 0);
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          // Check where we ended up
          rm_state = (abs_error <= REMOTE_DEADBAND) ? RM_SETTLED : RM_DRIVING;
        }
        // else: keep brake pulse running (pins already set when entering BRAKING)
        break;
      }

      case RM_SETTLED: {
        // In deadband — stay put unless target moves outside
        if (abs_error > REMOTE_DEADBAND) {
          rm_state = RM_DRIVING;
          // fall through to DRIVING below
        } else {
          // Motor off, hold position via hydraulic lock
          analogWrite(HBRIDGE_PWM_PIN, 0);
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          break;
        }
      }
      // intentional fall-through from SETTLED when error exceeds deadband
      // fall through

      case RM_DRIVING: {
        // Determine drive direction
        int8_t dir = 0;
        if      (error > 0) dir = +1;   // need stbd (increase ADC)
        else if (error < 0) dir = -1;   // need port (decrease ADC)

        // Respect hard and soft limits
        if (dir > 0 && (at_stbd_end || at_stbd_pilot)) dir = 0;
        if (dir < 0 && (at_port_end || at_port_pilot)) dir = 0;

        if (dir == 0) {
          // At limit or exactly on target — stop
          analogWrite(HBRIDGE_PWM_PIN, 0);
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          rm_state = RM_SETTLED;
          break;
        }

        // Check if we should transition to braking.
        // We're moving toward the target; is the distance to target centre
        // within the estimated braking distance?
        // Only brake if speed is meaningful (above minimum threshold) AND
        // we're moving toward the target (not away from it).
        bool moving_toward = (dir > 0 && rudder_speed_cps > 0) ||
                             (dir < 0 && rudder_speed_cps < 0);

        if (moving_toward && abs_speed >= BRAKE_MIN_SPEED_CPS &&
            abs_error <= est_brake_dist) {
          // Initiate reverse brake pulse
          rm_brake_dir      = -dir;  // opposite to travel direction
          rm_brake_dur_ms   = est_brake_ms;
          rm_brake_start_ms = now;
          rm_state          = RM_BRAKING;

          // Set H-bridge to brake direction at full duty
          if (rm_brake_dir > 0) {
            digitalWrite(HBRIDGE_RPWM_PIN, LOW);
            digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
          } else {
            digitalWrite(HBRIDGE_LPWM_PIN, LOW);
            digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
          }
          analogWrite(HBRIDGE_PWM_PIN, 255);
          break;
        }

        // Not yet at brake point — drive toward target at full duty
        if (dir > 0) {
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
        } else {
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
        }
        analogWrite(HBRIDGE_PWM_PIN, 255);
        break;
      }
    }
    return;
  } else {
    // Not in remote-manual mode — reset state machine so next entry starts clean
    rm_state = RM_SETTLED;
  }

  // ---- Manual override branch (AP disengaged) ----
  if (manual_jog_active) {
    // Respect limits
    if (manual_jog_dir < 0 && (at_port_end || at_port_pilot)) {
      // Trying to jog further to port, but at/near port end → stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }
    if (manual_jog_dir > 0 && (at_stbd_end || at_stbd_pilot)) {
      // Trying to jog further to stbd, but at/near stbd end → stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }

    const uint8_t duty = 255;  // full duty for now; you can tune later

    if (manual_jog_dir > 0) {
      // STBD: increase ADC
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
    } else if (manual_jog_dir < 0) {
      // PORT: decrease ADC
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
    } else {
      // No direction -> stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }

    analogWrite(HBRIDGE_PWM_PIN, duty);
    return;  // do not fall through to autopilot logic
  }

  // ---- Existing autopilot logic below (unchanged) ----

  // If AP is not active (remote ap.enabled false or Pi offline), don't drive motor
  if (!ap_active) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // ----- Autopilot motor drive from last_command_val -----
  // last_command_val: 0..2000, 1000 = stop
  // Deadband around neutral
  if (delta > -DEADBAND && delta < DEADBAND) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Don't drive further into soft limits
  if (delta > 0 && (at_stbd_end || at_stbd_pilot)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }
  if (delta < 0 && (at_port_end || at_port_pilot)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Duty mapping (keep MIN_DUTY = MAX_DUTY = 255 for your hydraulic pump: ON/OFF only)
  // If you ever experiment with PWM later, set MIN_DUTY somewhere >= 180 and MAX_DUTY = 255.
  const uint8_t MIN_DUTY = 255;
  const uint8_t MAX_DUTY = 255;

  int16_t abs_delta = (delta >= 0) ? delta : -delta;
  if (abs_delta > 1000) abs_delta = 1000;

  uint8_t duty = MIN_DUTY;
  if (MAX_DUTY == MIN_DUTY) {
    duty = MIN_DUTY;
  } else {
    int16_t span = 1000 - DEADBAND;
    if (span < 1) span = 1;
    int16_t effective = abs_delta - DEADBAND;
    if (effective < 0) effective = 0;

    duty = MIN_DUTY + (uint8_t)((effective * (MAX_DUTY - MIN_DUTY)) / span);
  }

  // Direction: delta > 0 => STBD (increase ADC), delta < 0 => PORT (decrease ADC)
  if (delta > 0) {
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
  } else {
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
  }

  analogWrite(HBRIDGE_PWM_PIN, duty);
}

// Process one CRC-valid frame
void process_packet() {
  flags |= SYNC;

  unsigned long now = millis();

  // Mark Pi controller online on first valid frame
  if (!pi_online) {
    pi_online = true;
    pi_ever_online = true;
    pi_online_time_ms = now;
  }

  // Update last time we heard from the Pi
  last_pi_frame_ms = now;
  
  uint16_t value = in_bytes[1] | (in_bytes[2] << 8);
  uint8_t  code  = in_bytes[0];

  switch (code) {
    case COMMAND_CODE:
      // 0..2000, 1000 = neutral
      last_command_val = value;
      last_command_ms = now;
      // Receiving COMMAND_CODE implies AP is engaged
      if (!ap_enabled_remote) {
        ap_enabled_remote = true;
        flags |= ENGAGED;
        if (ap_display_override_until_ms == 0) {
          ap_display = true;
        } else if (ap_display == true) {
          ap_display_override_until_ms = 0;
        }
      }
      break;

    case DISENGAGE_CODE:
      flags &= ~ENGAGED;
      // DISENGAGE_CODE means AP is off
      if (ap_enabled_remote) {
        ap_enabled_remote = false;
        if (ap_display_override_until_ms == 0) {
          ap_display = false;
        } else if (ap_display == false) {
          ap_display_override_until_ms = 0;
        }
      }
      break;

    case RESET_CODE:
      flags &= ~OVERCURRENT_FAULT;
      break;

    case PILOT_HEADING_CODE:
      pilot_heading_deg10 = value;
      pilot_heading_valid = true;
      break;

    case PILOT_COMMAND_CODE:
      pilot_command_deg10 = value;
      pilot_command_valid = true;
      break;

    case PILOT_RUDDER_CODE:
      pilot_rudder_deg10 = (int16_t)value; // interpret two's complement
      pilot_rudder_valid = true;
      break;

    case PILOT_RUDDER_PORT_LIM_CODE:
      pilot_port_lim_deg10 = (int16_t)value;
      pilot_port_lim_valid = true;
      break;
    
    case PILOT_RUDDER_STBD_LIM_CODE:
      pilot_stbd_lim_deg10 = (int16_t)value;
      pilot_stbd_lim_valid = true;
      break;

    case MANUAL_MODE_CODE:
      if (value != 0 && !remote_manual_active) {
        // Entering MANUAL: initialise speed estimator and brake state machine
        speed_prev_adc    = rudder_adc_smoothed;
        speed_prev_ms     = millis();
        rudder_speed_cps  = 0;
        rm_state          = RM_SETTLED;
      }
      remote_manual_active = (value != 0);
      if (!remote_manual_active) {
        // Exiting MANUAL: clear steer-loss state
        steer_loss_active   = false;
        steer_loss_silenced = false;
      }
      break;

    case MANUAL_RUD_TARGET_CODE:
      manual_rud_target_0_1000 = value;
      break;

    case WARNING_CODE:
      if ((uint8_t)value == WARN_AP_PRESSED) {
        ap_pressed_warn_active = true;
        ap_pressed_warn_ms     = millis();
        ap_warn_beep_end_ms    = millis() + 200UL;  // single 200ms beep
      } else if ((uint8_t)value == WARN_STEER_LOSS) {
        steer_loss_active   = true;
        steer_loss_silenced = false;
      } else if ((uint8_t)value == WARN_NONE) {
        steer_loss_active      = false;
        steer_loss_silenced    = false;
        ap_pressed_warn_active = false;
      }
      break;

    case BRIDGE_HELLO_CODE:
      send_frame(BRIDGE_HELLO_ACK_CODE, 0xBEEF);
      break;

    case BRIDGE_VERSION_CODE:
      bridge_build_num = value;
      bridge_build_valid = true;
      break;

    
    default:
      // Unhandled commands ignored for now
      break;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(RUDDER_PIN, INPUT);  // pot

  // IBT-2 pins
  pinMode(HBRIDGE_RPWM_PIN, OUTPUT);
  pinMode(HBRIDGE_LPWM_PIN, OUTPUT);
  pinMode(HBRIDGE_PWM_PIN, OUTPUT);
  digitalWrite(HBRIDGE_RPWM_PIN, LOW);
  digitalWrite(HBRIDGE_LPWM_PIN, LOW);
  analogWrite(HBRIDGE_PWM_PIN, 0);   // motor off

  // Clutch
  pinMode(CLUTCH_PIN, OUTPUT);
  digitalWrite(CLUTCH_PIN, LOW);    // clutch disengaged (active-HIGH)

  // Limits
  if (LIMIT_SWITCHES_ACTIVE) {
    pinMode(PORT_LIMIT_PIN, INPUT_PULLUP);  // NC -> GND
    pinMode(STBD_LIMIT_PIN, INPUT_PULLUP);
  }

  pinMode(PTM_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(38400);
  
  // Quick check: is Pi/pypilot already talking?
  // Look for any incoming serial for ~200ms
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 200UL) {
      if (Serial.available()) {
        pi_online_at_boot = true;
        break;
      }
    }
  }

  tempSensors.begin();
  tempSensors.setResolution(10);
  tempSensors.setWaitForConversion(false);
  temp_cycle_ms = 0;

  Wire.begin();
  oled_last_init_ms = millis();
  oled_try_init(true);

  // after splash, start boot timer reference
  boot_start_ms = millis();

  // Startup blink so we know this firmware is running
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  unsigned long now = millis();
  last_flags_ms  = now;
  last_rudder_ms = now;
  last_current_ms = now;
  last_voltage_ms = now;
  last_temp_ms = now;
}

void loop() {
  unsigned long now = millis();

  temp_service(now);
  if (!oled_ok && (now - oled_last_init_ms >= OLED_INIT_RETRY_MS)) {
    oled_last_init_ms = now;
    oled_try_init(false);
  }
  // Expire optimistic AP display override if remote didn't confirm in time
  if (ap_display_override_until_ms && now > ap_display_override_until_ms) {
    ap_display_override_until_ms = 0;
    ap_display = ap_enabled_remote;  // revert to truth
  }
  
  static unsigned long last_pi_ms = 0;
  if (now - last_pi_ms >= 200) {
    last_pi_ms = now;
    pi_voltage_v = read_pi_voltage_v();
    pi_overvolt_fault = (pi_voltage_v > PI_VOLT_HIGH_FAULT);
    pi_undervolt_fault = (pi_voltage_v < PI_VOLT_LOW_FAULT);
    pi_fault = pi_overvolt_fault || pi_undervolt_fault;
    vin_v = read_voltage_v();
    vin_low_fault  = (vin_v < VIN_LOW_FAULT_V);
    vin_high_fault = (vin_v > VIN_HIGH_FAULT_V);
    // Reset silence once all faults clear so next occurrence re-triggers alarm
    if (!pi_fault && !(flags & OVERTEMP_FAULT) && !vin_low_fault && !vin_high_fault) {
      pi_fault_alarm_silenced = false;
    }
  }

  static bool ptm_prev = false;
  static bool ptm_stable_pressed = false;
  static bool ptm_last_raw = false;
  static unsigned long ptm_last_change_ms = 0;
  bool ptm_raw_pressed = (digitalRead(PTM_PIN) == LOW);
  if (ptm_raw_pressed != ptm_last_raw) {
    ptm_last_raw = ptm_raw_pressed;
    ptm_last_change_ms = now;
  }
  if (now - ptm_last_change_ms >= PTM_DEBOUNCE_MS) {
    ptm_stable_pressed = ptm_raw_pressed;
  }
  bool ptm_edge = ptm_stable_pressed && !ptm_prev;
  ptm_prev = ptm_stable_pressed;

  if (ptm_edge && (pi_fault || (flags & OVERTEMP_FAULT) || vin_low_fault || vin_high_fault)) {
    pi_fault_alarm_silenced = true;
  }

  if (ptm_edge) {
    // Two-press STOP: first press silences active alarm if any; second (or first if no alarm) is full stop
    bool alarm_was_active = (steer_loss_active && !steer_loss_silenced)
                          || ap_pressed_warn_active
                          || (comms_crit_active && !comms_fault_silenced);

    if (alarm_was_active) {
      // First press: silence alarm only, AP stays engaged
      steer_loss_silenced    = true;
      ap_pressed_warn_active = false;
      comms_fault_silenced   = true;
      show_overlay("SIL");
    } else {
      // Full emergency stop
      flags &= ~ENGAGED;
      last_command_val       = 1000;
      remote_manual_active   = false;
      steer_loss_active      = false;
      steer_loss_silenced    = false;
      ap_pressed_warn_active = false;

      // Make OLED show OFF immediately (remote truth will follow)
      ap_display = false;
      ap_display_override_until_ms = millis() + 2000UL;

      // Tell the bridge/pypilot to disable ap.enabled
      send_button_event(BTN_EVT_STOP);

      // Show big STOP overlay
      show_overlay("STOP");
    }
  }

// ----- Button ladder on A6 (B1..B5) -----
static ButtonID last_stable_button = BTN_NONE;
static ButtonID last_raw_button    = BTN_NONE;
static unsigned long btn_last_change_ms = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 60UL;

int btn_adc = analogRead(BUTTON_ADC_PIN);
ButtonID raw_b = decode_button_from_adc(btn_adc);

if (raw_b != last_raw_button) {
  last_raw_button = raw_b;
  btn_last_change_ms = now;
}

ButtonID stable_b = last_stable_button;
if (now - btn_last_change_ms >= BUTTON_DEBOUNCE_MS) {
  stable_b = raw_b;
}

// Reset manual override by default; we’ll set it again below if needed
manual_override = false;
manual_dir      = 0;

// Treat AP as engaged only when remote ap.enabled is true AND Pi is alive
bool pi_alive = pi_ever_online && (now - last_pi_frame_ms <= PI_OFFLINE_TIMEOUT_MS);
bool ap_engaged = ap_enabled_remote && pi_alive;

// On a change of stable button state, act accordingly
if (stable_b != last_stable_button) {
  if (stable_b != BTN_NONE) {

    // B3 must always work (it is the AP toggle)
    if (stable_b == BTN_B3) {
      handle_button(stable_b);

    // Other buttons only send events when AP is engaged
    } else if (ap_engaged) {
      handle_button(stable_b);
    }
  }

  last_stable_button = stable_b;
}

// If AP is disengaged and not in remote MANUAL mode, use buttons as manual jog
if (!ap_engaged && !remote_manual_active) {
  if (stable_b == BTN_B1 || stable_b == BTN_B2) {
    // Define B1/B2 as starboard jog (positive direction)
    manual_override = true;
    manual_dir      = +1;
  } else if (stable_b == BTN_B4 || stable_b == BTN_B5) {
    // Define B4/B5 as port jog (negative direction)
    manual_override = true;
    manual_dir      = -1;
  }
}

  bool temp_valid = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
  if (temp_valid && temp_c > MAX_CONTROLLER_TEMP_C) {
    flags |= OVERTEMP_FAULT;
  } else {
    flags &= ~OVERTEMP_FAULT;
  }

  // Buzzer logic (priority: steer_loss > hw_fault > comms_fault > ap_warn > silent)
  bool hw_alarm_active   = pi_fault || (flags & OVERTEMP_FAULT) || vin_low_fault || vin_high_fault;
  bool hw_alarm_silenced = pi_fault_alarm_silenced;
  bool buzzer_on = false;

  if (steer_loss_active && !steer_loss_silenced) {
    // Continuous rapid 100ms beeping for steer loss (life-safety)
    static unsigned long sl_buzz_last = 0;
    static bool sl_buzz_on = false;
    if (now - sl_buzz_last >= 100UL) {
      sl_buzz_last = now;
      sl_buzz_on = !sl_buzz_on;
    }
    buzzer_on = sl_buzz_on;
  } else if (hw_alarm_active && !hw_alarm_silenced) {
    // Rapid 100ms beeping for hardware faults
    static unsigned long hw_buzz_last = 0;
    static bool hw_buzz_on = false;
    if (now - hw_buzz_last >= 100UL) {
      hw_buzz_last = now;
      hw_buzz_on = !hw_buzz_on;
    }
    buzzer_on = hw_buzz_on;
  } else if (comms_crit_active && !comms_fault_silenced) {
    // Slow 500ms on/off (1 Hz) for comms fault — distinguishable from rapid 100ms hw alarm
    static unsigned long cf_buzz_last = 0;
    static bool cf_buzz_on = false;
    if (now - cf_buzz_last >= 500UL) {
      cf_buzz_last = now;
      cf_buzz_on = !cf_buzz_on;
    }
    buzzer_on = cf_buzz_on;
  } else if (ap_pressed_warn_active && (now < ap_warn_beep_end_ms)) {
    // Single 200ms beep for AP-pressed warning
    buzzer_on = true;
  }

  digitalWrite(BUZZER_PIN, buzzer_on ? HIGH : LOW);

  // Report buzzer state change to bridge
  if (buzzer_on != last_buzzer_state) {
    last_buzzer_state = buzzer_on;
    send_frame(BUZZER_STATE_CODE, buzzer_on ? 1 : 0);
  }

  // ---- Comms-fault rate evaluation ----
  comms_err_bucket_tick(now);

  if (err_window_sum >= COMMS_CRIT_THRESH && crit_consec_s >= COMMS_CRIT_HOLD_S) {
    comms_warn_active = true;
    comms_crit_active = true;
  } else if (err_window_sum >= COMMS_WARN_THRESH) {
    comms_warn_active = true;
    comms_crit_active = false;
  } else {
    // Rate cleared: reset latches so next fault re-arms fully
    if (comms_warn_active || comms_crit_active) {
      comms_fault_disengaged = false;
      comms_fault_silenced   = false;
    }
    comms_warn_active = false;
    comms_crit_active = false;
  }

  if (comms_warn_active) flags |= COMMS_WARN_FAULT; else flags &= ~COMMS_WARN_FAULT;
  if (comms_crit_active) flags |= COMMS_CRIT_FAULT; else flags &= ~COMMS_CRIT_FAULT;

  // Autonomous AP disengage on CRITICAL (Nano-side safety plane, ~0ms latency)
  if (comms_crit_active && !comms_fault_disengaged) {
    ap_enabled_remote      = false;
    flags                 &= ~ENGAGED;
    last_command_val       = 1000;  // neutral
    ap_display             = false;
    comms_fault_disengaged = true;
  }

  if (pi_fault || vin_low_fault || vin_high_fault) {
    flags |= BADVOLTAGE_FAULT;
  } else {
    flags &= ~BADVOLTAGE_FAULT;
  }

  if ((flags & OVERTEMP_FAULT) || pi_fault || vin_low_fault || vin_high_fault) {
    flags &= ~ENGAGED;
    last_command_val = 1000;
  }

  // --- RX: parse incoming framed packets ---
  while (Serial.available()) {
    uint8_t c = Serial.read();
    any_serial_rx = true;
    last_serial_rx_ms = now;

    if (bridge_magic_state == 0) {
      if (c == BRIDGE_MAGIC1) {
        bridge_magic_state = 1;
      }
      continue;
    } else if (bridge_magic_state == 1) {
      if (c == BRIDGE_MAGIC2) {
        bridge_magic_state = 2;
        sync_b = 0;
      } else {
        bridge_magic_state = 0;
      }
      continue;
    }

    if (sync_b < 3) {
      in_bytes[sync_b++] = c;
    } else {
      // We have three bytes in in_bytes, this is the 4th (CRC)
      uint8_t crc_rx   = c;
      uint8_t crc_calc = crc8(in_bytes, 3);
      if (crc_rx == crc_calc) {
        // CRC-valid frame
        if (in_sync_count >= 2) {
          process_packet();
          rx_good_count++;
        } else {
          in_sync_count++;
          rx_sync_count++;
        }
        flags &= ~INVALID;
      } else {
        // CRC invalid: mark INVALID and resync to magic header
        flags |= INVALID;
        in_sync_count = 0;
        rx_crc_err_count++;
        comms_err_record();  // feed sliding-window rate detector
        // Capture error detail for forwarding to bridge (latest-wins, 1-slot)
        err_detail_value   = (uint16_t)in_bytes[0] | ((uint16_t)crc_rx << 8);
        err_detail_pending = true;
      }

      sync_b = 0;
      bridge_magic_state = 0;

      // Only process one frame per loop() iteration
      break;
    }
  }

  // --- Telemetry: periodic FLAGS and RUDDER ---
  if (now - last_flags_ms >= FLAGS_PERIOD_MS) {
    uint16_t out_flags = flags;
    flags &= ~REBOOTED;           // REBOOTED appears once
    send_frame(FLAGS_CODE, out_flags);
    last_flags_ms = now;
  }

  if (now - last_rudder_ms >= RUDDER_PERIOD_MS) {

    rudder_raw = read_rudder_scaled();
    send_frame(RUDDER_SENSE_CODE, rudder_raw);
    last_rudder_ms = now;
  }

  if (now - last_current_ms >= CURRENT_PERIOD_MS) {
    float current_a = read_current_a();
    int scaled = (int)(current_a * 100.0f + 0.5f);
    if (scaled < 0) {
      scaled = 0;
    } else if (scaled > 65535) {
      scaled = 65535;
    }
    send_frame(CURRENT_CODE, (uint16_t)scaled);
    last_current_ms = now;
  }

  if (now - last_voltage_ms >= VOLTAGE_PERIOD_MS) {
    float voltage_v = read_voltage_v();
    int scaled = (int)(voltage_v * 100.0f + 0.5f);
    if (scaled < 0) {
      scaled = 0;
    } else if (scaled > 65535) {
      scaled = 65535;
    }
    send_frame(VOLTAGE_CODE, (uint16_t)scaled);
    last_voltage_ms = now;
  }

  if (now - last_temp_ms >= TEMP_PERIOD_SEND_MS) {
    bool temp_valid = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
    if (temp_valid) {
      int scaled = (int)(temp_c * 100.0f + 0.5f);
      if (scaled < 0) {
        scaled = 0;
      } else if (scaled > 65535) {
        scaled = 65535;
      }
      send_frame(CONTROLLER_TEMP_CODE, (uint16_t)scaled);
    }
    last_temp_ms = now;
  }

  static unsigned long last_comms_diag_ms = 0;
  if (now - last_comms_diag_ms >= 1000UL) {
    // Pack: low byte = err_window_sum, high byte = crit_consec_s
    uint16_t diag = (uint16_t)err_window_sum | ((uint16_t)crit_consec_s << 8);
    send_frame(COMMS_DIAG_CODE, diag);
    last_comms_diag_ms = now;
  }

  // Send error detail frame to bridge (rate-limited to max 5/s)
  if (err_detail_pending && (now - err_detail_last_ms >= ERR_DETAIL_MIN_MS)) {
    send_frame(COMMS_ERR_DETAIL_CODE, err_detail_value);
    err_detail_pending = false;
    err_detail_last_ms = now;
  }

  // --- Motor + clutch control with limit logic ---
  update_motor_from_command();

  // Collect one settled A2 sample into the running average.  Placed here so
  // update_motor_from_command() uses the previous iteration's published value
  // (latency ≤ one loop iteration) and all other ADC channel work is already
  // complete, guaranteeing a channel switch into A2 every call.
  service_rudder_adc();

  static unsigned long last_draw = 0;
  if (oled_ok && (now - last_draw >= 1000)) {
    last_draw = now;
    oled_draw();
  }
}


















