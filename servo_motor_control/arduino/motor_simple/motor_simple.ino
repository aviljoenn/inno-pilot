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
const char INNOPILOT_VERSION[] = "v0.2.0_B10";
const uint16_t INNOPILOT_BUILD_NUM = 10;  // increment with each push during development

// Boot / online timing (user-tweakable)
const uint8_t AP_ENABLED_CODE = 0xE1;  // Bridge->Nano: ap.enabled state (0/1)
bool ap_enabled_remote = false;        // truth from Pi bridge
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

// ---- State / telemetry ----
uint16_t flags            = REBOOTED;   // reported once then cleared
uint16_t last_command_val = 1000;       // 0..2000, 1000 = neutral
unsigned long last_command_ms = 0;
uint16_t rudder_raw       = 0;          // 0..65535 (scaled)
int      rudder_adc_last  = 0;          // 0..1023

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

  // Rudder angle (rough local mapping for now)
  float rudder_deg = 0.0f;
  {
    float centre   = 0.5f * (RUDDER_ADC_PORT_END + RUDDER_ADC_STBD_END);
    float halfSpan = 0.5f * (RUDDER_ADC_STBD_END - RUDDER_ADC_PORT_END);
    if (halfSpan < 1.0f) halfSpan = 1.0f;
    rudder_deg = (rudder_adc_last - centre) * (RUDDER_RANGE_DEG / halfSpan);
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
  // Draw
  // ----------------------------
  display.setFont(System5x7);
  display.set1X();

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

  // --- Row 1: Pi online/offline + bridge version ---
  display.setCursor(0, 1);
  if (!within_boot_window) {
    bool pi_alive_now = pi_ever_online && !pi_timed_out;
    if (pi_alive_now) {
      if (bridge_build_valid) {
        char buf[22];
        snprintf(buf, sizeof(buf), "Pi: Online v0.2.0_B%u", bridge_build_num);
        display.print(buf);
      } else {
        display.print(F("Pi: Online"));
      }
    } else {
      display.print(F("Pi: Offline"));
    }
  }
  display.clearToEOL();

  // --- Rows 2-3: fault/warning display (priority: steer_loss > hw_fault > ap_warn > overlay) ---
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
        display.setCursor(x, 2);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 2); display.clearToEOL();
        display.setCursor(0, 3); display.clearToEOL();
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
        display.setCursor(x, 2);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 2); display.clearToEOL();
        display.setCursor(0, 3); display.clearToEOL();
      }
    } else if (ap_pressed_warn_active) {
      // AP-pressed warning: 1X "?AP Pressed?" (no flashing)
      display.setCursor(0, 2);
      display.print(F("?AP Pressed?"));
      display.clearToEOL();
      display.setCursor(0, 3); display.clearToEOL();
    } else if (overlay_active && (now - overlay_start_ms < OVERLAY_DURATION_MS)) {
      // Overlay (big transient button feedback) on rows 2-3
      display.set2X();
      uint8_t len = strlen(overlay_text);
      int16_t x = (SCREEN_WIDTH - (int16_t)len * 12) / 2;
      if (x < 0) x = 0;
      display.setCursor(x, 2);
      display.print(overlay_text);
      display.clearToEOL();
      display.set1X();
    } else {
      if (!overlay_active || (now - overlay_start_ms >= OVERLAY_DURATION_MS)) {
        overlay_active = false;
        display.setCursor(0, 2); display.clearToEOL();
        // Row 3: Rx diagnostics (Rx=good frames, Er=CRC errors, Age=seconds since last frame)
        display.setCursor(0, 3);
        {
          char dbuf[22];
          unsigned long age_ms = pi_ever_online ? (now - last_pi_frame_ms) : (now - boot_start_ms);
          uint8_t age_s = (age_ms < 25500UL) ? (uint8_t)(age_ms / 100) : 255;
          // age_s is tenths of seconds (0-255 => 0.0-25.5s)
          snprintf(dbuf, sizeof(dbuf), "Rx:%u Er:%u A:%u.%u",
                   rx_good_count, rx_crc_err_count,
                   age_s / 10, age_s % 10);
          display.print(dbuf);
        }
        display.clearToEOL();
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

  // Offline: show diagnostics in middle rows and return
  if (boot_offline || pi_timed_out) {
    // Row 4: Rx diagnostics (same as row 3 when online, but more visible here)
    display.setCursor(0, 4);
    {
      char dbuf[22];
      unsigned long age_ms = pi_ever_online ? (now - last_pi_frame_ms) : (now - boot_start_ms);
      uint8_t age_s = (age_ms < 25500UL) ? (uint8_t)(age_ms / 100) : 255;
      snprintf(dbuf, sizeof(dbuf), "Rx:%u Er:%u A:%u.%u",
               rx_good_count, rx_crc_err_count,
               age_s / 10, age_s % 10);
      display.print(dbuf);
    }
    display.clearToEOL();
    display.setCursor(0, 5); display.clearToEOL();
    display.setCursor(0, 6); display.clearToEOL();
    return;
  }

  // --- Online-only rows ---

  // Row 4: AP + clutch (or MAN: ON when remote manual mode active)
  display.setCursor(0, 4);
  if (remote_manual_active) {
    display.print(F("MAN: ON  Clutch: "));
    display.print(digitalRead(CLUTCH_PIN) == HIGH ? F("ON") : F("OFF"));
  } else {
    display.print(F("AP: "));
    display.print(ap_display ? F("ON") : F("OFF"));
    display.print(F("  Clutch: "));
    display.print(digitalRead(CLUTCH_PIN) == HIGH ? F("ON") : F("OFF"));
  }
  display.clearToEOL();

  // Row 5: Cmd / Heading
  display.setCursor(0, 5);
  display.print(F("Cmd: "));
  if (pilot_command_valid) display.print((pilot_command_deg10 + 5) / 10);
  else display.print(F("--.-"));
  display.print(F(" Head: "));
  if (pilot_heading_valid) display.print((pilot_heading_deg10 + 5) / 10);
  else display.print(F("--.-"));
  display.clearToEOL();

  // Row 6: Rudder (port limit / current / stbd limit)
  display.setCursor(0, 6);
  display.print(F("Rud:"));
  if (pilot_port_lim_valid) display.print(pilot_port_lim_deg10 / 10.0f, 1);
  else display.print(F("--.-"));
  display.print(F(" "));
  if (pilot_rudder_valid) display.print(pilot_rudder_deg10 / 10.0f, 1);
  else display.print(F("--.-"));
  display.print(F(" "));
  if (pilot_stbd_lim_valid) display.print(pilot_stbd_lim_deg10 / 10.0f, 1);
  else display.print(F("--.-"));
  display.clearToEOL();
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

  // Only block for the full splash if the Pi doesn't appear online yet
  if (allow_blocking_splash && !pi_online_at_boot) {
    delay(3000);   // Pi booting: OK to block
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
  // Invert ADC so higher counts always mean starboard movement.
  int raw = analogRead(RUDDER_PIN);   // 0..1023 (hardware polarity)
  return 1023 - raw;
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

  // ---- Remote MANUAL mode: bang-bang to ADC target from remote pot ----
  if (remote_manual_active) {
    // Map 0..1000 target to port-end..stbd-end ADC range
    int target_adc = RUDDER_ADC_PORT_END +
      (int)((long)(RUDDER_ADC_STBD_END - RUDDER_ADC_PORT_END) * manual_rud_target_0_1000 / 1000);
    const int REMOTE_DEADBAND = 15;  // ADC counts

    int8_t dir = 0;
    if      (a < target_adc - REMOTE_DEADBAND) dir = +1;   // need stbd
    else if (a > target_adc + REMOTE_DEADBAND) dir = -1;   // need port

    if (dir > 0 && (at_stbd_end || at_stbd_pilot)) dir = 0;
    if (dir < 0 && (at_port_end || at_port_pilot)) dir = 0;

    if (dir > 0) {
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
      analogWrite(HBRIDGE_PWM_PIN, 255);
    } else if (dir < 0) {
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
      analogWrite(HBRIDGE_PWM_PIN, 255);
    } else {
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    }
    return;
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
      break;

    case DISENGAGE_CODE:
      flags &= ~ENGAGED;
      break;

    case RESET_CODE:
      flags &= ~OVERCURRENT_FAULT;
      break;

    case AP_ENABLED_CODE: {
      bool en = (value != 0);
      ap_enabled_remote = en;
      if (en) {
        flags |= ENGAGED;
      } else {
        flags &= ~ENGAGED;
      }
      
      // If we are not in an override window, follow remote immediately.
      // If we ARE overriding (user just pressed B3), cancel override once remote matches.
      if (ap_display_override_until_ms == 0) {
        ap_display = en;
      } else if (ap_display == en) {
        ap_display_override_until_ms = 0;
      }
      break;
    }
    
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
    bool alarm_was_active = (steer_loss_active && !steer_loss_silenced) || ap_pressed_warn_active;

    if (alarm_was_active) {
      // First press: silence alarm only, AP stays engaged
      steer_loss_silenced    = true;
      ap_pressed_warn_active = false;
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

  // Buzzer logic (priority: steer_loss > hw_fault > ap_warn > silent)
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

  // --- Motor + clutch control with limit logic ---
  update_motor_from_command();

  static unsigned long last_draw = 0;
  if (oled_ok && (now - last_draw >= 200)) {
    last_draw = now;
    oled_draw();
  }
}


















