#define SSD1306_NO_SPLASH

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==============================
// Hardware pinout (matches IBT-2 Nano wiring)
// ==============================
const uint8_t PIN_RPWM   = 2;   // D2 -> RPWM (ON/OFF)
const uint8_t PIN_LPWM   = 3;   // D3 -> LPWM (ON/OFF)
const uint8_t PIN_EN     = 9;   // D9 -> R_EN + L_EN bridged
const uint8_t PIN_CLUTCH = 11;  // D11 -> clutch driver input

// Rudder position sensor
const uint8_t PIN_RUDDER = A2;  // potentiometer feedback input

// ==============================
// OLED
// ==============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oled_ok = false;

// ==============================
// Control constants
// ==============================
const uint16_t RUDDER_MIN_LIMIT = 0;
const uint16_t RUDDER_MAX_LIMIT = 1023;
const uint8_t ADC_SAMPLES = 8;
const uint8_t CHANGE_THRESHOLD = 1;     // minimum delta to consider movement
const uint16_t CENTER_TOLERANCE = 2;    // stop within +/- this value
const unsigned long STALL_TIMEOUT_MS = 700;
const unsigned long DISPLAY_PERIOD_MS = 200;
const unsigned long TELEMETRY_PERIOD_MS = 250;

// ==============================
// State
// ==============================
enum MotionState : uint8_t { STOPPED = 0, MOVING_NEG = 1, MOVING_POS = 2 };
enum RunState : uint8_t { FIND_MIN = 0, FIND_MAX = 1, CENTERING = 2, DONE = 3, ERROR = 4 };

MotionState motion = STOPPED;
RunState run_state = FIND_MIN;

uint16_t last_reading = 0;
unsigned long last_change_ms = 0;
bool last_reading_valid = false;

uint16_t min_reading = 1023;
uint16_t max_reading = 0;
uint16_t center_target = 512;

unsigned long last_display_ms = 0;
unsigned long last_telemetry_ms = 0;

String last_event = "";

// ==============================
// Motor and clutch helpers
// ==============================
void clutch_on() {
  digitalWrite(PIN_CLUTCH, HIGH);
}

void clutch_off() {
  digitalWrite(PIN_CLUTCH, LOW);
}

void motor_stop() {
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  motion = STOPPED;
}

void motor_move_negative() {
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_RPWM, HIGH);
  digitalWrite(PIN_EN, HIGH);
  motion = MOVING_NEG;
}

void motor_move_positive() {
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, HIGH);
  digitalWrite(PIN_EN, HIGH);
  motion = MOVING_POS;
}

// ==============================
// ADC helpers
// ==============================
uint16_t read_rudder() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += (uint16_t)analogRead(PIN_RUDDER);
    delayMicroseconds(200);
  }
  return (uint16_t)(sum / ADC_SAMPLES);
}

// ==============================
// Display and telemetry
// ==============================
const __FlashStringHelper *motion_str(MotionState m) {
  switch (m) {
    case MOVING_NEG: return F("NEG");
    case MOVING_POS: return F("POS");
    default: return F("STOP");
  }
}

const __FlashStringHelper *state_str(RunState s) {
  switch (s) {
    case FIND_MIN:  return F("FIND MIN");
    case FIND_MAX:  return F("FIND MAX");
    case CENTERING: return F("CENTER");
    case DONE:      return F("DONE");
    default:        return F("ERROR");
  }
}

void update_display(uint16_t reading) {
  if (!oled_ok) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("State: "));
  display.print(state_str(run_state));

  display.setCursor(0, 10);
  display.print(F("Motion: "));
  display.print(motion_str(motion));

  display.setCursor(0, 20);
  display.print(F("Rudder: "));
  display.print(reading);

  display.setCursor(0, 30);
  display.print(F("Min: "));
  display.print(min_reading);
  display.print(F(" Max: "));
  display.print(max_reading);

  display.setCursor(0, 40);
  display.print(F("Center: "));
  display.print(center_target);

  display.setCursor(0, 50);
  display.print(F("Event: "));
  display.print(last_event);

  display.display();
}

void telemetry(uint16_t reading) {
  Serial.print(F("state="));
  Serial.print(state_str(run_state));
  Serial.print(F(" motion="));
  Serial.print(motion_str(motion));
  Serial.print(F(" rudder="));
  Serial.print(reading);
  Serial.print(F(" min="));
  Serial.print(min_reading);
  Serial.print(F(" max="));
  Serial.print(max_reading);
  Serial.print(F(" center="));
  Serial.print(center_target);
  if (last_event.length()) {
    Serial.print(F(" event="));
    Serial.print(last_event);
  }
  Serial.println();
}

void setup() {
  Serial.begin(38400);

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_CLUTCH, OUTPUT);
  pinMode(PIN_RUDDER, INPUT);

  motor_stop();
  clutch_on();

  oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oled_ok) {
    display.clearDisplay();
    display.display();
  }

  last_event = "BOOT";
  uint16_t reading = read_rudder();
  last_reading = reading;
  last_reading_valid = true;
  last_change_ms = millis();

  motor_move_negative();
  run_state = FIND_MIN;
}

void loop() {
  unsigned long now = millis();
  uint16_t reading = read_rudder();

  while (Serial.available() > 0) {
    Serial.read();
  }

  if (last_reading_valid) {
    if ((uint16_t)abs((int)reading - (int)last_reading) >= CHANGE_THRESHOLD) {
      last_change_ms = now;
      last_reading = reading;
    }
  } else {
    last_reading = reading;
    last_reading_valid = true;
    last_change_ms = now;
  }

  bool at_limit_value = (reading == RUDDER_MIN_LIMIT || reading == RUDDER_MAX_LIMIT);
  bool stalled = (motion != STOPPED) && (now - last_change_ms > STALL_TIMEOUT_MS);

  if (motion != STOPPED && (at_limit_value || stalled)) {
    motor_stop();
    last_event = at_limit_value ? "LIMIT" : "STALL";

    if (run_state == FIND_MIN) {
      min_reading = reading;
      run_state = FIND_MAX;
      motor_move_positive();
      last_change_ms = now;
      last_reading = reading;
    } else if (run_state == FIND_MAX) {
      max_reading = reading;
      center_target = (uint16_t)((min_reading + max_reading) / 2);
      run_state = CENTERING;
      if (reading > center_target) {
        motor_move_negative();
      } else if (reading < center_target) {
        motor_move_positive();
      } else {
        run_state = DONE;
        clutch_off();
      }
      last_change_ms = now;
      last_reading = reading;
    } else if (run_state == CENTERING) {
      run_state = ERROR;
      clutch_off();
    }
  }

  if (run_state == CENTERING && motion != STOPPED) {
    if ((reading >= center_target - CENTER_TOLERANCE) &&
        (reading <= center_target + CENTER_TOLERANCE)) {
      motor_stop();
      run_state = DONE;
      last_event = "CENTERED";
      clutch_off();
    }
  }

  if ((motion == STOPPED) && (run_state == CENTERING)) {
    if (reading > center_target + CENTER_TOLERANCE) {
      motor_move_negative();
    } else if (reading < center_target - CENTER_TOLERANCE) {
      motor_move_positive();
    }
    last_change_ms = now;
    last_reading = reading;
  }

  if (run_state == FIND_MIN) {
    if (reading < min_reading) {
      min_reading = reading;
    }
  } else if (run_state == FIND_MAX) {
    if (reading > max_reading) {
      max_reading = reading;
    }
  }

  if (oled_ok && (now - last_display_ms >= DISPLAY_PERIOD_MS)) {
    update_display(reading);
    last_display_ms = now;
  }

  if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    telemetry(reading);
    last_telemetry_ms = now;
  }
}
