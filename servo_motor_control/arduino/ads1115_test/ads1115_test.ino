// ads1115_test.ino
// Exploratory sketch: probe ADS1115 on I2C, read AIN0 (rudder wiper pot),
// display raw counts and millivolts on SSD1306 OLED, echo to Serial.
//
// Wiring:
//   ADS1115 VDD  → 5V          ADS1115 GND  → GND
//   ADS1115 ADDR → GND         (forces I2C addr 0x48)
//   ADS1115 SCL  → Nano A5     ADS1115 SDA  → Nano A4
//   ADS1115 AIN0 → rudder pot wiper
//   OLED SDA → A4, SCL → A5   (shared I2C bus, addr 0x3C)
//
// Serial output format (38400 baud):
//   RAW:<int> MV:<int> N:<ulong>
//
// Compile: arduino-cli compile --fqbn arduino:avr:nano \
//            --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" .
// Upload:  arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .
// (Stop bridge/socat/pypilot services first.)

#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t ADS_ADDR  = 0x48;

// ADS1115 config: start single-shot, AIN0 vs GND, PGA ±6.144 V, 128 SPS,
// comparator disabled.
// Bit layout (MSB first):
//   OS=1  MUX=100(AIN0/GND)  PGA=000(±6.144V)  MODE=1(single)
//   DR=100(128SPS)  COMP_MODE=0  COMP_POL=0  COMP_LAT=0  COMP_QUE=11(disable)
// → MSB=0xC1  LSB=0x83
static const uint16_t ADS_CFG_SINGLE_AIN0 = 0xC183;

SSD1306AsciiWire oled;
static bool oled_found = false;
static bool ads_found  = false;

// ── I2C helpers ──────────────────────────────────────────────────────────────

// Returns true if a device ACKs at addr.
static bool i2c_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Write a 16-bit value to an ADS1115 register.
static bool ads_write_reg(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

// Set register pointer then read two bytes back.
static int16_t ads_read_reg(uint8_t reg) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0;
  Wire.requestFrom((uint8_t)ADS_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

// Kick off a single-shot conversion on AIN0.
static void ads_start_conversion() {
  ads_write_reg(0x01, ADS_CFG_SINGLE_AIN0);
}

// Return true when the OS bit in the config register indicates done.
static bool ads_conversion_ready() {
  return (ads_read_reg(0x01) & 0x8000) != 0;
}

// Read the conversion register (call only after ads_conversion_ready()).
static int16_t ads_read_raw() {
  return ads_read_reg(0x00);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

// Convert raw ADS1115 count to millivolts (integer, rounds down).
// With ±6.144 V FSR: 1 LSB = 6144 mV / 32768 = 0.1875 mV
// → mV = raw × 6144 / 32768  (use int32 to avoid overflow)
static int16_t raw_to_mv(int16_t raw) {
  return (int16_t)((int32_t)raw * 6144L / 32768L);
}

// ── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Wire.begin();
  Serial.begin(38400);

  // Probe both devices before touching the display.
  oled_found = i2c_probe(OLED_ADDR);
  ads_found  = i2c_probe(ADS_ADDR);

  Serial.print(F("OLED 0x3C: ")); Serial.println(oled_found ? F("FOUND") : F("MISS"));
  Serial.print(F("ADS  0x48: ")); Serial.println(ads_found  ? F("FOUND") : F("MISS"));

  if (oled_found) {
    oled.begin(&Adafruit128x64, OLED_ADDR);
    oled.setFont(System5x7);
    oled.clear();
    oled.println(F("ADS1115 Test"));
    oled.print(F("ADS  0x48: "));
    oled.println(ads_found ? F("FOUND") : F("MISS"));
    delay(1500);
    oled.clear();
    oled.println(F("ADS1115 Test"));
    oled.println();  // blank row before live values
  }
}

// ── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  static uint32_t n = 0;
  char buf[22];

  if (!ads_found) {
    // Retry probe — device may have been absent at boot.
    ads_found = i2c_probe(ADS_ADDR);
    if (!ads_found) {
      Serial.println(F("ADS1115 not found — check wiring"));
      if (oled_found) {
        oled.setCursor(0, 2);
        oled.println(F("ADS not found!"));
        oled.println(F("Check wiring."));
      }
      delay(1000);
      return;
    }
    Serial.println(F("ADS1115 found (late)"));
  }

  // Trigger conversion and wait (128 SPS → ~7.8 ms; 15 ms gives headroom).
  ads_start_conversion();
  delay(15);

  // Poll the OS/ready bit (up to 10 extra ms before giving up).
  uint8_t tries = 0;
  while (!ads_conversion_ready() && tries < 10) {
    delay(1);
    tries++;
  }

  int16_t raw = ads_read_raw();
  int16_t mv  = raw_to_mv(raw);
  n++;

  // Serial log — easy to parse from Pi.
  Serial.print(F("RAW:")); Serial.print(raw);
  Serial.print(F(" MV:"));  Serial.print(mv);
  Serial.print(F(" N:"));   Serial.println(n);

  // OLED update — three live rows.
  if (oled_found) {
    oled.setCursor(0, 2);
    snprintf(buf, sizeof(buf), "Raw: %-6d", (int)raw);
    oled.println(buf);

    snprintf(buf, sizeof(buf), "mV:  %-5d", (int)mv);
    oled.println(buf);

    snprintf(buf, sizeof(buf), "N:   %-6lu", n);
    oled.println(buf);
  }

  delay(200);  // ~5 Hz refresh
}
