// echo_upper.ino — loopback test sketch
// Receives a byte, sends back the uppercase version.
// Used to verify Pi <-> Nano serial TX/RX in both directions.

#define BAUD 38400

void setup() {
    Serial.begin(BAUD);
}

void loop() {
    if (Serial.available()) {
        uint8_t c = Serial.read();
        if (c >= 'a' && c <= 'z') {
            c -= 32;  // ASCII uppercase
        }
        Serial.write(c);
    }
}
