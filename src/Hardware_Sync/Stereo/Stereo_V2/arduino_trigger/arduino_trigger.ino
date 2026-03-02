/*
  Arduino hardware trigger pulse generator for stereo cameras, driven by LiDAR events.

  Protocol (from LiDAR host over USB serial):
  - One completed LiDAR frame is notified with: "F\n"
  - Each valid "F" line emits one pulse on A5.
*/

const uint8_t TRIGGER_PIN = A5;             // Uno A5 can be used as digital output.
const unsigned int PULSE_WIDTH_US = 100;    // 100 us trigger pulse width.

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) {
    ;  // Required on some boards; harmless on Uno.
  }
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "F") {
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(TRIGGER_PIN, LOW);
  }
}
