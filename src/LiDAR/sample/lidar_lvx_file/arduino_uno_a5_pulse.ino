/*
  Arduino Uno pulse trigger for LiDAR frame notifications.

  Protocol:
  - Host sends one line per completed frame: "F\n"
  - On each valid message, Arduino emits a pulse on pin A5.
*/

const uint8_t kPulsePin = A5;          // Arduino Uno: A5 can be used as digital pin 19
const unsigned long kPulseWidthUs = 1000;  // 1 ms pulse width

void setup() {
  pinMode(kPulsePin, OUTPUT);
  digitalWrite(kPulsePin, LOW);

  Serial.begin(115200);
  while (!Serial) {
    ;  // Safe no-op on Uno, required on some USB-native boards.
  }
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "F") {
    digitalWrite(kPulsePin, HIGH);
    delayMicroseconds(kPulseWidthUs);
    digitalWrite(kPulsePin, LOW);
  }
}

