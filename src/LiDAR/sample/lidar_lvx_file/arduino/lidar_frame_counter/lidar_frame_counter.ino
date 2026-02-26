/*
  LiDAR frame counter for Arduino.

  Protocol:
  - Host sends one line per completed frame: "F\n"
  - Arduino increments counter on each 'F' line and prints the count.
*/

unsigned long frame_counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for USB serial on boards that require it.
  }
  Serial.println("Arduino frame counter ready");
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "F") {
    frame_counter++;
    Serial.print("Frame count: ");
    Serial.println(frame_counter);
  }
}

