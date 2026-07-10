#include <ESP32Servo.h>

Servo esc;
int escPin = 13;   // <-- Put the pin of the NEW ESC motor here

void setup() {
  esc.attach(escPin, 1000, 2000);
  
  delay(2000);

  // Send MAX throttle
  esc.writeMicroseconds(2000);
  delay(5000);

  // Send MIN throttle
  esc.writeMicroseconds(1000);
  delay(5000);
}

void loop() {}
