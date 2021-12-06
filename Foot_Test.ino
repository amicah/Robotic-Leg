#include <Servo.h>
#define FOOT_PIN 15
#define LINEAR_MIN 1050
#define LINEAR_MAX 2000


Servo LINEAR;
bool swt = true;

void setup() {
  // put your setup code here, to run once:
  LINEAR.attach(FOOT_PIN, LINEAR_MIN, LINEAR_MAX);
  LINEAR.writeMicroseconds(LINEAR_MIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
  if (swt) {
    LINEAR.writeMicroseconds(LINEAR_MAX);
    swt = false;
  } else {
    LINEAR.writeMicroseconds(LINEAR_MIN);
    swt = true;
  }
}
