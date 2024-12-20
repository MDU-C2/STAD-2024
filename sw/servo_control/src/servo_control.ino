#include "DynamixelSerial.h"
#include "PWM.hpp"

PWM servo1(2);
PWM servo2(3);

void setup() {
  Dynamixel.begin(1000000);
  servo1.begin(true);
  servo2.begin(true);
}

void loop() {
  Dynamixel.move(1, servo1.getValue());
  Dynamixel.move(2, servo2.getValue());
  // delay(50);
}