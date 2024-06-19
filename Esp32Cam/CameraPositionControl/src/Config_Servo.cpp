#include <Arduino.h>

#include "Config_Servo.h"

Servo horizontal_servo; // Controls horizontal camera movement
Servo vertical_servo;   // Controls vertical camera movement

int horizontal_position = 90; // Variable to store the horizontal servo position
int vertical_position = 90;   // Variable to store the vertical servo position

void configure_initial_servo_positions() {
  horizontal_servo.setPeriodHertz(50); // Standard 50 Hz servo
  horizontal_servo.attach(SERVO_H, 1000, 2000);

  vertical_servo.setPeriodHertz(50); // Standard 50 Hz servo
  vertical_servo.attach(SERVO_V, 1000, 2000);

  horizontal_servo.write(horizontal_position);
  vertical_servo.write(vertical_position);
}
