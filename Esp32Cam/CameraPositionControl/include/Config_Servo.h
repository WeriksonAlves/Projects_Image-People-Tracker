#ifndef CONFIG_SERVO_H
#define CONFIG_SERVO_H

#include <Servo.h>

// Define servo pins
#define SERVO_H 13 // Example horizontal servo pin
#define SERVO_V 12 // Example vertical servo pin

extern Servo horizontal_servo; // Controls horizontal camera movement
extern Servo vertical_servo;   // Controls vertical camera movement

extern int horizontal_position; // Variable to store the horizontal servo position
extern int vertical_position;   // Variable to store the vertical servo position

void configure_initial_servo_positions();

#endif // CONFIG_SERVO_H
