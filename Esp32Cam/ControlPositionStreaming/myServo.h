// ===========================
// Settings Servos
// ===========================

#ifndef CONFIG_SERVO_H
#define CONFIG_SERVO_H

#include <ESP32Servo.h>

#define SERVO_H 12 // Servo for horizontal control
#define SERVO_V 13 // Vertical control servo

extern Servo horizontal_servo; // Controls horizontal camera movement
extern Servo vertical_servo;   // Controls vertical camera movement

void configure_initial_servo_positions();

#endif // CONFIG_SERVO_H
