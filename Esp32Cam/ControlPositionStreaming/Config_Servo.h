// ===========================
// Settings Servos
// ===========================

//#define DUMMY_SERVO1_PIN 14 // We need to create 2 dummy servos.
//#define DUMMY_SERVO2_PIN 15 // So that ESP32Servo library does not interfere with pwm channel and timer used by esp32 camera.
//
//Servo dummyServo1;
//Servo dummyServo2;



Servo servo_h; // Controls horizontal camera movement
Servo servo_v; // Controls vertical camera movement

void setUpPinModes() {
  servo_h.attach(SERVO_H);
  servo_v.attach(SERVO_V);

}
