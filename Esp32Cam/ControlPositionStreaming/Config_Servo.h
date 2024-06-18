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

int pos_h = 90;    // variable to store the servo position
int pos_v = 90;    // variable to store the servo position

void InitialServoConfiguration() {
  servo_h.setPeriodHertz(50);    // standard 50 hz servo
  servo_h.attach(SERVO_H, 1000, 2000);

  servo_v.setPeriodHertz(50);    // standard 50 hz servo
  servo_v.attach(SERVO_V, 1000, 2000);

  servo_h.write(pos_h);
  servo_v.write(pos_v);

}
