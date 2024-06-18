#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_camera.h> //<esp32cam.h>
#include <ArduinoOTA.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ESP32Servo.h>
#include <LedRGB.h>
#include "Config_OTA.h"
#include "Config_WiFi.h"
#include "Config_CameraPins.h"
#include "Config_Camera.h"
#include "Config_Servo.h"
#include "Config_ROS.h" // Include ROS configuration

// Define hardware pins and constants
#define SERVO_H 12 // Servo for horizontal control
#define SERVO_V 13 // Vertical control servo
#define FlashLedPin 04 // GPIO04
#define RedPin 15   // GPIO12
#define GreenPin 14 // GPIO13
#define BluePin 02  // GPIO14

// Define other global variables and objects
unsigned int frameCount = 0;
unsigned long lastMillis = 0;
float fps = 0;
const int time_delay = 500;
LedRGB LedRGB(RedPin, GreenPin, BluePin, 2, 3, 4, false); // Creating LedRGB object

void setup() {
  // Initializing Serial Communication
  Serial.begin(115200);
  Serial.println("Booting");

  // Progress indicators
  pinMode(FlashLedPin, OUTPUT);
  digitalWrite(FlashLedPin, LOW);
  LedRGB.Off();

  // Initialize the camera
  Serial.print("Initializing the camera module...");
  configInitCamera();
  Serial.println("Ok!");
  LedRGB.Red();
  delay(time_delay);

  // Connect to Wi-Fi
  connectWIFI();
  LedRGB.Green();
  delay(time_delay);

  // Initialize OTA
  OTA();
  LedRGB.Blue();
  delay(time_delay);

  // Start camera server
  startCameraServer();
  LedRGB.Red();
  digitalWrite(FlashLedPin, HIGH);
  delay(time_delay);

  // Initialize ROS
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_hor_rot);
  nh.subscribe(sub_ver_rot);
  LedRGB.Green();
  delay(time_delay);

  // Set up servos
  configure_initial_servo_positions();
  LedRGB.Blue();
  delay(time_delay);

  // Finish setup
  LedRGB.Off();
  digitalWrite(FlashLedPin, LOW);
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Handle ROS communication
  nh.spinOnce();

  // Your main loop code here

}

// Calculate FPS function
void calculateFPS() {
  frameCount++;
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastMillis = currentMillis;
  }
}
