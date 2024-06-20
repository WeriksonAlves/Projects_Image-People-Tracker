#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_camera.h>
#include <ArduinoOTA.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ESP32Servo.h>
#include <LedRGB.h>

// Configs Define
#define StaticIP // Static IP address
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define SERVO_H 12 // Servo for horizontal control
#define SERVO_V 13 // Vertical control servo
#define FlashLedPin 4 // GPIO04
#define RedPin 15 // GPIO15
#define GreenPin 14 // GPIO14
#define BluePin 2 // GPIO2

// Including Libraries: Configs
#include "Config_OTA.h"
#include "Config_WiFi.h"
#include "Config_CameraPins.h"
#include "Config_Camera.h"
#include "Config_Servo.h"
#include "Config_ROS.h"

// Outhers Configs

// Variables for function 'calculateFPS'
unsigned int frameCount = 0;
unsigned long lastMillis = 0;
float fps = 0;
const int time_delay = 500;

void startCameraServer();
void setupLedFlash(int pin);

LedRGB Led_RGB(RedPin,GreenPin,BluePin, 2,3,4,false); // Creating LedRGB object

void setup() {
    // Initializing Serial Communication
    Serial.begin(115200);

    // Progress indicators
    pinMode(FlashLedPin, OUTPUT);
    digitalWrite(FlashLedPin, LOW);
    Led_RGB.Off();

    // Initialize the camera
    Serial.print("Initializing the camera module...");
    configInitCamera();
    Serial.println("Ok!");
    Led_RGB.Red();
    delay(time_delay);

    // Connect to WiFi
    connectWIFI();
    Led_RGB.Green();
    delay(time_delay);


    // Initializing OTA
    OTA();
    Led_RGB.Blue();
    delay(time_delay);

    // Start Camera Server
    startCameraServer();
    Led_RGB.Red();
    digitalWrite(FlashLedPin, HIGH);
    delay(time_delay);

    // Initialize ROS
    setupROS();
    Led_RGB.Green();
    delay(time_delay);

    // Set up servos
    configure_initial_servo_positions();
    Led_RGB.Blue();
    delay(time_delay);

    Led_RGB.Off();
    digitalWrite(FlashLedPin, LOW);
}

void loop() {
    // Calling function to update OTA
    ArduinoOTA.handle();

    nh.spinOnce();
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
