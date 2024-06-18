// ===========================
// Including Libraries: Default

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_camera.h> //<esp32cam.h>
#include <ArduinoOTA.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ESP32Servo.h>
#include <LedRGB.h>
// ===========================


// ===========================
//Configs Define

#define StaticIP // Static IP adress

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#define SERVO_H 12 // Servo for horizontal control
#define SERVO_V 13 // Vertical control servo

#define FlashLedPin 04 // GPIO04
#define RedPin 15   // GPIO12
#define GreenPin 14 // GPIO13
#define BluePin 02  // GPIO14
// ===========================


// ===========================
// Including Libraries: Configs

#include "Config_OTA.h"
#include "Config_WiFi.h"
#include "Config_CameraPins.h"
#include "Config_Camera.h"
#include "Config_Servo.h"
#include "Config_ROS.h"
// ===========================



// ===========================
// Outhers Configs

// Variables for function 'calculefps'
unsigned int frameCount = 0;
unsigned long lastMillis = 0;
float fps = 0;

const int time_delay = 500;

void startCameraServer();
void setupLedFlash(int pin);

void messageCb();

LedRGB LedRGB(RedPin,GreenPin,BluePin, 2,3,4,false); // Creating LedRGB object
// ===========================



void setup() {
  // ===========================
  // Initializing Serial Communication
  Serial.begin(115200);
  // ===========================

  // ===========================
  // Progress indicators
  pinMode(FlashLedPin, OUTPUT);
  digitalWrite(FlashLedPin, LOW);
  LedRGB.Off();
  // ===========================
  
  
  
  // ===========================
  //Initialize the camera  
  Serial.print("Initializing the camera module...");
  configInitCamera();
  Serial.println("Ok!");

  LedRGB.Red();
  delay(time_delay);
  // ===========================



  // ===========================
  // Configuring static IP address
  #ifdef StaticIP
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
    }
  #endif

  // Connecting to access point
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  
  // Changing Wi-fi Channel
  Serial.print("Default WiFi-Channel: ");
  Serial.println(WiFi.channel()); 
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);   
  Serial.print("Updated WiFi-Channel: ");
  Serial.println(WiFi.channel());

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  LedRGB.Green();
  delay(time_delay);
  // ===========================
  
  
  
  // ===========================
  // Initializing OTA
  OTA();

  LedRGB.Blue();
  delay(time_delay);
  // ===========================
  
  
  
  // ===========================
  startCameraServer();

  LedRGB.Red();
  digitalWrite(FlashLedPin, HIGH);
  delay(time_delay);
  // ===========================
  
  
  
  // ===========================
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_hor_rot);
  nh.subscribe(sub_ver_rot);

  LedRGB.Green();
  delay(time_delay);
  // ===========================


  
  // ===========================
  // Set up servos
  InitialServoConfiguration();
  
  LedRGB.Blue();
  delay(time_delay);
  // ===========================

  LedRGB.Off();
  digitalWrite(FlashLedPin, LOW);

  
  
}

void loop() { 

  // Calling function to update OTA
  ArduinoOTA.handle();

  nh.spinOnce();
  delay(10);
  
}


// ===========================
// Calculate FPS function
// ===========================
void calculateFPS() {
  frameCount++;
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastMillis = currentMillis;
  }
}
