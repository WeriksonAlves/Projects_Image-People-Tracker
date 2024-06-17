// ===========================
// Including Libraries: Default

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_camera.h> //<esp32cam.h>
#include <ArduinoOTA.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <ESP32Servo.h>
// ===========================


// ===========================
//Configs Define

#define StaticIP // Static IP adress

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#define Servo_H 12 
#define Servo_V 13 

#define RedLedPin 14   // GPIO13
#define GreenLedPin 15 // GPIO15
#define BlueLedPin 02  // GPIO02
#define FlashLedPin 04 // GPIO04
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

void startCameraServer();
void setupLedFlash(int pin);
// ===========================



void setup() {
  // ===========================
  // Initializing Serial Communication
  Serial.begin(115200);
  // ===========================

  // ===========================
  // Progress indicators
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  pinMode(FlashLedPin, OUTPUT);

  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, LOW);
  digitalWrite(BlueLedPin,  LOW);
  digitalWrite(FlashLedPin, LOW);
  // ===========================
  
  
  
  // ===========================
  //Initialize the camera  
  Serial.print("Initializing the camera module...");
  configInitCamera();
  Serial.println("Ok!");

  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, LOW);
  digitalWrite(BlueLedPin,  LOW);
  digitalWrite(FlashLedPin, HIGH);
  delay(1000);
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

  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, LOW);
  digitalWrite(BlueLedPin,  HIGH);
  digitalWrite(FlashLedPin, LOW);
  delay(1000);
  // ===========================
  
  
  
  // ===========================
  // Initializing OTA
  OTA();

  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, HIGH);
  digitalWrite(BlueLedPin,  LOW);
  digitalWrite(FlashLedPin, LOW);
  delay(1000);
  // ===========================
  
  
  
  // ===========================
  startCameraServer();

  digitalWrite(RedLedPin,   HIGH);
  digitalWrite(GreenLedPin, LOW);
  digitalWrite(BlueLedPin,  LOW);
  digitalWrite(FlashLedPin, LOW);
  delay(1000);
  // ===========================
  
  
  
  // ===========================
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_hor_rot);
  nh.subscribe(sub_ver_rot);

  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, LOW);
  digitalWrite(BlueLedPin,  HIGH);
  digitalWrite(FlashLedPin, HIGH);
  delay(1000);
  // ===========================


  
  // ===========================
  //  // Set up servos
  //  setUpPinModes();
  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, HIGH);
  digitalWrite(BlueLedPin,  LOW);
  digitalWrite(FlashLedPin, HIGH);
  delay(1000);
  // ===========================

  digitalWrite(RedLedPin,   LOW);
  digitalWrite(GreenLedPin, LOW);
  digitalWrite(BlueLedPin,  LOW);
  digitalWrite(FlashLedPin, LOW);

  
  
}

void loop() { 
  /*
  Code...
  */
  
  // Calling function to update OTA
  ArduinoOTA.handle();

  nh.spinOnce();
  delay(10);

  /*
  Code...
  */
}


void messageCb(const std_msgs::String& data, Servo& servo) {
  String action = data.data;
  if (action == "0") {
    // No action
  } else if (action == "+1") {
    servo.write(servo.read() - 10); // Turn counterclockwise
  } else if (action == "-1") {
    servo.write(servo.read() + 10); // Turn clockwise
  } else if (action == "+2") {
    servo.write(servo.read() - 10); // Turn clockwise (for vertical)
  } else if (action == "-2") {
    servo.write(servo.read() + 10); // Turn counterclockwise (for vertical)
  } else {
    Serial.println("Invalid direction.");
  }
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
