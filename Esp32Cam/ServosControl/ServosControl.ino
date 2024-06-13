// ===========================
// Board used
// ===========================
#include "esp_camera.h"
// ===========================



// ===========================
// Information about WiFi
// ===========================
#include <WiFi.h>
#include <esp_wifi.h>

const char* ssid = "NERo-Arena";
const char* password = "BDPsystem10";
// ===========================



// ===========================
// Static IP adress
// ===========================
#define StaticIP 
#ifdef StaticIP
  IPAddress local_IP(192, 168, 0, 111); // IP address - NERo
  IPAddress gateway(192, 168, 0, 1); // Gateway 
  IPAddress subnet(255, 255, 255, 0); // Subnet
  IPAddress primaryDNS(8, 8, 8, 8); // optional
  IPAddress secondaryDNS(8, 8, 4, 4); // optional
#endif
// ===========================



/// ===========================
// Servo settings
// ===========================
#include <ESP32Servo.h>

// ===========================



// ===========================
// ROS settings
// ===========================
#include <ros.h>
#include <std_msgs/String.h>

IPAddress server(192,168,0,125); // MASTER IP
const uint16_t serverPort = 11411; // CONEXAO TCP

ros::NodeHandle nh;
//std_msgs::String msg;

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
    // No action   Serial.println("Invalid direction.");
  }
}

void horRotCb(const std_msgs::String& data) {
//  messageCb(data, servo_h);
}

void verRotCb(const std_msgs::String& data) {
//  messageCb(data, servo_v);
}

ros::Subscriber<std_msgs::String> sub_hor_rot("/SPS/hor_rot", &horRotCb);
ros::Subscriber<std_msgs::String> sub_ver_rot("/SPS/ver_rot", &verRotCb);
// ===========================



// ===========================
// Camera settings
// ===========================
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

#define LED_GPIO_NUM   4 // 4 for flash led or 33 for normal led
// ===========================



// ===========================
// Extras
// ===========================

#define RedLedPin 14   // GPIO13
#define GreenLedPin 15  // GPIO15
#define BlueLedPin 02   // GPIO02
#define FlashLedPin 04  // GPIO04
#define BoardLedPin 33  // GPIO33

void startCameraServer();
void setupLedFlash(int pin);

// Variables for function 'calculefps'
unsigned long lastMillis = 0;
unsigned int frameCount = 0;
const int CHANNEL = 13;
float fps = 0;
// ===========================




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
