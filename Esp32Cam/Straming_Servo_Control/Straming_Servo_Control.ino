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
// Camera settings
// ===========================
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"
// ===========================





/// ===========================
// Servo settings
// ===========================
#include <ESP32Servo.h>
//
////#define DUMMY_SERVO1_PIN 14 // We need to create 2 dummy servos.
////#define DUMMY_SERVO2_PIN 15 // So that ESP32Servo library does not interfere with pwm channel and timer used by esp32 camera.
////
////Servo dummyServo1;
////Servo dummyServo2;
//
//#define SERVO_H 12 // We need to create 2 dummy servos.
//#define SERVO_V 13 // So that ESP32Servo library does not interfere with pwm channel and timer used by esp32 camera.
//
//Servo servo_h; // Controls horizontal camera movement
//Servo servo_v; // Controls vertical camera movement

// ===========================

#define RedLedPin 14   // GPIO13
#define GreenLedPin 15  // GPIO15
#define BlueLedPin 02   // GPIO02
#define FlashLedPin 04  // GPIO04
#define BoardLedPin 33  // GPIO33



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
//    Serial.println("Invalid direction.");
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
// Extras
// ===========================
void startCameraServer();
void setupLedFlash(int pin);

// Variables for function 'calculefps'
unsigned long lastMillis = 0;
unsigned int frameCount = 0;
const int CHANNEL = 13;
float fps = 0;
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





// ===========================
// Setup
// ===========================
void setup() {
  // Initializing Serial Communication
//  Serial.begin(115200);
//  Serial.setDebugOutput(true);
//  Serial.println();

  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  pinMode(FlashLedPin, OUTPUT);
  pinMode(BoardLedPin, OUTPUT);

  digitalWrite(RedLedPin,LOW);
  digitalWrite(GreenLedPin,LOW);
  digitalWrite(BlueLedPin,LOW);
  delay(1000);
  
  //Initialize the camera  
//  Serial.print("Initializing the camera module...");
  configInitCamera();
//  Serial.println("Ok!");

  // Configuring static IP address
  #ifdef StaticIP
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//      Serial.println("STA Failed to configure");
    }
  #endif
  
  // Connecting to access point
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  
  // Changing Wi-fi Channel
//  Serial.print("Default WiFi-Channel: ");
//  Serial.println(WiFi.channel()); 
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);   
//  Serial.print("Updated WiFi-Channel: ");
//  Serial.println(WiFi.channel());


  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
//  Serial.println("");
//  Serial.println("WiFi connected");
  
  digitalWrite(RedLedPin,HIGH);
  digitalWrite(GreenLedPin,HIGH);
  digitalWrite(BlueLedPin,LOW);
  delay(1000);
  
  startCameraServer();

//  Serial.print("Camera Ready! Use 'http://");
//  Serial.print(WiFi.localIP());
//  Serial.println("' to connect");

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_hor_rot);
  nh.subscribe(sub_ver_rot);

  digitalWrite(RedLedPin,LOW);
  digitalWrite(GreenLedPin,LOW);
  digitalWrite(BlueLedPin,HIGH);
  delay(1000);
  
//  // Set up servos
//  setUpPinModes();

  digitalWrite(RedLedPin,LOW);
  digitalWrite(GreenLedPin,LOW);
  digitalWrite(BlueLedPin,LOW);
}
// ===========================





// ===========================
// Loop
// ===========================
void loop() {

  nh.spinOnce();
  delay(10);
}
// ===========================





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
// ===========================





// ===========================
// Stores the camera configuration parameters
// ===========================
void configInitCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  config.xclk_freq_hz = 30000000;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming  //YUV422,GRAYSCALE,RGB565,JPEG

  // Select lower framesize if the camera doesn't support PSRAM
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 20; //10-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }  

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
//    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 2); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  
}




//
//// ===========================
//// Settings Servos
//// ===========================
//void setUpPinModes() {
////  dummyServo1.attach(DUMMY_SERVO1_PIN);
////  dummyServo2.attach(DUMMY_SERVO2_PIN);  
//  servo_h.attach(SERVO_H);
//  servo_v.attach(SERVO_V);
//
//}
