//#include "esp_camera.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp32cam.h>
// ===========================





// ===========================
// Enter your WiFi credentials
// ===========================
#include "Credentials.h"
// ===========================





// ===========================
// Select camera model
// ===========================
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

static auto Res = esp32cam::Resolution::find(640, 480);

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
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initializing Camera
  using namespace esp32cam;
  Config cfg;
  cfg.setPins(pins::AiThinker);
  cfg.setResolution(Res);
  cfg.setBufferCount(1);
  cfg.setJpeg(25);
  


  // Camera Status
  bool ok = Camera.begin(cfg);
  Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");

  if (!esp32cam::Camera.changeResolution(Res)) {
    Serial.println("SET RES FAIL");
    return;
  }

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

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}
// ===========================





// ===========================
// Loop
// ===========================
void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}
// ===========================





// ===========================
// Calculate FPS function
// ===========================
void CalculateFPS() {
  frameCount++;
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastMillis = currentMillis;
  }
}
