// Including Libraries
#include <Led.h>
#include <WiFi.h>
#include "Wire.h"
#include <LedRGB.h>
#include <esp_wifi.h>
#include <esp32cam.h>
#include <Joystick.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>           
#include "Credentials.h" // Remember to uncomment the respective credentials
#include <Electromagnet.h>
#include <WebSocketsServer.h>
                   
// Creating webSocket object
WebSocketsServer webSocket = WebSocketsServer(81);

// Creating server object
WiFiServer server(80);

// Serial Pinout
//#define RXPin 03 // GPIO03       
//#define TXPin 00 // GPIO00   

// Electromagnet Pinout
//#define ElectromagnetPin 01 // GPIO01

// Joystick Pinout
//#define VRxPin 13 // GPIO13
//#define VRyPin 12 // GPIO12

// Leds Pinout
//#define RedLedPin 02    // GPIO02
//#define GreenLedPin 14  // GPIO14
//#define BlueLedPin 15   // GPIO15
#define FlashLedPin 04  // GPIO04
#define BoardLedPin 33  // GPIO33

// I2C pinout
#define I2C_SDA 12 // GPIO12
#define I2C_SCL 13 // GPIO13

// Comment the line below -> dynamic IP/Uncomment the line below -> static IP
//#define StaticIP 

// Variables and constants
bool ButtonState = false;
unsigned long lastMillis = 0;
unsigned int frameCount = 0;
const int CHANNEL = 13;
float fps = 0;
bool flashState = false;
bool redState = false;
bool greenState = false;
bool blueState = false;
bool boardState = false;
bool electromagnetState = false;
bool buttonState;
int buttonValue;

static auto loRes = esp32cam::Resolution::find(320, 240);
static auto midRes = esp32cam::Resolution::find(350, 530);
static auto hiRes = esp32cam::Resolution::find(800, 600);

// Creating Wire Object
//TwoWire I2C = TwoWire(0);

// Creating Gyro Object
//Gyroscope Gyro;

// Creating Leds objects
//LedRGB LedRGB(RedLedPin,GreenLedPin,BlueLedPin,false);
Led FlashLed(FlashLedPin,false);
Led BoardLed(BoardLedPin,true);

// Creating Joystick Object
//Joystick Joystick(VRxPin,VRyPin);

// Creating Electromagnet Object
//Electromagnet Electromagnet(ElectromagnetPin);

// Static IP adress
#ifdef StaticIP
  IPAddress local_IP(192, 168, 0, 117); // IP address - NERo
  //IPAddress local_IP(192, 168, 0, 188); // IP address - NERo
  IPAddress gateway(192, 168, 0, 1); // Gateway 
  IPAddress subnet(255, 255, 255, 0); // Subnet
  IPAddress primaryDNS(8, 8, 8, 8); // optional
  IPAddress secondaryDNS(8, 8, 4, 4); // optional
#endif

void setup() {
  // Initializing Serial Communication
  //Serial.begin(115200,SERIAL_8N1,RXPin,TXPin);
  Serial.begin(115200);

  // Initializing Camera
  using namespace esp32cam;
  Config cfg;
  cfg.setPins(pins::AiThinker);
  cfg.setResolution(loRes);
  cfg.setBufferCount(2);
  cfg.setJpeg(80);

  // Camera Status
  bool ok = Camera.begin(cfg);
  Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");

  if (!esp32cam::Camera.changeResolution(loRes)) {
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
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  // Printing the IP address
  Serial.println("ESP32-CAM IP: " + WiFi.localIP().toString());

  // Initializing server and webSocket
  server.begin();  
  webSocket.begin();
  webSocket.onEvent(WebsocketCallback);

  // Initializing OTA
  OTA();

  // Deactivating Electromagnet
  //Electromagnet.deactivate();

  // Deactivating Board Led
  BoardLed.off();

  // Deactivating RGB Led
  //LedRGB.Off();

  // Calculate Offsets
  //delay(2000);
  //Gyro.calcOffsets(true,true); 
  //Gyro.setFilterGyroCoef(0.98);

  //Joystick.begin();
}

void loop() { 
  // Calling function to update websocket
  webSocket.loop();

  // Calling function to update OTA
  ArduinoOTA.handle();

  // Calling function to send information using websocket
  SendWebsocket();

  // Calling HTML Page (for debugging purposes)
  HTMLPage();
}

// Websocket Callback Function
void WebsocketCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      //Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      //Serial.printf("[%u] Connection from ", num);
      //Serial.println(ip.toString());
    } break;
    case WStype_TEXT:
      //Serial.printf("[%u] Received text: %s\n", num, payload);
      ReadWebsocket(payload);
      break;
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void ReadWebsocket(uint8_t *receivedData) {
  switch ((char)receivedData[0]) {
    case 'R':
      redState = !redState;
      greenState = false;
      blueState = false;
      if (redState) {
        //LedRGB.Red();
      } 
      else {
        //LedRGB.Off();
      }   
      break;
    case 'G':
      redState = false;
      greenState = !greenState;
      blueState = false;
      if (greenState) {
       // LedRGB.Green();
      } 
      else {
       // LedRGB.Off();
      }   
      break;
    case 'B':
      redState = false;
      greenState = false;
      blueState = !blueState;
      if (blueState) {
       // LedRGB.Blue();
      } 
      else {
        //LedRGB.Off();
      }   
      break;
    case 'P':
      boardState = !boardState;
      if (boardState) {
        BoardLed.on();
      } else {
        BoardLed.off();
      }
      break;
    case 'F':
      flashState = !flashState;
      if (flashState) {
        FlashLed.on();
      } else {
        FlashLed.off();
      }
      break;
    case 'E':
      electromagnetState = !electromagnetState;
      if (electromagnetState) {
        //Electromagnet.activate();
      } 
      else {
       //Electromagnet.deactivate();
      }
      break;
    default:
      break;
  }
}

// Send Websocket Function
void SendWebsocket() {
  auto frame = esp32cam::capture();

  if (frame != nullptr) {
    const uint8_t* imgData = frame->data();
    size_t imgSize = frame->size();

    webSocket.broadcastBIN(imgData, imgSize);

    CalculateFPS();
  }

  String jsonData = "";                           
  StaticJsonDocument<200> doc;                     
  JsonObject object = doc.to<JsonObject>();        
  object["X"] = random(100);     
  object["Y"] = random(100);    
  object["B"] = String(buttonValue);                   
  object["FPS"] = String(fps);  
  serializeJson(doc, jsonData);                  

  webSocket.broadcastTXT(jsonData);
}

// Calculate FPS function
void CalculateFPS() {
  frameCount++;
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastMillis = currentMillis;
  }
}

// HTML Page Function
void HTMLPage() {
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/html");
      client.println("Connection: close");
      client.println();
      client.println("<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'>");
      client.println("<meta http-equiv='X-UA-Compatible' content='IE=edge'>");
      client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
      client.println("<title>ESP32-CAM BDP-FLY</title></head><body>");
      client.println("<h1 style='text-align:center;'>ESP32-CAM BDP-FLY</h1>");
      client.println("<div style='display:flex; justify-content: space-between;'>");
      client.println("  <div style='text-align:left;'><img id='cameraImage' width='800' height='600' alt='Camera Image'></div>");
      client.println("  <div style='text-align:right;'>");
      client.println("    <p id='fps'>FPS: </p>");
      client.println("    <p id='sensorDataX'>X: </p>");
      client.println("    <p id='sensorDataY'>Y: </p>");
      client.println("    <p id='sensorDataB'>B: </p>");
      client.println("    <label for='ledColor'>Led:</label>");
      client.println("    <select id='ledColor'>");
      client.println("      <option value='R'>Red</option>");
      client.println("      <option value='G'>Green</option>");
      client.println("      <option value='B'>Blue</option>");
      client.println("      <option value='F'>Flash</option>");
      client.println("      <option value='P'>Board</option>");
      client.println("    </select>");
      client.println("    <button onclick='sendLedData()'>Send</button>");
      client.println("    <button onclick='sendElectromagnetData()'>Electromagnet</button>");
      client.println("  </div>");
      client.println("</div>");
      client.println("<script>const socket = new WebSocket('ws://' + window.location.hostname + ':81/');");
      client.println("socket.addEventListener('message', function (event) {");
      client.println("  if (event.data instanceof Blob) {");
      client.println("    const image = document.getElementById('cameraImage');");
      client.println("    image.src = URL.createObjectURL(event.data);");
      client.println("  } else {");
      client.println("    console.log('Received message:', event.data);");
      client.println("    const data = JSON.parse(event.data);");
      client.println("    if (data.type === 'sensor') {");
      client.println("      const fpsElement = document.getElementById('fps');");
      client.println("      fpsElement.innerText = 'FPS: ' + data.fps;");
      client.println("      const sensorDataX = document.getElementById('sensorDataX');");
      client.println("      const sensorDataY = document.getElementById('sensorDataY');");
      client.println("      const sensorDataB = document.getElementById('sensorDataB');");
      client.println("      sensorDataX.innerText = 'X: ' + data.x;");
      client.println("      sensorDataY.innerText = 'Y: ' + data.y;");
      client.println("      sensorDataB.innerText = 'B: ' + data.b;");
      client.println("    }");
      client.println("  }");
      client.println("});");
      client.println("function sendLedData() {");
      client.println("  const ledColor = document.getElementById('ledColor').value;");
      client.println("  socket.send(ledColor);");
      client.println("}");
      client.println("function sendElectromagnetData() {");
      client.println("  socket.send('E');");
      client.println("}</script>");
      client.println("</body></html>");
      delay(10);
      client.stop();
    }
  }
}