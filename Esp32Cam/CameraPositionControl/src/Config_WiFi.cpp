#include "Config_WiFi.h"

// WiFi credentials
const char* ssid = "NERo-Arena";
const char* password = "BDPsystem10";
const int CHANNEL = 13;

#ifdef StaticIP
  IPAddress local_IP(192, 168, 0, 111); // IP address - NERo
  IPAddress gateway(192, 168, 0, 1); // Gateway 
  IPAddress subnet(255, 255, 255, 0); // Subnet
  IPAddress primaryDNS(8, 8, 8, 8); // optional
  IPAddress secondaryDNS(8, 8, 4, 4); // optional
#endif

IPAddress server(192, 168, 0, 125); // MASTER IP

void connectWIFI() {
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
}
