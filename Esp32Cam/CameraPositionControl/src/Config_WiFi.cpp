#include <Arduino.h>

#include "Config_WiFi.h"

const char* ssid = "NERo-Arena";
const char* password = "BDPsystem10";
const int CHANNEL = 13;
IPAddress server(192, 168, 0, 125); // Update with your server IP

#ifdef StaticIP
IPAddress local_IP(192, 168, 0, 111);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);
#endif

void connectWIFI() {
    #ifdef StaticIP
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("STA Failed to configure");
    }
    #endif

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
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
