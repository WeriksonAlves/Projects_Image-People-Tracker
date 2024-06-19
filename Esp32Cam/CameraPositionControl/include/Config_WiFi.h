#include <Arduino.h>

#ifndef CONFIG_WIFI_H
#define CONFIG_WIFI_H

#include <WiFi.h>
#include <esp_wifi.h>

extern const char* ssid; // Declaration
extern const char* password; // Declaration
extern const int CHANNEL; // Declaration

#ifdef StaticIP
extern IPAddress local_IP; // Declaration
extern IPAddress gateway; // Declaration
extern IPAddress subnet; // Declaration
extern IPAddress primaryDNS; // Declaration
extern IPAddress secondaryDNS; // Declaration
#endif

extern IPAddress server; // Declaration

void connectWIFI();

#endif // CONFIG_WIFI_H
