// ===========================
// Information about WiFi
// ===========================

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

IPAddress server(192,168,0,125); // MASTER IP
