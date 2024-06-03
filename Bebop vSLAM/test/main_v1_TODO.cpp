
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
====> Code ESP32-Cam - Visual SLAM <====
by Leonardo Fagundes ______Date: Dec, 17 of 2023

Tasks:   - Read image from sensor
//       - Connect to the Wi-Fi router
//       - Send and receive data using ROS
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- */

/* ==> Ocuped Pins in this project:
- 2 => on-off and indicate the logical state of connection status

=======================================================================*/


// Definitions
#define ROSSERIAL_ARDUINO_TCP

// Used Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <Led.h>
#include <LedRGB.h>
#include <ros.h>
#include <esp_wifi.h> // #include <esp32cam.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/String.h>
#include <driver/ledc.h>


#if defined(ESP8266) || defined(ESP32)
#include <EEPROM.h>
#endif

// Wi-Fi credentials
const char* ssid = "NERO";
const char* password = "BDPsystem10";

IPAddress ip(192, 168, 0, 131);     // IP da ESP 32
IPAddress dns(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// Defina o endereço IP do servidor do soquete (rosserial) | IP do ROS Master
IPAddress server(192, 168, 0, 100);

// Defina a porta do servidor do soquete (rosserial)
const uint16_t serverPort = 11412;
ros::NodeHandle nh;

// Callback functions:
int command_state = 0;            // estado do ímã
void callback(const std_msgs::Int32 &command){
  command_state = command.data;
}

int command_LEDstate = 0;         // estado do LED  
void callback_LED(const std_msgs::Int32 &command_LED){
  command_LEDstate = command_LED.data;
}

int command_color[3] = {255, 255, 255};
void callback_color(const std_msgs::Int64MultiArray &color){
  command_color[0] = color.data[0];
  command_color[1] = color.data[1];
  command_color[2] = color.data[2];
}

// Habilitando a mensagem a ser enviada
std_msgs::Int32 ima_state;
ros::Publisher pubIma_state("loadActuator/Ima_state/", &ima_state);

std_msgs::Float32 sensor_load;
ros::Publisher pubSensor_load("loadSensor/Sensor_load/", &sensor_load);

std_msgs::Float32MultiArray sensor_angle;
ros::Publisher pubSensor_angle("loadSensor/Sensor_angle/", &sensor_angle);

std_msgs::Float32 sensor_angle_alpha;
ros::Publisher pubSensor_angle_alpha("loadSensor/Sensor_angle_alpha/", &sensor_angle_alpha);

std_msgs::Float32 sensor_angle_beta;
ros::Publisher pubSensor_angle_beta("loadSensor/Sensor_angle_beta/", &sensor_angle_beta);

std_msgs::Int32 command;
ros::Subscriber<std_msgs::Int32> subIma_command("loadActuator/ima_command/", &callback);

// ------------ LED functions:
std_msgs::Int32 led_state_ROS;
ros::Publisher pubLED_state("commonState/LED_state/", &led_state_ROS);

std_msgs::Int32 command_LED; 
ros::Subscriber<std_msgs::Int32> subLED_command("commonState/LED_command/", &callback_LED);

std_msgs::Int64MultiArray color[3];
ros::Subscriber<std_msgs::Int64MultiArray> subLED_color("commonState/LED_color/", &callback_color);


// Port settings used
const int ledPin = 2; // Pino GPIO conectado ao LED embutido

#define button 1         // Botão do joystick
#define led 15           // LED de sinalização
#define ima 2            // atuador eletro-ímã

#define LED_R 19 // Set the LED pin R
#define LED_G 21 // Set the LED pin G
#define LED_B 18 // Set the LED pin B

// Define the LEDC channels for each color
const int RED_CHANNEL = 0;
const int GREEN_CHANNEL = 1;
const int BLUE_CHANNEL = 2;

// Joystick sensor ports
ReadPot_BDPFly Vrx(32);  // alpha
ReadPot_BDPFly Vry(36);  // beta

// mensagem que será enviada
bool state_Led;          // estado atual do LED
bool last_command;       // comando ROS para acionar/desligar eletro-ímã
float alpha = 0.0;       // inicializção do joystick (ângulo Vrx)
float beta = 0.0;        // inicializção do joystick (ângulo Vry)
float alpha_atual = 0.0;       
float beta_atual = 0.0;        
float angles[2] = {0.0, 0.0};  // vetor de  ângulos
float factor = 0.25;      // fator de ponderação do filtro residual

float one = 1.0;         // variável auxiliar

int LEDstate = 0;
int red = 0;
int green = 0;
int blue = 0;

// Variable definitions
float time_write = 0;    // timer para controle de loop de tempo real
const int calVal_eepromAdress = 0; // calibração da célula de carga
unsigned long t2 = 0;    // timer para exibir informações no serial monitor

bool var_on_off = false;  // Flag for controlling the LED state
bool var_blink = false;   // Flag for controlling the LED state
bool on_off = false;      // Flag for controlling the LED state
bool blink = false;       // Flag for controlling the LED state

bool var_on_off_RGB = false;  // Flag for controlling the LED state
bool var_blink_RGB = false;   // Flag for controlling the LED state
bool on_off_RGB = false;      // Flag for controlling the LED state
bool blink_RGB = false;       // Flag for controlling the LED state

int freq = 10;            // Frequency in Hertz of the blink

int limitLoadWeight = 350;   // peso limite -> o drone não deve conseguir carregar
int warningLoadWeight = 200; // peso excedendo o permitido -> o drone deve conseguir carregar com certa dificuldade
int emptyWeight = 30;        // peso a vazio (sem carga acoplada) [g]

// pins:
const int HX711_dout = 23; // mcu > HX711 dout pin
const int HX711_sck = 22;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Declaration of functions
void On_OFF(int var_on_off_RGB, int red, int green, int blue, int pinR, int pinG, int pinB);
void Blink_LED(int var_blink_RGB, int var_on_off_RGB, int red, int green, int blue, int pinR, int pinG, int pinB, int freq);
void setColor(int red, int green, int blue);

void On_OFF_communicationLED(int var_on_off, int pin);
void Blink_communicationLED(int var_blink, int var_on_off, int pin, int freq);



// Initialization, Configuration code, to be executed once:
void setup() {

  // Configure IN/OUTPUT channels
  // pinMode(button, INPUT);
  // pinMode(ledPin, OUTPUT); // Configura o pino do LED como saída
  pinMode(led, OUTPUT);
  pinMode(ima, OUTPUT);
  state_Led = 0;

  // Configure LEDC PWM channels
  ledcSetup(RED_CHANNEL, 5000, 8);   // 5 kHz PWM, 8-bit resolution
  ledcSetup(GREEN_CHANNEL, 5000, 8);
  ledcSetup(BLUE_CHANNEL, 5000, 8);

  // Attach LEDC channels to the corresponding GPIO pins
  ledcAttachPin(LED_R, RED_CHANNEL);
  ledcAttachPin(LED_G, GREEN_CHANNEL);
  ledcAttachPin(LED_B, BLUE_CHANNEL); 

  // Usando o monitor serial do ESP
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("\n\n\n----------------------- Starting ESP 32 on ROS node:\n\n");
  Serial.print("Connecting on Wi-Fi router: ");
  Serial.print(ssid);
  delay(100);
  Serial.println();
  
  // Connect the ESP board to Wi-Fi 
  WiFi.config(ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  Serial.println(""); // ("Connecting to WiFi...");
  var_blink = true;
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.print(".");
    // Test LED during Wi-Fi connection
    On_OFF_communicationLED(var_on_off, led);
    Blink_communicationLED(var_blink, var_on_off, led, freq);
  }
  delay(250);

  var_blink = false;
  var_on_off = true;

  On_OFF_communicationLED(var_on_off, led);

  // Start the server
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP()); //192.168.0.125
  Serial.println("");
  
   // Liga o LED demonstrando que tudo está Ok até aqui
  var_on_off_RGB = false;
  var_blink_RGB = true;
  On_OFF(var_on_off_RGB, 255, 255, 255, LED_R, LED_G, LED_B);
  Blink_LED(var_blink_RGB, var_on_off_RGB, 255, 255, 255, LED_R, LED_G, LED_B, freq);

  // Definindo a conexão com o servidor do soquete (rosserial)
  nh.getHardware() -> setConnection(server,serverPort);
  nh.initNode();

  // Start da msg
  nh.advertise(pubIma_state);
  nh.advertise(pubSensor_load);
  
  sensor_angle.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  sensor_angle.layout.dim[0].label = "angulos";
  sensor_angle.layout.dim[0].size = 2;
  sensor_angle.layout.dim[0].stride = 1*2;
  //sensor_angle.layout.dim_length = 1;
  sensor_angle.layout.data_offset = 0;

  sensor_angle.data = (float *)malloc(sizeof(float)*2);
  sensor_angle.data_length = 2;

  nh.advertise(pubSensor_angle);
  nh.advertise(pubSensor_angle_alpha);
  nh.advertise(pubSensor_angle_beta);
  nh.subscribe(subIma_command);

  // Start da msg
  nh.advertise(pubLED_state);
  nh.subscribe(subLED_command);
  nh.subscribe(subLED_color);

  delay(100);

  // Load Cell
  LoadCell.begin();
  // LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue;   // calibration value (see example file "Calibration.ino")
  calibrationValue = 681.0; // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266) || defined(ESP32)
  // EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
      Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
      while (1)
          ;
  }
  else
  {
      LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
      Serial.println("Startup is complete");
  }

  delay(100);

  // Desliga o LED para iniciar o Loop Principal
  var_on_off = false;
  var_blink = false;
  On_OFF_communicationLED(var_on_off, led);
  On_OFF(var_on_off_RGB, 0, 0, 0, LED_R, LED_G, LED_B);
  Blink_LED(var_blink_RGB, var_on_off_RGB, 0, 0, 0, LED_R, LED_G, LED_B, freq);

}



// Main loop:
void loop() {
  unsigned long t = millis();
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; // increase value to slow down serial print activity

  if (nh.connected()) {
    // ------------------------------ ROS topics
    on_off = true;

    // - Control Electromaget
    last_command = state_Led;
    state_Led = command_state;

    if (state_Led != last_command){
      digitalWrite(ima, state_Led);
    }

    ima_state.data = state_Led;
    pubIma_state.publish( &ima_state );
    // -----------------------

    // - Angles (values joystick)
    // show the state ima and values joystick
    if (t - time_write >= 30){ // sample time of angles joystick = 30 mili seconds
      alpha_atual = Vrx.getAngle();
      beta_atual = Vry.getAngle();

      // filtro de resíduo:
      alpha = factor*alpha + (1.0-factor)*alpha_atual;
      beta = factor*beta + (1.0-factor)*beta_atual;

      // Alocando no vetor de ângulos
      angles[0] = alpha;
      angles[1] = beta;

      // Publicando no ROS            
      sensor_angle.data[0] = alpha;
      sensor_angle.data[1] = beta;
      pubSensor_angle.publish( &sensor_angle );

      sensor_angle_alpha.data = alpha;
      pubSensor_angle_alpha.publish( &sensor_angle_alpha );

      sensor_angle_beta.data = beta;
      pubSensor_angle_beta.publish( &sensor_angle_beta );

      time_write = t;
    }

    // -----------------------
    // check for new data/start next conversion:
    if (LoadCell.update())
        newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady){
        float i = LoadCell.getData();

        sensor_load.data = i;
        pubSensor_load.publish( &sensor_load );

        newDataReady = 0;

        // Exibição do dado no monitor Serial
        if (millis() - t2 >= serialPrintInterval){
            // envio da mensagem
            Serial.println("ROS Command: " + String(command_state));
    
            // show values in Serial monitor
            Serial.print("State Ima: ");
            Serial.print(state_Led);
            Serial.print(" | alpha = ");
            Serial.print(angles[0]);
            Serial.print(" , ");
            Serial.print("beta = ");
            Serial.print(angles[1]);
            Serial.print(" | Load_cell output val (Kg): ");
            Serial.println(i);
    
            t2 = millis();
        }
    }

    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
            LoadCell.tareNoDelay();
    }
    // check if last tare operation is complete:
    if (LoadCell.getTareStatus() == true)
    {
        Serial.println("Tare complete");
    }
    // -----------------------

    // Serial.print("PASSOU AQUIII!!!! -----------------------------------------------------------------");

    // - Control LED state
    LEDstate = command_LEDstate;
    led_state_ROS.data = LEDstate;
    pubLED_state.publish( &led_state_ROS );

    red = command_color[0];
    green = command_color[1];
    blue = command_color[2];

    // leitura da mensagem:
    // String message("OFF");
    if (command_LEDstate == 1) { 
      // String message("ON"); 
      on_off_RGB = true;
      blink_RGB = false;
    } else if (command_LEDstate == 2) { 
      // String message("BLINK ON"); 
      blink_RGB = true;
      on_off_RGB = false; 
    } else if (command_LEDstate == 0) { 
      // String message("OFF"); 
      on_off_RGB = false; 
      blink_RGB = false; 
    }


  } else {
    Serial.println("Não conectado ao ROS Master\n");
    on_off = false;
  }

  // Checks if any of the variables have changed state and if so, calls the functions accordingly.
  if (var_on_off != on_off) {
    var_on_off = on_off;
    On_OFF_communicationLED(var_on_off, led);
  }
  if (var_blink != blink) {
    var_blink = blink;
    Blink_communicationLED(var_blink, var_on_off, led, freq);
  }
  // Call the functions ON_OFF and BLINK_LED in the loop based on the flags
  On_OFF_communicationLED(var_on_off, led);
  Blink_communicationLED(var_blink, var_on_off, led, freq);

  if (var_on_off_RGB != on_off_RGB) {
    var_on_off_RGB = on_off_RGB;
    On_OFF(var_on_off_RGB, red, green, blue, LED_R, LED_G, LED_B);
  }
  if (var_blink_RGB != blink_RGB) {
    var_blink_RGB = blink_RGB;
    Blink_LED(var_blink_RGB, var_on_off_RGB, red, green, blue, LED_R, LED_G, LED_B, freq);
  }
  // Call the functions ON_OFF and BLINK_LED in the loop based on the flags
  On_OFF(var_on_off_RGB, red, green, blue, LED_R, LED_G, LED_B);
  Blink_LED(var_blink_RGB, var_on_off_RGB, red, green, blue, LED_R, LED_G, LED_B, freq);

  
  nh.spinOnce();

  // Loop para performance
  delay(5);
}




// Creating the functions
// Function to set the RGB color
void setColor(int red, int green, int blue) {
  ledcWrite(RED_CHANNEL, red);
  ledcWrite(GREEN_CHANNEL, green);
  ledcWrite(BLUE_CHANNEL, blue);
}


// Creating the functions
void On_OFF(int var_on_off_RGB, int red, int green, int blue, int pinR, int pinG, int pinB) {
  /*
  Lights up the LED if the flag is true
  var_on_off: flag  for controlling the LED
  pin: pin of the LED
  */
  if (var_on_off_RGB) { 
    setColor(red,green,blue);
  } else if (!var_on_off_RGB){ 
    setColor(0,0,0); 
    } else {
    Serial.println("Error 1\n");
  } 
}

void Blink_LED(int var_blink_RGB, int var_on_off_RGB, int red, int green, int blue, int pinR, int pinG, int pinB, int freq) {
  /*
  Flashes the LED with a freq frequency if the flag is true
  var_blink: flag for controlling the LED
  pin: pin of the LED
  freq: frequency in Hertz of the blink
  */
  if (var_blink_RGB) {
    setColor(red,green,blue);
    delay(1000/freq);
    setColor(0,0,0);
    delay(1000/freq);
  } else if (!var_blink_RGB & !var_on_off_RGB){
    setColor(0,0,0);
  } else {
    Serial.println("Error 2\n");
  }
}


// Creating the functions
void On_OFF_communicationLED(int var_on_off, int pin) {
  /*
  Lights up the LED if the flag is true
  var_on_off: flag  for controlling the LED
  pin: pin of the LED
  */
  if (var_on_off) { 
    digitalWrite(pin, HIGH);
    delay(5);
  } else if (!var_on_off){ 
    digitalWrite(pin, LOW); 
    delay(5);
  } else {
    Serial.println("Error 1");
  } 
}

void Blink_communicationLED(int var_blink, int var_on_off, int pin, int freq) {
  /*
  Flashes the LED with a freq frequency if the flag is true
  var_blink: flag for controlling the LED
  pin: pin of the LED
  freq: frequency in Hertz of the blink
  */
  if (var_blink) {
    digitalWrite(pin, HIGH);
    delay(1000/freq);
    digitalWrite(pin, LOW);
    delay(1000/freq);
  } else if (!var_blink & !var_on_off){
    digitalWrite(pin, LOW);
  } else {
    Serial.println("Error 2");
  }
}