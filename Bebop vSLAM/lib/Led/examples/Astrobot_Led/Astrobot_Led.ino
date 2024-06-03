// Including Library
#include <Led.h>

// Defining Leds pinout
#define ForwardLedPin 26  // GPIO26
#define RightLedPin 25    // GPIO25
#define LeftLedPin 33     // GPIO33
#define FunctionLedPin 01 // GPIO01 (TX)
#define PlayLedPin 03     // GPIO03 (RX)

// Creating Led objects
Led ForwardLed(ForwardLedPin);
Led RightLed(RightLedPin);
Led LeftLed(LeftLedPin);
Led FunctionLed(FunctionLedPin);
Led PlayLed(PlayLedPin);

void setup() {

}
 
void loop() {
    ForwardLed.on();
    delay(1000);
    ForwardLed.off();
    delay(1000);
    RightLed.on();
    delay(1000);
    RightLed.off();
    delay(1000);
    LeftLed.on();
    delay(1000);
    LeftLed.off();
    delay(1000);
    FunctionLed.on();
    delay(1000);
    FunctionLed.off();
    delay(1000);
    PlayLed.on();
    delay(1000);
    PlayLed.off();
    delay(1000);
}