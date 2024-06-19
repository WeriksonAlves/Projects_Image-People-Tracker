// Including Library
#include <LedRGB.h>

// Defining RGB pinout
#define RedPin 12   // GPIO12
#define GreenPin 13 // GPIO13
#define BluePin 14  // GPIO14

// Creating LedRGB object
LedRGB LedRGB(RedPin,GreenPin,BluePin);

void setup() {

}
 
void loop() {
    LedRGB.Off();
    delay(1000);
    LedRGB.Red();
    delay(1000);
    LedRGB.Green();
    delay(1000);
    LedRGB.Blue();
    delay(1000);
}