#include <Led.h>
#include <LedRGB.h>
//#include <Servo.h>

// Leds Pinout
#define RedLedPin 14   // GPIO13
#define GreenLedPin 15  // GPIO15
#define BlueLedPin 02   // GPIO02

#define FlashLedPin 04  // GPIO04
#define BoardLedPin 33  // GPIO33

LedRGB LedRGB(RedLedPin,GreenLedPin,BlueLedPin,2,3,4,false);

Led FlashLed(FlashLedPin,false);
Led BoardLed(BoardLedPin,true);


// Servo myservo1;  // create servo object to control a servo
// Servo myservo2;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos1 = 0;    // variable to store the servo position
int pos2 = 0;    // variable to store the servo position

void setup() {
  // myservo1.attach(12);  // attaches the servo on pin 9 to the servo object
  // myservo2.attach(13);  // attaches the servo on pin 9 to the servo object
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

    BoardLed.on();
    delay(1000);
    BoardLed.off();  
     
    FlashLed.on();
    delay(1000);
    FlashLed.off();

  // for (pos1 = 0; pos1 <= 180; pos1 += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }

  // for (pos2 = 0; pos2 <= 180; pos2 += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo2.write(pos2);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }

  // for (pos1 = 180; pos1 >= 0; pos1 -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }

 
  // for (pos2 = 180; pos2 >= 0; pos2 -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo2.write(pos2);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }
}
