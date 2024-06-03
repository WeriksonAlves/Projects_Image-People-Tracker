#include "Led.h"
#include "Arduino.h"

Led::Led(int LedPin, bool IsInverted){
  pin_led = LedPin;
  _IsInverted = IsInverted;
  pinMode(pin_led,OUTPUT);
}

void Led::on(){
  if (_IsInverted){
    digitalWrite(pin_led,LOW);
  }
  else {
    digitalWrite(pin_led,HIGH);
  }
}

void Led::off(){
  if (_IsInverted){
    digitalWrite(pin_led,HIGH);
  }
  else {
    digitalWrite(pin_led,LOW);
  }
}