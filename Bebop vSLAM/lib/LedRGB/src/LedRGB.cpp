#include "LedRGB.h"
#include "Arduino.h"

LedRGB::LedRGB(int RedPin, int GreenPin, int BluePin){
  pin_r = RedPin;
  pin_g = GreenPin;
  pin_b = BluePin;
  pinMode(pin_r, OUTPUT);
  pinMode(pin_g, OUTPUT);
  pinMode(pin_b, OUTPUT);
}

void LedRGB::Red(){
  digitalWrite(pin_r, HIGH);
  digitalWrite(pin_g, LOW);
  digitalWrite(pin_b, LOW);
}

void LedRGB::Green(){
  digitalWrite(pin_r, LOW);
  digitalWrite(pin_g, HIGH);
  digitalWrite(pin_b, LOW);
}

void LedRGB::Blue(){
  digitalWrite(pin_r, LOW);
  digitalWrite(pin_g, LOW);
  digitalWrite(pin_b, HIGH);
}

void LedRGB::Off(){
  digitalWrite(pin_r, LOW);
  digitalWrite(pin_g, LOW);
  digitalWrite(pin_b, LOW);
}