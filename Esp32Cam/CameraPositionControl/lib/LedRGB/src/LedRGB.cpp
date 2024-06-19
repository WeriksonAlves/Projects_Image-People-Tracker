#include "LedRGB.h"
#include "Arduino.h"

LedRGB::LedRGB(int RedPin, int GreenPin, int BluePin, int RedChannel, int GreenChannel, int BlueChannel, bool isPWM){
  // Assign values
  _RedPin = RedPin;
  _GreenPin = GreenPin;
  _BluePin = BluePin;
  _RedChannel = RedChannel;
  _GreenChannel = GreenChannel;
  _BlueChannel = BlueChannel;

  _isPWM = isPWM;

  if (_isPWM){
    // configure LED PWM functionalitites
    //ledcSetup(_RedChannel, freq, resolution);
    //ledcSetup(_GreenChannel, freq, resolution);
    //ledcSetup(_BlueChannel, freq, resolution);
  
    // attach the channel to the GPIOS
    ledcAttachPin(_RedPin, _RedChannel);
    ledcAttachPin(_GreenPin, _GreenChannel);
    ledcAttachPin(_BluePin, _BlueChannel);
  }
  else {
    pinMode(_RedPin, OUTPUT);
    pinMode(_GreenPin, OUTPUT);
    pinMode(_BluePin, OUTPUT);
  }
}

void LedRGB::Red(){
  if (!_isPWM){
    digitalWrite(_RedPin, HIGH);
    digitalWrite(_GreenPin, LOW);
    digitalWrite(_BluePin, LOW);
  }
}

void LedRGB::Green(){
  if (!_isPWM){
    digitalWrite(_RedPin, LOW);
    digitalWrite(_GreenPin, HIGH);
    digitalWrite(_BluePin, LOW);
  }
}

void LedRGB::Blue(){
  if (!_isPWM){
    digitalWrite(_RedPin, LOW);
    digitalWrite(_GreenPin, LOW);
    digitalWrite(_BluePin, HIGH);
  }
}

void LedRGB::Off(){
   if (_isPWM){
    ledcWrite(_RedChannel, 0);
    ledcWrite(_GreenChannel, 0);
    ledcWrite(_BlueChannel, 0);
  }
  else {
    digitalWrite(_RedPin, LOW);
    digitalWrite(_GreenPin, LOW);
    digitalWrite(_BluePin, LOW);
  }
}

void LedRGB::Color(int RedPWM, int GreenPWM, int BluePWM){
 if (_isPWM){
    ledcWrite(_RedChannel, RedPWM);
    ledcWrite(_GreenChannel, GreenPWM);
    ledcWrite(_BlueChannel, BluePWM);
 }
}
