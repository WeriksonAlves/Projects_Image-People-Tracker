#ifndef LedRGB_h
#define LedRGB_h
#include "Arduino.h"

class LedRGB{
  private:
    int _RedPin, _GreenPin, _BluePin, _RedChannel, _GreenChannel, _BlueChannel;
    bool _isPWM;

    // PWM properties
    const int freq = 5000;
    const int resolution = 8;
  public:
    LedRGB(int RedPin, int GreenPin, int BluePin, int RedChannel, int GreenChannel, int BlueChannel, bool isPWM);
    void Red();
    void Green();
    void Blue();
    void Off();
    void Color(int RedPWM, int GreenPWM, int BluePWM);
};

#endif