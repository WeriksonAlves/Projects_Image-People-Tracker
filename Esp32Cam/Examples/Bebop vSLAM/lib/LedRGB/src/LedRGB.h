#ifndef LedRGB_h
#define LedRGB_h
#include "Arduino.h"

class LedRGB{
  private:
    int pin_r, pin_g, pin_b;
  public:
    LedRGB(int RedPin, int GreenPin, int BluePin);
    void Red();
    void Green();
    void Blue();
    void Off();
};

#endif