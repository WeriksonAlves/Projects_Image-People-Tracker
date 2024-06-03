#ifndef Led_h
#define Led_h
#include "Arduino.h"

class Led{
  private:
    int pin_led;
    bool _IsInverted;
  public:
    Led(int LedPin, bool IsInverted);
    void on();
    void off();
};

#endif