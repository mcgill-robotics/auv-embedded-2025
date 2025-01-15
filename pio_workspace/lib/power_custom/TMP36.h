#ifndef TMP36_H
#define TMP36_H

#include "Arduino.h"

class TMP36 {
  public:
    TMP36(int pin, float ref);

    void begin();
    float readTemperature();
    
  private:
    int sensorPin;
    float refVoltage;
    float volts;
    float temperature;
};

#endif
