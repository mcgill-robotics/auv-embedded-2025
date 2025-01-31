/* 
Revision 1.0
Commenter: Dorothy Ma

HEADER FILE FOR TEMPERATURE SENSOR (TMP36GT9Z)
*/

#ifndef TMP36_H
#define TMP36_H

#include "Arduino.h"

class TMP36 {
  public:
    // Constructor for TMP36 class
    TMP36(int pin, float ref);

    // Initialize TMP36 class
    void begin();

    // Performs one reading of the sensor
    float readTemperature();
    
  private:
    int sensorPin; // Pin number to which the sensor is connected
    float refVoltage; // Reference voltage for voltage conversion (Vcc)
    float volts; // Voltage from reading
    float temperature; // Temperature from reading
};

#endif
