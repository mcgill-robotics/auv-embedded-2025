/* 
Revision 1.0
Commenter: Dorothy Ma

HEADER FILE FOR ADC_SENSOR FOR CURRENT & VOLTAGE SENSING
*/

#ifndef ADC_SENSORS_H
#define ADC_SENSORS_H

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_ADS1X15.h"

#define ADC_CURRENT1_ADDR 0x48
#define ADC_CURRENT2_ADDR 0x49
#define ADC_VOLTAGE_ADDR  0x4B

class ADCSensors {
  public:
    // Constructor for ADCSensor class
    ADCSensors();

    // Initialization of ADCSensor
    bool begin(bool enableVoltage, bool enableCurrent, TwoWire* wire);
    
    // Returns an array pointer to current measurements
    float* senseVoltage();
    float* senseCurrent();
    
  private:
    // Flags to indicate which component to measure
    bool voltageEnabled;
    bool currentEnabled;

    // Create Adafruit_ADS1115 sensor drivers
    Adafruit_ADS1115 adcCurrent1;
    Adafruit_ADS1115 adcCurrent2;
    Adafruit_ADS1115 adcVoltage;

    // Arrays for raw measurements from the ADC
    int16_t rawADCVoltage[2];
    float computedADCVoltage[2];

    // Arrays for measurements from ADC converted by the ADS1X15 computeVolts method
    int16_t rawADCCurrent[8];
    float computedADCCurrent[8];

    // Arrays for V and I values calculated using real-world components with meaningful values
    float voltageValues[2];
    float currentValues[8];

    // Reads the ADC values and uses computeVolts and stores into arrays
    void refreshVoltage();
    void refreshCurrent();

    // Converts computed values to real-world values and stores into arrays
    float convertVoltage(float adcVoltage);
    float convertCurrent(float adcCurrent);
};

#endif
