/* 
Revision 1.0
Commenter: Dorothy Ma
Date: Jan 27, 2025

HEADER FILE FOR ADC_SENSOR FOR CURRENT & VOLTAGE SENSING
* adcCurrent1, adcCurrent2, adcVoltage: Adafruit_ADS1115 sensor drivers
METHODS:
* senseVoltage, senseCurrent: returns an array pointer to current measurements
* refreshVoltage, refreshCurrent: reads the ADC values and uses computeVolts and stores into arrays
* convertVoltage, convertCurrent: converts computed values to real-world values and stores into arrays

VARIABLES:
* voltageEnabled, currentEnabled: flags to indicate which component to measure
* rawADCVoltage, rawADCCurrent: arrays for raw measurements from the ADC
* computedADCVoltage, computedADCCurrent: arrays for measurements from ADC converted by the ADS1X15 computeVolts method
* voltageValues, currentValues: arrays for V and I values calculated using real-world components with meaningful values
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
    ADCSensors();

    bool begin(bool enableVoltage, bool enableCurrent, TwoWire* wire = &Wire1);
    float* senseVoltage();
    float* senseCurrent();
    
  private:
    bool voltageEnabled;
    bool currentEnabled;

    Adafruit_ADS1115 adcCurrent1;
    Adafruit_ADS1115 adcCurrent2;
    Adafruit_ADS1115 adcVoltage;

    int16_t rawADCVoltage[2];
    float computedADCVoltage[2];

    int16_t rawADCCurrent[8];
    float computedADCCurrent[8];

    float voltageValues[2];
    float currentValues[8];

    void refreshVoltage();
    void refreshCurrent();

    float convertVoltage(float adcVoltage);
    float convertCurrent(float adcCurrent);
};

#endif
