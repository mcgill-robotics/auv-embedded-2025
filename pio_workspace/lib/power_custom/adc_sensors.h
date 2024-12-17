#ifndef ADC_SENSORS_H
#define ADC_SENSORS_H

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_ADS1X15.h"

#define ADC_VOLTAGE_ADDR  0x4B
#define ADC_CURRENT1_ADDR 0x48
#define ADC_CURRENT2_ADDR 0x49

class ADCSensors {
  public:
    ADCSensors();

    bool begin(bool enableVoltage, bool enableCurrent, TwoWire* wire = &Wire1);
    float* senseVoltage();
    float* senseCurrent();
    
  private:
    bool voltageEnabled;
    bool currentEnabled;

    Adafruit_ADS1115 adcVoltage;
    Adafruit_ADS1115 adcCurrent1;
    Adafruit_ADS1115 adcCurrent2;

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
