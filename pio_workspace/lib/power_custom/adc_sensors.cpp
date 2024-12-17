#include "adc_sensors.h"

ADCSensors::ADCSensors() {
  voltageEnabled = false;
  currentEnabled = false;

  for (int i = 0; i < 2; i++) {
    rawADCVoltage[i] = 0;
    computedADCVoltage[i] = 0.0;
    voltageValues[i] = 0.0;
  }

  for (int i = 0; i < 8; i++) {
    rawADCCurrent[i] = 0;
    computedADCCurrent[i] = 0.0;
    currentValues[i] = 0.0;
  }
}

bool ADCSensors::begin(bool enableVoltage, bool enableCurrent, TwoWire* wire) {
  voltageEnabled = enableVoltage;
  currentEnabled = enableCurrent;

  bool success = true;

  if (voltageEnabled) {
    success &= adcVoltage.begin(ADC_VOLTAGE_ADDR, wire);
  }

  if (currentEnabled) {
    success &= adcCurrent1.begin(ADC_CURRENT1_ADDR, wire);
    success &= adcCurrent2.begin(ADC_CURRENT2_ADDR, wire);
  }

  return success;
}

float* ADCSensors::senseVoltage() {
  if (voltageEnabled) {
    refreshVoltage();

    voltageValues[0] = convertVoltage(computedADCVoltage[0]);
    voltageValues[1] = convertVoltage(computedADCVoltage[1]);

    return voltageValues;
  } else {
    return nullptr;
  }
}

float* ADCSensors::senseCurrent() {
  if (currentEnabled) {
    refreshCurrent();

    for (int i = 0; i < 8; i++) {
      currentValues[i] = convertCurrent(computedADCCurrent[i]);
    }

    return currentValues;
  } else {
    return nullptr;
  }
}

void ADCSensors::refreshVoltage() {
  rawADCVoltage[0] = adcVoltage.readADC_SingleEnded(0);
  rawADCVoltage[1] = adcVoltage.readADC_SingleEnded(1);

  computedADCVoltage[0] = adcVoltage.computeVolts(rawADCVoltage[0]);
  computedADCVoltage[1] = adcVoltage.computeVolts(rawADCVoltage[1]);
}

void ADCSensors::refreshCurrent() {
  for (int i = 0; i < 4; i++) {
    rawADCCurrent[i] = adcCurrent1.readADC_SingleEnded(i);
    computedADCCurrent[i] = adcCurrent1.computeVolts(rawADCCurrent[i]);
  }

  for (int i = 4; i < 8; i++) {
    rawADCCurrent[i] = adcCurrent2.readADC_SingleEnded(i - 4);
    computedADCCurrent[i] = adcCurrent2.computeVolts(rawADCCurrent[i]);
  }
}

float ADCSensors::convertVoltage(float adcVoltage) {
  return (adcVoltage - 0.08) * (16.8 - 12.8) / (1.27 - 0.08) + 12.8;
}

float ADCSensors::convertCurrent(float adcCurrent) {
  return (((adcCurrent * 2 ) / 50 ) / 0.005);
}
