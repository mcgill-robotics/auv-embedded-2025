/* 
Revision 1.0
Commenter: Dorothy Ma

FUNCTIONS TO USE VOLTAGE- AND CURRENT-SENSING WITH ADCS:
1) call "begin()" to initialize the ADCs and indicate whether to enableVoltage- and or enableCurrent-sensing
2) call "senseVoltage()" and "senseCurrent()" to retrieve updated measurements
*/

#include "adc_sensors.h"
/* 
  Constructor for empty ADCSensor object:
  1) voltageEnabled and currentEnabled flags are set to false
  2) the arrays for holding sensor values are initialized to 0
*/
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

/*
Initializes the I2C buses for sensors using their respective addresses for current and voltage sensing
Arguments: flags for voltage sensing, current sensing, and wire
Returns: true if the sensors are successfully initialized
*/
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

/* 
Returns meaningful voltage values into an array [2]
Will be -ve if voltageEnabled = false
*/
float* ADCSensors::senseVoltage() {
  if (voltageEnabled) {
    refreshVoltage();
    voltageValues[0] = convertVoltage(computedADCVoltage[0]);
    voltageValues[1] = convertVoltage(computedADCVoltage[1]);
  } else {
    voltageValues[0] = -1.0;
    voltageValues[1] = -1.0;
  }
  return voltageValues;
}

/*
Returns meaningful current values into an array [8]
Will be -ve if currentEnabled = false
*/
float* ADCSensors::senseCurrent() {
  if (currentEnabled) {
    refreshCurrent();
    for (int i = 0; i < 8; i++) {
      currentValues[i] = convertCurrent(computedADCCurrent[i]);
    }
  } else {
    for (int i = 0; i < 8; i++) {
      currentValues[i] = -1.0;
    }
  }
  return currentValues;
}

/*
Performs one reading of the ADC for voltage
Read and stores raw ADC value from channels 0-1 and computed voltage values into arrays
*/
void ADCSensors::refreshVoltage() {
  rawADCVoltage[0] = adcVoltage.readADC_SingleEnded(0);
  rawADCVoltage[1] = adcVoltage.readADC_SingleEnded(1);

  computedADCVoltage[0] = adcVoltage.computeVolts(rawADCVoltage[0]);
  computedADCVoltage[1] = adcVoltage.computeVolts(rawADCVoltage[1]);
}

/* 
Performs one reading of the ADC for current
Read and stores raw ADC value for channels 0-3 for both adcCurrent objects
Stores adcCurrent1 values into 0-3 and adcCurrent2 values into 4-7
*/
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

// Convert ADC values into meaningful real-life voltage values based on circuit components
float ADCSensors::convertVoltage(float adcVoltage) {
  /* 
  1+R41/R42 = 3
  VBAT LOW: 12.8  | ADC INPUT: 0.18
  VBAT NOM: 14.8  | ADC INPUT: 1.384
  VBAT HIGH: 16.8 | ADC INPUT: 2.586
  */
  return (adcVoltage * 2) * (16.8 - 12.8) / (2.586 - 0.180) + 12.5;
}

// Convert ADC values into meaningful real-life current values based on circuit components
float ADCSensors::convertCurrent(float adcCurrent) {
  /* 
  Rs * Imax * Gain = Vomax
  Vomax = 0.005 * 10 * 50 = 2.5
  */
  return (((adcCurrent * 2 ) / 50 ) / 0.005);
}
