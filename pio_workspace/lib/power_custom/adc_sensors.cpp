/* 
Revision 1.0
Commenter: Dorothy Ma

FUNCTIONS TO USE VOLTAGE- AND CURRENT-SENSING WITH ADCS:
1) call "begin()" to initialize the ADCs and indicate whether to enableVoltage- and or enableCurrent-sensing
2) call "senseVoltage()" and "senseCurrent()" to retrieve updated measurements
*/

#include "adc_sensors.h"

ADCSensors::ADCSensors() {
  /* 
  CONSTRUCTOR TO CREATE AN "EMPTY" OBJECT:
  1) voltageEnabled and currentEnabled flags are set to false
  2) the arrays for holding raw, computed, and meaningful-to-human values of the sensor are initialized to 0
  */
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
  /*
  INTIALIZATION OF THE SENSOR FOR VOLTAGE AND OR CURRENTS SENSING
  Initializes the I2C buses for sensors using their respective addresses (defined in header)
  ARGUMENTS: flags for voltage sensing, current sensing, and wire
  RETURNS: true if the sensors are successfully initialized
  */
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
  /*
  STORES MEANINGFUL-TO-HUMANS VOLTAGE VALUES USING convertVoltage FUNCTION IN voltageValues ARRAY
  Returns: pointer to voltageValues[0-1] array with the converted voltages
  Will be -ve if voltageEnabled = false
  */
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

float* ADCSensors::senseCurrent() {
  /*
  STORES MEANINGFUL-TO-HUMANS CURRENT VALUES USING convertCurrent FUNCTION IN currentValues ARRAY
  Returns: pointer to currentValues[0-7] array with the converted currents
  Will be -ve if currentEnabled = false
  */
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

void ADCSensors::refreshVoltage() {
  /*
    PERFORMS ONE READING OF THE ADC FOR VOLTAGE AND UPDATES THE computedADCVoltage array
    Read and stores the raw ADC value from channels 0 and 1 to rawADCVoltage[0-1] array
    Stores the voltage computed by "computeVolts" function to computedADCVoltage array
  */
  rawADCVoltage[0] = adcVoltage.readADC_SingleEnded(0);
  rawADCVoltage[1] = adcVoltage.readADC_SingleEnded(1);

  computedADCVoltage[0] = adcVoltage.computeVolts(rawADCVoltage[0]);
  computedADCVoltage[1] = adcVoltage.computeVolts(rawADCVoltage[1]);
}

void ADCSensors::refreshCurrent() {
  /*
    PERFORMS ONE READING OF THE ADC FOR CURRENTS 1 and 2 AND UPDATES THE computedADCCurrent array
    Read and stores the raw ADC value from channels 0 to 3 to rawADCCurrent[0-7] array
    Stores the current computed by "computeVolts" function to computedADCCurrent array
    Stores adcCurrent1 values into 0-3 and adcCurrent2 values into 4-7
  */
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
  /* 
  CONVERT ADC VALUES TO REAL-LIFE VOLTAGE VALUES BASED ON CIRCUIT COMPONENTS
  1+R41/R42 = 3
  VBAT LOW: 12.8  | ADC INPUT: 0.18
  VBAT NOM: 14.8  | ADC INPUT: 1.384
  VBAT HIGH: 16.8 | ADC INPUT: 2.586
  */
  return (adcVoltage * 2) * (16.8 - 12.8) / (2.586 - 0.180) + 12.5;
}

float ADCSensors::convertCurrent(float adcCurrent) {
  /* 
  CONVERT ADC VALUES TO REAL-LIFE CURRENT VALUES BASED ON CIRCUIT COMPONENTS
  Rs * Imax * Gain = Vomax
  Vomax = 0.005 * 10 * 50 = 2.5
  */
  return (((adcCurrent * 2 ) / 50 ) / 0.005);
}
