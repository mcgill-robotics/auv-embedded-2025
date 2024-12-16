#include "TMP36.h"

TMP36::TMP36(int pin, float ref) {
  sensorPin = pin;
  refVoltage = ref;
  volts = 0;
  temperature = 0;
}

void TMP36::begin() {
  pinMode(sensorPin, INPUT);
}

float TMP36::readTemperature() {
  int reading = analogRead(sensorPin);
  volts = (reading * refVoltage) / 1024.0;
  temperature = (volts - 0.5) * 100;
  
  return temperature;
}
