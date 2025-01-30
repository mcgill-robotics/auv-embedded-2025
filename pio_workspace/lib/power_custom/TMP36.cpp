/* 
Revision 1.0
Commenter: Dorothy Ma

TEMPERATURE SENSOR CONTROLS:
0) indicate pin number and reference voltage when creating TMP36 object
1) call "begin()" to setup the pin with microcontroller
2) call "readTemperature()" to retrieve updated measurements
*/

#include "TMP36.h"

/*
  CONSTRUCTOR TO SET UP TEMPERATURE SENSOR VARIABLES:
  ARGUMENTS: pin number, reference voltage (Vcc)
  Initializes the volts and temperature variables to 0
*/
TMP36::TMP36(int pin, float ref) {
  sensorPin = pin;
  refVoltage = ref;
  volts = 0;
  temperature = 0;
}

// INITIALIZES THE TEMP SENSOR BY SETTING IT UP WITH THE RIGHT pinMode
void TMP36::begin() {
  pinMode(sensorPin, INPUT);
}

/*
PERFORMS ONE READING OF THE TEMPERATURE
* calculates the temperature based on formula and reference voltage from the setup
* code source: https://botland.store/content/147-Read-temperature-with-an-Arduino-and-sensor-TMP36GT9Z
RETURNS: temperature reading
*/
float TMP36::readTemperature() {
  int reading = analogRead(sensorPin);
  volts = (reading * refVoltage) / 1024.0;
  temperature = (volts - 0.5) * 100;
  
  return temperature;
}
