/* 
Revision 1.0
Commenter: Dorothy Ma

TEMPERATURE SENSOR CONTROLS:
0) indicate pin number and ref (Vcc) voltage when creating TMP36 object
1) call "begin()" to setup the pin with microcontroller
2) call "readTemperature()" to retrieve updated measurements
*/

#include "TMP36.h"

// Arguments: pin number, reference voltage (Vcc)
TMP36::TMP36(int pin, float ref) {
  sensorPin = pin;
  refVoltage = ref;
  // Initializes volts and temperature variables to 0
  volts = 0;
  temperature = 0;
}

// Initializes the temp sensor by setting it up with the right pin
void TMP36::begin() {
  pinMode(sensorPin, INPUT);
}


// Returns reading of the temperature sensor
float TMP36::readTemperature() {
  // calculates the temperature based on formula and reference voltage from the setup
  // code source: https://botland.store/content/147-Read-temperature-with-an-Arduino-and-sensor-TMP36GT9Z

  int reading = analogRead(sensorPin);
  volts = (reading * refVoltage) / 1024.0;
  temperature = (volts - 0.5) * 100;
  
  return temperature;
  
}
