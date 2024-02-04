/*

DO NOT FLASH TO TEENSY OR TO ARDUINO

*/

#include <EEPROM.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

const int buttonPin = 2;
int buttonState = 0;

const int torightbuttonPin = 3;
int torightbuttonState = 0;


int RorW = 0;
// read is 0
// write is 1
int address = 1;
int pressure = 0;

int modeChosen = 0;

void setup()
{
  Serial.begin(9600);
  delay(500);
  pinMode(buttonPin, INPUT);
  pinMode(torightbuttonPin, INPUT);

  pinMode(13, OUTPUT);
  //RorW = EEPROM.read(0);
  //digitalWrite(LED_BUILTIN, RorW);
  //RorW = !RorW;
  //if (RorW == 0) {
    //EEPROM.write(0, 0);
  //} else {
    //EEPROM.write(0, 1);
  //}


  // setup for pressure sensor
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  // .init sets the sensor model for us but we can override it if required.
  // Uncomment the next line to force the sensor model to the MS5837_30BA.
  sensor.setModel(MS5837::MS5837_30BA);

  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater) was 997
}

void loop()
{
  while (modeChosen == 0) {
    buttonState = digitalRead(buttonPin);
    torightbuttonState = digitalRead(torightbuttonPin);
    if (buttonState) {
      Serial.print("left");
      Serial.print("\n");
      RorW = 0;
      modeChosen = 1;
      digitalWrite(13, HIGH);
      delay(10);
      digitalWrite(13, LOW);
      delay(10);
      digitalWrite(13, HIGH);
      delay(10);
      digitalWrite(13, LOW);
      delay(10);
      digitalWrite(13, HIGH);
      delay(10);
      digitalWrite(13, LOW);
      delay(10);
      digitalWrite(13, HIGH);
      delay(10);
      digitalWrite(13, LOW);
      delay(10);
      break;
    }
    if (torightbuttonState) {
      Serial.print("right");
      Serial.print("\n");
      RorW = 1;
      modeChosen = 1;
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      delay(1000);
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      break;
    }
  }
  Serial.print(address);
  Serial.print("\n");
  if (RorW==0) {
    //Serial.print("Writing");
    //Serial.print("\n");
    if (address > 1001) {
      // do nothing
    }
    sensor.read();
    pressure = round(sensor.pressure()/10);
    Serial.print(pressure);
    Serial.print("\n");
    EEPROM.write(address, pressure);
    address = address + 1;
    delay(8000);
  } else {
    //Serial.print("Reading");
    //Serial.print("\n");
    // read from eeprom and display to serial monitor
    pressure = EEPROM.read(address);
    Serial.print(pressure);
    Serial.print("\n");
    address = address + 1;
    delay(100);
  }
}