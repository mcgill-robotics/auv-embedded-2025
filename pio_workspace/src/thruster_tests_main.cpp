#ifdef THRUSTER_TESTS_H

#include "thruster_tests_main.h"

#include <Arduino.h>

#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>

#define LED_PIN 13
#define BUTTON 2

float thrusterForce = -1;

#define YELLOW_LED 6
#define GREEN_LED 7

// sensor code
#include <HX711_ADC.h>

const int HX711_dout = 4; // mcu > HX711 dout pin
const int HX711_sck = 5;  // mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

void sensor_setup() {
    LoadCell.begin();
    float calibrationValue;
    calibrationValue = 696.0;
    unsigned long stabilizingtime = 2000;
    boolean _tare = true;
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag()) {
        while (1);
    } else {
        LoadCell.setCalFactor(calibrationValue);
    }
}

void sensor_loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      thrusterForce = i;
      newDataReady = 0;
      t = millis();
    }
  }

  if (digitalRead(BUTTON) == HIGH) {
    LoadCell.tareNoDelay();
    digitalWrite(YELLOW_LED, HIGH);
  }

  if (LoadCell.getTareStatus() == true) {
    digitalWrite(GREEN_LED, HIGH);
    delay(500);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }
}
// end of sensor code

std_msgs::Float32 thruster_force_msg;

uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
void commandCb(const auv_msgs::ThrusterMicroseconds &tc) {
    memcpy(microseconds, tc.microseconds, 8 * sizeof(uint16_t));
}

ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Publisher thruster_force("/thruster/force", &thruster_force_msg);

void publishData() {
    thruster_force_msg.data = thrusterForce;
    thruster_force.publish(&thruster_force_msg);
}

void thruster_tests_setup() {
    pinMode(BUTTON, INPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    sensor_setup();

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(thruster_force);
}

void thruster_tests_loop() {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    sensor_loop();

    publishData();
    nh.spinOnce();
    delay(10);
}

#endif
