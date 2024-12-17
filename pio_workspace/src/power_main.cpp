#include "power_main.h"

#include "TMP36.h"
#include "adc_sensors.h"
#include "ThrusterControl.h"
#include "power_micro_ros.h"

#define LED_PIN 13
#define TEMP_SENSE 23

TMP36 temperature_sensor(TEMP_SENSE, 3.3);

void power_setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    initThrusters();

    micro_ros_init();

    temperature_sensor.begin();
  
    delay(2000);
}

void power_loop() {
    updateThrusters(microseconds);

    power_board_temperature_publish(temperature_sensor.readTemperature());

    spin_micro_ros();
    
    delay(10);
}