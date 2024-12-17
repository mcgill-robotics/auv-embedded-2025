#include "power_main.h"

#include "TMP36.h"
#include "adc_sensors.h"
#include "ThrusterControl.h"
#include "power_micro_ros.h"

#define LED_PIN 13

void power_setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    initThrusters();

    micro_ros_init();
  
    delay(2000);
}

void power_loop() {
    updateThrusters(microseconds);

    spin_micro_ros();
    
    delay(10);
}