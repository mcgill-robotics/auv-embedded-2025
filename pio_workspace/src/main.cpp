#include <Arduino.h>

#ifdef ACTUATOR_H
#include "actuator_main.h"
#elif DISPLAY_H
#include "display_main.h"
#elif POWER_H
#include "power_main.h"
#endif

void setup() {
  #ifdef ACTUATOR_H
    actuator_setup();
  #elif DISPLAY_H
    display_setup();
  #elif POWER_H
    power_setup();
  #endif
}

void loop() {
  #ifdef ACTUATOR_H
    actuator_loop();
  #elif DISPLAY_H
    display_loop();
  #elif POWER_H
    power_loop();
  #endif
}
