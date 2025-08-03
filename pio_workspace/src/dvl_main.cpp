#ifdef DVL_H

#include "dvl_main.h"

// RX2 -> Pin 7 (receives data from DVL)
// TX2 -> Pin 8 (sends data to DVL)

void dvl_setup(){
    Serial.begin(115200);
    Serial2.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void dvl_loop(){
    // Forward data from DVL (Serial2) to Jetson (Serial)
    if (Serial2.available()) {
        int inByte = Serial2.read();
        Serial.write(inByte);
    }
    
    // Forward data from Jetson (Serial) to DVL (Serial2)
    if (Serial.available()) {
        int inByte = Serial.read();
        Serial2.write(inByte);
    }
}

#endif
