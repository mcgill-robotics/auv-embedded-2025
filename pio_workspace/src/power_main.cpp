#include "power_main.h"

#include "TMP36.h"
#include "adc_sensors.h"
#include "ThrusterControl.h"

#define LED 13
#define TEMP_SENSE 23
#define THRUSTER_DELAY 1000  // Delay time in milliseconds

ADCSensors ADCs;
TMP36 TEMPSENSOR(TEMP_SENSE, 3.3);

// A variable to track the current thruster being updated
int currentThruster = 0;

void power_setup() {
    // an interrupt would generally be setup for water detection

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    initThrusters();

    Serial.begin(115200);
    Serial.println("Initializing ADCs...");
    if (!ADCs.begin(true, true, &Wire1)) {
        Serial.println("Failed to init ADCs");
    }

    Serial.println("Initializing Temperature Sensor");
    TEMPSENSOR.begin();
}

void power_loop() {
    // Reset the microseconds array to default value (1500) for all thrusters
    for (int i = 0; i < 8; i++) {
        microseconds[i] = 1500;
    }

    // Update the current thruster
    microseconds[currentThruster] = 1560;
    updateThrusters(microseconds);

    // Increment to the next thruster for the next loop
    currentThruster = (currentThruster + 1) % 8;

    // Print the voltages, currents, and temperature
    float* voltages = ADCs.senseVoltage();
    if (voltages) {
        Serial.print("Voltage 1: ");
        Serial.println(voltages[0]);
        Serial.print("Voltage 2: ");
        Serial.println(voltages[1]);
    } else {
        Serial.println("Failed to read voltages");
    }

    // Print the currents
    float* currents = ADCs.senseCurrent();
    if (currents) {
        for (int i = 0; i < 8; i++) {
            Serial.print("Current ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println(currents[i]);
        }
    } else {
        Serial.println("Failed to read currents");
    }

    // Print the temperature
    float temperature = TEMPSENSOR.readTemperature();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    // Delay after updating one thruster
    delay(THRUSTER_DELAY);

    // Reset the current thruster to the off state (1500)
    microseconds[currentThruster] = 1500;
    updateThrusters(microseconds);
}
