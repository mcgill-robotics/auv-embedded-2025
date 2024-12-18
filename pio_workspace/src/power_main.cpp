#include "power_main.h"

#include <Arduino.h>

#include "ThrusterControl.h"
#include "adc_sensors.h"
#include "TMP36.h"
#include "micro_ros_wrapper.h"

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32.h>

#define LED_PIN 13

MicroROSWrapper micro_ros("power_node");

void propulsion_microseconds_callback(const void *msgin) {
    const std_msgs__msg__Int16MultiArray *propulsion_microseconds_msg = (const std_msgs__msg__Int16MultiArray *)msgin;
    // Ensure we don't exceed the size of the `microseconds` array
    for (size_t i = 0; i < 8 && i < propulsion_microseconds_msg->data.size; i++) {
        microseconds[i] = propulsion_microseconds_msg->data.data[i]; // Access the data correctly
    }
}

Publisher power_board_temprature_publisher = micro_ros.createPublisher(
    "/power/board/temperature",
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32));

std_msgs__msg__Float32 power_board_temperature_msg;

Subscriber propulsion_microseconds_subscriber = micro_ros.createSubscriber(
    "/propulsion/microseconds",
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    propulsion_microseconds_callback);
    
std_msgs__msg__Int16MultiArray propulsion_microseconds_msg;

TMP36 temperatureSensor(23, 3.3);

void power_setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    initThrusters();

    // init all messages
    power_board_temperature_msg.data = 0.0;

    propulsion_microseconds_msg.data.data = (int16_t *)malloc(8 * sizeof(int16_t));
    propulsion_microseconds_msg.data.size = 8;
    propulsion_microseconds_msg.data.capacity = 8;

    temperatureSensor.begin();
}

void power_loop() {
    power_board_temperature_msg.data = temperatureSensor.readTemperature();
    power_board_temprature_publisher.publish(&power_board_temperature_msg);

    updateThrusters(microseconds);

    micro_ros.spin();
}
