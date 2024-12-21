#include "power_main.h"

#include <Arduino.h>

#include "ThrusterControl.h"
#include "adc_sensors.h"
#include "TMP36.h"
#include "micro_ros_wrapper.h"

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#define LED_PIN 13

#define ENABLE_VOLTAGE_SENSE false
#define ENABLE_CURRENT_SENSE false

MicroROSWrapper micro_ros;
ADCSensors adcSensors;
TMP36 temperatureSensor(23, 3.3);

// Define the subscribers and publishers as rcl_publisher_t and rcl_subscription_t
rcl_subscription_t* propulsion_microseconds_subscriber;
rcl_publisher_t* power_thrusters_current_publisher;
rcl_publisher_t* power_batteries_voltage_publisher;
rcl_publisher_t* power_board_temperature_publisher;
rcl_publisher_t* power_teensy_temperature_publisher;

std_msgs__msg__Int16MultiArray propulsion_microseconds_msg;
std_msgs__msg__Float32MultiArray power_thrusters_current_msg;
std_msgs__msg__Float32MultiArray power_batteries_voltage_msg;
std_msgs__msg__Float32 power_board_temperature_msg;
std_msgs__msg__Float32 power_teensy_temperature_msg;

void propulsion_microseconds_callback(const void *msgin) {
  const std_msgs__msg__Int16MultiArray *propulsion_microseconds_msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  for (size_t i = 0; i < 8; i++) {
    microseconds[i] = propulsion_microseconds_msg->data.data[i]; // Access the data correctly
  }
}

bool create_entities() {
  micro_ros.init("power_node");
  
  // Create subscribers and publishers using the MicroROSWrapper class
  propulsion_microseconds_subscriber = micro_ros.createSubscriber(
      "/propulsion/microseconds",
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      propulsion_microseconds_callback
  );

  power_thrusters_current_publisher = micro_ros.createPublisher(
      "/power/thrusters/current",
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray)
  );

  power_batteries_voltage_publisher = micro_ros.createPublisher(
      "/power/batteries/voltage",
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray)
  );

  power_board_temperature_publisher = micro_ros.createPublisher(
      "/power/board/temperature",
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32)
  );

  power_teensy_temperature_publisher = micro_ros.createPublisher(
      "/power/teensy/temperature",
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32)
  );
  return true;
}

void computeAndPublish() {
  power_board_temperature_msg.data = temperatureSensor.readTemperature();
  micro_ros.publishData(&power_board_temperature_msg, *power_board_temperature_publisher);  // Dereference pointer

  power_teensy_temperature_msg.data = tempmonGetTemp();
  micro_ros.publishData(&power_teensy_temperature_msg, *power_teensy_temperature_publisher);  // Dereference pointer

  float* current_data = adcSensors.senseCurrent();
  for (size_t i = 0; i < 8; i++) {
      power_thrusters_current_msg.data.data[i] = current_data[i];
  }
  micro_ros.publishData(&power_thrusters_current_msg, *power_thrusters_current_publisher);  // Dereference pointer

  float* voltage_data = adcSensors.senseVoltage();
  for (size_t i = 0; i < 2; i++) {
      power_batteries_voltage_msg.data.data[i] = voltage_data[i];
  }
  micro_ros.publishData(&power_batteries_voltage_msg, *power_batteries_voltage_publisher);  // Dereference pointer
}

void power_setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  initThrusters();

  adcSensors.begin(ENABLE_VOLTAGE_SENSE, ENABLE_CURRENT_SENSE, &Wire1);
  temperatureSensor.begin();

  // init all messages
  power_board_temperature_msg.data = 0.0;
  power_teensy_temperature_msg.data = 0.0;
  power_thrusters_current_msg.data.data = (float *)malloc(8 * sizeof(float));
  power_batteries_voltage_msg.data.data = (float *)malloc(2 * sizeof(float));
  propulsion_microseconds_msg.data.data = (int16_t *)malloc(8 * sizeof(int16_t));

  power_thrusters_current_msg.data.size = 8;
  power_batteries_voltage_msg.data.size = 2;
  propulsion_microseconds_msg.data.size = 8;

  create_entities();
}

void power_loop() {
  if (micro_ros.pingAgent()) {
    computeAndPublish();
    micro_ros.spin();
    updateThrusters(microseconds);

  } else {
    updateThrusters(offCommand);
    micro_ros.destroyMicroROS();
    
    while (!micro_ros.pingAgent()) {

    }
    delay(10);
    create_entities();
  }
  
  delay(10);
}
