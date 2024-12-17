#include "power_micro_ros.h"

rcl_subscription_t propulsion_microseconds_subscriber;
std_msgs__msg__Int16MultiArray propulsion_microseconds_msg;

rcl_publisher_t power_board_temperature_publisher;
std_msgs__msg__Float32 power_board_temperature_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void micro_ros_init() {
    set_microros_transports();

    allocator = rcl_get_default_allocator();

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "power_node", "", &support));

    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &propulsion_microseconds_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/propulsion/microseconds"));

    propulsion_microseconds_msg.data.data = (int16_t *)malloc(8 * sizeof(int16_t));
    propulsion_microseconds_msg.data.size = 8;
    propulsion_microseconds_msg.data.capacity = 8;

    // create publisher
  RCCHECK(rclc_publisher_init_default(
    &power_board_temperature_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/power/board/temperature"));

    power_board_temperature_msg.data = 0.0;

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &propulsion_microseconds_subscriber, &propulsion_microseconds_msg, &propulsion_microseconds_subscription_callback, ON_NEW_DATA));
}

void spin_micro_ros() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void error_loop() {
  // Ensure msg.data.data is allocated before freeing
  if (propulsion_microseconds_msg.data.data != NULL) {
    free(propulsion_microseconds_msg.data.data);
  }

  power_board_temperature_msg.data = 0.0;

  int error = 0;
  
  while(error <= 20) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);

    error += 1;
  }

  digitalWrite(LED_PIN, HIGH);
}

void power_board_temperature_publish(float power_board_temperature) {
  power_board_temperature_msg.data = power_board_temperature;
  RCSOFTCHECK(rcl_publish(&power_board_temperature_publisher, &power_board_temperature_msg, NULL));
}

void propulsion_microseconds_subscription_callback(const void * msgin) {
    const std_msgs__msg__Int16MultiArray * propulsion_microseconds_msg = (const std_msgs__msg__Int16MultiArray *)msgin;

    // Ensure we don't exceed the size of the `microseconds` array
    for (size_t i = 0; i < 8 && i < propulsion_microseconds_msg->data.size; i++) {
        microseconds[i] = propulsion_microseconds_msg->data.data[i]; // Access the data correctly
    }
}
