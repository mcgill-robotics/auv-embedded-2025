#include "micro_ros_wrapper.h"

#include <Arduino.h>

// MicroROSWrapper Implementation
MicroROSWrapper::MicroROSWrapper(const char* node_name) {
    set_microros_transports();
    allocator_ = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
    RCCHECK(rclc_node_init_default(&node_, node_name, "", &support_));
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));
}

rcl_publisher_t* MicroROSWrapper::createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support) {
    RCCHECK(rclc_publisher_init_default(&publishers_[publisher_count], &node_, type_support, topic_name));
    publisher_count++;
    return &publishers_[publisher_count - 1];  // Return a pointer to the publisher
}


void MicroROSWrapper::publishData(const void* msg, rcl_publisher_t &publisher) {
    RCSOFTCHECK(rcl_publish(&publisher, msg, NULL));
}

rcl_subscription_t* MicroROSWrapper::createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*)) {
    RCCHECK(rclc_subscription_init_default(&subscriptions_[subscription_count], &node_, type_support, topic_name));
    RCCHECK(rclc_executor_add_subscription(&executor_, &subscriptions_[subscription_count], nullptr, callback, ON_NEW_DATA));
    subscription_count++;
    return &subscriptions_[subscription_count - 1];
}

void MicroROSWrapper::spin() {
    RCCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100)));
}

void MicroROSWrapper::errorLoop() {
    int error = 0;

    while (error < 20) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);

        error++;
    }

    digitalWrite(LED_BUILTIN, HIGH);
}
