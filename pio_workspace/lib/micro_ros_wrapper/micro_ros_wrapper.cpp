#include "micro_ros_wrapper.h"

#include <Arduino.h>

// Publisher Implementation
Publisher::Publisher(rcl_node_t* node, const char* topic_name, const rosidl_message_type_support_t* type_support) {
    RCCHECK(rclc_publisher_init_default(&publisher_, node, type_support, topic_name));
}

void Publisher::publish(const void* msg) {
    RCSOFTCHECK(rcl_publish(&publisher_, msg, NULL));
}

// Subscriber Implementation
Subscriber::Subscriber(rcl_node_t* node, const char* topic_name, const rosidl_message_type_support_t* type_support, rclc_executor_t* executor, void (*callback)(const void*)) {
    RCCHECK(rclc_subscription_init_default(&subscriber_, node, type_support, topic_name));
    RCCHECK(rclc_executor_add_subscription(executor, &subscriber_, nullptr, callback, ON_NEW_DATA));
}

// MicroROSWrapper Implementation
MicroROSWrapper::MicroROSWrapper(const char* node_name) {
    set_microros_transports();
    allocator_ = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
    RCCHECK(rclc_node_init_default(&node_, node_name, "", &support_));
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));
}

Publisher MicroROSWrapper::createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support) {
    return Publisher(&node_, topic_name, type_support);
}

Subscriber MicroROSWrapper::createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*)) {
    return Subscriber(&node_, topic_name, type_support, &executor_, callback);
}

void MicroROSWrapper::spin() {
    RCCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100)));
}

void errorLoop() {
    int error = 0;

    while (error < 20) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);

        error++;
    }

    digitalWrite(LED_BUILTIN, HIGH);
}
