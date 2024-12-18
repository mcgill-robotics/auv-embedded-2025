#ifndef MICRO_ROS_WRAPPER_H
#define MICRO_ROS_WRAPPER_H

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { errorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void errorLoop();

class Publisher {
public:
    Publisher(rcl_node_t* node, const char* topic_name, const rosidl_message_type_support_t* type_support);
    void publish(const void* msg);

private:
    rcl_publisher_t publisher_;
};

class Subscriber {
public:
    Subscriber(rcl_node_t* node, const char* topic_name, const rosidl_message_type_support_t* type_support, rclc_executor_t* executor, void (*callback)(const void*));
    
private:
    rcl_subscription_t subscriber_;
};

class MicroROSWrapper {
public:
    MicroROSWrapper(const char* node_name);
    void spin();
    Publisher createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support);
    Subscriber createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*));

private:
    rclc_executor_t executor_;
    rclc_support_t support_;
    rcl_allocator_t allocator_;
    rcl_node_t node_;
};

#endif
