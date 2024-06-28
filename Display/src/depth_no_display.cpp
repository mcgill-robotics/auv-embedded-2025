#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
//#include <cmath>
#include <math.h>

// Define pin configurations
#define TFT_DC 9
#define TFT_CS 10

// Define rotation constants
#define ILI9341_ROTATION_0 0
#define ILI9341_ROTATION_90 1
#define ILI9341_ROTATION_180 2
#define ILI9341_ROTATION_270 3

// Initialize sensor object
MS5837 sensor;

// Initialize ROS node handle
ros::NodeHandle nh;

// Define publisher message variable
std_msgs::Float64 depth_msg;

// Define publishers and subscribers
// Publishes depth
// Subscribes to battery voltages, thruster microseconds, device statuses, status message, and tether status
ros::Publisher DEPTH("/sensors/depth/z", &depth_msg);

// Function to calculate and publish depth
void publish_depth() {
  depth_msg.data = sensor.depth();
  DEPTH.publish(&depth_msg);
}

void setup() {
  // Initialize I2C communication with sensor
  Wire.begin();
  sensor.init();
  delay(1000);

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);

  // Initialize ROS node handle
  nh.initNode();

  // Advertise ROS publisher
  nh.advertise(DEPTH);

}

void loop() {
  // Handle ROS communication
  nh.spinOnce();

  // Read sensor data and publish depth
  sensor.read();
  publish_depth();

  // Delay for stability
  delay(10);
}
