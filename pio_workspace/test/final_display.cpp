#include "SPI.h"
#include "Adafruit_GFX.h"
#include <iostream>
#include "Adafruit_ILI9341.h"
#include "XPT2046_Touchscreen.h"
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <cmath>

// Define pin configurations
#define TFT_DC 9
#define TFT_CS 10
#define TOUCH_CS 8
#define TOUCH_IRQ 2

// Create objects for display and touchscreen
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(TOUCH_CS);

// Colors for the rectangles
#define BATTERY_COLOR ILI9341_YELLOW
#define NUM_COLOR ILI9341_CYAN
#define LABEL_COLOR ILI9341_MAGENTA
#define MAIN_RECT_COLOR ILI9341_WHITE
#define DRY_TEST_COLOR ILI9341_GREEN
#define BACKGROUND_COLOR ILI9341_BLACK

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define DARK_GRAY 0x2104
#define LIGHT_GRAY 0xC618

// Display dimensions and rotation
#define HEIGHT 240
#define WIDTH 320
#define ILI9341_ROTATION_270 1

//UPDATE//
// Initialize sensor object
MS5837 sensor;

// Initialize ROS node handle
ros::NodeHandle nh;

// Define global variables for ros handling
int devices_new[] = { 0, 0, 0, 0, 0, 0, 0 };
int thrusters_new[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
// Define global variables for display functions
// Maintains version of variables on screen
float devices_old[] = { -1, -1, -1, -1, -1, -1, -1 };
float thrusters_old[] = { -1, -1, -1, -1, -1, -1, -1, -1 };

struct Button {
  int x, y, width, height;
  uint16_t color;
  String label;
};

// Declare buttons
Button buttons[] = {
  {0, 0, 155, 50, BATTERY_COLOR, "Battery"},
  {160, 0, 155, 50, BATTERY_COLOR, "Battery"},
  //{210, 10, 100, 50, DRY_TEST_COLOR, "Dry Test"}, // Dry Test button
  // Second Row
  {0, 55, 38, 50, NUM_COLOR, "1"},
  {40, 55, 38, 50, NUM_COLOR, "2"},
  {80, 55, 38, 50, NUM_COLOR, "3"},
  {120, 55, 38, 50, NUM_COLOR, "4"},
  {160, 55, 38, 50, NUM_COLOR, "5"},
  {200, 55, 38, 50, NUM_COLOR, "6"},
  {240, 55, 38, 50, NUM_COLOR, "7"},
  {280, 55, 38, 50, NUM_COLOR, "8"},
  // Third Row
  {0, 110, 44, 48, LABEL_COLOR, "IMU"},
  {46, 110, 44, 48, LABEL_COLOR, "P"},
  {92, 110, 44, 48, LABEL_COLOR, "H"},
  {138, 110, 44, 48, LABEL_COLOR, "A"},
  {184, 110, 44, 48, LABEL_COLOR, "FC"},
  {230, 110, 44, 48, LABEL_COLOR, "DC"},
  {276, 110, 44, 48, LABEL_COLOR, "DVL"},
  // Fourth Row
  {0, 160, 320, 30, MAIN_RECT_COLOR, "Touch 2nd row for surprise"},
  // Fifth Row
  {0, 195, 78, 40, ILI9341_BLUE, "Box1"},
  {80, 195, 78, 40, ILI9341_BLUE, "Box2"},
  {160, 195, 78, 40, ILI9341_BLUE, "Box3"},
  {240, 195, 78, 40, ILI9341_BLUE, "Box4"},
};

bool wasTouched = false; // Track previous touch state
bool isInDryTestMode = false; // Flag for Dry Test mode

int thruster_states[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // All thrusters initially off

// Main page setup
void initMainPage() {
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOR); // Main screen background
  // Draw buttons
  for (const Button &btn : buttons) {
    tft.fillRoundRect(btn.x, btn.y, btn.width, btn.height, 8, btn.color);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(btn.x + 5, btn.y + 10);
    tft.setTextSize(2);
    tft.print(btn.label);
  }
}

// Function to update thrusters display
void thrusters(int T1, int T2, int T3, int T4, int T5, int T6, int T7, int T8) {
  int temp_thrusters[] = { T1, T2, T3, T4, T5, T6, T7, T8 };
  uint16_t thruster_colors[] = {DARK_GRAY, CYAN};

  for (int i = 0; i < 8; i++) {
      uint16_t color = thruster_colors[temp_thrusters[i]];

      if (temp_thrusters[i] != thrusters_old[i]) {
        buttons_thrusters[i].color = color;
      }
    }

  for (const Button &btn : buttons_thrusters) {
    tft.fillRoundRect(btn.x, btn.y, btn.width, btn.height, 8, btn.color);
    tft.fillRoundRect(btn.x, btn.y, btn.width, btn.height, 8, btn.color);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(btn.x + 5, btn.y + 10);
    tft.setTextSize(2);
    tft.print(btn.label);
  }


  for (int i = 0; i < 8; i++) {
    thrusters_old[i] = temp_thrusters[i];
  }
}

// function to update thruster statuses
void thrusterStatus(int Sthrusters[]) {
  for (int i = 0; i < 8; i++) {
    if (microseconds[i] == 1500) {
      Sthrusters[i] = 0;
    } else {
      Sthrusters[i] = 1;
    }
  }
}

//UPDATE DEVICES DISPLAY//
void device(int IMU, int DVL, int PS, int HYD, int ACT, int FC, int DC) {
  int temp_devices[] = {IMU, DVL, PS, HYD, ACT, FC, DC};
  uint16_t device_colors[] = {RED, GREEN}; // RED = offline, GREEN = online
  int device_x[] = {0, 276, 46, 92, 138, 184, 230};
  int device_y = 110;
  int device_width = 44;
  int device_height = 48;

  for (int i = 0; i < 7; i++) {
    if (temp_devices[i] != devices_old[i]) {
      uint16_t color = device_colors[temp_devices[i]];
      tft.fillRoundRect(device_x[i], device_y, device_width, device_height, 8, color);

      // Re-draw text for visibility
      tft.setCursor(device_x[i] + 12, device_y + 15);
      tft.setTextColor(WHITE);
      tft.setTextSize(2);
      switch (i) {
        case 0: tft.print("IMU"); break;
        case 1: tft.print("DVL"); break;
        case 2: tft.print("P"); break;
        case 3: tft.print("H"); break;
        case 4: tft.print("A"); break;
        case 5: tft.print("FC"); break;
        case 6: tft.print("DC"); break;
      }
      devices_old[i] = temp_devices[i]; // Update only after successful draw
    }
  }
}

// Callback function that updates microseconds array with values from ros
void commandCb(const auv_msgs::ThrusterMicroseconds& tc){
  memcpy(microseconds, tc.microseconds, 8*sizeof(uint16_t));
  thrusterStatus(Sthrusters);
}

std_msgs::Float64 depth_msg;

// Callback function for IMU message
void devicesIMUMessageCallback(const std_msgs::Int32& msg) {
  devices_new[0] = msg.data;
}

// Callback function for DVL message
void devicesDVLMessageCallback(const std_msgs::Int32& msg) {
  devices_new[1] = msg.data;
}

// Callback function for Pressure Sensor message
void devicesPSMessageCallback(const std_msgs::Int32& msg) {
  devices_new[2] = msg.data;
}

// Callback function for Hydrophones message
void devicesHYDMessageCallback(const std_msgs::Int32& msg) {
  devices_new[3] = msg.data;
}

// Callback function for Actuator message
void devicesACTMessageCallback(const std_msgs::Int32& msg) {
  devices_new[4] = msg.data;
}

// Callback function for Front Camera message
void devicesFCMessageCallback(const std_msgs::Int32& msg) {
  devices_new[5] = msg.data;
}

// Callback function for Down Camera message
void devicesDCMessageCallback(const std_msgs::Int32& msg) {
  devices_new[6] = msg.data;
}

ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);

ros::Subscriber<std_msgs::Int32> DEVICEIMU("/sensors/imu/status", &devicesIMUMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDVL("/sensors/dvl/status", &devicesDVLMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEPS("/sensors/depth/status", &devicesPSMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEHYD("/sensors/hydrophones/status", &devicesHYDMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEACT("/sensors/actuator/status", &devicesACTMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEFC("/sensors/front_camera/status", &devicesFCMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDC("/sensors/down_camera/status", &devicesDCMessageCallback);

void display_setup() {
  tft.begin();
  ts.begin();
  ts.setRotation(1); // Ensure touch orientation is consistent
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.fillScreen(WHITE);

  initMainPage();

  // Initialize ROS node handle
  nh.initNode();

  // Subscribe to ROS topics
  nh.subscribe(sub);

  nh.subscribe(DEVICEIMU);
  nh.subscribe(DEVICEDVL);
  nh.subscribe(DEVICEPS);
  nh.subscribe(DEVICEHYD);
  nh.subscribe(DEVICEACT);
  nh.subscribe(DEVICEFC);
  nh.subscribe(DEVICEDC);
}

void display_loop() {
  static unsigned long lastRosUpdate = 0;

  if (millis() - lastRosUpdate > 100) {
    nh.spinOnce();
    lastRosUpdate = millis();
  }
  //update display with new data
  device(devices_new[0], devices_new[1], devices_new[2], devices_new[3], devices_new[4], devices_new[5], devices_new[6]);
  thrusters(Sthrusters[0], Sthrusters[1], Sthrusters[2], Sthrusters[3], Sthrusters[4], Sthrusters[5], Sthrusters[6], Sthrusters[7]);

  //delay for stability
  delay(10);
}




