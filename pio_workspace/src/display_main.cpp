#include "SPI.h"
#include "Adafruit_GFX.h"
#include <iostream>
#include "Adafruit_ILI9341.h"
#include "XPT2046_Touchscreen.h"
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
#include <math.h>
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
#define THRUSTER_SPEED 1540

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

// ==== ROS ====
auv_msgs::ThrusterMicroseconds cmd_msg;
auv_msgs::ThrusterMicroseconds reset_cmd;
ros::Publisher pub("/propulsion/microseconds", &cmd_msg);

// Define global variables for ros handling
// Maintains newest version of thruster pwm values and status
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int Sthrusters[8];

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

// Declare Buttons for Thrusters
Button buttons_thrusters[] = {
  {0, 55, 38, 50, NUM_COLOR, "1"},
  {40, 55, 38, 50, NUM_COLOR, "2"},
  {80, 55, 38, 50, NUM_COLOR, "3"},
  {120, 55, 38, 50, NUM_COLOR, "4"},
  {160, 55, 38, 50, NUM_COLOR, "5"},
  {200, 55, 38, 50, NUM_COLOR, "6"},
  {240, 55, 38, 50, NUM_COLOR, "7"},
  {280, 55, 38, 50, NUM_COLOR, "8"},
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
  if (isInDryTestMode) return;  // Skip drawing if in dry test mode

  int temp_thrusters[] = { T1, T2, T3, T4, T5, T6, T7, T8 };
  uint16_t thruster_colors[] = {WHITE, CYAN};

  for (int i = 0; i < 8; i++) {
    uint16_t color = thruster_colors[temp_thrusters[i]];
    if (temp_thrusters[i] != thrusters_old[i]) {
      buttons_thrusters[i].color = color;
    }
  }

  for (const Button &btn : buttons_thrusters) {
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

// *** STUFF for second dry test page ***

// ==== Init ROS Messages ====
void initializeThrusterMessages() {
  for (int i = 0; i < 8; i++) {
    cmd_msg.microseconds[i] = 1500;
    reset_cmd.microseconds[i] = 1500;
  }
}

// ==== Dry Test ====
void optimized_dry_test(int t) {
  for (int i = 0; i < 8; i++) {
    if (i == t) {
      cmd_msg.microseconds[i] = THRUSTER_SPEED;
    } else {
      cmd_msg.microseconds[i] = 1500;
    }
  }

  if (nh.connected()) {
    pub.publish(&cmd_msg);
    delay(1000);
    pub.publish(&reset_cmd);
  } 
}

void updateThrusters_page2() {
  uint16_t thruster_colors[] = {DARK_GRAY, CYAN};

  for (int i = 0; i < 8; i++) {
    int row = i / 4;
    int col = i % 4;
    int x = 43 + col * 60;
    int y = 65 + row * 60;
    uint16_t color = thruster_colors[thruster_states[i]];

    // Update thruster display
    tft.fillRoundRect(x + 2, y + 2, 46, 46, 10, color);

    // Display thruster number
    tft.setCursor(x + 15, y + 12);
    tft.setTextColor(DARK_GRAY);
    tft.setTextSize(2);
    tft.print(i + 1);
  }
}

// Dry Test page setup
void initDryTestPage() {
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOR); // Dry Test page background
  

  // Draw thruster layout
  for (int i = 0; i < 8; i++) {
    int row = i / 4;
    int col = i % 4;
    int x = 43 + col * 60;
    int y = 65 + row * 60;
    tft.drawRoundRect(x, y, 50, 50, 10, LIGHT_GRAY); // Thruster button outline
  }

  // Title text
  tft.setCursor(WIDTH / 2 - 110, HEIGHT / 3 - 30);
  tft.setTextColor(LIGHT_GRAY);
  tft.setTextSize(1);
  tft.println("-------------Dry Test Mode-------------");

  // Back button layout
  int backButtonX = 8;
  int backButtonY = 10;
  int backButtonWidth = 60;
  int backButtonHeight = 30;
  tft.fillRoundRect(backButtonX, backButtonY, backButtonWidth, backButtonHeight, 5, LIGHT_GRAY);
  tft.drawRoundRect(backButtonX, backButtonY, backButtonWidth, backButtonHeight, 5, WHITE);
  tft.setCursor(backButtonX + 10, backButtonY + 8);
  tft.setTextColor(DARK_GRAY);
  tft.setTextSize(2);
  tft.print("BACK");

}
// *** ***

void handleTouch() {
   // Update touch detection

  if (ts.touched()) {
    if (!wasTouched) { // Only register a touch when first detected
      wasTouched = true;

      TS_Point p = ts.getPoint();

      // Use your specified mapping values
      p.x = map(p.x, 300, 4000, 320, 0);
      p.y = map(p.y, 200, 4000, 240, 0);
      int16_t x = p.x;  
      int16_t y = p.y;

      // Serial.print("Touch: ("); Serial.print(x);
      // Serial.print(", "); Serial.print(y);
      // Serial.println(")");

      if (!isInDryTestMode) {
        // Main screen button press detection
        if (x >= 0 && x <= 300 && y >= 60 && y <= 110) {  // Dry Test button
          isInDryTestMode = true;
          
          initDryTestPage();
        }
      } else {
        // Dry Test Mode button presses
        if (x >= 8 && x <= 68 && y >= 10 && y <= 40) { // BACK button on Dry Test page
          isInDryTestMode = false;
          initMainPage();
        } else {
          // Check thruster button presses
          for (int i = 0; i < 8; i++) {
            int row = i / 4;
            int col = i % 4;
            int x_start = 43 + col * 60;
            int y_start = 65 + row * 60;

            if (x >= x_start && x <= x_start + 50 && y >= y_start && y <= y_start + 50) {
              thruster_states[i] = !thruster_states[i]; // Toggle thruster state
              updateThrusters_page2();           // Show as "on"

              optimized_dry_test(i);     // Waits 1s internally

              thruster_states[i] = 0;
              updateThrusters_page2();           // Show as "off"
              break; // Exit after handling one touch
            }
          }
        }
      }
    }
  } else {
    wasTouched = false; // Reset flag when touch is released
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
  if (isInDryTestMode) {
    return;
  }
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

  nh.advertise(pub);
  initializeThrusterMessages();
}

void display_loop() {
  static unsigned long lastRosUpdate = 0;

  if (millis() - lastRosUpdate > 100) {
    if (!isInDryTestMode) {
      nh.spinOnce();
    }
    lastRosUpdate = millis();
  }
  handleTouch();
  //update display with new data
  device(devices_new[0], devices_new[1], devices_new[2], devices_new[3], devices_new[4], devices_new[5], devices_new[6]);
  thrusters(Sthrusters[0], Sthrusters[1], Sthrusters[2], Sthrusters[3], Sthrusters[4], Sthrusters[5], Sthrusters[6], Sthrusters[7]);

  //delay for stability
  delay(10);
}




