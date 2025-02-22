/*#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
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

// Define color constants
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// Define display dimensions
#define HEIGHT 240
#define WIDTH 320
#define ILI9341_ROTATION_270 1

// Initialize ROS node handle
ros::NodeHandle nh;

// Define global variables for ros handling
// Maintains newest version of thruster pwm values and status
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int Sthrusters[8];

// Define global variables for ros handling
// Maintains newest version of all variables
int thrusters_new[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// Define global variables for display functions
// Maintains version of variables on screen
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
void thrusters(int T1, int T2, i\nt T3, int T4, int T5, int T6, int T7, int T8) {
  int temp_thrusters[] = { T1, T2, T3, T4, T5, T6, T7, T8 };
  uint16_t thruster_colors[] = { RED, GREEN };

  for (int i = 0; i < 8; i++) {
    int x = i * 40; // to modify according to new disposition of the screen
    uint16_t color = thruster_colors[temp_thrusters[i]];

    if (temp_thrusters[i] != thrusters_old[i]) {
      buttons_thrusters[i].color = color
    }
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


// Callback function that updates microseconds array with values from ros
void commandCb(const auv_msgs::ThrusterMicroseconds& tc){
  memcpy(microseconds, tc.microseconds, 8*sizeof(uint16_t));
  thrusterStatus(Sthrusters);
}


// Define publisher message variable
std_msgs::Float64 depth_msg;

// Define publishers and subscribers
// Publishes depth
// Subscribes to battery voltages, thruster microseconds, device statuses, status message, and tether status

ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);

void setup() {
  // Initialize display
  tft.begin();
  ts.setRotation(1); // Ensure touch orientation is consistent
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.fillScreen(WHITE);

  // Draw basic display structure
  initMainPage();

  // Initialize ROS node handle
  nh.initNode();

  // Subscribe to ROS topics
  nh.subscribe(sub);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();

  // Update display with new data
  thrusters(Sthrusters[0], Sthrusters[1], Sthrusters[2], Sthrusters[3], Sthrusters[4], Sthrusters[5], Sthrusters[6], Sthrusters[7]);

  // Delay for stability
  delay(10);
}*/