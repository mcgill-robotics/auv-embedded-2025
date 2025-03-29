#include <SPI.h>       
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>

//UPDATE MIA
//including ROS libraries
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <cmath>



// Pin Definitions
#define TFT_DC 9
#define TFT_CS 10
#define TOUCH_CS 8
#define TOUCH_IRQ 2

// Create objects for display and touchscreen
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
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

//UPDATE MIA 
//Define the number of samples to use in the moving average
#define MOVING_AVERAGE_SAMPLES 10

// ROS Node Handle
ros::NodeHandle nh;

// Global Variables for Battery Data
float batt_voltage_1_new = 0;
float batt_voltage_2_new = 0;
float voltages_old[] = { -1, -1 };
float voltages_new[] = { -1, -1 };
uint16_t batt_colours[] = { WHITE, WHITE };

// Arrays to store previous voltage readings for moving average
float voltage_buffer1[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index1 = 0;
float voltage_buffer2[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index2 = 0;

// Function to perform moving average filtering for battery 1
float movingAverage1(float newValue) {
  static float sum = 0;
  sum -= voltage_buffer1[voltage_buffer_index1];
  voltage_buffer1[voltage_buffer_index1] = newValue;
  sum += newValue;
  voltage_buffer_index1 = (voltage_buffer_index1 + 1) % MOVING_AVERAGE_SAMPLES;
  return sum / MOVING_AVERAGE_SAMPLES;
}


// Function to perform moving average filtering for battery 2
float movingAverage2(float newValue) {
  static float sum = 0;
  sum -= voltage_buffer2[voltage_buffer_index2];
  voltage_buffer2[voltage_buffer_index2] = newValue;
  sum += newValue;
  voltage_buffer_index2 = (voltage_buffer_index2 + 1) % MOVING_AVERAGE_SAMPLES;
  return sum / MOVING_AVERAGE_SAMPLES;
}

// Callback function for battery 1 message
void batt1MessageCallback(const std_msgs::Float32& msg) {
  batt_voltage_1_new = msg.data;
}

// Callback function for battery 2 message
void batt2MessageCallback(const std_msgs::Float32& msg) {
  batt_voltage_2_new = msg.data;
}

// ROS Subscribers
ros::Subscriber<std_msgs::Float32> BATT1("/display/batteries/voltage/1", &batt1MessageCallback);
ros::Subscriber<std_msgs::Float32> BATT2("/display/batteries/voltage/2", &batt2MessageCallback);

// Function to update battery 1 display
void batt1(float V1) {
  V1 = round(V1 * 10.0) / 10.0;
  V1 = movingAverage1(V1);
  V1 = round(V1 * 10.0) / 10.0;

  voltages_new[0] = V1;
  if (voltages_old[0] != voltages_new[0]) {
    voltages_old[0] = voltages_new[0];
    if (V1 <= 14.8) {
      batt_colours[0] = RED;
    } else if (V1 <= 15.8) {
      batt_colours[0] = YELLOW;
    } else {
      batt_colours[0] = GREEN;
    }

    char buffer[6];
    dtostrf(V1, 4, 1, buffer);

    tft.setTextColor(WHITE);
    tft.fillRect(1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[0]);
    tft.setCursor(10, 20);
    tft.setTextSize(6);
    tft.println(buffer);
  }
}

// Function to update battery 2 display
void batt2(float V2) {
  V2 = round(V2 * 10.0) / 10.0;
  V2 = movingAverage2(V2);
  V2 = round(V2 * 10.0) / 10.0;

  voltages_new[1] = V2;
  if (voltages_old[1] != voltages_new[1]) {
    voltages_old[1] = voltages_new[1];
    if (V2 <= 14.8) {
      batt_colours[1] = RED;
    } else if (V2 <= 15.8) {
      batt_colours[1] = YELLOW;
    } else {
      batt_colours[1] = GREEN;
    }

    char buffer[6];
    dtostrf(V2, 4, 1, buffer);

    tft.setTextColor(WHITE);
    tft.fillRect(WIDTH / 2 + 1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[1]);
    tft.setCursor(WIDTH / 2 + 10, 20);
    tft.setTextSize(6);
    tft.println(buffer);
  }
}
//END OF UPDATE MIA 


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

      Serial.print("Touch: ("); Serial.print(x);
      Serial.print(", "); Serial.print(y);
      Serial.println(")");

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
              updateThrusters();
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
  tft.println("Dry Test Mode");

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

  // Update thruster display
  updateThrusters();
}

// Update thruster display
void updateThrusters() {
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

void setup() {
  Serial.begin(38400);  // For debugging
  tft.begin();
  ts.begin();
  ts.setRotation(1); // Ensure touch orientation is consistent

  //UPDATE MIA
  // Initialize ROS node handle
  nh.initNode();

  //subscribe to ROS TOPIC
  nh.subscribe(BATT1);
  nh.subscribe(BATT2);

  // Initialize voltage buffer arrays
  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
    voltage_buffer1[i] = 0;
    voltage_buffer2[i] = 0;
  }

  //END UPDATE MIA



  initMainPage();
}

void loop() {
  //Mia update
  handleTouch();

  //UPDATE MIA 
  nh.spinOnce();

  // Update display with new data
  batt1(batt_voltage_1_new);
  batt2(batt_voltage_2_new);

  //assuming that bat1 and batt2 are std_msgs::Float32  publishers
  //batt1_msg.data = batt_voltage_1_new;
  //batt2_msg.data = batt_voltage_2_new;

  //batt1.publish(&batt1_msg);
  //batt2.publish(&batt2_msg);

  //END UPDATE MIA 
  delay(10);
  //handleTouch();
}