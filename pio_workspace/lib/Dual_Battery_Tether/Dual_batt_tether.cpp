#include <SPI.h>       
#include <Adafruit_ILI9341.h>
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

// Forward declarations (tell the compiler these functions exist)
void initMainPage();

void handleTouch();

// Pin Definitions
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

// Define display dimensions
#define HEIGHT 240
#define WIDTH 320
#define ILI9341_ROTATION_270 1

// ROS Node Handle
ros::NodeHandle nh;

//global variables for tether and dual battery
int tether_new = 0;
int tether_old = -1;
int dual_batt_old = -1;

// Function to update tether and dual battery status
void tether_dual_battery(float tether_status, float batt1_V, float batt2_V) {
  uint16_t custom_colors[] = { RED, GREEN };

  int temp_tether_status = tether_status;

  if (temp_tether_status != tether_old) {
    uint16_t tether_color = custom_colors[temp_tether_status];

    tft.fillRect(1, HEIGHT - 40 + 1, WIDTH / 2 - 2, 40 - 2, tether_color);

    tft.setTextColor(WHITE);
    tft.setTextSize(3);

    tft.setCursor(26, HEIGHT - 30);
    tft.println("TETHER");
    
    tether_old = temp_tether_status;
  }

  float battery_difference = batt2_V - batt1_V;
  bool temp_battery_status = battery_difference > 0.52 && battery_difference < 0.63;

  if (temp_battery_status != dual_batt_old) {
    uint16_t dual_batt_color = custom_colors[temp_battery_status];

    tft.fillRect(1 + WIDTH / 2, HEIGHT - 40 + 1, WIDTH / 2 - 2, 40 - 2, dual_batt_color);

    tft.setTextColor(WHITE);
    tft.setTextSize(3);

    tft.setCursor(8 + WIDTH / 2, HEIGHT - 30);
    tft.println("DUAL BAT");
    
    dual_batt_old = temp_battery_status;
  }
}

// Callback function for tether

void tetherStatusMessageCallback(const std_msgs::Int32& msg) {
  tether_new = msg.data;
}

//define subscribers
ros::Subscriber<std_msgs::Int32> TETHER("/tether/status", &tetherStatusMessageCallback);

struct Button {
  int x, y, width, height;
  uint16_t color;
  String label;
};

// Declare buttons
Button buttons[] = {
  {0, 0, 155, 50, BATTERY_COLOR, "Battery 1"},
  {160, 0, 155, 50, BATTERY_COLOR, "Battery 2"},
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
  {0, 195, 78, 40, ILI9341_BLUE, "DUAL BATT "},
  {80, 195, 78, 40, ILI9341_BLUE, "TETHER"},
  {160, 195, 78, 40, ILI9341_BLUE, "Box3"},
  {240, 195, 78, 40, ILI9341_BLUE, "Box4"},
};

bool wasTouched = false; // Track previous touch state


void handleTouch() {
  if (!ts.touched()) return;

  TS_Point p = ts.getPoint();
  int x = map(p.y, 200, 3800, 0, tft.width());
  int y = map(p.x, 200, 3800, 0, tft.height());

  for (int i = 0; i < sizeof(buttons) / sizeof(Button); i++) {
    Button btn = buttons[i];
    if (x > btn.x && x < btn.x + btn.width && y > btn.y && y < btn.y + btn.height) {
      if (btn.label == "TETHER") {
        tether_dual_battery(tether_new, 0.0, 0.0); // battery values don't matter here
      } else if (btn.label == "DUAL BATT ") {
        // simulate battery input for visual feedback
        tether_dual_battery(0, 11.1, 11.7);  // test values, tweak as needed
      }
    }
  }
}

void initMainPage() {
  tft.fillScreen(BACKGROUND_COLOR);

  for (int i = 0; i < sizeof(buttons) / sizeof(Button); i++) {
    Button btn = buttons[i];
    tft.fillRect(btn.x, btn.y, btn.width, btn.height, btn.color);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.setCursor(btn.x + 5, btn.y + 15);
    tft.print(btn.label);
  }

  // Draw initial tether/dual battery areas
  tether_dual_battery(tether_new, 0.0, 0.0); // initial display
}

void display_setup() {
  tft.begin();
  tft.setRotation(ILI9341_ROTATION_270);
  ts.begin();
  ts.setRotation(1);

  nh.initNode();
  nh.subscribe(TETHER);

  initMainPage();
}

void display_loop() {
  handleTouch();
  nh.spinOnce();
  delay(10);  // prevent overload
}

//this code doesnt have the real battery voltages, later when we combine ev