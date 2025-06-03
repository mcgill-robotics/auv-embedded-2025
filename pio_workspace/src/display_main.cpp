
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

//Define the number of samples to use in the moving average
#define MOVING_AVERAGE_SAMPLES 10//idk why we have this from old code

// ROS Node Handle
ros::NodeHandle nh;

//global variables for tether and dual battery
int tether_new = 0;
int tether_old = -1;
int dual_batt_old = -1;

// Global Variables for Battery Data
float batt_voltage_1_new = 0.0;
float batt_voltage_2_new = 0.0;


float voltages_old[] = { -1, -1 };
float voltages_new[] = { -1, -1 };
uint16_t batt_colours[] = { WHITE, WHITE };

// Arrays to store previous voltage readings for moving average
float voltage_buffer1[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index1 = 0;
float voltage_buffer2[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index2 = 0;

// --------- ROS Callbacks ----------
void tetherStatusMessageCallback(const std_msgs::Int32& msg) {
  tether_new = msg.data;
}

void battery1Callback(const std_msgs::Float32& msg) {
  batt_voltage_1_new = msg.data;
}

void battery2Callback(const std_msgs::Float32& msg) {
  batt_voltage_2_new = msg.data;
}

// --------- ROS Subscribers ----------
ros::Subscriber<std_msgs::Int32> sub_tether("/tether/status", &tetherStatusMessageCallback);
ros::Subscriber<std_msgs::Float32> BATT1("/battery1/voltage", &battery1Callback);
ros::Subscriber<std_msgs::Float32> BATT2("/battery2/voltage", &battery2Callback);

struct Button {
  int x, y, width, height;
  uint16_t color;
  String label;
};


Button buttons[] = {
  {0, 0, 155, 50, BATTERY_COLOR, "Battery 1"},
  {160, 0, 155, 50, BATTERY_COLOR, "Battery 2"},
  {0, 55, 38, 50, NUM_COLOR, "1"},
  {40, 55, 38, 50, NUM_COLOR, "2"},
  {80, 55, 38, 50, NUM_COLOR, "3"},
  {120, 55, 38, 50, NUM_COLOR, "4"},
  {160, 55, 38, 50, NUM_COLOR, "5"},
  {200, 55, 38, 50, NUM_COLOR, "6"},
  {240, 55, 38, 50, NUM_COLOR, "7"},
  {280, 55, 38, 50, NUM_COLOR, "8"},
  {0, 110, 44, 48, LABEL_COLOR, "IMU"},
  {46, 110, 44, 48, LABEL_COLOR, "P"},
  {92, 110, 44, 48, LABEL_COLOR, "H"},
  {138, 110, 44, 48, LABEL_COLOR, "A"},
  {184, 110, 44, 48, LABEL_COLOR, "FC"},
  {230, 110, 44, 48, LABEL_COLOR, "DC"},
  {276, 110, 44, 48, LABEL_COLOR, "DVL"},
  {0, 160, 320, 30, MAIN_RECT_COLOR, "Touch 2nd row for surprise"},
  {160, 200, 78, 35, BLUE, "Box3"},
  {240, 200, 78, 35, BLUE, "Box4"},
};

//functionsss

//DUAL BATTERY AND TETHER
void tether_dual_battery(float tether_status, float batt1_V, float batt2_V) {
  uint16_t custom_colors[] = { ILI9341_RED, ILI9341_GREEN };

  int temp_tether_status = tether_status;
  float battery_difference = batt2_V - batt1_V;
  bool temp_battery_status = battery_difference > 0.52 && battery_difference < 0.63;


  if (temp_tether_status != tether_old) {
    uint16_t tether_color = custom_colors[temp_tether_status];

    tft.fillRoundRect(0, 200, 78, 35, 6, tether_color);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 210);
    tft.print("T");
    
    tether_old = temp_tether_status;
  }

  if (temp_battery_status != dual_batt_old) {
    uint16_t dual_batt_color = custom_colors[temp_battery_status];

    tft.fillRoundRect(80, 200, 78, 35, 6, dual_batt_color);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.setCursor(90, 210);
    tft.print("DB");
    
    dual_batt_old = temp_battery_status;
  }
}

//BATTERY 
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

// Function to update battery 1 display
void batt1(float V1) {
  V1 = round(V1 * 10.0) / 10.0;
  V1 = movingAverage1(V1);
  V1 = round(V1 * 10.0) / 10.0;

  voltages_new[0] = V1;
  if (voltages_old[0] != voltages_new[0]) {
    voltages_old[0] = voltages_new[0];

    uint16_t color;
    if (V1 <= 14.8) {
      color = RED;
    } else if (V1 <= 15.8) {
      color= YELLOW;
    } else {
      color = GREEN;
    }

    buttons[0].color = color;
    tft.fillRoundRect(buttons[0].x, buttons[0].y, 
      buttons[0].width, buttons[0].height, 8, color);


    //display voltage text
    char buffer[6];
    dtostrf(V1, 4, 1, buffer);
    String voltageText = String(buffer) + "V";

  

    tft.setTextColor(WHITE);
    tft.setTextSize(3);

    // Calculate centered position
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(voltageText, 0, 0, &x, &y, &w, &h);
    tft.setCursor(buttons[0].x + (buttons[0].width - w)/2, 
                 buttons[0].y + (buttons[0].height - h)/2);
    tft.print(voltageText);
    
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

    // Determine color based on voltage
    uint16_t color;
    if (V2 <= 14.8) {
      color = RED;
    } else if (V2 <= 15.8) {
      color = YELLOW;
    } else {
      color = GREEN;
    }

    // Update the button color
    buttons[1].color = color;
    tft.fillRoundRect(buttons[1].x, buttons[1].y, 
      buttons[1].width, buttons[1].height, 8, color);


    //display voltage text
    char buffer[6];
    dtostrf(V2, 4, 1, buffer);
    String voltageText = String(buffer) + "V";


    tft.setTextColor(WHITE);
    tft.setTextSize(3);

    // Calculate centered position
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(voltageText, 0, 0, &x, &y, &w, &h);
    tft.setCursor(buttons[1].x + (buttons[1].width - w)/2, 
                 buttons[1].y + (buttons[1].height - h)/2);
    tft.print(voltageText);
  }
}


bool wasTouched = false;

/*
void handleTouch() {
  if (!ts.touched()) {
    wasTouched = false;
    return;
  }

  if (!wasTouched) {
    wasTouched = true;

    TS_Point p = ts.getPoint();
    int16_t x = map(p.y, 200, 3800, 0, tft.width());
    int16_t y = map(p.x, 200, 3800, 0, tft.height());
    //tether stuff
    if (x >= 0 && x <= 78 && y >= 200 && y <= 235) {
      tether_dual_battery(tether_new, 0.0, 0.0);
      return;
    }
    //dual battery
    if (x >= 80 && x <= 158 && y >= 200 && y <= 235) {
      tether_dual_battery(0, 11.1, 11.7);
      return;
    }

    for (const Button &btn : buttons) {
      if (x > btn.x && x < btn.x + btn.width && y > btn.y && y < btn.y + btn.height) {
        // Add logic here for each button if needed
      }
    }
  }
}
*/

void handleTouch() {
  if (!ts.touched()) {
    wasTouched = false;
    return;
  }

  if (!wasTouched) {
    wasTouched = true;
    TS_Point p = ts.getPoint();
    int16_t x = map(p.y, 200, 3800, 0, tft.width());
    int16_t y = map(p.x, 200, 3800, 0, tft.height());

    //tether button pressed uses latest tether/battery values
    if (x >= 0 && x <= 78 && y >= 200 && y <= 235) {
      tether_dual_battery(tether_new, batt_voltage_1_new, batt_voltage_2_new);
      return;
    }

    //dual Battery button pressed, uses latest battery voltages
    if (x >= 80 && x <= 158 && y >= 200 && y <= 235) {
      tether_dual_battery(tether_new, batt_voltage_1_new, batt_voltage_2_new);
      return;
    }

    // Other buttons default kinda
    for (const Button &btn : buttons) {
      if (x > btn.x && x < btn.x + btn.width && y > btn.y && y < btn.y + btn.height) {
        // Handle other buttons if needed
      }
    }
  }
}

void initMainPage() {
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOR);

  for (const Button &btn : buttons) {
    tft.fillRoundRect(btn.x, btn.y, btn.width, btn.height, 8, btn.color);
    tft.setTextColor(BLACK);
    tft.setTextSize(2);

    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(btn.label, 0, 0, &x, &y, &w, &h);
    tft.setCursor(btn.x + (btn.width - w) / 2, btn.y + (btn.height - h) / 2);
    tft.print(btn.label);
  }
}


void display_setup() {
  tft.begin();
  ts.begin();
  ts.setRotation(1);

  nh.initNode();
  nh.subscribe(BATT1);
  nh.subscribe(BATT2);
  nh.subscribe(sub_tether);

  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
    voltage_buffer1[i] = 0;
    voltage_buffer2[i] = 0;
  }

  initMainPage();
}

void display_loop() {
  handleTouch();
  nh.spinOnce();
  batt1(batt_voltage_1_new);
  batt2(batt_voltage_2_new);
  delay(10);
}

