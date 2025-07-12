#ifdef DISPLAY_H

#include "SPI.h"
#include "Adafruit_GFX.h"
#include <iostream>
#include "Adafruit_ILI9341.h"
#include "XPT2046_Touchscreen.h"
#include <SPI.h>       
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
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

// Colors
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

// Display dimensions
#define HEIGHT 240
#define WIDTH 320
#define ILI9341_ROTATION_270 1
#define THRUSTER_SPEED 1540
#define MOVING_AVERAGE_SAMPLES 10

// Initialize sensor object
MS5837 sensor;

// ROS Node Handle
ros::NodeHandle nh;

// ===== Battery/Tether Variables =====
int tether_new = 0;
int tether_old = -1;
int dual_batt_old = -1;
float batt_voltage_1_new = 0.0;
float batt_voltage_2_new = 0.0;
float voltages_old[] = { -1, -1 };
float voltages_new[] = { -1, -1 };
uint16_t batt_colours[] = { WHITE, WHITE };
float voltage_buffer1[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index1 = 0;
float voltage_buffer2[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index2 = 0;

// ===== Thruster/Devices Variables =====
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int Sthrusters[8];
int devices_new[] = { 0, 0, 0, 0, 0, 0, 0 };
int thrusters_new[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float devices_old[] = { -1, -1, -1, -1, -1, -1, -1 };
float thrusters_old[] = { -1, -1, -1, -1, -1, -1, -1, -1 };
bool wasTouched = false;
bool isInDryTestMode = false;
int thruster_states[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// ROS Messages
auv_msgs::ThrusterMicroseconds cmd_msg;
auv_msgs::ThrusterMicroseconds reset_cmd;
std_msgs::Float64 depth_msg;

// Button structure
struct Button {
  int x, y, width, height;
  uint16_t color;
  String label;
};

// Main page buttons
Button buttons[] = {
  {0, 0, 155, 50, BATTERY_COLOR, "0.0"},
  {160, 0, 155, 50, BATTERY_COLOR, "0.0"},
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
  {0, 200, 78, 35, BLUE, "T"},
  {80, 200, 78, 35, BLUE, "DB"},
  {160, 200, 78, 35, BLUE, "Box3"},
  {240, 200, 78, 35, BLUE, "Box4"},
};

// Thruster buttons for main page
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

// ===== ROS Publishers =====
ros::Publisher DEPTH("/sensors/depth/z", &depth_msg);
ros::Publisher pub("/propulsion/microseconds", &cmd_msg);

// ===== Battery/Tether Functions =====
void tetherStatusMessageCallback(const std_msgs::Int32& msg) {
  tether_new = msg.data;
}

void battery1Callback(const std_msgs::Float32& msg) {
  batt_voltage_1_new = msg.data;
}

void battery2Callback(const std_msgs::Float32& msg) {
  batt_voltage_2_new = msg.data;
}

void publish_depth() {
  depth_msg.data = sensor.depth();
  DEPTH.publish(&depth_msg);
}

void tether_dual_battery(float tether_status, float batt1_V, float batt2_V) {
  uint16_t custom_colors[] = { ILI9341_RED, ILI9341_GREEN };

  int temp_tether_status = tether_status;
  float battery_difference = batt2_V - batt1_V;
  bool temp_battery_status = (battery_difference > -0.05 && battery_difference < 0.05) && (batt1_V >= 12.8 && batt2_V >= 12.8);

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

float movingAverage1(float newValue) {
  static float sum = 0;
  sum -= voltage_buffer1[voltage_buffer_index1];
  voltage_buffer1[voltage_buffer_index1] = newValue;
  sum += newValue;
  voltage_buffer_index1 = (voltage_buffer_index1 + 1) % MOVING_AVERAGE_SAMPLES;
  return sum / MOVING_AVERAGE_SAMPLES;
}

float movingAverage2(float newValue) {
  static float sum = 0;
  sum -= voltage_buffer2[voltage_buffer_index2];
  voltage_buffer2[voltage_buffer_index2] = newValue;
  sum += newValue;
  voltage_buffer_index2 = (voltage_buffer_index2 + 1) % MOVING_AVERAGE_SAMPLES;
  return sum / MOVING_AVERAGE_SAMPLES;
}

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
      color = YELLOW;
    } else {
      color = GREEN;
    }

    buttons[0].color = color;
    tft.fillRoundRect(buttons[0].x, buttons[0].y, 
      buttons[0].width, buttons[0].height, 8, color);

    char buffer[6];
    dtostrf(V1, 4, 1, buffer);
    String voltageText = String(buffer) + "V";

    tft.setTextColor(WHITE);
    tft.setTextSize(3);

    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(voltageText, 0, 0, &x, &y, &w, &h);
    tft.setCursor(buttons[0].x + (buttons[0].width - w)/2, 
                 buttons[0].y + (buttons[0].height - h)/2);
    tft.print(voltageText);
  }
}

void batt2(float V2) {
  V2 = round(V2 * 10.0) / 10.0;
  V2 = movingAverage2(V2);
  V2 = round(V2 * 10.0) / 10.0;

  voltages_new[1] = V2;
  if (voltages_old[1] != voltages_new[1]) {
    voltages_old[1] = voltages_new[1];

    uint16_t color;
    if (V2 <= 14.8) {
      color = RED;
    } else if (V2 <= 15.8) {
      color = YELLOW;
    } else {
      color = GREEN;
    }

    buttons[1].color = color;
    tft.fillRoundRect(buttons[1].x, buttons[1].y, 
      buttons[1].width, buttons[1].height, 8, color);

    char buffer[6];
    dtostrf(V2, 4, 1, buffer);
    String voltageText = String(buffer) + "V";

    tft.setTextColor(WHITE);
    tft.setTextSize(3);

    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(voltageText, 0, 0, &x, &y, &w, &h);
    tft.setCursor(buttons[1].x + (buttons[1].width - w)/2, 
                 buttons[1].y + (buttons[1].height - h)/2);
    tft.print(voltageText);
  }
}

void thrusterStatus(int Sthrusters[]) {
  for (int i = 0; i < 8; i++) {
    if (microseconds[i] == 1500) {
      Sthrusters[i] = 0;
    } else {
      Sthrusters[i] = 1;
    }
  }
}

// ===== Thruster/Devices Functions =====
void commandCb(const auv_msgs::ThrusterMicroseconds& tc){
  if (isInDryTestMode) {
    return;
  }
  memcpy(microseconds, tc.microseconds, 8*sizeof(uint16_t));
  thrusterStatus(Sthrusters);
}

void devicesIMUMessageCallback(const std_msgs::Int32& msg) {
  devices_new[0] = msg.data;
}

void devicesDVLMessageCallback(const std_msgs::Int32& msg) {
  devices_new[1] = msg.data;
}

void devicesPSMessageCallback(const std_msgs::Int32& msg) {
  devices_new[2] = msg.data;
}

void devicesHYDMessageCallback(const std_msgs::Int32& msg) {
  devices_new[3] = msg.data;
}

void devicesACTMessageCallback(const std_msgs::Int32& msg) {
  devices_new[4] = msg.data;
}

void devicesFCMessageCallback(const std_msgs::Int32& msg) {
  devices_new[5] = msg.data;
}

void devicesDCMessageCallback(const std_msgs::Int32& msg) {
  devices_new[6] = msg.data;
}

// ===== ROS Subscribers =====
ros::Subscriber<std_msgs::Int32> sub_tether("/tether/status", &tetherStatusMessageCallback);
ros::Subscriber<std_msgs::Float32> BATT1("/power/voltage/battery/1", &battery1Callback);
ros::Subscriber<std_msgs::Float32> BATT2("/power/voltage/battery/2", &battery2Callback);
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Subscriber<std_msgs::Int32> DEVICEIMU("/sensors/imu/status", &devicesIMUMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDVL("/sensors/dvl/status", &devicesDVLMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEPS("/sensors/depth/status", &devicesPSMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEHYD("/sensors/hydrophones/status", &devicesHYDMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEACT("/sensors/actuator/status", &devicesACTMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEFC("/sensors/front_camera/status", &devicesFCMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDC("/sensors/down_camera/status", &devicesDCMessageCallback);

void thrusters(int T1, int T2, int T3, int T4, int T5, int T6, int T7, int T8) {
  if (isInDryTestMode) return;

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


void device(int IMU, int DVL, int PS, int HYD, int ACT, int FC, int DC) {
  int temp_devices[] = {IMU, DVL, PS, HYD, ACT, FC, DC};
  uint16_t device_colors[] = {RED, GREEN};
  int device_x[] = {0, 276, 46, 92, 138, 184, 230};
  int device_y = 110;
  int device_width = 44;
  int device_height = 48;

  for (int i = 0; i < 7; i++) {
    if (temp_devices[i] != devices_old[i]) {
      uint16_t color = device_colors[temp_devices[i]];
      tft.fillRoundRect(device_x[i], device_y, device_width, device_height, 8, color);

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
      devices_old[i] = temp_devices[i];
    }
  }
}

// ===== Dry Test Functions =====
void initializeThrusterMessages() {
  for (int i = 0; i < 8; i++) {
    cmd_msg.microseconds[i] = 1500;
    reset_cmd.microseconds[i] = 1500;
  }
}

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

    tft.fillRoundRect(x + 2, y + 2, 46, 46, 10, color);

    tft.setCursor(x + 15, y + 12);
    tft.setTextColor(DARK_GRAY);
    tft.setTextSize(2);
    tft.print(i + 1);
  }
}

void initDryTestPage() {
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOR);
  
  for (int i = 0; i < 8; i++) {
    int row = i / 4;
    int col = i % 4;
    int x = 43 + col * 60;
    int y = 65 + row * 60;
    tft.drawRoundRect(x, y, 50, 50, 10, LIGHT_GRAY);
  }

  tft.setCursor(WIDTH / 2 - 110, HEIGHT / 3 - 30);
  tft.setTextColor(LIGHT_GRAY);
  tft.setTextSize(1);
  tft.println("-------------Dry Test Mode-------------");

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

// ===== Display Functions =====
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

void handleTouch() {
  if (ts.touched()) {
    if (!wasTouched) {
      wasTouched = true;
      TS_Point p = ts.getPoint();
      p.x = map(p.x, 300, 4000, 320, 0);
      p.y = map(p.y, 200, 4000, 240, 0);
      int16_t x = p.x;  
      int16_t y = p.y;

      if (!isInDryTestMode) {
        // Main screen button press detection
        if (x >= 0 && x <= 78 && y >= 200 && y <= 235) { // Tether button
          tether_dual_battery(tether_new, batt_voltage_1_new, batt_voltage_2_new);
        }
        else if (x >= 80 && x <= 158 && y >= 200 && y <= 235) { // Dual Battery button
          tether_dual_battery(tether_new, batt_voltage_1_new, batt_voltage_2_new);
        }
        else if (x >= 0 && x <= 300 && y >= 60 && y <= 110) { // Dry Test button
          isInDryTestMode = true;
          initDryTestPage();
        }
      } else {
        // Dry Test Mode button presses
        if (x >= 8 && x <= 68 && y >= 10 && y <= 40) { // BACK button
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
              thruster_states[i] = !thruster_states[i];
              updateThrusters_page2();
              optimized_dry_test(i);
              thruster_states[i] = 0;
              updateThrusters_page2();
              break;
            }
          }
        }
      }
    }
  } else {
    wasTouched = false;
  }
}

void display_setup() {
  // Initialize I2C communication with sensor
  Wire.begin();
  sensor.init();
  delay(1000);

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);

  tft.begin();
  ts.begin();
  ts.setRotation(1);
  
  nh.initNode();
  nh.subscribe(BATT1);
  nh.subscribe(BATT2);
  nh.subscribe(sub_tether);
  nh.subscribe(sub);
  nh.subscribe(DEVICEIMU);
  nh.subscribe(DEVICEDVL);
  nh.subscribe(DEVICEPS);
  nh.subscribe(DEVICEHYD);
  nh.subscribe(DEVICEACT);
  nh.subscribe(DEVICEFC);
  nh.subscribe(DEVICEDC);

  nh.advertise(DEPTH);
  nh.advertise(pub);
  initializeThrusterMessages();

  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
    voltage_buffer1[i] = 0;
    voltage_buffer2[i] = 0;
  }

  initMainPage();
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
  
  // Read sensor data and publish depth
  sensor.read();
  publish_depth();

  // Update display with new data
  batt1(batt_voltage_1_new);
  batt2(batt_voltage_2_new);
  tether_dual_battery(tether_new, batt_voltage_1_new, batt_voltage_2_new);
  device(devices_new[0], devices_new[1], devices_new[2], devices_new[3], 
         devices_new[4], devices_new[5], devices_new[6]);
  thrusters(Sthrusters[0], Sthrusters[1], Sthrusters[2], Sthrusters[3], 
           Sthrusters[4], Sthrusters[5], Sthrusters[6], Sthrusters[7]);

  delay(10);
}

#endif