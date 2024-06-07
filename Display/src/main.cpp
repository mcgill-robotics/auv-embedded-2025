#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

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

// Define rotation constants
#define ILI9341_ROTATION_0 0
#define ILI9341_ROTATION_90 1
#define ILI9341_ROTATION_180 2
#define ILI9341_ROTATION_270 3

// Initialize display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Initialize sensor object
MS5837 sensor;

// Initialize ROS node handle
ros::NodeHandle nh;

// Define global variables for ros handling
// Maintains newest version of all variables
float batt_voltage_1_new = 0;
float batt_voltage_2_new = 0;
int thrusters_new[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int devices_new[] = { 0, 0, 0, 0, 0, 0, 0 };
String status_new = "I am Douglas!";
float quaternions_new[] = { 0, 0, 0, 0 };

// Define global variables for display functions
// Maintains version of variables on screen
float voltages_old[] = { -1, -1 };
float voltages_new[] = { -1, -1 };
uint16_t batt_colours[] = { WHITE, WHITE };
float thrusters_old[] = { -1, -1, -1, -1, -1, -1, -1, -1 };
float devices_old[] = { -1, -1, -1, -1, -1, -1, -1 };
String status_old = "";
String quaternions_old[] = { "", "", "", "" };

// Function to initialize main screen layout
void initMainScreen() {
  // Clears screen
  tft.fillScreen(WHITE);

  // Batteries 1 and 2 section
  tft.drawRect(0, 0, WIDTH / 2, HEIGHT / 3, BLACK);
  tft.drawRect(WIDTH / 2, 0, WIDTH / 2, HEIGHT / 3, BLACK);

  // Thrusters section
  for (int i = 0; i < 8; i++) {
    int x = i * 40;
    tft.drawRect(x, HEIGHT / 3, 40, 40, BLACK);
  }

  // Devices section
  for (int i = 0; i < 7; i++) {
    if (i == 0) {
      tft.drawRect(0, HEIGHT / 3 + 40, WIDTH / 7 + 3, WIDTH / 7, BLACK);
    } else if (i == 1) {
      tft.drawRect(3 + i * (WIDTH / 7), HEIGHT / 3 + 40, WIDTH / 7 + 3, WIDTH / 7, BLACK);
    } else {
      int x = 6 + i * (WIDTH / 7);
      tft.drawRect(x, HEIGHT / 3 + 40, WIDTH / 7, WIDTH / 7, BLACK);
    }
  }

  // Status message section
  tft.drawLine(0, HEIGHT / 3 + 40 + 45, WIDTH, HEIGHT / 3 + 40 + 45, BLACK);
  tft.drawLine(0, HEIGHT - 41, WIDTH, HEIGHT - 41, BLACK);

  // Quaternions section
  for (int i = 0; i < 4; i++) {
    int x = i * WIDTH / 4;
    tft.drawRect(x, HEIGHT - 40, WIDTH / 4, 40, BLACK);
    tft.setCursor(x + 2, HEIGHT - 38);
    tft.setTextSize(1);
    if (i == 0) {
      tft.print("W");
    } else if (i == 1) {
      tft.print("X");
    } else if (i == 2) {
      tft.print("Y");
    } else if (i == 3) {
      tft.print("Z");
    }
  }

  // Draws full rectangle on edges
  tft.drawRect(0, 0, WIDTH, HEIGHT, BLACK);
}

// Function to update battery 1 display
void batt1(float V1) {
  voltages_new[0] = V1;
  if (voltages_old[0] != voltages_new[0]) {
    voltages_old[0] = V1;
    if (V1 <= 12.6) {
      tft.setTextColor(BLACK);
      batt_colours[0] = WHITE;
      tft.fillRect(1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[0]);
      tft.setCursor(8, 25);
      tft.setTextSize(5);
      tft.println("EMPTY");
      return;
    } else if (V1 <= 14.8) {
      batt_colours[0] = RED;
    } else if (V1 <= 15.8) {
      batt_colours[0] = YELLOW;
    } else {
      batt_colours[0] = GREEN;
    }

    char buffer[6];
    dtostrf(V1, 4, 1, buffer);

    tft.fillRect(1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[0]);
    tft.setCursor(10, 20);
    tft.setTextSize(6);
    tft.println(buffer);
  }
}

// Function to update battery 2 display
void batt2(float V2) {
  voltages_new[1] = V2;
  if (voltages_old[1] != voltages_new[1]) {
    voltages_old[1] = V2;
    if (V2 <= 12.6) {
      tft.setTextColor(BLACK);
      batt_colours[1] = WHITE;
      tft.fillRect(WIDTH / 2 + 1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[1]);
      tft.setCursor(WIDTH / 2 + 8, 25);
      tft.setTextSize(5);
      tft.println("EMPTY");
      return;
    } else if (V2 <= 14.8) {
      batt_colours[1] = RED;
    } else if (V2 <= 15.8) {
      batt_colours[1] = YELLOW;
    } else {
      batt_colours[1] = GREEN;
    }

    char buffer[6];
    dtostrf(V2, 4, 1, buffer);

    tft.fillRect(WIDTH / 2 + 1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[1]);
    tft.setCursor(WIDTH / 2 + 10, 20);
    tft.setTextSize(6);
    tft.println(buffer);
  }
}

// Function to update thrusters display
void thrusters(int T1, int T2, int T3, int T4, int T5, int T6, int T7, int T8) {
  int temp_thrusters[] = { T1, T2, T3, T4, T5, T6, T7, T8 };
  uint16_t thruster_colors[] = { RED, GREEN };

  for (int i = 0; i < 8; i++) {
    int x = i * 40;
    uint16_t color = thruster_colors[temp_thrusters[i]];

    if (temp_thrusters[i] != thrusters_old[i]) {
      tft.fillRect(x + 1, HEIGHT / 3 + 1, 38, 38, color);

      tft.setCursor(x + 15, HEIGHT / 3 + 14);
      tft.setTextColor(WHITE);
      tft.setTextSize(2);
      tft.println(i + 1);
    }
  }

  for (int i = 0; i < 8; i++) {
    thrusters_old[i] = temp_thrusters[i];
  }
}

// Function to update devices display
void devices(int IMU, int DVL, int PS, int HYD, int ACT, int FC, int DC) {
  int temp_devices[] = { IMU, DVL, PS, HYD, ACT, FC, DC };
  uint16_t device_colors[] = { RED, GREEN };

  for (int i = 0; i < 7; i++) {
    uint16_t color = device_colors[temp_devices[i]];
    int x = 6 + i * (WIDTH / 7);

    if (temp_devices[i] != devices_old[i]) {
      if (i == 0) {
        tft.fillRect(1, HEIGHT / 3 + 40 + 1, WIDTH / 7 + 3 - 2, WIDTH / 7 - 2, color);
      } else if (i == 1) {
        tft.fillRect(3 + i * (WIDTH / 7) + 1, HEIGHT / 3 + 40 + 1, WIDTH / 7 + 3 - 2, WIDTH / 7 - 2, color);
      } else if (i == 6) {
        tft.fillRect(x + 1, HEIGHT / 3 + 40 + 1, WIDTH / 7 - 3, WIDTH / 7 - 2, color);
      } else {
        tft.fillRect(x + 1, HEIGHT / 3 + 40 + 1, WIDTH / 7 - 2, WIDTH / 7 - 2, color);
      }
    }

    tft.setTextColor(WHITE);
    tft.setTextSize(2);

    if (i == 0) {
      tft.setCursor(x, HEIGHT / 3 + 16 + 40);
      tft.println("IMU");
    } else if (i == 1) {
      tft.setCursor(x + 5, HEIGHT / 3 + 16 + 40);
      tft.println("DVL");
    } else if (i == 2) {
      tft.setCursor(x + 17, HEIGHT / 3 + 16 + 40);
      tft.println("P");
    } else if (i == 3) {
      tft.setCursor(x + 17, HEIGHT / 3 + 16 + 40);
      tft.println("H");
    } else if (i == 4) {
      tft.setCursor(x + 17, HEIGHT / 3 + 16 + 40);
      tft.println("A");
    } else if (i == 5) {
      tft.setCursor(x + 12, HEIGHT / 3 + 16 + 40);
      tft.println("FC");
    } else if (i == 6) {
      tft.setCursor(x + 12, HEIGHT / 3 + 16 + 40);
      tft.println("DC");
    }
  }

  for (int i = 0; i < 7; i++) {
    devices_old[i] = temp_devices[i];
  }
}

// Function to update message status display
void status(String status) {
  String temp_status = status;

  if (temp_status != status_old) {
    tft.fillRect(1, HEIGHT / 3 * 2 + 6, WIDTH - 2, 33, WHITE);

    tft.setCursor(5, HEIGHT / 3 * 2 + 10);
    tft.setTextColor(BLACK);
    tft.setTextSize(3);

    tft.println(status);
  }

  status_old = temp_status;
}

// Function to update quaternions display
void quaternions(float W, float X, float Y, float Z) {
  char bufferW[5], bufferX[5], bufferY[5], bufferZ[5];

  if (W == 1.0) {
    strcpy(bufferW, "1.0");
  } else {
    dtostrf(W, 3, 2, bufferW);
    if (bufferW[0] == '0') {
      memmove(bufferW, bufferW + 1, strlen(bufferW));
    }
  }

  if (X == 1.0) {
    strcpy(bufferX, "1.0");
  } else {
    dtostrf(X, 3, 2, bufferX);
    if (bufferX[0] == '0') {
      memmove(bufferX, bufferX + 1, strlen(bufferX));
    }
  }

  if (Y == 1.0) {
    strcpy(bufferY, "1.0");
  } else {
    dtostrf(Y, 3, 2, bufferY);
    if (bufferY[0] == '0') {
      memmove(bufferY, bufferY + 1, strlen(bufferY));
    }
  }

  if (Z == 1.0) {
    strcpy(bufferZ, "1.0");
  } else {
    dtostrf(Z, 3, 2, bufferZ);
    if (bufferZ[0] == '0') {
      memmove(bufferZ, bufferZ + 1, strlen(bufferZ));
    }
  }

  String temp_quaternions[] = { bufferW, bufferX, bufferY, bufferZ };

  for (int i = 0; i < 4; i++) {
    int x = i * WIDTH / 4;

    if (temp_quaternions[i] != quaternions_old[i]) {
      tft.fillRect(x + 1 + 8, HEIGHT - 40 + 1 + 6, WIDTH / 4 - 2 - 8, 38 - 6, WHITE);

      tft.setCursor(x + 5, HEIGHT - 33);
      tft.setTextColor(BLACK);
      tft.setTextSize(4);
      tft.println(temp_quaternions[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    quaternions_old[i] = temp_quaternions[i];
  }
}

// Callback function for battery 1 message
void batt1MessageCallback(const std_msgs::Float32& msg) {
  batt_voltage_1_new = msg.data;
}

// Callback function for battery 2 message
void batt2MessageCallback(const std_msgs::Float32& msg) {
  batt_voltage_2_new = msg.data;
}

// Callback function for thruster 1 message
void thruster1MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[0] = msg.data;
}

// Callback function for thruster 2 message
void thruster2MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[1] = msg.data;
}

// Callback function for thruster 3 message
void thruster3MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[2] = msg.data;
}
// Callback function for thruster 4 message
void thruster4MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[3] = msg.data;
}

// Callback function for thruster 5 message
void thruster5MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[4] = msg.data;
}

// Callback function for thruster 6 message
void thruster6MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[5] = msg.data;
}

// Callback function for thruster 7 message
void thruster7MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[6] = msg.data;
}

// Callback function for thruster 8 message
void thruster8MessageCallback(const std_msgs::Int32& msg) {
  thrusters_new[7] = msg.data;
}

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

// Callback function for status message
void statusMessageCallback(const std_msgs::String& msg) {
  status_new = msg.data;
}

// Callback function for quaternion W message
void quaternionWMessageCallback(const std_msgs::Float32& msg) {
  quaternions_new[0] = msg.data;
}

// Callback function for quaternion X message
void quaternionXMessageCallback(const std_msgs::Float32& msg) {
  quaternions_new[1] = msg.data;
}

// Callback function for quaternion Y message
void quaternionYMessageCallback(const std_msgs::Float32& msg) {
  quaternions_new[2] = msg.data;
}

// Callback function for quaternion Z message
void quaternionZMessageCallback(const std_msgs::Float32& msg) {
  quaternions_new[3] = msg.data;
}

// Define publisher message variable
std_msgs::Float64 depth_msg;

// Define publishers and subscribers
// Publishes depth
// Subscribes to battery voltages, thruster statuses, device statuses, status message, and quaternions
ros::Publisher DEPTH("/sensors/depth/z", &depth_msg);
ros::Subscriber<std_msgs::Float32> BATT1("/batteries/voltage/1", &batt1MessageCallback);
ros::Subscriber<std_msgs::Float32> BATT2("/batteries/voltage/2", &batt2MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER1("/thrusters/status/1", &thruster1MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER2("/thrusters/status/2", &thruster2MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER3("/thrusters/status/3", &thruster3MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER4("/thrusters/status/4", &thruster4MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER5("/thrusters/status/5", &thruster5MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER6("/thrusters/status/6", &thruster6MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER7("/thrusters/status/7", &thruster7MessageCallback);
ros::Subscriber<std_msgs::Int32> THRUSTER8("/thrusters/status/8", &thruster8MessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEIMU("/sensors/imu/status", &devicesIMUMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDVL("/sensors/dvl/status", &devicesDVLMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEPS("/sensors/depth/status", &devicesPSMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEHYD("/sensors/hydrophones/status", &devicesHYDMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEACT("/sensors/actuator/status", &devicesACTMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEFC("/sensors/front_camera/status", &devicesFCMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDC("/sensors/down_camera/status", &devicesDCMessageCallback);
ros::Subscriber<std_msgs::String> STATUSMESSAGE("/mission_display", &statusMessageCallback);
ros::Subscriber<std_msgs::Float32> QUATERNIONW("/quaternions/W", &quaternionWMessageCallback);
ros::Subscriber<std_msgs::Float32> QUATERNIONX("/quaternions/X", &quaternionXMessageCallback);
ros::Subscriber<std_msgs::Float32> QUATERNIONY("/quaternions/Y", &quaternionYMessageCallback);
ros::Subscriber<std_msgs::Float32> QUATERNIONZ("/quaternions/Z", &quaternionZMessageCallback);

// Function to calculate and publish depth
void publish_depth() {
  depth_msg.data = sensor.depth();
  DEPTH.publish(&depth_msg);
}

void setup() {
  // Initialize I2C communication with sensor
  Wire.begin();
  while (!sensor.init()) {
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);

  // Initialize display
  tft.begin();
  tft.setRotation(ILI9341_ROTATION_90);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.fillScreen(WHITE);

  // Draw basic display structure
  initMainScreen();

  // Initialize ROS node handle
  nh.initNode();

  // Subscribe to ROS topics
  nh.subscribe(BATT1);
  nh.subscribe(BATT2);
  nh.subscribe(THRUSTER1);
  nh.subscribe(THRUSTER2);
  nh.subscribe(THRUSTER3);
  nh.subscribe(THRUSTER4);
  nh.subscribe(THRUSTER5);
  nh.subscribe(THRUSTER6);
  nh.subscribe(THRUSTER7);
  nh.subscribe(THRUSTER8);
  nh.subscribe(DEVICEIMU);
  nh.subscribe(DEVICEDVL);
  nh.subscribe(DEVICEPS);
  nh.subscribe(DEVICEHYD);
  nh.subscribe(DEVICEACT);
  nh.subscribe(DEVICEFC);
  nh.subscribe(DEVICEDC);
  nh.subscribe(STATUSMESSAGE);
  nh.subscribe(QUATERNIONW);
  nh.subscribe(QUATERNIONX);
  nh.subscribe(QUATERNIONY);
  nh.subscribe(QUATERNIONZ);

  // Advertise ROS publisher
  nh.advertise(DEPTH);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();

  // Read sensor data and publish depth
  sensor.read();
  publish_depth();

  // Update display with new data
  batt1(batt_voltage_1_new);
  batt2(batt_voltage_2_new);
  thrusters(thrusters_new[0], thrusters_new[1], thrusters_new[2], thrusters_new[3], thrusters_new[4], thrusters_new[5], thrusters_new[6], thrusters_new[7]);
  devices(devices_new[0], devices_new[1], devices_new[2], devices_new[3], devices_new[4], devices_new[5], devices_new[6]);
  status(status_new);
  quaternions(quaternions_new[0], quaternions_new[1], quaternions_new[2], quaternions_new[3]);

  // Delay for stability
  delay(10);
}
