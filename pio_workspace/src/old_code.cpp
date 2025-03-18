Matt
mattou981
Do Not Disturb
in:#auv-electrical 
Matt Levasseur — 11/23/2024 1:51 PM
#include <ILI9341_t3.h>
#include <font_Arial.h> // from ILI9341_t3
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

#define CS_PIN  8
Expand
functionaltouchcode.ino
2 KB
Yuheng Liu — 11/23/2024 1:52 PM


#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>       // this is needed for display
#include <Adafruit_ILI9341.h>
#include <Arduino.h>      // this is needed for FT6206
Expand
message.txt
5 KB
Yuheng Liu — 1/18/2025 12:43 PM


#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>       // this is needed for display
#include <Adafruit_ILI9341.h>
#include <Arduino.h>      // this is needed for FT6206
Expand
message.txt
5 KB
Yuheng Liu — 1/18/2025 2:15 PM
#include <SPI.h>       // Needed for display
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>

// Pins and hardware SPI setup
#define TFT_DC 9
Expand
message.txt
3 KB
Celine Shao — 1/24/2025 5:23 PM
Hi guys are you able to meet tmrw? @Yuheng Liu @Matt Levasseur @Masa El-Soufi @Mia
Masa El-Soufi — 1/24/2025 5:25 PM
I can’t tomorrow I have an appointment…
But just to know what to we have left to do for the display?
Yuheng Liu — 1/24/2025 5:33 PM
Yur
Celine Shao — 1/24/2025 5:39 PM
We have the code for both pages, we just need to combine and figure out localized touch, and then do microros
Matt Levasseur — 1/24/2025 6:08 PM
I'll be there!
Mia — 1/24/2025 11:26 PM
Ill be there too Trying to figure out microros lol
John-Paul Chouery | Elec Lead — 1/24/2025 11:27 PM
Slight tip
Copy off the example code as is from here
https://github.com/micro-ROS/micro_ros_arduino/tree/jazzy/examples
GitHub
micro_ros_arduino/examples at jazzy · micro-ROS/micro_ros_arduino
micro-ROS library for Arduino. Contribute to micro-ROS/micro_ros_arduino development by creating an account on GitHub.
micro_ros_arduino/examples at jazzy · micro-ROS/micro_ros_arduino
On main from embedded repo
There is microros code that works for power board
U can also copy off
But it'll be too complicated since my code implements the publisher, subscriber, and reconnection example codes all at once
Go step by step instead please by copying from the link i sent u above 
Mia — 1/25/2025 11:24 AM
Thanks JP! We will try that!
Yuheng Liu — 1/25/2025 1:58 PM
#include <SPI.h>       
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>

// Pins and hardware SPI setup
#define TFT_DC 9
Expand
message.txt
4 KB
Celine Shao — 1/25/2025 2:17 PM
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// Pin Definitions
#define TFT_DC 9
Expand
display_board_code_jan_2025.ino
3 KB
Yuheng Liu — 1/25/2025 2:24 PM
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// Pin Definitions
#define TFT_DC 9
Expand
message.txt
3 KB
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// Pin Definitions
#define TFT_DC 9
Expand
message.txt
3 KB
Celine Shao — 1/28/2025 6:12 PM
#include <SPI.h>       
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>

// Pin Definitions
#define TFT_DC 9
Expand
combined_display_code_2025.ino
6 KB
Yuheng Liu — 2/4/2025 6:49 PM
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

// Define rotation constants
#define ILI9341_ROTATION_0 0
#define ILI9341_ROTATION_90 1
#define ILI9341_ROTATION_180 2
#define ILI9341_ROTATION_270 3

// Define the number of samples to use in the moving average
#define MOVING_AVERAGE_SAMPLES 10

// Initialize display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Initialize sensor object
MS5837 sensor;

// Initialize ROS node handle
ros::NodeHandle nh;

// Define global variables for ros handling
// Maintains newest version of thruster pwm values and status
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int Sthrusters[8];

// Define global variables for ros handling
// Maintains newest version of all variables
float batt_voltage_1_new = 0;
float batt_voltage_2_new = 0;
int thrusters_new[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int devices_new[] = { 0, 0, 0, 0, 0, 0, 0 };
String status_new = "Hello World!";
int tether_new = 0;

// Define global variables for display functions
// Maintains version of variables on screen
float voltages_old[] = { -1, -1 };
float voltages_new[] = { -1, -1 };
uint16_t batt_colours[] = { WHITE, WHITE };
float thrusters_old[] = { -1, -1, -1, -1, -1, -1, -1, -1 };
float devices_old[] = { -1, -1, -1, -1, -1, -1, -1 };
String status_old = "";
int tether_old = -1;
int dual_batt_old = -1;

// Status of batteries
boolean BATT1_EMPTY = false;
boolean BATT2_EMPTY = false;

// Arrays to store previous voltage readings for moving average
float voltage_buffer1[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index1 = 0;
float voltage_buffer2[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index2 = 0;

// Function to perform moving average filtering for battery 1
float movingAverage1(float newValue) {
  static float sum = 0;

  // Subtract oldest value from sum
  sum -= voltage_buffer2[voltage_buffer_index2];

  // Add new value to sum
  voltage_buffer2[voltage_buffer_index2] = newValue;
  sum += newValue;

  // Move to the next index in the buffer
  voltage_buffer_index2 = (voltage_buffer_index2 + 1) % MOVING_AVERAGE_SAMPLES;

  // Calculate and return the average
  return sum / MOVING_AVERAGE_SAMPLES;
... (440 lines left)
Collapse
message.txt
16 KB
Celine Shao — 2/11/2025 7:13 PM
most up to date code with new changes (removed dry test button, made all buttons bigger)
#include <SPI.h>       
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>

// Pin Definitions
#define TFT_DC 9
Expand
combined_display_code_2025.ino
6 KB
Image
Mia — 2/11/2025 10:20 PM
Yessirrrrr
John-Paul Chouery | Elec Lead — 2/11/2025 10:43 PM
We're you guys able to simulate the microros environment?
Matt Levasseur — 2/11/2025 11:40 PM
I installed microros but we haven't implemented microros in our code yet
John-Paul Chouery | Elec Lead — 2/11/2025 11:41 PM
Okay
To be honest guys
Considering you guys are early on the timeline
I'd like it if you can also implement the board in rosserial.
Either split up or do it successively.
Software is still in ros 1 so the board will stay in ros 1 till software updates
Matt Levasseur — 2/18/2025 7:10 PM
roscore --> starts ros
rosrun rosserial_python serial_node.py /dev/ttyACM0 --> starts node on USB device
rostopic list --> view list of topics
rostopic echo /thetopicname --> prints data to terminal
rostopic pub /sensors/imu/status std_msgs/Int32 "data: 1" --> publishes an integer to the specific topic name
Expand
roscore -- starts ros.txt
1 KB
^ running ros on windows and publishing data to the node from your computer
Matt Levasseur — 2/18/2025 7:19 PM
"attach, bind, and manage USB devices over IP in a Linux-based environment"

usbipd list # list available items
usbipd attach --busid "the device you want to attach" #makes the usb device available for the network
usbipd bind --busid "the device you want to attach" #bind the device for usb sharing
usbipd attach --wsl --busid "the device you want to attach" #for use with wsl in a linux env
usbipd.txt
1 KB
Yuheng Liu — 2/22/2025 11:30 AM
Good morning!
I caught a cold:(( but I will be working from home on the devices stuff for ros
Mia — 2/22/2025 11:59 AM
Feel better soon!
John-Paul Chouery | Elec Lead — 2/22/2025 12:07 PM
Hope u get better
Matt Levasseur — 2/22/2025 1:22 PM
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include "MS5837.h"
#include <ros.h>
Expand
sample_thrusters_rosserial.txt
5 KB
Matt Levasseur — 2/22/2025 2:24 PM
#Guide to Connecting USB port to Microros and creating node

(The administrator privilege terminal can be accessed from 

1. In a terminal WITH administrator privileges, run the following commands:
   1a. usbipd list # list available items
   1b. usbipd bind --busid <busid> #bind the device for usb sharing
   1c. usbipd attach --wsl --busid <busid> #for use with wsl in a linux env

Here you're connecting the device. Disconnect the device and reconnect it to see which one it is.

2. Open another terminal, then do the following:
   2a. Type in "wsl".
   2b. Type in roscore to initialize a ros session.
   2b. Type in lsusb to make sure that your device has been connected.
   2c. Run the following command as is (this is a python-based command and executable therefore it should be python): rosrun rosserial_python serial_node.py /dev/ttyACM0

NOTE: The ttyACMO may be specific to your device. To determine.

   2d. Run the following command: "rostopic list", to determine which topic you want to publish to
   2e. Run the following command: rostopic pub /sensors/imu/status std_msgs/Int32 "data: 1"

The above command allows you to pulish AS a topic to the display node, therefore allowing you to receive data and test the display. 

Collapse
#microros_guide.txt
2 KB
Yuheng Liu — 2/25/2025 7:01 PM
#include <SPI.h>       
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>

//UPDATE
#include <Wire.h>
Expand
display_ros_devices.ino
10 KB
not sure if it works for ros 100%
Mia — 3/1/2025 11:00 AM
Hey guys ill be like 30 min late, thus buses arent cooperating :(((
Celine Shao — 3/1/2025 11:42 AM
Sorry guys I’m not feeling too good I dont think I’ll make it today
Mia — 3/1/2025 12:01 PM
Alright celine no worries feel better soon! Ill
Try to install docker for the ros thing on my computer
Otherwise ill use the virtual macjine
The metro is getting delayed just my luck :/
Mia — 3/1/2025 12:45 PM
@Yuheng Liu you coming today?
Mia — 3/1/2025 1:40 PM
finally, able to run ROS, with docker at least I think and hope so
Image
Celine Shao — 3/10/2025 2:59 PM
hey guys i made a shared doc for the design report we could all fill for tonight
https://docs.google.com/document/d/1hRRRcjpRgbMND8BPO4sXv8UMrq2DM5CJAOeLVm_cqp0/edit?usp=sharing
Google Docs
Design Report - Display Board
Design Report Display Board Member Names (Mech + Elec + Software): Mia, Yuheng, Matt, Céline 1. Overview of your subteam’s project (1 paragraph) Briefly summarize your sub-team’s project in the AUV project 2. Progress + Key Accomplishments (6+ bullets + explanations) Write out an outline of the ...
Image
Celine Shao — 3/10/2025 8:33 PM
@Yuheng Liu @Matt Levasseur Mia and I filled in most of the parts but pls add if you think there’s more!
Mia — 3/10/2025 9:15 PM
hey guys i put the docs on the AUV documentation
you can always add or modify stuff
Celine Shao — 3/11/2025 6:46 PM
docker pull ros:noetic-ros-core
Celine Shao — 3/11/2025 7:54 PM
https://www.pjrc.com/store/display_ili9341_touch.html
Celine Shao — 3/15/2025 11:19 AM
Hey guys I’ll be there but like an hour late!
Mia — 3/15/2025 11:32 AM
👍
Yuheng Liu — 3/15/2025 11:54 AM
I just woke up lol
Be there soon
Mia — 3/15/2025 2:09 PM
socat -d -d pty,raw,echo=0,link=/tmp/ttyTeensy COM8
docker run -it --rm \
  --privileged \
  -v /tmp/ttyTeensy:/dev/ttyACM0 \
  ros:noetic
Yuheng Liu — 3/15/2025 2:09 PM
socat -d -d pty,raw,echo=0,link=/tmp/ttyTeensy COM8
Error: package 'rosserial_python' not found
Mia — 3/15/2025 2:14 PM
socat -d -d pty,raw,echo=0,link=/tmp/ttyTeensy /dev/tty.usbmodem12345
Yuheng Liu — 3/15/2025 2:16 PM
N PTY is /dev/ttys001
2025/03/15 14:15:26 socat[17517] E unlink("/tmp/ttyTeensy"): Operation not permitted
2025/03/15 14:15:26 socat[17517] N exit(1)
Mia — 3/15/2025 2:17 PM
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyTeensy /dev/tty.usbmodem12345
Yuheng Liu — 3/15/2025 2:18 PM
lrwxr-xr-x@ 1 root  wheel  11 15 Dec  2023 /tmp -> private/tmp
Mia — 3/15/2025 2:19 PM
ls -ld /private/tmp
sudo socat -d -d pty,raw,echo=0,link=$HOME/ttyTeensy /dev/tty.usbmodem12345
/dev/tty.usbmodem139422701 this the number of the teensy (that works we think)
anna joy — 3/15/2025 2:34 PM
JP wants you in medn now
Mia — 3/15/2025 3:08 PM
Yuheng
@Yuheng Liu
You forgot your charger
Yuheng Liu — 3/15/2025 3:12 PM
Oh it’s ok
I can get it Tuesday lol
Thx tho
Mia — 3/15/2025 5:18 PM
ok ok
Yuheng Liu — Today at 12:05 PM
@Mia r u going to be a trot this afternoon? I can’t make it to tonight’s work session anymore:( gotta study more for a midterm
Mia — Today at 12:06 PM
no i wont be unfortunately, right now Im close to the dollarama thats in like metro mcgill
wanna meet me there
your charger is in my bag
ill be there until like 2:15 ish pm
Yuheng Liu — Today at 12:11 PM
OkOk I can meet you there or McGill metro at 12:50
Thanks
Yuheng Liu — Today at 12:51 PM
@Mia I’m at McGill metro. How can I find u lol
Mia — Today at 12:53 PM
umm go towards les promenades smt where the dollorama is
then instead of going towards the dolloram
go the opposite way where lots of people are eating
Yuheng Liu — Today at 12:54 PM
Hmmm is it in eaton centre
Mia — Today at 12:54 PM
no
go towards the entrence of the eaton center
then turn left
Yuheng Liu — Today at 12:54 PM
Image
This dollars a
Mia — Today at 12:55 PM
i have no idea honestly since there are two dollaramas undergound
one in montreal trust
and another one in mcgill station
Yuheng Liu — Today at 12:55 PM
I’m so confused
Okay Saturday then lmaoooo
😂😂😂
R u going to the work session
Mia — Today at 12:56 PM
yeah I think ill go
﻿
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

// Define rotation constants
#define ILI9341_ROTATION_0 0
#define ILI9341_ROTATION_90 1
#define ILI9341_ROTATION_180 2
#define ILI9341_ROTATION_270 3

// Define the number of samples to use in the moving average
#define MOVING_AVERAGE_SAMPLES 10

// Initialize display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Initialize sensor object
MS5837 sensor;

// Initialize ROS node handle
ros::NodeHandle nh;

// Define global variables for ros handling
// Maintains newest version of thruster pwm values and status
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int Sthrusters[8];

// Define global variables for ros handling
// Maintains newest version of all variables
float batt_voltage_1_new = 0;
float batt_voltage_2_new = 0;
int thrusters_new[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int devices_new[] = { 0, 0, 0, 0, 0, 0, 0 };
String status_new = "Hello World!";
int tether_new = 0;

// Define global variables for display functions
// Maintains version of variables on screen
float voltages_old[] = { -1, -1 };
float voltages_new[] = { -1, -1 };
uint16_t batt_colours[] = { WHITE, WHITE };
float thrusters_old[] = { -1, -1, -1, -1, -1, -1, -1, -1 };
float devices_old[] = { -1, -1, -1, -1, -1, -1, -1 };
String status_old = "";
int tether_old = -1;
int dual_batt_old = -1;

// Status of batteries
boolean BATT1_EMPTY = false;
boolean BATT2_EMPTY = false;

// Arrays to store previous voltage readings for moving average
float voltage_buffer1[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index1 = 0;
float voltage_buffer2[MOVING_AVERAGE_SAMPLES];
int voltage_buffer_index2 = 0;

// Function to perform moving average filtering for battery 1
float movingAverage1(float newValue) {
  static float sum = 0;

  // Subtract oldest value from sum
  sum -= voltage_buffer2[voltage_buffer_index2];

  // Add new value to sum
  voltage_buffer2[voltage_buffer_index2] = newValue;
  sum += newValue;

  // Move to the next index in the buffer
  voltage_buffer_index2 = (voltage_buffer_index2 + 1) % MOVING_AVERAGE_SAMPLES;

  // Calculate and return the average
  return sum / MOVING_AVERAGE_SAMPLES;
}

// Function to perform moving average filtering for battery 2
float movingAverage2(float newValue) {
  static float sum = 0;

  // Subtract oldest value from sum
  sum -= voltage_buffer1[voltage_buffer_index1];

  // Add new value to sum
  voltage_buffer1[voltage_buffer_index1] = newValue;
  sum += newValue;

  // Move to the next index in the buffer
  voltage_buffer_index1 = (voltage_buffer_index1 + 1) % MOVING_AVERAGE_SAMPLES;

  // Calculate and return the average
  return sum / MOVING_AVERAGE_SAMPLES;
}

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

  // Tether and dual battery section
  for (int i = 0; i < 2; i++) {
    int x = i * WIDTH / 2;
    tft.drawRect(x, HEIGHT - 40, WIDTH / 2, 40, BLACK);
  }

  // Draws full rectangle on edges
  tft.drawRect(0, 0, WIDTH, HEIGHT, BLACK);
}

// Function to update battery 1 display
void batt1(float V1) {
  V1 = round(V1 * 10.0) / 10.0;
  V1 = movingAverage1(V1);
  V1 = round(V1 * 10.0) / 10.0;

  voltages_new[0] = V1;
  if (voltages_old[0] != voltages_new[0]) {
    voltages_old[0] = voltages_new[0];
    // if (V1 <= 12.8) {
    //   if (!BATT1_EMPTY) {
    //     tft.setTextColor(BLACK);
    //     batt_colours[0] = WHITE;
    //     tft.fillRect(1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[0]);
    //     tft.setCursor(8, 25);
    //     tft.setTextSize(5);
    //     tft.println("EMPTY");
    //     BATT1_EMPTY = true;
    //   }
    //   return;
    // } else 
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
    
    BATT1_EMPTY = false;
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
    // if (V2 <= 12.8) {
    //   if (!BATT2_EMPTY) {
    //     tft.setTextColor(BLACK);
    //     batt_colours[1] = WHITE;
    //     tft.fillRect(WIDTH / 2 + 1, 1, WIDTH / 2 - 2, HEIGHT / 3 - 2, batt_colours[1]);
    //     tft.setCursor(WIDTH / 2 + 8, 25);
    //     tft.setTextSize(5);
    //     tft.println("EMPTY");
    //     BATT2_EMPTY = true;
    //   }
    //   return;
    // } else 
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

    BATT2_EMPTY = false;
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

// Callback function for battery 1 message
void batt1MessageCallback(const std_msgs::Float32& msg) {
  batt_voltage_1_new = msg.data;
}

// Callback function for battery 2 message
void batt2MessageCallback(const std_msgs::Float32& msg) {
  batt_voltage_2_new = msg.data;
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

void tetherStatusMessageCallback(const std_msgs::Int32& msg) {
  tether_new = msg.data;
}

// Define publisher message variable
std_msgs::Float64 depth_msg;

// Define publishers and subscribers
// Publishes depth
// Subscribes to battery voltages, thruster microseconds, device statuses, status message, and tether status
ros::Publisher DEPTH("/sensors/depth/z", &depth_msg);
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Subscriber<std_msgs::Float32> BATT1("/display/batteries/voltage/1", &batt1MessageCallback);
ros::Subscriber<std_msgs::Float32> BATT2("/display/batteries/voltage/2", &batt2MessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEIMU("/sensors/imu/status", &devicesIMUMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDVL("/sensors/dvl/status", &devicesDVLMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEPS("/sensors/depth/status", &devicesPSMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEHYD("/sensors/hydrophones/status", &devicesHYDMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEACT("/sensors/actuator/status", &devicesACTMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEFC("/sensors/front_camera/status", &devicesFCMessageCallback);
ros::Subscriber<std_msgs::Int32> DEVICEDC("/sensors/down_camera/status", &devicesDCMessageCallback);
ros::Subscriber<std_msgs::String> STATUSMESSAGE("/mission_display", &statusMessageCallback);
ros::Subscriber<std_msgs::Int32> TETHER("/tether/status", &tetherStatusMessageCallback);

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

  // Initialize display
  tft.begin();
  tft.setRotation(ILI9341_ROTATION_270);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.fillScreen(WHITE);

  // Draw basic display structure
  initMainScreen();

  // Initialize ROS node handle
  nh.initNode();

  // Subscribe to ROS topics
  nh.subscribe(sub);
  nh.subscribe(BATT1);
  nh.subscribe(BATT2);
  nh.subscribe(DEVICEIMU);
  nh.subscribe(DEVICEDVL);
  nh.subscribe(DEVICEPS);
  nh.subscribe(DEVICEHYD);
  nh.subscribe(DEVICEACT);
  nh.subscribe(DEVICEFC);
  nh.subscribe(DEVICEDC);
  nh.subscribe(STATUSMESSAGE);
  nh.subscribe(TETHER);

  // Advertise ROS publisher
  nh.advertise(DEPTH);

  // Initializes voltage buffer arrays
  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
    voltage_buffer1[i] = 0;
    voltage_buffer2[i] = 0;
  }
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
  thrusters(Sthrusters[0], Sthrusters[1], Sthrusters[2], Sthrusters[3], Sthrusters[4], Sthrusters[5], Sthrusters[6], Sthrusters[7]);
  devices(devices_new[0], devices_new[1], devices_new[2], devices_new[3], devices_new[4], devices_new[5], devices_new[6]);
  status(status_new);
  tether_dual_battery(tether_new, batt_voltage_1_new, batt_voltage_2_new);

  // Delay for stability
  delay(10);
}
message.txt
16 KB