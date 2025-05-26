#include <SPI.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <cmath>

// ==== ROS ====
ros::NodeHandle nh;
auv_msgs::ThrusterMicroseconds cmd_msg;
auv_msgs::ThrusterMicroseconds reset_cmd;
ros::Publisher pub("/propulsion/microseconds", &cmd_msg);

// ==== Display + Touch ====
#define TFT_DC 9
#define TFT_CS 10
#define CS_PIN 8
#define TIRQ_PIN 2

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(CS_PIN);

// ==== Colors ====
#define BLACK 0x0000
#define DARK_GRAY 0x2104
#define LIGHT_GRAY 0xC618
#define CYAN 0x07FF
#define WHITE 0xFFFF

// ==== Screen ====
#define HEIGHT 240
#define WIDTH 320
#define ILI9341_ROTATION_270 1

#define THRUSTER_SPEED 1540

int thruster_states[8] = {0};
bool wasTouched = false;

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
    delay(3000);
    pub.publish(&reset_cmd);
  } 
}

// ==== UI ====
void initMainScreen() {
  tft.fillScreen(DARK_GRAY);
  for (int i = 0; i < 3; i++) {
    tft.drawRoundRect(1 + i, 1 + i, WIDTH - 2 * i, HEIGHT - 2 * i, 15, LIGHT_GRAY);
  }
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

  tft.fillRoundRect(8, 10, 60, 30, 5, LIGHT_GRAY);
  tft.drawRoundRect(8, 10, 60, 30, 5, WHITE);
  tft.setCursor(18, 18);
  tft.setTextColor(DARK_GRAY);
  tft.setTextSize(2);
  tft.print("BACK");
}

void drawThrusters() {
  uint16_t colors[] = {DARK_GRAY, CYAN};
  for (int i = 0; i < 8; i++) {
    int row = i / 4;
    int col = i % 4;
    int x = 43 + col * 60;
    int y = 65 + row * 60;
    uint16_t color = colors[thruster_states[i]];
    tft.fillRoundRect(x + 2, y + 2, 46, 46, 10, color);
    tft.setCursor(x + 15, y + 12);
    tft.setTextColor(DARK_GRAY);
    tft.setTextSize(2);
    tft.print(i + 1);
  }
}

// ==== Touch ====
void handleTouch() {
  if (ts.touched()) {
    if (!wasTouched) {
      wasTouched = true;
      TS_Point p = ts.getPoint();
      p.x = map(p.x, 300, 4000, WIDTH, 0);
      p.y = map(p.y, 200, 4000, HEIGHT, 0);

      for (int i = 0; i < 8; i++) {
        int row = i / 4;
        int col = i % 4;
        int x_start = 43 + col * 60;
        int y_start = 65 + row * 60;

        if (p.x >= x_start && p.x <= x_start + 50 &&
            p.y >= y_start && p.y <= y_start + 50) {
          thruster_states[i] = 1;
          drawThrusters();           // Show as "on"

          optimized_dry_test(i);     // Waits 1s internally

          thruster_states[i] = 0;
          drawThrusters();           // Show as "off"
          break;                     
        }
      }
    }
  } else {
    wasTouched = false;
  }
}


// ==== Setup ====
void display_setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub);
  initializeThrusterMessages();

  tft.begin();
  ts.begin();
  ts.setRotation(1);
  tft.setRotation(ILI9341_ROTATION_270);
  initMainScreen();
  drawThrusters();
}

// ==== Loop ====
void display_loop() {
  handleTouch();
  nh.spinOnce();
  delay(10);
}
