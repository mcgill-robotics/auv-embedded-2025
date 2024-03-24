/*
  Simple Display fireware for ILI9341 LCD

*/
#include <ros.h>
// #include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 9
#define TFT_CS 10
bool update = true;
float voltage;
float oldVoltage;
long unsigned timer;
float dis_voltage;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

ros::NodeHandle nh;

void messageCb(const std_msgs::Float32 & voltage_level){
  voltage = voltage_level.data;
}
ros::Subscriber<std_msgs::Float32> sub_voltage("voltage_level", &messageCb);

 void setup() {
//-------------------------------------------
  nh.initNode();
  nh.subscribe(sub_voltage);
  tft.begin();
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(10, 10);
  tft.setTextSize(4);
}

void loop() {
  check_for_update();
  display();
  nh.spinOnce();
}

void check_for_update(){
  if(millis()-timer>=500){
    update = true;
  }
}

void display(){
  if(update){
    tft.setTextColor(ILI9341_WHITE);
    display_voltage(oldVoltage);
    tft.setTextColor(ILI9341_RED);
    display_voltage(voltage);
    timer = millis();
    update = false;
  }
}

void display_voltage(float voltageValue){
  tft.setCursor(70,120);
  if(voltageValue == voltage){
    oldVoltage = voltage;
  }
  tft.println(voltageValue);
}
