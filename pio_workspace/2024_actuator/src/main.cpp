#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


#define CURRENT_PIN 16
#define SERVO_PIN 11

// constant data types
bool contact = 0;
bool closing = 0;
bool new_close_msg = 0;

// variables
int contact_current = 20;
int max_position = 180;
int pos = 0;    // variable to store the servo position 
float current = 0;

// set messagd types
std_msgs::Bool grabber_contact_msg;
std_msgs::Bool close_msg;

// update var from subscribed message
void update_close_msg( const std_msgs::Bool &close){
  closing = close.data;
  new_close_msg = true;
}

// set ros pub/subscriber
ros::NodeHandle nh;
ros::Publisher grabber_contact("contact", &grabber_contact_msg);
ros::Subscriber<std_msgs::Bool> sub_close("close", &update_close_msg);

// create servo object
Servo servo1;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 

unsigned long previousMillis = 0; // stores last time the servo was updated
unsigned long previousServoMillis = 0; // stores last time the servo was updated
const long interval = 1000; // interval at which to switch direction (milliseconds)
const long servoInterval = 2;

bool direction = false;

void setup() 
{ 
  // set ros
  nh.initNode();
  nh.subscribe(sub_close);
  nh.advertise(grabber_contact);

  // set data rate
  Serial.begin(9600); 

  // set i/o pins
  pinMode(16, INPUT); // pin16 is IC input
  pinMode(LED_BUILTIN, OUTPUT); // builtin LED on teesny
  servo1.attach(11); // servo to control grabber
} 
 

void loop()
{
  //nh.spinOnce(); // happens in function

  if (new_close_msg == 1) {
    if (closing == 1) {
      close_actuator();
    } else if (closing == 0) {
      open_actuator();
    }
    new_close_msg = 0;  // clear flag
  }
  nh.spinOnce();
  delay(10);
  // check for messages
  // if statements

}

void close_actuator() {
  for (pos = 0; pos <= max_position; pos += 1) { // position maximum should be set to whatever would see grabber fully closed
    
    // check current reading
    int currentReading = analogRead(CURRENT_PIN);
    Serial.println(currentReading);

    // use current reading to check if made contact
    if(currentReading > contact_current){
      grabber_contact_msg.data = 1; // contact message is TRUE
      grabber_contact.publish( &grabber_contact_msg ); // publish contact message
      digitalWrite(LED_BUILTIN, HIGH);
      // STOP THE SERVO
    }
    else{ // servo keeps going
      grabber_contact_msg.data = 0; // contact message is FALSE
      grabber_contact.publish( &grabber_contact_msg ); // publish contact message
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // ros thingie to make happen
    nh.spinOnce();
    // move servo
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

}

void open_actuator() {
  for (pos = max_position; pos >= 0; pos -= 1) { // position maximum should be set to whatever would see grabber fully closed
    
    // check current reading
    int currentReading = analogRead(CURRENT_PIN);
    Serial.println(currentReading);

    // use current reading to check if made contact
    if(currentReading > contact_current){
      grabber_contact_msg.data = 1; // contact message is TRUE
      grabber_contact.publish( &grabber_contact_msg ); // publish contact message
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else{ // servo keeps going
      grabber_contact_msg.data = 0; // contact message is FALSE
      grabber_contact.publish( &grabber_contact_msg ); // publish contact message
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // ros thingie to make happen
    nh.spinOnce();
    // move servo
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

}
