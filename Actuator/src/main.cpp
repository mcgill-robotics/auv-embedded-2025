#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


#define CURRENT_PIN 16
#define SERVO_PIN 11

// constant data types
bool contact = 0;
bool close = 0;

// variables
int contact_current = 20;
int max_position = 180;
int pos = 0;    // variable to store the servo position 
float current = 0;

// set messagd types
std_msgs::Bool grabber_contact_msg;
std_msgs::Bool close_msg;

// set ros pub/subscriber
ros::NodeHandle nh;
ros::Publisher grabber_contact("/actuators/grabber/contact", &grabber_contact_msg);
ros::Subscriber<std_msgs::Bool> sub("/actuators/grabber/close", &update_close_msg);

// creat servo object
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

  if (grabber_contact_msg.data == 1) {
    close_msg();
  }
  delay(10);
  // check for messages
  // if statements

}

void update_close_msg( const std::msgsBool& close){
  close = close.data;
}

void close( const std_msgs::Bool& close_msg ) {
  for (pos = 0; pos <= max_position; pos += 1) { // position maximum should be set to whatever would see grabber fully closed
    
    // check current reading
    int currentReading = analogRead(CURRENT_PIN);
    Serial.println(currentReading);

    // use current reading to check if made contact
    if(currentReading > contact_current){
      grabber_contact_msg.data = 1; // contact message is TRUE
      grabber_contact.publish( &grabber_contact_msg.data ) // publish contact message
      digitalWrite(LED_BUILTIN, HIGH);
      // STOP THE SERVO
    }
    else{ // servo keeps going
      grabber_contact_msg.data = 0; // contact message is FALSE
      grabber_contact.publish( &grabber_contact_msg.data ) // publish contact message
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // ros thingie to make happen
    nh.spinOnce();
    // move servo
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

}
//void loop()
//{

  //for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    
    //int currentReading = analogRead(CURRENT_PIN);
    //Serial.println(currentReading);

    //if(currentReading > 20){

      //contact = 1;
      //digitalWrite(LED_BUILTIN, HIGH);
    //}
    //else{
     // contact = 0;
      //digitalWrite(LED_BUILTIN, LOW);
    //}
    // in steps of 1 degree
    //grabber_contact_msg.data = contact;

    //grabber_contact.publish( &grabber_contact_msg );

    //nh.spinOnce();


    //myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(15);                       // waits 15ms for the servo to reach the position
  //}




}
