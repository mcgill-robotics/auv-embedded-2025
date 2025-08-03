
// PRINTS CURRENT READINGS FROM MOTOR RUNNING OFF J3

#include <Servo.h>
#define CURRENT_PIN 17 //CHANGED FROM 16
#define SERVO_PIN 10
#define GREEN_LED 15
#define RED_LED 14
#define pos_global
#define pos


// variables
int contact_current = 400;
int max_position = 180;
int pos = 0;    // variable to store the servo position 
float current = 0;



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

  // set data rate
  Serial.begin(9600); 

  // set i/o pins
  pinMode(CURRENT_PIN, INPUT); // pin16 is IC input
  pinMode(LED_BUILTIN, OUTPUT); // builtin LED on teesny
  servo1.attach(SERVO_PIN); // servo to control grabber
} 
 

void loop()
{

  close_actuator();

}

void close_actuator() {
  for (pos = 0; pos <= max_position; pos += 1) { // position maximum should be set to whatever would see grabber fully closed
    
    // check current reading
    int currentReading = analogRead(CURRENT_PIN);
    Serial.println(currentReading);

    // use current reading to check if made contact
    if(currentReading > contact_current){
      
      stop();

      // STOP THE SERVO
    }
    else{ // servo keeps going
      
      digitalWrite(GREEN_LED, HIGH);
    }
    
    // move servo
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    pos = pos;
  }

}

void open_actuator() {
  for (pos = pos_global; pos >=0; pos -= 1) { // position maximum should be set to whatever would see grabber fully closed
    servo1.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}


  void stop() {
    for (;;) {
      digitalWrite(RED_LED, HIGH);
      pos_global = pos;
      open_actuator();
      
    }
  }
