#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(8);  // attaches the servo on pin 3 to the Servo object
}

void loop() {
  for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to x degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
 
  //for (pos = 300; pos >= 0; pos -= 1) { // goes from x degrees to 0 degrees
   // myservo.write(pos);              // tell servo to go to position in variable 'pos'
   // delay(15);                       // waits 15 ms for the servo to reach the position
 // }
}
