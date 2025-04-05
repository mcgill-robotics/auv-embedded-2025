// USING SERVO CONNECTED TO ACTUATOR BOARD

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(11);  // attaches the servo on pin x to the Servo object
  Serial.begin(115200); // connection speed (req field for actuator board)
}


void loop() {
  for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to x degrees
    // in steps of 1 degree
    Serial.println("working");
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
 
  //for (pos = 300; pos >= 0; pos -= 1) { // goes from x degrees to 0 degrees
   // myservo.write(pos);              // tell servo to go to position in variable 'pos '
   // delay(15);                       // waits 15 ms for the servo to reach the position
 // }
}
