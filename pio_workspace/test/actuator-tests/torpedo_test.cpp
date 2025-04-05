// ******UNFINISHED **********

// first button press, fires first torpedo
// second button press, fires second torpedo
// then, immedietly resets system

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int buttonState = 0;  // variable for reading the pushbutton status
int systemState = 1; // position of motor, corresponds to number of torpedos fired
//add ammounts to move and start points as array that's indexed by system state variable, since 
//want to change and might not be uniform between states

void setup() {
  myservo.attach(3);  // attaches the servo on pin 3 to the Servo object
  pinMode(5, INPUT);
}

void loop() {
// read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH && systemState < 3 ) {
    systemState++; // increment system state
     
    for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to x degrees
    // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15 ms for the servo to reach the position
  }
    
    // move servo
    
  } else {
    // 
  }
}

  for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to x degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
