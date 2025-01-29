#include <Servo.h>

const int buttonPin = 2;  // Pin number for the pushbutton
Servo myservo;            // Servo object to control a servo

// Variables for button and motor state
int buttonState = 0;
int lastButtonState = 0;
bool motorOpen = false;    // Track whether the motor is open or closed
unsigned long lastDebounceTime = 0; // Debounce timer
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds

void setup() {
  myservo.attach(3);       // Attach the servo to pin 3
  myservo.write(0);        // Start with the motor in the closed position
  pinMode(buttonPin, INPUT_PULLUP); // Initialize the button pin with pull-up resistor
}

void loop() {
  // Read the current state of the button
  int reading = digitalRead(buttonPin);

  // Check for button state changes with debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // Reset the debounce timer
  }

  // If the debounce time has passed, process the button press
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed, toggle the motor state
    if (reading == LOW && buttonState == HIGH) { // Button pressed
      motorOpen = !motorOpen;

      if (motorOpen) {
        // Open the motor (0° to 180°)
        for (int pos = 0; pos <= 180; pos++) {
          myservo.write(pos);
          delay(10);
        }
      } else {
        // Close the motor (180° to 0°)
        for (int pos = 180; pos >= 0; pos--) {
          myservo.write(pos);
          delay(10);
        }
      }
    }
    buttonState = reading; // Update button state
  }

  lastButtonState = reading; // Update last button state
}
