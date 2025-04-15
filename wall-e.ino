#include <Servo.h>
#include "IR_remote.h"

// Define IR Receiver and Servo Pins
const int irRecvPin = 3;

// Define Servo pins
const int servoNeckPin = 9;
const int servoLeftArmPin = 8;
const int servoRightArmPin = 7;

// Servo objects
Servo servoNeck, servoLeftArm, servoRightArm;

// Motor control pins
const int motorA1Pin = 5; // PWM control for right motors (A)
const int motorA2Pin = 2; // Direction control for right motors (A)
const int motorB1Pin = 6; // PWM control for left motors (B)
const int motorB2Pin = 4; // Direction control for left motors (B)

// IR Remote object
IRremote irRemote(irRecvPin);

// Ultrasonic and infrared sensor pins
const int ultrasonicTrigPin = 12;
const int ultrasonicEchoPin = 13;
const int infraredLeftPin = A1;
const int infraredRightPin = A2;

const int threshold = 200; 
bool obstacleAvoidanceEnabled = false;

int motorSpeed = 50; // Default slower speed

void setup() {
  Serial.begin(9600);
  servoNeck.attach(servoNeckPin);
  servoLeftArm.attach(servoLeftArmPin);
  servoRightArm.attach(servoRightArmPin);

  irRemote.begin();

  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorB1Pin, OUTPUT);
  pinMode(motorB2Pin, OUTPUT);

  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);

  pinMode(infraredLeftPin, INPUT);
  pinMode(infraredRightPin, INPUT);
}

void loop() {
  if (irRemote.decode()) {
    handleIRCommand(irRemote.value);
    irRemote.begin();  // Ready to receive the next IR command
  }

  if (obstacleAvoidanceEnabled) {
    performObstacleAvoidance();
  }
}

void handleIRCommand(unsigned long cmd) {
  Serial.print("Received IR code: ");
  Serial.println(cmd, HEX);
  switch (cmd) {
    case 0xB946FF00: // Forward
      moveForward();
      break;
    case 0xEA15FF00: // Backward
      moveBackward();
      break;
    case 0xBB44FF00: // Left
      turnLeft();
      break;
    case 0xBC43FF00: // Right
      turnRight();
      break;
    case 0xE916FF00: // Rotate Left
      rotateLeft();
      break;
    case 0xF20DFF00: // Rotate Right
      rotateRight();
      break;
    case 0xBD42FF00: // Neck Left
      servoNeck.write(135); // Inverted
      break;
    case 0xB54AFF00: // Neck Right
      servoNeck.write(45); // Inverted
      break;
    case 0xF30CFF00: // Left Arm Up
      servoLeftArm.write(0); // Inverted
      break;
    case 0xF708FF00: // Left Arm Down
      servoLeftArm.write(90); // Inverted
      break;
    case 0xA15EFF00: // Right Arm Up
      servoRightArm.write(180); // Inverted
      break;
    case 0xA55AFF00: // Right Arm Down
      servoRightArm.write(90); // Inverted
      break;
    case 0xAD52FF00: // Toggle Obstacle Avoidance
      toggleObstacleAvoidance();
      break;
    case 0xBF40FF00: // Stop All
      stopAll();
      break;
    case 0xFF02FD: // Speed Up
      motorSpeed = 255;
      break;
    case 0xFF629D: // Slow Down
      motorSpeed = 150;
      break;
    case 0xFF22DD: // Turn Off Obstacle Avoidance
      obstacleAvoidanceEnabled = false;
      break;
  }
}

void moveForward() {
  // Corrected directions for left and right motors
  digitalWrite(motorA2Pin, HIGH); // Right motors forward
  digitalWrite(motorB2Pin, LOW);  // Left motors forward (inverted)
  analogWrite(motorA1Pin, motorSpeed);
  analogWrite(motorB1Pin, motorSpeed);
}

void moveBackward() {
  // Corrected directions for left and right motors
  digitalWrite(motorA2Pin, LOW);  // Right motors backward
  digitalWrite(motorB2Pin, HIGH); // Left motors backward (inverted)
  analogWrite(motorA1Pin, motorSpeed);
  analogWrite(motorB1Pin, motorSpeed);
}

void turnLeft() {
  // Corrected so both motors drive to turn the bot left
  digitalWrite(motorA2Pin, HIGH); // Right motor forward
  digitalWrite(motorB2Pin, HIGH); // Left motor backward (inverted)
  analogWrite(motorA1Pin, 50);    // Reduced speed for smooth rotation
  analogWrite(motorB1Pin, 50);
}

void turnRight() {
  // Corrected so both motors drive to turn the bot right
  digitalWrite(motorA2Pin, LOW);  // Right motor backward
  digitalWrite(motorB2Pin, LOW);  // Left motor forward (inverted)
  analogWrite(motorA1Pin, 50);    // Reduced speed for smooth rotation
  analogWrite(motorB1Pin, 50);
}

void rotateLeft() {
  // Spin in place to the left
  digitalWrite(motorA2Pin, HIGH); // Right motor forward
  digitalWrite(motorB2Pin, HIGH); // Left motor backward (inverted)
  analogWrite(motorA1Pin, 50);    // Constant slow speed for rotation
  analogWrite(motorB1Pin, 50);
  delay(3500); // Time to complete a 360-degree spin
  stopAll();
}

void rotateRight() {
  // Spin in place to the right
  digitalWrite(motorA2Pin, LOW);  // Right motor backward
  digitalWrite(motorB2Pin, LOW);  // Left motor forward (inverted)
  analogWrite(motorA1Pin, 50);    // Constant slow speed for rotation
  analogWrite(motorB1Pin, 50);
  delay(3500); // Time to complete a 360-degree spin
  stopAll();
}

void stopAll() {
  digitalWrite(motorA1Pin, LOW);
  digitalWrite(motorB1Pin, LOW);
}

void toggleObstacleAvoidance() {
  obstacleAvoidanceEnabled = !obstacleAvoidanceEnabled;
  Serial.print("Obstacle Avoidance Mode: ");
  Serial.println(obstacleAvoidanceEnabled ? "ON" : "OFF");
}

void performObstacleAvoidance() {
  int distance = getUltrasonicDistance();
  if (distance < 20 || digitalRead(infraredLeftPin) == LOW || digitalRead(infraredRightPin) == LOW) {
    stopAndRedirect();
  } else {
    moveForward();
  }
}

int getUltrasonicDistance() {
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);
  long duration = pulseIn(ultrasonicEchoPin, HIGH);
  return duration / 58;
}

void stopAndRedirect() {
  stopAll();
  delay(500);
  turnRight();
  delay(500);
  moveForward();
}
