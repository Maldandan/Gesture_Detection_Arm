#include <Wire.h>
#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

const char DELIMITER = ';';

// Define max and min values for servos
int M1;
int M2;
int M3;
int M4;
int M5;
int M6;
int M1_max = 180;
int M2_max = 165;
int M3_max = 180;
int M4_max = 180;
int M5_max = 180;
int M6_max = 73;
int M2_min = 15;
int M6_min = 10;

// Variables to store previous values of motors
int prev_M1;
int prev_M2;
int prev_M3;
int prev_M4;
int prev_M5;
int prev_M6;

void setup() {
  Serial.begin(9600);
  Braccio.begin();
}

void loop() {
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil(DELIMITER);
    processCommand(command);
    delay(100); // Reduced delay between commands
  }
}

void processCommand(String command) {
  // Store previous values
  prev_M1 = M1;
  prev_M2 = M2;
  prev_M3 = M3;
  prev_M4 = M4;
  prev_M5 = M5;
  prev_M6 = M6;

  // Perform actions based on received commands
  if (command == "like") {
    Braccio.ServoMovement(20, 20, 20, 20, 20, 20, 50);
  } else if (command == "dislike") {
    Braccio.ServoMovement(20, 20, 50, 25, 50, 25, 50);
  } else if (command == "call") {
    Braccio.ServoMovement(20, 50, 20, 50, 20, 50, 35);
  } else if (command == "mute") {
    Braccio.ServoMovement(20, 50, 50, 50, 50, 50, M6_max);
  } else if (command == "ok") {
    Braccio.ServoMovement(20, 70, 70, 70, 70, 70, M6_max);
  } else if (command == "peace") {
    Braccio.ServoMovement(20, 50, 70, 50, 70, 50, M6_max);
  } else if (command == "stop") {
    // Release the gripper (Motor 6)
    Braccio.ServoMovement(20, prev_M1, prev_M2, prev_M3, prev_M4, prev_M5, M6_min);
    M6 = M6_min; // Update the current value of M6 to M6_min
  } else if (command == "fist") {
    // Grip with the gripper (Motor 6)
    Braccio.ServoMovement(20, prev_M1, prev_M2, prev_M3, prev_M4, prev_M5, M6_max);
    M6 = M6_max; // Update the current value of M6 to M6_max
  } else if (command == "peace_inverted") {
    Braccio.ServoMovement(20, 100, 50, 100, 50, 100, M6_max);
  } else if (command == "stop_inverted") {
    Braccio.ServoMovement(20, 50, 140, 50, 140, 50, M6_max);
  }
  // Add more conditions for other gestures and their corresponding actions
}
