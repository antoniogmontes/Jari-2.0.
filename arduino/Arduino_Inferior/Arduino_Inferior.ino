#include "AFMotor.h"
#include "time.h"

// --------------------- MOTORS VARIABLES ---------------------------
// IR sensor variables
int irSensors[4];  // 0 - Front, 1 - Left, 2 - Right, 3 - Back

// DC motor variables
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// ------------------- VARIABLES COMMANDOS ---------------------------
int receivedCommand = 0;
bool newCommandReceived  = false;
uint32_t currentState = 0; // Default state

// Command enum
enum Movement {
  Forward = 17,
  Backward = 11,
  LEFT = 12,
  RIGHT = 13,
  STOP = 14,
  TURN_RIGHT = 15,
  TURN_LEFT = 16,
  OFF = 0
};

// Command enum
enum Emotion {
  HAPPY = 3,
  VERY_HAPPY = 2,
  SERIOUS = 1,
  SAD = 5,
  VERY_SAD = 4,
  DISGUSTED = 6,
  ANGRY = 7,
  SCARED = 8,
  SURPRISED = 9,
  EMOTION_OFF = 0
};

enum MovementDirection { forward, backward, left, right, turnRight, turnLeft, stop };

void setup() {
  Serial.begin(9600); // Initialize serial communication
  
  clearSerialBuffer(Serial);
  
  initializeSystem();
}

void loop() {
  receiveCommand();

  if (newCommandReceived) {
    processCommand();

    // Clean the Serial Buffer for the net command
    clearSerialBuffer(Serial);
  }
}

// ------------------- FUNCTION DEFINITIONS ---------------------------
// Clear any unwanted data from the serial buffer
void clearSerialBuffer(HardwareSerial &serialPort) {
  while (serialPort.available() > 0) {
    serialPort.read();
  }
}

// Initialize the system
void initializeSystem() {
  Serial.println("Initializing Arduino...");
  stopMotors();
}

// Read commands from Serial2
void receiveCommand() {
  if (Serial.available() > 0) {
    receivedCommand = Serial.parseInt();
    Serial.println("-------------------");
    Serial.println(receivedCommand);
    newCommandReceived = true;
    if (receivedCommand == 0) {
      currentState = OFF;
    }
  }
}

// Process the received command
void processCommand() {
  Serial.println("ProcessCommand");

  switch(receivedCommand) {
    // Emotions
    case Emotion::SERIOUS:
      currentState = SERIOUS;
      Serial.println("Serious");
      moveRobotSteps(stop, 0, 125, 1000);
      break;
    case Emotion::VERY_HAPPY:
      currentState = VERY_HAPPY;
      Serial.println("Very happy");
      moveRobotSteps(turnRight, 0, 125, 3000);
      break;
    case Emotion::HAPPY:
      currentState = HAPPY;
      Serial.println("Happy");
      moveRobotSteps(forward, 0, 125, 250);
      moveRobotSteps(stop, 0, 125, 1000);
      moveRobotSteps(backward, 3, 125, 250);
      break;
    case Emotion::VERY_SAD:
      currentState = VERY_SAD;
      Serial.println("Very sad");
      moveRobotSteps(turnLeft, 3, 125, 1000);
      moveRobotSteps(stop, 0, 125, 2000);
      moveRobotSteps(turnLeft, 3, 125, 1000);
      break;
    case Emotion::SAD:
      currentState = SAD;
      Serial.println("Sad");
      moveRobotSteps(backward, 3, 250, 5000);
      break;
    case Emotion::DISGUSTED:
      currentState = DISGUSTED;
      Serial.println("Disgusted");
      moveRobotSteps(turnLeft, 3, 125, 250);
      moveRobotSteps(turnRight, 0, 125, 500);
      moveRobotSteps(turnLeft, 3, 125, 250);
      break;
    case Emotion::ANGRY:
      currentState = ANGRY;
      Serial.println("Angry");
      moveRobotSteps(forward, 0, 200, 250);
      break;
    case Emotion::SCARED:
      currentState = SCARED;
      Serial.println("Scared");
      moveRobotSteps(backward, 3, 50, 500);
      moveRobotSteps(forward, 0, 50, 250);
      moveRobotSteps(backward, 3, 50, 500);
      break;
    case Emotion::SURPRISED:
      currentState = SURPRISED;
      Serial.println("Surprised"); 
      moveRobotSteps(turnLeft, 3, 125, 500);
      moveRobotSteps(stop, 0, 50, 500);
      moveRobotSteps(turnRight, 0, 125, 500);
      break;
    // Movements
    case Movement::Forward:
      currentState = forward;
      moveRobot(forward, 0, 100);
      break;
    case Movement::Backward:
      currentState = backward;
      moveRobot(backward, 3, 100);
      break;
    case Movement::LEFT:
      currentState = left;
      moveRobot(left, 2, 100);
      break;
    case Movement::RIGHT:
      currentState = right;
      moveRobot(right, 1, 100);
      break;
    case Movement::TURN_RIGHT:
      currentState = turnRight;
      moveRobot(turnRight, 0, 100);
      break;
    case Movement::TURN_LEFT:
      currentState = turnLeft;
      moveRobot(turnLeft, 3, 100);
      break;    
    case Movement::STOP:
      currentState = OFF;
      stopMotors();
      break;
    case Movement::OFF:
      currentState = OFF;
      stopMotors();
      break;
    default:
      currentState = OFF;
      Serial.println("Error: Invalid command");
      break;
  }
  newCommandReceived = false;
}

// Read IR sensor values
void readIRSensors() {
    irSensors[0] = analogRead(A0);
    irSensors[1] = analogRead(A1);
    irSensors[2] = analogRead(A2);
    irSensors[3] = analogRead(A3);
}

// Stop all motors
void stopMotors() {
  motor1.setSpeed(0); motor1.run(RELEASE);
  motor2.setSpeed(0); motor2.run(RELEASE);
  motor3.setSpeed(0); motor3.run(RELEASE);
  motor4.setSpeed(0); motor4.run(RELEASE);

  Serial.println("Motors stopped.");
}

// Configure motors based on the movement direction
void configureMotors(MovementDirection direction, int speed) {
  switch (direction) {
    case forward:
      Serial.print("Forward");
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case backward:
      Serial.print("Backward");
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case left:
      Serial.print("Left");
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case right:
      Serial.print("Right");
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case turnRight:
      Serial.print("TurnRight");
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case turnLeft:
      Serial.print("TurnLeft");
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case stop:
      Serial.print("Stop");
      motor1.setSpeed(speed); motor1.run(RELEASE);
      motor2.setSpeed(speed); motor2.run(RELEASE);
      motor3.setSpeed(speed); motor3.run(RELEASE);
      motor4.setSpeed(speed); motor4.run(RELEASE);
      break;
    default:
      stopMotors();
      break;
  }
}

// Move the robot in a specific direction
void moveRobot(MovementDirection direction, int sensorToCheck, int speed) {
  configureMotors(direction, speed);
  
  // Clean the Serial Buffer for the net command
  Serial.println("  Start");
  Serial.println(currentState);
  clearSerialBuffer(Serial);
  
  while (true) {
    receiveCommand();
    //readIRSensors();
    // --------------- Ejemplo ---------------------
    irSensors[0] = 1000;
    irSensors[1] = 1000;
    irSensors[2] = 1000;
    irSensors[3] = 1000;

    if (currentState == OFF || irSensors[sensorToCheck] < 200) {
      Serial.println("End");
      stopMotors();
      return;
    }
    delay(250);
  }
}

// Move the robot a specific seconds
void moveRobotSteps(MovementDirection direction, int sensorToCheck, int speed, int mSeconds) {
  configureMotors(direction, speed);
  readIRSensors();
  
  // --------------- Ejemplo ---------------------
  irSensors[0] = 1000;
  irSensors[1] = 1000;
  irSensors[2] = 1000;
  irSensors[3] = 1000;

  if (currentState == OFF || irSensors[sensorToCheck] < 200) {
    Serial.println("Obstacle detected");
    stopMotors();
    return;
  }

  Serial.print("  Start");
  delay(mSeconds);
  Serial.println("  End");
  stopMotors();
}
