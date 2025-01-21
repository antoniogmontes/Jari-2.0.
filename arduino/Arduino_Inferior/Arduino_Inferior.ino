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
int receivedComand = 0;
bool newCommandReceived  = false;
uint32_t currentState = 0; // Default state

// Command enum
enum Movement {
  Forward = 10,
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

enum MovementDirection { forward, backward, left, right, turnRight, turnLeft };


void setup() {
  Serial.begin(9600); // Initialize serial communication
  Serial2.begin(9600);
  
  clearSerialBuffer(Serial);
  clearSerialBuffer(Serial2);
  
  initializeSystem();
}

void loop() {
  receiveCommand();

  if (newCommandReceived) {
    processCommand();
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
  if (Serial2.available() > 0) {
    receivedCommand = Serial.parseInt();
    Serial.println(receivedCommand);
    newCommandReceived = true;
    if (receivedCommand == 0) {
      currentState = OFF;
    }
  }
}

// Process the received command
void processCommand() {
  switch(receivedCommand) {
    // Emotions
    case Emotion::SERIOUS:
      currentState = SERIOUS;
      moveRobotSteps(stop, 0, 125, 1000);
      break;
    case Emotion::VERY_HAPPY:
      currentState = VERY_HAPPY;
      moveRobotSteps(girader, 0, 125, 3000);
      break;
    case Emotion::HAPPY:
      currentState = HAPPY;
      moveRobotSteps(adelante, 0, 125, 250);
      moveRobotSteps(stop, 0, 125, 1000);
      moveRobotSteps(atras, 3, 125, 250);
      break;
    case Emotion::VERY_SAD:
      currentState = VERY_SAD;
      moveRobotSteps(giraizq, 3, 125, 1000);
      moveRobotSteps(stop, 0, 125, 2000);
      moveRobotSteps(giraizq, 3, 125, 1000);
      break;
    case Emotion::SAD:
      currentState = SAD;
      moveRobotSteps(atras, 3, 50, 500);
      break;
    case Emotion::DISGUSTED:
      currentState = DISGUSTED;
      moveRobotSteps(giraizq, 3, 125, 250);
      moveRobotSteps(girader, 0, 125, 500);
      moveRobotSteps(giraizq, 3, 125, 250);
      break;
    case Emotion::ANGRY:
      currentState = ANGRY;
      moveRobotSteps(adelante, 0, 200, 250);
      break;
    case Emotion::SCARED:
      currentState = SCARED;
      moveRobotSteps(atras, 3, 50, 500);
      moveRobotSteps(adelante, 0, 50, 250);
      moveRobotSteps(atras, 3, 50, 500);
      break;
    case Emotion::SURPRISED:
      currentState = SURPRISED; 
      moveRobotSteps(giraizq, 3, 125, 500);
      moveRobotSteps(stop, 0, 50, 500);
      moveRobotSteps(girader, 0, 125, 500);
      break;
    // Movements
    case Movement::Forward:
      currentState = forward;
      moveRobot(forward, 0, 125);
      break;
    case Movement::Backward:
      currentState = backward;
      moveRobot(backward, 3, 125);
      break;
    case Movement::LEFT:
      currentState = left;
      moveRobot(left, 2, 125);
      break;
    case Movement::RIGHT:
      currentState = right;
      moveRobot(right, 1, 125);
      break;
    case Movement::TURN_RIGHT:
      currentState = turnRight;
      moveRobot(turnRight, 0, 125);
      break;
    case Movement::TURN_LEFT:
      currentState = turnLeft;
      moveRobot(turnLeft, 3, 125);
      break;    
    case Movement::STOP:
      currentState = OFF;
      stop();
      break;
    case Movement::OFF:
      currentState = OFF;
      stop();
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
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case backward:
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case left:
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case right:
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case turnRight:
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case turnLeft:
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    default:
      stopMotors();
      break;
  }
}

// Move the robot in a specific direction
void moveRobot(MovementDirection direction, int sensorToCheck, int speed) {
  configureMotors(direction, speed);

  while (true) {
    receiveCommand();
    readIRSensors();

    if (currentState == OFF || irSensors[sensorToCheck] < 200) {
      stopMotors();
      return;
    }
    delay(250);
  }
}

// Move the robot a specific seconds
void moveRobotSteps(MovementDirection direction, int sensorToCheck, int speed, int mSeconds) {
  configureMotors(direction, speed);

  if (currentState == OFF || irSensors[sensorToCheck] < 200) {
      stopMotors();
      return;
    }

  delay(mSeconds);
  stop();
}
