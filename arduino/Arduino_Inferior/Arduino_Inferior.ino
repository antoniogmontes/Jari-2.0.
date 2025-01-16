#include "AFMotor.h"
#include "time.h"


// --------------------- VARIABLES MOTORES ---------------------------
// Variables sensores IR
int sensores[4];  // 0 - delante, 1 - izquierda, 2 - derecha, 3 - detras

// Variables motores DC
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);


// ------------------- VARIABLES COMMANDOS ---------------------------
int receivedComand = 0;
bool newData = false;
int currentState = 0;


enum MovementDirection { adelante, atras, izquierda, derecha, girader, giraizq};


void setup() {
  Serial.begin(9600); // Start serial port
  
  while (Serial.available() > 0){
    ; // Liberar puerto serial por si tiene basura
  }

  Serial2.begin(9600); // Start serial port
  
  while (Serial2.available() > 0){
    ; // Liberar puerto serial por si tiene basura
  }
  

  // Initial configuration
  start();
}

void loop() {
  receivedEvents();

  if (newData) {
    decoderCommand();
  }
  
}

void start() {  // Initial configuration
  Serial.println("Iniciando Arduino...");
  stop();
}


void receiveEvents() {
  if (Serial2.available() > 0) {
    receivedCommand = Serial.parseInt();
    currentState = receivedCommand
    newData = true;
  }
}

void decoderCommand() {
  switch(currentState) {
    case 1:
    case 2:
  }
 
}


void lectura() { // Lectura de los sensores IR
  sensores[0]=analogRead(A0);
  sensores[1]=analogRead(A1);
  sensores[2]=analogRead(A2);
  sensores[3]=analogRead(A3);  
}

void stop() { // Stop the movement
  motor1.setSpeed(0); motor1.run(RELEASE); // Parada de los motores
  motor2.setSpeed(0); motor2.run(RELEASE); 
  motor3.setSpeed(0); motor3.run(RELEASE); 
  motor4.setSpeed(0); motor4.run(RELEASE); 
  
  Serial.println("Parando");
}

void moveRobot(MovementDirection direction, int sensorToCheck, int speed) { // Move the robot
  // Configuración inicial de los motores según la dirección
  switch (direction) {
    case adelante:  // FORWARD
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case atras: // BACKWARD
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case izquierda: // LEFT
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    case derecha: // RIGHT
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case girader: // TURN RIGHT
      motor1.setSpeed(speed); motor1.run(BACKWARD);
      motor2.setSpeed(speed); motor2.run(FORWARD);
      motor3.setSpeed(speed); motor3.run(FORWARD);
      motor4.setSpeed(speed); motor4.run(BACKWARD);
      break;
    case giraizq:  // TURN LEFT
      motor1.setSpeed(speed); motor1.run(FORWARD);
      motor2.setSpeed(speed); motor2.run(BACKWARD);
      motor3.setSpeed(speed); motor3.run(BACKWARD);
      motor4.setSpeed(speed); motor4.run(FORWARD);
      break;
    default: 
      stop();
      break;
  }

  // Bucle de movimiento
  while (!stopMovement) {
    receiveEvents();  // Comprueba si hay nuevas órdenes
    lectura();        // Lee los sensores IR
    delay(250);       // Delay de medio segundo

    // Condición de parada
    if (currentState == 0 || sensores[sensorToCheck] < 200) {
      stop();           // Detener los motores
      stopMovement = true;
    }
  }
}
