// -------- LIBRERIAS ---------- 

#include "AFMotor.h"
#include "time.h"

// --------- VARIABLES ------------

// Variables sensores IR
int sensores[4];  // 0 - delante, 1 - izquierda, 2 - derecha, 3 - detras

// Variables motores DC
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// 

// Variables detector
String receivedCommand = "";  // receivedCommand that receives commands
String contentM = "";         // receivedCommand para almacenar el contenido después de "M_"
String contentE = "";         // receivedCommand para almacenar el contenido después de "E_"
bool stringComplete = false;  // True si la cadena recibida está completa

// Bool Acciones
bool stopMovement = false;    // True si queremos q pare el movimiento
bool stopEmotion = false;     // True si queremos q pare la emocion

// ------------ ENUM COMANDOS ---------------

// Command enum
enum Movement {
  Forward = 'F',
  Backward = 'B',
  LEFT = 'L',
  RIGHT = 'R',
  STOP = 'S',
  TURN = 'T',
  NONE_M = '0'
};

// Command enum
enum Emotion {
  HAPPY = 'A',
  VERY_HAPPY = 'C',
  SERIOUS = 'D',
  SAD = 'K',
  VERY_SAD = 'E',
  DISGUSTED = 'N',
  ANGRY = 'O',
  SCARED = 'P',
  SURPRISED = 'Q',
  NONE_E = '0'
};

enum MovementDirection { adelante, atras, izquierda, derecha, girader, giraizq};

// ----------------------------------------
// ------------- VOID SETUP ---------------
// ----------------------------------------

void setup()
{
  Serial.begin(9600); // Start serial port
  while (Serial.available() > 0){
    ; // Liberar puerto serial por si tiene basura
  }

  // Setup function
  start();
}

// ----------------------------------------
// ------------- VOID LOOP ----------------
// ----------------------------------------

void loop() {
  
  receiveEvents(); // Extrae informacion de la cadena
  
  if (stringComplete) {
    analiceCommand(); // Analiza los valores de la cadena entrante
  }

  stringComplete = false;
}

void start() {  // Función inicio del programa
  Serial.println("Iniciando Arduino...");
  stop();
}

void receiveEvents(){ // Analiza el mensaje recibido
  if (Serial.available() > 0){
    String receivedCommand = Serial.readStringUntil('\n');
    // receivedCommand.remove(0, 1);  
    // Delete index 0. Posible solucion si no funciona el readStringUntil()
    // y coge un ultimo caracter con basura  

    detect(receivedCommand, contentM, contentE);
    stringComplete = true;
    stopMovement = false;
    stopEmotion = false;
  }
}

void detect(String S, String &C, String &P) { //Separate the message
  if (true) {

    // Buscar el primer separador "+" que tenga contenido a su izquierda y a su derecha
    int separatorPos = S.indexOf('+');

    while (separatorPos != -1) {
      // Verificar si hay caracteres tanto a la izquierda como a la derecha del separador "+"
      if (separatorPos > 0 && separatorPos < S.length() - 1) {
        // Extraer las subcadenas antes y después del separador "+"
        String contentBeforePlus = S.substring(0, separatorPos);
        String contentAfterPlus = S.substring(separatorPos + 1);

        int idenM = contentBeforePlus.indexOf("M_");
        int idenE = contentAfterPlus.indexOf("E_");

        if (idenM > 0 && idenM < contentBeforePlus.length() - 1) {
          //Extraer la subcadena despues de M_
          String contentAfterM_ = contentBeforePlus.substring(idenM + 1);
        }

        if (idenE > 0 && idenE < contentBeforePlus.length() - 1) {
          //Extraer la subcadena despues de E_
          String contentAfterE_ = contentAfterPlus.substring(idenE + 1);
        }
        // Salir del bucle una vez que se haya procesado un separador "+"
        break;
      }
      // Buscar el siguiente separador "+" que cumpla la condición
      separatorPos = S.indexOf('+', separatorPos + 1);
    }

    Serial.println("------------------------------");
    Serial.println(contentM);
    Serial.println(contentE);
  }
}

void lectura() { // Lectura de los sensores "ir
  sensores[0]=analogRead(A0);
  sensores[1]=analogRead(A1);
  sensores[2]=analogRead(A2);
  sensores[3]=analogRead(A3);  
}

void analiceCommand() {  // Función indica moviminetno y emocion a hacer
  // Analice movement
  stopMovement = false;
  switch (contentM.charAt(0)) {
    case Movement::Forward  : Serial.println("Adelante");  moveRobot(adelante, 0, 125); break;
    case Movement::Backward : Serial.println("Atras");     moveRobot(atras, 3, 125); break;
    case Movement::LEFT     : Serial.println("Derecha");   moveRobot(derecha, 2, 125); break;
    case Movement::RIGHT    : Serial.println("Izquierda"); moveRobot(izquierda, 1, 125); break;
    case Movement::TURN     :      
      if (contentM.charAt(1) == 'W') { Serial.println("Girar Dcha"); moveRobot(girader, 0, 125); break; }
      else if (contentM.charAt(1) == 'M') { Serial.println("Girar Izq"); moveRobot(giraizq, 3, 125); break; }
    case Movement::STOP     : Serial.println("Stop"); stop(); break;
    case Movement::NONE_M   : Serial.println("Nada"); stop(); break;
    default                 : Serial.println("Error: Invalid movement"); break;
  }

  stopMovement = true;
  // Analice emotion
  switch (contentE.charAt(0)) {
    case Emotion::HAPPY: 
      Serial.println("Feliz"); 
      moveRobot(adelante, 0, 125); delay (250);
      stop(); delay(1000);
      moveRobot(atras, 3, 125); delay (250);
      stop();
      break;
    case Emotion::VERY_HAPPY: 
      Serial.println("Muyfeliz"); 
      moveRobot(girader, 0, 125); delay(3000);
      stop();
      break;
    case Emotion::SERIOUS: 
      Serial.println("Serio");
      stop();
      break;
    case Emotion::SAD: 
      Serial.println("Triste");
      moveRobot(giraizq, 3, 100); delay(1000);
      stop(); delay(2000);
    case Emotion::VERY_SAD  :
      Serial.println("MuyTriste");
      moveRobot(giraizq, 3, 125); delay(1000);
      stop(); delay(2000);
      moveRobot(giraizq, 3, 125); delay(1000);
      stop();
      break;
    case Emotion::DISGUSTED:
      Serial.println("Asqueado");
      moveRobot(giraizq, 3, 125); delay(250);
      moveRobot(girader, 0, 125); delay(500);
      moveRobot(giraizq, 0, 125); delay(250);
      stop();
      break;
    case Emotion::ANGRY:
      Serial.println("Enfadado");
      moveRobot(adelante, 0, 200);
      break;
    case Emotion::SCARED:
      Serial.println("Asustado");
      // Incluir movimiento
      break;
    case Emotion::SURPRISED:
      Serial.println("Sorprendido");
      // Incluir movimiento
      break;
    case Emotion::NONE_E:
      Serial.println("Nada");
      break;
    default:
      Serial.println("Error: Invalid emotion");
      break;
  }

  stringComplete = false;
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
    if (contentM.charAt(0) == Movement::STOP || sensores[sensorToCheck] < 200) {
      stop();           // Detener los motores
      stopMovement = true;
    }
  }
}