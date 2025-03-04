#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Clear the Serial Port
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void loop() {
  receiveCommand();

}

void receiveCommand() {
  if (Serial.available() > 0) {
    int receivedCommand = Serial.parseInt();
    Serial.println(receivedCommand);

    sendToArduino(receivedCommand);
  }
}


void sendToArduino(int cmd) {
  Serial2.println(cmd);
  Serial.println("Comando enviado al Arduino: ");
  Serial.print(cmd);

  while (Serial.available() > 0) {
    Serial.read();
  }
}