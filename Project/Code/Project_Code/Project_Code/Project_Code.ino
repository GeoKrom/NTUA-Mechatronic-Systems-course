/*  Semester Project for course DPMS_AS/2204 - Mechatronic Systems
 *  1: Semi-autonomous Differential Drive mobile robot for avoiding obstacles.
 *  2: Telemanipulated Differential Drive Mobile Manipulator for picking
 * objects. 
 * Team 6: 
 *        Georgios Krommydas 
 *        Lampis Papakostas 
 * Co-Team 2 (4 DoF Manipulator): 
 *        Georgios Kassavetakis 
 *        Aggelos Kousanas 
 *        Konstantinos Manolis
 */

// KASA Tx = 10 , Rx = 9 

#include <SoftwareSerial.h>

const byte txPin = 7; //Maybe 0 or 1
const byte rxPin = 8; //Maybe 0 or 1
SoftwareSerial BTSerial(rxPin, txPin); // RX, TX

String command;

#define enableEncoderA 12
#define enableEncoderB 13
#define enableMotorA 3
#define enableMotorB 11
#define dcMotor1IN1 2
#define dcMotor1IN2 4
#define dcMotor2IN3 6 // 7
#define dcMotor2IN4 5 //8

#define wheelSpeed 100.0
#define Buffer_Size 8

uint8_t commands[Buffer_Size];

void setup() {

  pinMode(rxPin,INPUT);
  pinMode(txPin,OUTPUT);
  BTSerial.begin(38400);

  Serial.begin(38400);
  setPins();
  // delay(1000);
}

void loop() {
  if (Serial.available() == Buffer_Size) {
    Serial.println(Serial.available());
    for (int i = 0; i< Buffer_Size ; i++){
      commands[i] = uint8_t(Serial.read());
    }
    if(true){
      Serial.print("Command_7: ");
      Serial.println(commands[7]);
      Serial.print("Command_1: ");
      Serial.println(commands[1]-128);
      Serial.print("Command_2: ");
      Serial.println(commands[2]-128);
    }

    if (commands[7] == 1){  
      commands[1] -=128;
      commands[2] -=128;
      // FRONT <--> BACK
      if (commands[1] == 'F'){
        wheelForward(5);
      } else if (commands[1] == 'B') {
        wheelReverse(5);
      } else if (commands[1] == 'A') {
        Serial.println("No Move");
      }

      if (commands[2] == 'L') {
        wheelLeft(5);
      } else if (commands[2] == 'R') {
        wheelRight(5);
      } else if (commands[2] == 'A') {
        Serial.println("No Move");
      }
    }
  }
  wheelStop(1);
}

void setPins() {
  pinMode(enableEncoderA, INPUT);
  pinMode(enableEncoderB, INPUT);
  pinMode(enableMotorA, OUTPUT);
  pinMode(enableMotorB, OUTPUT);
  pinMode(dcMotor1IN1, OUTPUT); // Left DC Motor
  pinMode(dcMotor1IN2, OUTPUT); // Left DC Motor
  pinMode(dcMotor2IN3, OUTPUT); // Right DC Motor
  pinMode(dcMotor2IN4, OUTPUT); // Right DC Motor
}

void wheelForward(int a) {               // Motors Rotate Forward
  Serial.println("Forward...");
  digitalWrite(dcMotor1IN1, LOW);        // ON - Forward
  digitalWrite(dcMotor1IN2, HIGH);       // OFF - Forward
  analogWrite(enableMotorA, wheelSpeed); // Set the output speed(PWM)
  digitalWrite(dcMotor2IN3, LOW);        // ON - Forward
  digitalWrite(dcMotor2IN4, HIGH);       // OFF - Forward
  analogWrite(enableMotorB, wheelSpeed); // Set the output speed(PWM)
  delay(a * 50);
}

void wheelStop(int f) {                  // Both Motors Stop
  digitalWrite(dcMotor1IN1, LOW);        // OFF - Stop
  digitalWrite(dcMotor1IN2, LOW);        // OFF - Stop
  analogWrite(enableMotorA, wheelSpeed); // Set the output speed(PWM)
  digitalWrite(dcMotor2IN3, LOW);        // OFF - Stop
  digitalWrite(dcMotor2IN4, LOW);        // OFF - Stop
  analogWrite(enableMotorB, wheelSpeed); // Set the output speed(PWM)
  delay(f * 50);
}

void wheelLeft(int d) {                  // Rotate Left (Hard Turn)
  Serial.println("Left...");
  digitalWrite(dcMotor1IN1, LOW);        // OFF - Reverse
  digitalWrite(dcMotor1IN2, HIGH);       // ON - Reverse
  analogWrite(enableMotorA, wheelSpeed); // Set the output speed(PWM)
  digitalWrite(dcMotor2IN3, HIGH);       // ON - Forward
  digitalWrite(dcMotor2IN4, LOW);        // OFF - Forward
  analogWrite(enableMotorB, wheelSpeed); // Set the output speed(PWM)
  delay(d * 50);
}

void wheelRight(int e) {                 // Rotate Right (Hard Turn)
  Serial.println("Right...");
  digitalWrite(dcMotor1IN1, HIGH);       // ON - Forward
  digitalWrite(dcMotor1IN2, LOW);        // OFF - Forward
  analogWrite(enableMotorA, wheelSpeed); // Set the output speed(PWM)
  digitalWrite(dcMotor2IN3, LOW);        // OFF - Reverse
  digitalWrite(dcMotor2IN4, HIGH);       // ON - Reverse
  analogWrite(enableMotorB, wheelSpeed); // Set the output speed(PWM)
  delay(e * 50);
}

void wheelReverse(int g) {               // Motors Rotate Reverse
  Serial.println("Reverse...");
  digitalWrite(dcMotor1IN1, HIGH);       // OFF - Reverse
  digitalWrite(dcMotor1IN2, LOW);        // ON - Reverse
  analogWrite(enableMotorA, wheelSpeed); // Set the output speed(PWM)
  digitalWrite(dcMotor2IN3, HIGH);       // OFF - Reverse
  digitalWrite(dcMotor2IN4, LOW);        // ON - Reverse
  analogWrite(enableMotorB, wheelSpeed); // Set the output speed(PWM)
  delay(g * 50);
}

float readMotorsEncoder() {

  float speedEncoderA = analogRead(enableEncoderA);
  float speedEncoderB = analogRead(enableEncoderB);

  Serial.print("Encoder A: ");
  Serial.println(speedEncoderA);
  Serial.print("Encoder b: ");
  Serial.println(speedEncoderB);
}
