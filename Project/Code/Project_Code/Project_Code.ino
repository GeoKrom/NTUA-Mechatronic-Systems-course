/*  Semester Project for course DPMS_AS/2204 - Mechatronic Systems
 *  1: Semi-autonomous Differential Drive mobile robot for avoiding obstacles.
 *  2: Telemanipulated Differential Drive Mobile Manipulator for picking objects.
 *  Team 6:
 *        Georgios Krommydas
 *        Lampis Papakostas
 *  Co-Team 2 (4 DoF Manipulator):
 *        Georgios Kassavetakis
 *        Aggelos Kousanas
 *        Konstantinos Manolis
 */

#include <PID_v1.h>
#include <HCSR04.h>
#include <SoftwareSerial.h>

#define enableEncoderA 12
#define enableEncoderB 13
#define enableMotorA 3
#define enableMotorB 5
#define dcMotor1IN1 2
#define dcMotor1IN2 4
#define dcMotor1IN3 7
#define dcMotor1IN4 8

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setPins();
}

void loop() {
  // put your main code here, to run repeatedly:

}

int Controller(){

  return 0;
}

void setDCMotors(int EN_DC_A, int EN_DC_B, int IN1, int IN2, int IN3, int IN4){

}
void setPins(){

  pinMode(enableEncoderA, INPUT);
  pinMode(enableEncoderB, INPUT);
  pinMode(enableMotorA, OUTPUT);
  pinMode(enableMotorA, OUTPUT);
  pinMode(dcMotor1IN1, OUTPUT);
  pinMode(dcMotor1IN2, OUTPUT);
  pinMode(dcMotor1IN3, OUTPUT);
  pinMode(dcMotor1IN4, OUTPUT);

}

float readMotorsEncoder(){

  float speedEncoderA = digitalRead(enableEncoderA);
  float speedEncoderB = digitlaRead(enableEncoderB);

  float encoderSpeed[2] = {speedEncoderA, speedEncoderB};

  return encoderSpeed;

}