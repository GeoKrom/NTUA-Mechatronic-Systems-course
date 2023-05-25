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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

}


void readMotorEncoders(){
  int b1 = digitalRead(EnA);
  int b2 = digitalRead(EnB);
}