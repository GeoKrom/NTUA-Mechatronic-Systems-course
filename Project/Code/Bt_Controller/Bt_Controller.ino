/* ARM CONTROLLER CODE*/
/* LIBRARY INCLUDES*/
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <EEPROM.h> //Used to store the value of q even when the Arduino is Closed 
#define pi 3.1415926
/* HARDWARE DEFINES */
#define NofSERVOS 4
#define NofJoints 3
#define FREQ 50 // Analog servos run at ~50 Hz updates
#define GripperPin 3
#define EnablePin 8
#define ReadyPin 9
/* MEMORY DEFINES */
#define Buffer_Size 8
#define NofPoints 8 //Memmory size
/* GRIPPER ANGLES DEFINES */
#define closed_angle 105
#define opened_angle 150

//Bluetooth communication Parameters
#define txPin 12      //Maybe 0 or 1
#define rxPin 13      //Maybe 0 or 1
SoftwareSerial BTSerial(rxPin, txPin);  // RX, TX
// #define rover_txPin 10      //Maybe 0 or 1
// #define rover_rxPin 9      //Maybe 0 or 1
// SoftwareSerial RoverSerial(rover_rxPin, rover_txPin);  // RX, TX
//Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Servo Calibration Section
const int SERVOMIN[] = {475,480,250,125}; // this is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX[] = {450,282,462,575}; // this is the 'maximum' pulse length count (out of 4096)
const int q_SERVOMIN[] = {40,0,0,0}; // this is the desired angle for the SERVOMIN pulse
const int q_SERVOMAX[] = {72,90,90,180}; // this is the desired angle for the SERVOMAX pulse

/* With calble on the left hand, joistick of left hand is one(1) and other is two(2) 
  1X: index 0 (x, axis)
  1Y: index 1 (y, axis)
  2X: index 3 (gripper)
  2Y: index 2 (z, axis)

  Based on the Forward and Inverse Kinematic the Joints used are:
  q[0]: Small Link (elbow)
  q[1]: Big Link (shoulder)
  q[2]: Z rotation 
*/

//DEBUG MODE
boolean DEBUG = false;

//Time Parameters
unsigned long commStartTime = 0;  //Current time of Communication
unsigned long commEndTime = 0;    //Last time of Communication
unsigned long saveStartTime = 0;  //Current time of Communication
unsigned long saveEndTime = 0;    //Last time of Communication
unsigned int savePeriod = 10000;   //Period of Backup Save on EEPROM in ms
unsigned long curr_time = 0;
unsigned long prev_time = 0;
unsigned int roverCommPeriod = 1000;   //Period of UART Comm
const int speedDelay = 10;        //ms (SERVO max speed is 1.6ms/1deg)
const int pointDelay = 1000;      //ms
const int gripperDelay = 100;     //ms

//Global Variables Used
boolean moveEnabled;

//Struct Used to transfer Data
struct Data_Package {
  int8_t pot_1X;
  int8_t pot_1Y;
  int8_t pot_2X;
  int8_t pot_2Y;
  int8_t Button1;  //Joint space to Cartesian space
  int8_t Button2;  //Button to store the values
  int8_t Button3;  //Button for Automatic Move
  int8_t Button4;  //Button for Automatic Move
};

//Union Used to transfer Data
union inputUnion {
  Data_Package data;
  byte dataLine[Buffer_Size];  //Taking 'Buffer_Size' bytes of data
};

//Struct for cartesian Position and kinematics
struct cartesian_position {
  int x;
  int y;
  int z;
};

// void readySignal(boolean val){
//   if(val){
//     digitalWrite(ReadyPin,HIGH);
//   }else{
//     digitalWrite(ReadyPin,LOW);
//   }
// }

class manipulator {
private:
  cartesian_position maximum;
  cartesian_position minimum;
  cartesian_position pos;
  const float L1 = 32.7; //mm
  const float L2 = 79.6; //mm
  const float L5 = 80.7; //mm
  const float L6 = 29.0; //mm
  const float L7 = 12.2; //mm
  const float L8 = 23.7; //mm
  int q_max[NofSERVOS], q_min[NofSERVOS]; //Joint Limits 
  int q_init[NofSERVOS];
  int q[NofJoints], gripper;
  boolean autoMode, controlMode;
  int Memory[NofSERVOS][NofPoints];
  int count;  //Stores the Number of saved Positions
public:
  manipulator() {

    autoMode = false;
    controlMode = false;
    count = 0;

    //q values based on the Kinematic Analysis
    q_init[0] = 60; 
    q_init[1] = 60;
    q_init[2] = 0;
    q_init[3] = opened_angle; //Need to find those
    q_max[0] = 72; //LIMIT ON THOSE!!
    q_max[1] = 90; //LIMIT ON THOSE!!
    q_max[2] = 90; //LIMIT ON THOSE!!
    q_max[3] = opened_angle;
    q_min[0] = 38; //LIMIT ON THOSE!!
    q_min[1] = 0;  //LIMIT ON THOSE!!
    q_min[2] = -90;//LIMIT ON THOSE!!
    q_min[3] = closed_angle;

    int q_memory[NofSERVOS];
    boolean init_pos = true;
    for(int i=0; i<NofSERVOS; i++){
      q_memory[i] = (int)map(EEPROM.read(i),0,180,-90,90);

      if(q_memory[i] != q_init[i]){
        init_pos = false;
      }
    }

    if(!init_pos){
      for(int i=0;i<NofSERVOS;i++){
        q[i] = q_memory[i];
      }
      if(q_memory[3]>0){
        gripper = opened_angle;
      }else{
        gripper = closed_angle;
      }
    }

    //Setting Maximum Limit of Cartesian Space
    int q_forMax [NofJoints];
    q_forMax[0] = 38;
    q_forMax[1] = 0;
    q_forMax[2] = 0;
    maximum = forwardKinematic(q_forMax);
    int q_forMin [NofJoints];
    q_forMin[0] = 38;
    q_forMin[1] = 90;
    q_forMin[2] = 90;
    minimum = forwardKinematic(q_forMin);
  }

  boolean initPosition(){
    boolean init_pos = true;
    for(int i=0; i<NofJoints; i++){
      if(q[i] != q_init[i]){
        init_pos = false;
      }
    }
    if(gripper != q_init[3]){
      init_pos = false;
    }   
    return init_pos;
  }

  float deg2rad(float val){
    return (val/180.0)*pi;
  }

  float rad2deg(float val){
    return (val/pi)*180;
  }

  float int2float(int v,float fmin,float fmax){
    //Used to convert the incoming byte ot Cartesian position into x,y,z coord
    float val = map((float)v,0.0,255.0,fmin,fmax);
    return val;
  }
  
  int angleToPulse(float ang,int i){
    //Mapping function
    int pulse = (int)map(ang, q_SERVOMIN[i], q_SERVOMAX[i], SERVOMIN[i], SERVOMAX[i]);// map angle of 0 to 180 to Servo min and Servo max 
    // if(DEBUG){
    //   Serial.print("Angle: ");Serial.print(ang);
    //   Serial.print(" pulse: ");Serial.println(pulse);
    // }
    return pulse;
  }

  void open_gripper() {
    gripper = opened_angle;  //45 degrees opens the gripper

    //Arm Gripper control Actuation
    pwm.setPWM(GripperPin, 0, angleToPulse(gripper,GripperPin));
    delay(5);
  }

  void close_gripper() {
    gripper = closed_angle;  //23 degrees closes the gripper

    //Arm Gripper control Actuation
    pwm.setPWM(GripperPin, 0, angleToPulse(gripper,GripperPin));
    delay(5);
  }

  void setControlMode(boolean mode) {
    controlMode = mode;
  }

  boolean getControlMode() {
    //Returns 0 for Joint Space and 1 for Cartesian space
    return controlMode;
  }

  void setAutoMode(boolean mode){
    autoMode = mode;
  }

  boolean getAutoMode(){
    //Returns 0 for Teleoperation and 1 for Control from learned points
    return autoMode;
  }

  void jointsToEEPROM(){
    for(int i=0; i<NofJoints; i++){
      byte value = (byte)map(q[i],-90,90,0,180);
      EEPROM.update(i, value);
    }
    EEPROM.update(NofJoints, gripper);
  }

  void storePosition(boolean save){
    boolean newPos = false;
    if (save) {
      if (count == 0) {
        //First Save
        newPos = true;
      } else {
        //Next Saves
        for (int i = 0; i < NofJoints; i++) {
          if (q[i] != Memory[i][count - 1]) {
            newPos = true;
            break;
          }
        }
        if (gripper != Memory[NofSERVOS - 1][count - 1]) {
          newPos = true;
        }
      }
    }

    //Saving the Position into the Memory Matrix
    if (newPos) {
      for (int i = 0; i < NofJoints; i++) {
        Memory[i][count] = q[i];
      }
      Memory[NofSERVOS - 1][count] = gripper;
      count++;
    }
  }

  void move() {
    //This Method Does the job and takes the q[0-2] int PWM value mapping degrees to PWM length
    //Arm Joints control
    for (int i = 0; i < NofJoints; i++) {
      int pulse = angleToPulse(q[i],i);
      // if(DEBUG){
      //   Serial.print("SETTING PWM with Pulse: ");Serial.print(i);
      //   Serial.print("_"); Serial.println(pulse);
      // }
      pwm.setPWM(i, 0, pulse);
      delay(2); //Small delay for compatibility
    }
  }

  cartesian_position forwardKinematic(int* joint_point) {
    //This methods uses the q of the manipulator to return the End Effector Position
    cartesian_position E;
    /* HERE COMES THE F-K*/
    E.x = cos(deg2rad(joint_point[2]))*(L6+L8+L5*cos(deg2rad(joint_point[0]))+L2*cos(deg2rad(joint_point[1])));
    E.y = sin(deg2rad(joint_point[2]))*(L6+L8+L5*cos(deg2rad(joint_point[0]))+L2*cos(deg2rad(joint_point[1])));
    E.z = -L7-L5*sin(deg2rad(joint_point[0]))+L2*sin(deg2rad(joint_point[1]));
    /* HERE ENDS THE F-K*/
    return E;
  }

  int* inverseKinematic(cartesian_position P) {
    //This methods uses the given x,y,z coordinates of the class and returns an int array of joints
    static int joint_point[NofJoints];
    /* HERE COMES THE I-K*/
    float A = sqrt(pow(P.x,2)+pow(P.y,2))-L6-L8;
    float B = P.z+L7 ;
    float C = (pow(A,2)+pow(B,2)-pow(L5,2)-pow(L2,2))/(2*L5*L2);

    joint_point[1] = rad2deg(atan2(B,A)+atan(L5*sqrt(1-pow(C,2))/(L2+L5*C)));
    joint_point[0] = rad2deg(acos(C)-deg2rad(joint_point[1]));
    joint_point[2] = rad2deg(atan2(P.y,P.x));
    /* HERE ENDS THE I-K*/
    return joint_point;
  }

  void automaticMode() {
    //Movement based on the Learned Points
    // readySignal(false);
    //Variables used Init
    boolean pointsReached[NofJoints];
    boolean newPointReached = false;
    for (int k = 0; k < count; k++) {
      //Setting the pointsReached Array to false
      int q_ref[NofJoints];
      for(int i=0;i<NofJoints;i++){
        q_ref[i] = Memory[i][k];
      }
      smoothMotion(q_ref);
      //Changing the gripper place
      if (Memory[4][k]<opened_angle){
        close_gripper();
      } else {
        open_gripper();
      }
      delay(pointDelay);
    }
    //Saved Movement Finished
    toInitPosition();
  }

  void telemanipulation(int* in, boolean grip) {
    //Telemanipulation Control
    // readySignal(false);
    //Variables used Init
    boolean pointsReached[NofJoints];
    boolean newPointReached = false;


    if (!controlMode) {
      //Joint space Telemanipulation
      smoothMotion(in);

    } else {
      //Cartesian space Telemanipulation
      cartesian_position pos_ref;
      pos_ref.x = int2float(in[0],minimum.x,maximum.x);
      pos_ref.y = int2float(in[1],minimum.y,maximum.y);
      pos_ref.z = int2float(in[2],minimum.z,maximum.z);
      int* q_ref = inverseKinematic(pos_ref);

      smoothMotion(q_ref);

    }

    //Gripper Handling
    delay(gripperDelay);

    if (!grip) {
      close_gripper();
    } else {
      open_gripper();
    }
    // readySignal(true);
  }

  void smoothMotion(int* input){
    //Variables used Init
    boolean pointsReached[NofJoints];
    boolean newPointReached = false;

    while(!newPointReached){
      for (int i = 0; i < NofJoints; i++) {
        if (q[i] > input[i]) {
          pointsReached[i] = false;
          q[i]--;
        } else if (q[i] < input[i]) {
          pointsReached[i] = false;
          q[i]++;
        }else{
          pointsReached[i] = true;
        }
      }

      //Drive Action and Calculation of the end Effector Position
      move();
      pos = forwardKinematic(q);

      //Speed Delay and Finish condition
      delay(speedDelay);
      newPointReached = (pointsReached[0] && pointsReached[1] && pointsReached[2]);
    }
  }

  void toInitPosition(){
    //Setting the Robot to initial comfiguration
    smoothMotion(q_init);
    close_gripper();
    //readySignal(true);
  }
  void reset(){
    for(int i=0;i<NofJoints;i++){
      q[i] = q_init[i];
    }
    move();
    close_gripper();

  }
};

//Manipulator & Communication Buffer Init
manipulator RobotArm;
inputUnion inputBuffer;

//Setup function
void setup() {

  //BT Communication Init
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  BTSerial.begin(38400);

  //Debugging Init
  Serial.begin(38400);

  //Signal Pins Init
  pinMode(EnablePin,INPUT);
  moveEnabled = true;
  pinMode(ReadyPin,OUTPUT);

  //Driver Init
  pwm.begin();
  pwm.setPWMFreq(FREQ);  // Analog servos run at ~50 Hz updates

  //Robot on Initial Position
  if(RobotArm.initPosition()){
    RobotArm.reset();
  }else{
    RobotArm.toInitPosition();
  }

  delay(1000);
}


void loop() {
  //Testing The Enable Input before communication
  // if(!DEBUG) moveEnabled = digitalRead(EnablePin); //CAN BECOME TRUE ONLY IF I AM READY!!!!
  //ALTERNATIVE: moveEnabled =inputBuffer.data.Button4>0;

  /*COMMUNICATION Section*/

  // Data Parcing
  BTcommunication();
  //Robot Mode and Control Method setting
  RobotArm.setControlMode(inputBuffer.data.Button1>0);
  RobotArm.setAutoMode(inputBuffer.data.Button3>0);
  //Robot Position Save
  RobotArm.storePosition(inputBuffer.data.Button2>0);
  if(inputBuffer.data.Button4){
    RoverCommunication();
  }
  

  /*ARM MOTION Section*/
  if (inputBuffer.data.Button4) {
    if (!RobotArm.getAutoMode()){
      // Telemanipulation Mode
      int in[3];
      in[0] = map((int)inputBuffer.data.pot_1X,0,255,-128,127);
      in[1] = map((int)inputBuffer.data.pot_1Y,0,255,-128,127);
      in[2] = map((int)inputBuffer.data.pot_2X,0,255,-128,127);
      boolean gripper = map((int)inputBuffer.data.pot_2Y,0,255,-128,127) > 0;

      RobotArm.telemanipulation(in, gripper);

    } else {
      //Automatic Mode
      RobotArm.automaticMode();
      RobotArm.toInitPosition();
    }
  } else {
    //Stopped from outside due to rover signal
    //Fault Handle
    if(RobotArm.getAutoMode())RobotArm.toInitPosition();
  }


  /*EEPROM SAVE Section*/
  saveStartTime = millis();
  if(saveStartTime-saveEndTime>savePeriod){
    RobotArm.jointsToEEPROM();
    saveEndTime = saveStartTime;
  }

}
void RoverCommunication(){
  curr_time = millis();
  if(curr_time-prev_time>roverCommPeriod){
    //Writing to Tx Rx PINS for UART Communication
    Serial.write(inputBuffer.dataLine,Buffer_Size);
    prev_time = curr_time;
  }
}

void BTcommunication() {
  // Check whether we keep receving data, or we have a connection between the two modules
  commStartTime = millis();
  // if(DEBUG){
  //   Serial.print("Communication Delay:");
  //   Serial.println(commStartTime-commEndTime);
  //   if (BTSerial.available() < Buffer_Size)
  //   Serial.print("Wrong Buffer Size");
  // }
  if (BTSerial.available() >= Buffer_Size) {
    //Reading Buffer and transfering the data into the Union
    for (int i = 0; i < Buffer_Size; i++) {
      inputBuffer.dataLine[i] = BTSerial.read();  //Reading bytes from buffer 1 by 1
    }
    if(DEBUG){
      for (int i = 0; i < Buffer_Size; i++) {
          Serial.print("Index ");
          Serial.print(i);
          Serial.print(" is ");
          if(i<NofSERVOS){
            Serial.println(map((uint8_t)inputBuffer.dataLine[i],0,255,-128,127));
          }else{
            Serial.println((int)inputBuffer.dataLine[i]);
          }
          
      }
      Serial.println("============");
      delay(500);
    }
    commEndTime = millis();  // time we have received the data
  }
}
