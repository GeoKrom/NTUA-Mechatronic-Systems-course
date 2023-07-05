/* MAIN ARDUINO CODE*/
#include <SoftwareSerial.h>
#include <math.h>
#define pi 3.1415926
#define Buffer_Size 8
#define NofJoints 3
#define NofButtons 3
#define closed_angle 0   //-128
#define opened_angle 255 // 127 //Bluetooth communication Parameters
const byte txPin = 13;   // Maybe 0 or 1
const byte rxPin = 12;   // Maybe 0 or 1
SoftwareSerial BTSerial(rxPin, txPin); // RX, TX
#define GripperPin 3                   // getting the pot_2Y Pin
int ButtonPin[] = {6, 7, 5, 9};        // SET THOSE CORRECTLY BASED ON CIRCUITs

// With calble on the left hand, joistick of left hand is one(1) and other is
// two(2) 1X: index 0 (x, axis) 1Y: index 1 (y, axis) 2X: index 3 (gripper) 2Y:
// index 2 (z, axis)

// Based on the Forward and Inverse Kinematic the Joints used are:
// q[0]: Small Link (elbow)
// q[1]: Big Link (shoulder)
// q[2]: Z rotation
const int readDelay = 10;

// DEBUG MODE
boolean DEBUG = false;

// Struct Used to transfer Data
struct Data_Package {
  byte pot_1X;
  byte pot_1Y;
  byte pot_2X;
  byte pot_2Y;
  byte Button1; // Joint space to Cartesian space
  byte Button2; // Button to store the values
  byte Button3; // Button for Automatic Move
  byte Button4; // Button for Rover Mode
};

// Union Used to transfer Data
union outputUnion {
  Data_Package data;
  byte dataLine[Buffer_Size]; // Taking 'Buffer_Size' bytes of data
};

// Object for cartesian Position and kinematics
class cartesian_position {
public:
  float x, y, z; // mm
  cartesian_position() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }

  void setPosition(int index, float value) {
    if (index == 0) {
      x = value;
    } else if (index == 1) {
      y = value;
    } else if (index == 2) {
      z = value;
    } else {
      return;
    }
  }

  float getPosition(int index) {
    if (index == 0) {
      return x;
    } else if (index == 1) {
      return y;
    } else if (index == 2) {
      return z;
    } else {
      return;
    }
  }
};

class controller {
private:
  cartesian_position maximum;
  cartesian_position minimum;
  cartesian_position pos;
  const float L1 = 32.7; // mm
  const float L2 = 79.6; // mm
  const float L5 = 80.7; // mm
  const float L6 = 29.0; // mm
  const float L7 = 12.2; // mm
  const float L8 = 23.7; // mm
  int q_max[NofJoints + 1], q_min[NofJoints + 1];
  int q_init[NofJoints + 1]; //+1 for the gripper
  boolean autoMode, controlMode, roverMode, store;
  int q[NofJoints], gripper;
  int q_memory[NofJoints + 1];

public:
  controller() {
    autoMode = false;
    controlMode = false;
    roverMode = false;

    // q values based on the Kinematic Analysis
    q_init[0] = 60;
    q_init[1] = 60;
    q_init[2] = 0;
    q_init[3] = opened_angle; // Need to find those
    q_max[0] = 72;            // LIMIT ON THOSE!!
    q_max[1] = 90;            // LIMIT ON THOSE!!
    q_max[2] = 90;            // LIMIT ON THOSE!!
    q_max[3] = opened_angle;
    q_min[0] = 38;  // LIMIT ON THOSE!!
    q_min[1] = -10; // 0;  //LIMIT ON THOSE!!
    q_min[2] = -90; // LIMIT ON THOSE!!
    q_min[3] = closed_angle;

    for (int i = 0; i < NofJoints; i++) {
      q[i] = q_init[i];
      q_memory[i] = q_init[i];
    }
    gripper = q_init[NofJoints];
    q_memory[NofJoints] = q_init[NofJoints];

    // Setting Maximum Limit of Cartesian Space
    int q_forMax[3];
    q_forMax[0] = 38;
    q_forMax[1] = 0;
    q_forMax[2] = 0;
    maximum = forwardKinematic(q_forMax);
    int q_forMin[3];
    q_forMin[0] = 38;
    q_forMin[1] = 90;
    q_forMin[2] = 90;
    minimum = forwardKinematic(q_forMin);

    pos = forwardKinematic(q);
  }

  float deg2rad(float val) { return (val / 180.0) * pi; }

  float rad2deg(float val) { return (val / pi) * 180; }

  int8_t float2int(float f, float fmin, float fmax) {
    float val = map(f, fmin, fmax, -128.0, 127.0);
    return (int8_t)round(val);
  }

  void readGripper() {
    int value = analogRead(GripperPin);
    if (value < 412) {
      store = false;
      gripper = closed_angle; // 45 degrees opens the gripper
    } else if (value > 612) {
      store = false;
      gripper = opened_angle; // 23 degrees closes the gripper
    }
  }

  void setControlMode(boolean mode) {
    if (mode) {
      // We are in Cartesian space and we want to go to
      // Joint space.
      int *q_ref = inverseKinematic(pos);
      for (int i = 0; i < NofJoints; i++) {
        q[i] = q_ref[i];
      }
    } else {
      pos = forwardKinematic(q);
    }
    controlMode = mode;
  }

  boolean getControlMode() {
    // Returns 0 for Joint Space and 1 for Cartesian space
    return controlMode;
  }

  void setAutoMode(boolean mode) { autoMode = mode; }

  boolean getAutoMode() {
    // Returns 0 for Teleoperation and 1 for Control from learned points
    return autoMode;
  }

  void setRoverMode(boolean mode) { roverMode = mode; }

  boolean getRoverMode() { return roverMode; }

  cartesian_position forwardKinematic(int *joint_point) {
    // This methods uses the q of the manipulator to return the End Effector
    // Position
    /* Start of the F-K*/
    cartesian_position E;
    E.x = cos(deg2rad(joint_point[2])) *
          (L6 + L8 + L5 * cos(deg2rad(joint_point[0])) +
           L2 * cos(deg2rad(joint_point[1])));
    E.y = sin(deg2rad(joint_point[2])) *
          (L6 + L8 + L5 * cos(deg2rad(joint_point[0])) +
           L2 * cos(deg2rad(joint_point[1])));
    E.z = -L7 - L5 * sin(deg2rad(joint_point[0])) +
          L2 * sin(deg2rad(joint_point[1]));
    /* End of the F-K*/
    return E;
  }

  int *inverseKinematic(cartesian_position P) {
    // This methods uses the given x,y,z coordinates of the class and returns an
    // int array of joints
    static int joint_point[NofJoints];
    /* Start of the I-K*/
    float A = sqrt(pow(P.x, 2) + pow(P.y, 2)) - L6 - L8;
    float B = P.z + L7;
    float C = (pow(A, 2) + pow(B, 2) - pow(L5, 2) - pow(L2, 2)) / (2 * L5 * L2);

    joint_point[1] =
        rad2deg(atan2(B, A) + atan(L5 * sqrt(1 - pow(C, 2)) / (L2 + L5 * C)));
    joint_point[0] = rad2deg(acos(C) - deg2rad(joint_point[1]));
    joint_point[2] = rad2deg(atan2(P.y, P.x));
    /* End of the I-K*/
    return joint_point;
  }

  void setStore(boolean trigger) { store = trigger; }

  boolean getStore() { return store; }

  void readJoint() { // Use Pin fro 0 to 3
    // Joystick Input

    for (int Pin = 0; Pin < NofJoints; Pin++) {
      int value = analogRead(Pin);
      if (value > 612) {
        // Using the simple way: Adding value while the signal is true
        if (q[Pin] > q_min[Pin]) {
          store = false;
          --q[Pin];
        }
        // TODO: Take speed from value
      } else if (value < 412) {
        // Using the simple way: Adding value while the signal is true
        if (q[Pin] < q_max[Pin]) {
          store = false;
          ++q[Pin];
        }
        // TODO: Take speed from value
      }
    }
    delay(readDelay);
  }

  void readCartesian() { // Use Pin fro 0 to 3
    for (int Pin = 0; Pin < NofJoints; Pin++) {
      // Joystick Input
      int value = analogRead(Pin);
      float delta =
          (maximum.getPosition(Pin) - minimum.getPosition(Pin)) / 10.0;

      // float delta = 1.0;
      if (value > 612) {
        // Using the simple way: Adding value while the signal is true
        if (pos.getPosition(Pin) > minimum.getPosition(Pin)) {
          store = false;
          pos.setPosition(Pin, pos.getPosition(Pin) - delta);
        }

        // TODO: Take speed from value
      } else if (value < 412) {
        // Using the simple way: Adding value while the signal is true
        if (pos.getPosition(Pin) < maximum.getPosition(Pin)) {
          store = false;
          pos.setPosition(Pin, pos.getPosition(Pin) + delta);
        }
        // TODO: Take speed from value
      }
    }
  }

  void readRoverCmds() {
    int fwd_val = analogRead(1); // PIN 0
    if (fwd_val > 612) {
      q[1] = int('B');
    } else if (fwd_val < 412) {
      q[1] = int('F');
    } else {
      q[1] = int('A');
    }

    int turn_val = analogRead(3); // PIN 2
    if (turn_val > 612) {
      q[2] = int('R');
    } else if (turn_val < 412) {
      q[2] = int('L');
    } else {
      q[2] = int('A');
    }
    Serial.println(String(q[1]) + " " + String(q[2]));
    Serial.println("---------");
    // delay(500);
  }

  void setPosition(int *q_new) {
    // Not used yet
    for (int i = 0; i < NofJoints; i++) {
      q[i] = q_new[i];
    }
  }

  void setMemoryPosition() {
    // Not used yet
    for (int i = 0; i < NofJoints; i++) {
      q_memory[i] = q[i];
    }
    q_memory[3] = gripper;
  }

  void getFromMemory() {
    for (int i = 0; i < NofJoints; i++) {
      q[i] = q_memory[i];
    }
    gripper = q_memory[3];
  }

  int *getPosition() {
    if (!controlMode) {
      // Joint Space
      //  if(DEBUG){
      //    Serial.println("Values of q:");
      //    Serial.println(q[0]);
      //    Serial.println(q[1]);
      //    Serial.println(q[2]);
      //  }
      return q;
    } else {
      // Cartesian Space
      if (DEBUG) {
        Serial.println("Values of q:");
        Serial.println(q[0]);
        Serial.println(q[1]);
        Serial.println(q[2]);
      }
      static int value[3];
      for (int i = 0; i < NofJoints; i++) {
        // Typecasting a float into a 0 to 255 integer
        value[i] = float2int(pos.getPosition(i), minimum.getPosition(i),
                             maximum.getPosition(i));
      }
      return value;
    }
  }

  int getGripper() { return gripper; }

  void reset() {
    // Sending to Robot the initial comfiguration
    controlMode = false;
    autoMode = false;
    q[0] = q_init[0];
    q[1] = q_init[1];
    q[2] = q_init[2];
    gripper = opened_angle;
  }
};

// Setup function
void setup() {

  // Communication Init
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  BTSerial.begin(38400);

  // Serial Init for DEBUG
  Serial.begin(38400);

  // Init PinModes for Buttons
  for (int i = 0; i < NofButtons; i++) {
    pinMode(ButtonPin[i], INPUT_PULLUP);
  }
}

// Joystick Init
controller Joystick;

// Communication Variables
outputUnion outputBuffer;
int8_t outData[Buffer_Size];
boolean error_flag = false;

// Time Variables
const int timeDelay = 30;                       // 30ms
const unsigned long communicationPeriod = 1000; // 1000ms
unsigned long current_time = 0;
unsigned long past_time = 0;

void loop() {

  if (!error_flag) {
    // Data Reading from Joystick

    // Reading Buttons
    readButton1();
    readButton2();
    readButton3();
    readButton4();
    // Reading Joysticks
    if (Joystick.getRoverMode()) {
      // Serial.println("Check Rover");
      Joystick.readRoverCmds();
    } else {
      if (!Joystick.getControlMode()) {
        // if(DEBUG) Serial.print("Check_Joint");
        Joystick.readJoint();
      } else {
        // if(DEBUG) Serial.println("Check_Cart");
        Joystick.readCartesian();
      }
      Joystick.readGripper();
    }

    // Communication every communicationPeriod ms
    current_time = millis();
    if (current_time - past_time >= communicationPeriod) {

      // Send Data to the Robot Arduino
      //  Serial.println("Sending DATA");
      sendData();
      if (Joystick.getStore()) {
        // Mayde open LED or something to show we stored the position
        if (DEBUG)
          Serial.println("SAVE BUTTON PRESSED");
        // Joystick.setStore(false);
      }

      past_time = current_time;
    }
  }
}

void sendData() {

  // Putting Data to buffer
  outputBuffer.data.pot_1X = (byte)map(Joystick.getPosition()[0],-128,127,0,255);
  outputBuffer.data.pot_1Y = (byte)map(Joystick.getPosition()[1],-128,127,0,255);
  outputBuffer.data.pot_2X = (byte)map(Joystick.getPosition()[2],-128,127,0,255);
  outputBuffer.data.pot_2Y = (byte)Joystick.getGripper();
  outputBuffer.data.Button1 = Joystick.getControlMode();
  outputBuffer.data.Button2 = Joystick.getStore();
  outputBuffer.data.Button3 = Joystick.getAutoMode();
  outputBuffer.data.Button4 = Joystick.getRoverMode();

  // Sending the Data
  BTSerial.write(outputBuffer.dataLine, Buffer_Size);
  // if(DEBUG){
  for (int i = 0; i < Buffer_Size/2 ; i++) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(" is ");
    Serial.println(map((int)outputBuffer.dataLine[i],0,255,-128,127));
  }
    for (int i = Buffer_Size /2; i < Buffer_Size ; i++) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(" is ");
    Serial.println(outputBuffer.dataLine[i]);
  }
  Serial.println("====================");
}

// Button Polling Section
void readButton1() {
  // Change Control Mode
  static boolean value_prev = false;
  boolean value = digitalRead(ButtonPin[0]);

  if (!value && value_prev) {
    Joystick.setControlMode(!Joystick.getControlMode());
  }
  value_prev = value;
}

void readButton2() {
  static boolean value_prev = false;
  boolean value = digitalRead(ButtonPin[1]);

  if (!value && value_prev) {
    Joystick.setStore(true);
  }
  value_prev = value;
}

void readButton3() {
  // Change Auto Mode
  static boolean value_prev = false;
  boolean value = digitalRead(ButtonPin[2]);

  if (!value && value_prev) {
    Joystick.setAutoMode(!Joystick.getAutoMode());
  }
  value_prev = value;
}

void readButton4() {
  // Change to ROVER
  static boolean value_prev = false;
  boolean value = digitalRead(ButtonPin[3]);

  boolean is_in_rover_mode = Joystick.getRoverMode();

  if (!value && value_prev) {
    if (is_in_rover_mode) {
      Joystick.getFromMemory();
    } else {
      Joystick.setMemoryPosition();
    }
    Joystick.setRoverMode(!is_in_rover_mode);    
  }
  
  value_prev = value;

  // 
}
