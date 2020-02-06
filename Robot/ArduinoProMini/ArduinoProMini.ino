#include <Servo.h>
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
/* If the Encoder API is not installed in your Arduino Environement:
  - Go to 'Sketch' -> 'Include Library' -> 'Manage Libraries...'
  - Search 'Encoder'
  - Install 'Encoder by Paul Stoffregen' (This project uses version 1.4.1)
*/
#include <Encoder.h>
/* If the PID API is not installed in your Arduino Environment:
  - Go to 'Sketch' -> 'Include Library' -> 'Manage Libraries...'
  - Search 'PID'
  - Install 'PID by Brett Beauregard' (This project uses version 1.2.0)
*/
#include <PID_v1.h>

/*==============================GLOBAL VARS===================================*/
//Motor Pins
const unsigned char leftPin = 5;
const unsigned char rightPin = 6;
const unsigned char armPin = 9;

// Aux Pins for Motors or I/O
// NOTE: Uncomment these if you're using them
// const unsigned char aux1Pin = 10;
// const unsigned char aux2Pin = 11;

// Lights
const unsigned char boardLedPin = 13;
// LED Strip
const unsigned char ledPinBlue = 16;
const unsigned char ledPinRed = 14;
const unsigned char ledPinGreen = 15;
enum LEDColor {
  OFF,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  PURPLE,
  BLUEGREEN,
  WHITE
};

// Encoder Pins
const unsigned char inttEncPinA = 2;
const unsigned char inttEncPinB = 3;
// NOTE: Not using the polling encoder
// const unsigned char pollEncPinA = 7;
// const unsigned char pollEncPinB = 8;

// Using Servo API for ESCs
Servo leftSrvo;
Servo rightSrvo;
Servo armSrvo;
// NOTE: Uncomment these if you want to run additional motors
// Servo aux1Srvo;
// Servo aux2Srvo;

// If MANUAL, arm input = motor direction and speed (default)
// If ANGLE, arm input = angle
// If HOLD, arm input = amount by which to change the PID setpoint
enum ArmMode {
  ARM_MODE_MANUAL,
  ARM_MODE_ANGLE,
  ARM_MODE_HOLD
};
ArmMode armMode = ARM_MODE_MANUAL;

// The encoder value at which the arm is considered to be
// all the way down at "zero" angle.
int armZeroPoint = 0;

// The multiplier used to convert the arm angle (0 to 250)
// to the desired encoder value.
double armAngleScale = 20;

// The multiplier used to convert the arm joystick input (0 to 254)
// to an incremental change in the hold setpoint.
double armHoldScale = 0.5;

// Encoders
Encoder inttEnc(inttEncPinB, inttEncPinA);
// NOTE: Not using the polling encoder
// Encoder pollEnc(pollEncPinA, pollEncPinB);

// Interrupt limiting
int interruptsDisabledLoopCount = 0;
int numLoopsBetweenActivatingInterrupts = 1;

// PID object for arm
double myPID_Setpoint, myPID_Input, myPID_Output;
double Kp=0.43, Ki=0.0001, Kd=0.05;  // Gains
PID myPID(&myPID_Input, &myPID_Output, &myPID_Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);

// Setpoint for the arm when in the hold mode. This value gets set
// to the arm's current position upon entering the hold mode.
double holdSetpoint = 0;

// For checking if comms are lost
unsigned long lastTimeRX = 0;

// Used for comms protocol
bool waitingForStartByte = true;

// Used in local testing
int loopCount = 0;

/*=================================SET UP=====================================*/
void setup() {
  // 57600 baud, pin 13 is an indicator LED
  pinMode(boardLedPin, OUTPUT);
  Serial.begin(57600);
  Serial.setTimeout(510);

  // Set the modes on the motor pins
  leftSrvo.attach(leftPin);
  rightSrvo.attach(rightPin);
  armSrvo.attach(armPin);

  // Set the modes of the feedback LED pins
  pinMode(ledPinBlue,OUTPUT);
  pinMode(ledPinRed,OUTPUT);
  pinMode(ledPinGreen,OUTPUT);

  // Write initial values to the pins
  leftSrvo.writeMicroseconds(1500);
  rightSrvo.writeMicroseconds(1500);
  armSrvo.writeMicroseconds(1500);
  
  digitalWrite(ledPinBlue, LOW);
  digitalWrite(ledPinRed, LOW);
  digitalWrite(ledPinGreen, LOW);

  // Set up arm PID
  myPID_Setpoint = 0.0;
  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(-500, 500);

  // Give a few blinks to show that the code is up and running
  blinkBoardLed(2, 200);
}

/*=================================LOOP=======================================*/
void loop() {
  // Poll the encoder
  // NOTE: Not using the polling encoder
  // inttEnc.read();

  if (waitingForStartByte) {
    // Look for the start byte (255, or 0xFF)
    if (serialRead() == 255) {
      lastTimeRX = millis();
      waitingForStartByte = false;
    }
  }

  if (!waitingForStartByte && serialAvailable(3)) {
    waitingForStartByte = true;
    // Read the message body (3 bytes)
    int left = serialRead();
    int right = serialRead();
    int arm = serialRead();
    if (left == 126 && right == 126 && arm == 126) {
      processSetup();
    } else if (left < 255 && right < 255 && arm < 255) {
      processCmd(left, right, arm);
    }
  }
 
  if (millis() - lastTimeRX > 250) {
    idle();
  }
}


/*============================CUSTOM FUNC=====================================*/
void processCmd(int left, int right, int arm) {
  // Debug output
//  Serial.print("L: ");
//  Serial.print(left);
//  Serial.print(", R:");
//  Serial.print(right);
//  Serial.print(", A:");
//  Serial.print(arm);
//  Serial.print(", Enc:");
//  Serial.print(inttEnc.read());
//  Serial.print(", Count:");
//  Serial.print(++loopCount);
//  Serial.println("");

  // Indicate that we have signal by illuminating the on-board LED
  digitalWrite(boardLedPin, HIGH);

  // Special values: 123 to 127
  // If left/right/arm all == 126, then it is a setup cmd - see main loop
  if (arm == 123) {        // reset current arm encoder value as zero point
    armZeroPoint = inttEnc.read();
    setLEDColor(BLUEGREEN);
  } else if (arm == 124) { // set arm in angle mode
    armMode = ARM_MODE_ANGLE;
    myPID.SetMode(AUTOMATIC);
  } else if (arm == 125) { // set arm in manual mode (default)
    armMode = ARM_MODE_MANUAL;
    myPID.SetMode(MANUAL);
  } else if (arm == 127) { // set arm in hold mode
    armMode = ARM_MODE_HOLD;
    // Reset the hold setpoint to the current arm position.
    holdSetpoint = (double) (inttEnc.read() - armZeroPoint);
    myPID.SetMode(AUTOMATIC);
  } else if (armMode == ARM_MODE_ANGLE) {
    setArmAngle(arm);
  } else if (armMode == ARM_MODE_HOLD) {
    moveArmHold(arm);
  } else {
    moveArm(arm);
  }
  runWheels(left, right);
}

int runArmPID(double setpoint) {
  myPID_Setpoint = setpoint;

  // Subtract the armZeroPoint from the current encoder value
  // to get the current encoder angle.
  myPID_Input = (double) (inttEnc.read() - armZeroPoint);
  myPID.Compute();

  // Cap the PID command at the endpoints of the command range
  double armMicroseconds = 1500 - myPID_Output;
  if (armMicroseconds > 2000) armMicroseconds = 2000;
  if (armMicroseconds < 1000) armMicroseconds = 1000;

  // Debug output
//  Serial.print("  PID setpoint: ");
//  Serial.print(myPID_Setpoint);
//  Serial.print(", input: ");
//  Serial.print(myPID_Input);
//  Serial.print(", output:");
//  Serial.print(armMicroseconds);
//  Serial.println("");

  return ((int) armMicroseconds);
}

void setArmAngle(int arm) {
  // Omit the reserved values listed in processCmd (123 - 127)
  // so that the arm value represents a contiguous range of values.
  // arm == 128 is valid angle, but represents idle.
  if (arm > 123) arm -= 5;

  // Multiply the desired arm angle by armAngleScale
  // to get the desired encoder angle.
  double setpoint = ((double) arm) * armAngleScale;

  // Run the PID, and set the arm motor value accordingly.
  armSrvo.writeMicroseconds(runArmPID(setpoint));
  setLEDColor(YELLOW);
}

void moveArmHold(int arm) {
  // Recenter the arm command from (0, 254) to (-127, 127)
  double armRecentered = (double)arm - 127.0;

  // Multiply the arm command by the armHoldScale to get the
  // desired incremental change to the hold setpoint.
  holdSetpoint = holdSetpoint + armRecentered * armHoldScale;

  // Run the PID, and set the arm motor value accordingly.
  armSrvo.writeMicroseconds(runArmPID(holdSetpoint));
  setLEDColor(PURPLE);
}

void moveArm(int arm) {
  arm = map(arm, 0, 254, 1000, 2000);
  armSrvo.writeMicroseconds(arm);

  if (arm > 1505) {
    setLEDColor(BLUE);
  } else if(arm < 1495) {
    setLEDColor(GREEN);
  } else {
    setLEDColor(WHITE);
  }
}

void runWheels(int left, int right) {
  left = map(left, 0, 254, 1000, 2000);
  right = map(right, 0, 254, 1000, 2000);

  leftSrvo.writeMicroseconds(left);
  rightSrvo.writeMicroseconds(right);
}

void idle() {
  // Set all motors to neutral
  rightSrvo.writeMicroseconds(1500);
  leftSrvo.writeMicroseconds(1500);
  armSrvo.writeMicroseconds(1500);

  // Indicate that we have lost comms by turning off the on-board LED
  digitalWrite(boardLedPin, LOW);
  setLEDColor(RED);
}

void setLEDColor(LEDColor color) {
  switch (color) {
    case OFF:
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinGreen, LOW);
      digitalWrite(ledPinBlue, LOW);
      break;
    case RED:
      digitalWrite(ledPinRed, HIGH);
      digitalWrite(ledPinGreen, LOW);
      digitalWrite(ledPinBlue, LOW);
      break;
    case GREEN:
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinGreen, HIGH);
      digitalWrite(ledPinBlue, LOW);
      break;
    case BLUE:
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinGreen, LOW);
      digitalWrite(ledPinBlue, HIGH);
      break;
    case YELLOW:
      digitalWrite(ledPinRed, HIGH);
      digitalWrite(ledPinGreen, HIGH);
      digitalWrite(ledPinBlue, LOW);
      break;
    case PURPLE:
      digitalWrite(ledPinRed, HIGH);
      digitalWrite(ledPinGreen, LOW);
      digitalWrite(ledPinBlue, HIGH);
      break;
    case BLUEGREEN:
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinGreen, HIGH);
      digitalWrite(ledPinBlue, HIGH);
      break;
    case WHITE:
      digitalWrite(ledPinRed, HIGH);
      digitalWrite(ledPinGreen, HIGH);
      digitalWrite(ledPinBlue, HIGH);
      break;
  }
}

void blinkLED(int count, int duration, LEDColor color) {
  for (int index = 0; index < count; ++index) {
    setLEDColor(color);
    delay(duration);
    setLEDColor(WHITE);
    delay(duration);
  }
}

void blinkBoardLed(int count, int duration) {
  for (int index = 0; index < count; ++index) {
    digitalWrite(boardLedPin, HIGH);
    delay(duration);
    digitalWrite(boardLedPin, LOW);
    delay(duration);
  }
}

// Specialized command for setting up the PID controller.
// Command format:
//   255, 126, 126, 126,
//     <Proportional char>,    <--- either "E" for proportional on error
//                                      or "M" for proportional on measurement
//     <# chars> <Kp>,         <--- string representing value of Kp
//     <# chars> <Ki>,         <--- string representing value of Ki
//     <# chars> <Kd>,         <--- string representing value of Kd
//     <# chars> <armAngleScale>,   <--- string representing value of armAngleScale
//     <checksum>              <--- sum all chars excluding lengths & 0xFF
void processSetup() {
  // Simple checksum
  // (If needed, consider Fletcher checksum, instead)
  int checksum = 0;

  waitSerialAvailable(1);
  char proportional = serialRead();
  checksum += proportional;

  // Read 5 strings and convert to doubles
  double values[5];
  for (int valueIndex = 0; valueIndex < 5; ++valueIndex) {
    // Read string as # characters followed by characters
    waitSerialAvailable(1);
    int strLen = serialRead();
    char strChars[strLen + 1];
    waitSerialAvailable(strLen);
    for (int strIndex = 0; strIndex < strLen; ++strIndex) {
      char ch = serialRead();
      strChars[strIndex] = ch;
      checksum += ch;
    }
    strChars[strLen] = 0;
    String valueStr = String(strChars);
    values[valueIndex] = valueStr.toDouble();
  }

  // Simple sanity check - abort if checksum does not match
  waitSerialAvailable(1);
  if ((checksum & 0xFF) != serialRead()) {
    blinkLED(3, 250, RED);
    return;
  }

  Kp = values[0];
  Ki = values[1];
  Kd = values[2];
  armAngleScale = values[3];
  armHoldScale = values[4];
  int pOn;
  if (proportional == 'E') {
    pOn = P_ON_E;
  } else {
    pOn = P_ON_M;
  }
  myPID.SetTunings(Kp, Ki, Kd, pOn);
  blinkLED(5, 50, BLUEGREEN);
  setLEDColor(BLUEGREEN);
}

/*============================LOCAL TEST=====================================*/
//int serialData[] = {255, 127, 127, 128};
//int serialIndex = 0;
//
//char setupData[][10] = { // max string size + 1 for null terminator
// {(char) 255, 126, 126, 126, 'E'}, // setup signal + proportional
// "0.1", // Kp
// "0", // Ki
// "0", // Kd
// "40", // armAngleScale
// "1", // armHoldScale
//};
//int setupChecksum = 152;
//int setupIndex = 0;
//int setupCount = -1;

// Block the main loop until the specified number of bytes is available
void waitSerialAvailable(int count) {
  while (!serialAvailable(count)) {
    delay(1);
  }
}

bool serialAvailable(int count) {
  return Serial.available() >= count;

  // Local test
// return true;
}

int serialRead() {
  return Serial.read();

//  // Local test
//  // Simulate client setup command
//  if (setupCount >= 0) {
//    while (setupCount < 5) {
//      if (setupIndex == -1) {
//        ++setupIndex;
//        return String(setupData[setupCount]).length();
//      }
//      unsigned char ch = setupData[setupCount][setupIndex++];
//      if (ch != 0) {
//        return ch;
//      }
//      setupIndex = -1;
//      ++setupCount;
//    }
//    setupCount = -1;
//    return setupChecksum;
//  }
//
//  // Simulate joystick input from the keyboard
//  if (Serial.available() > 0) {
//    int serialCmd = Serial.read();
//    Serial.println(serialCmd);
//
//    if (serialCmd == '0') {               // send setup message
//      setupCount = 0;
//
//    } else if (serialCmd == 'f') {        // forward
//      serialData[1] = min(serialData[1] + 10, 254);
//      serialData[2] = min(serialData[2] + 10, 254);
//    } else if (serialCmd == 'g') {        // forward small
//      serialData[1] = min(serialData[1] + 2, 254);
//      serialData[2] = min(serialData[2] + 2, 254);
//    } else if (serialCmd == 'h') {        // forward full
//      serialData[1] = 254;
//      serialData[2] = 254;
//    } else if (serialCmd == 'r') { // reverse
//      serialData[1] = max(serialData[1] - 10, 0);
//      serialData[2] = max(serialData[2] - 10, 0);
//    } else if (serialCmd == 't') { // reverse small
//      serialData[1] = max(serialData[1] - 2, 0);
//      serialData[2] = max(serialData[2] - 2, 0);
//    } else if (serialCmd == 'y') { // reverse full
//      serialData[1] = 0;
//      serialData[2] = 0;
//    } else if (serialCmd == 's') { // stop wheels
//      serialData[1] = 128;
//      serialData[2] = 128;
//
//    } else if (serialCmd == 'z') { // reset arm zero point
//      serialData[3] = 123;
//    } else if (serialCmd == 'x') { // put arm in manual mode
//      serialData[3] = 125;
//    } else if (serialCmd == 'c') { // put arm in angle mode
//      serialData[3] = 124;
//    } else if (serialCmd == 'v') { // put arm in hold mode
//      serialData[3] = 127;
//
//    } else if (serialCmd == 'u') {  // up
//      int arm = min(serialData[3] + 10, 254);
//      if (arm >= 123 && arm <= 127) arm = 128;
//      serialData[3] = arm;
//    } else if (serialCmd == 'i') {  // up small
//      int arm = min(serialData[3] + 1, 254);
//      if (arm >= 123 && arm <= 127) arm = 128;
//      serialData[3] = arm;
//    } else if (serialCmd == 'o') {  // up full
//      serialData[3] = 254;
//    } else if (serialCmd == 'j') {  // down
//      serialData[3] = max(serialData[3] - 10, 0);
//    } else if (serialCmd == 'k') {  // down small
//      serialData[3] = max(serialData[3] - 1, 0);
//    } else if (serialCmd == 'l') {  // down full
//      serialData[3] = 0;
//    } else if (serialCmd == 'm') {  // stop arm
//      serialData[3] = 128;
//     
//    } else if (serialCmd == '1') { // set arm angle to 1 degree
//      serialData[3] = 1;
//    } else if (serialCmd == '2') { // set arm angle to 10 degree
//      serialData[3] = 10;
//    } else if (serialCmd == '3') { // set arm angle to 80 degree
//      serialData[3] = 80;
//    } else if (serialCmd == '4') { // set arm angle to 120 degree
//      serialData[3] = 120;
//    }
//  }
//  int value = serialData[serialIndex];
//  ++serialIndex;
//  if (serialIndex > 3) {
//    serialIndex = 0;
//  }
//  return value;
}
