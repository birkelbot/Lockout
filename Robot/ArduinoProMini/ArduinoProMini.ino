#include <Servo.h>

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

// Using Servo API for ESCs
Servo leftSrvo;
Servo rightSrvo;
Servo armSrvo;

// For checking if comms are lost
unsigned long lastTimeRX = 0;

// Used for comms protocol
bool waitingForStartByte = true;

// Used in local testing/simulation
int simulatedSerialData[] = {255, 127, 127, 127};
int simulatedSerialCurrIndex = 0;
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

  // Give a few blinks to show that the code is up and running
  blinkBoardLed(2, 200);
}

/*=================================LOOP=======================================*/
void loop() {
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
    processCmd(left, right, arm);
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
//  Serial.print(", Count:");
//  Serial.print(++loopCount);
//  Serial.println("");

  // Indicate that we have signal by illuminating the on-board LED
  digitalWrite(boardLedPin, HIGH);

  runWheels(left, right);
  moveArm(arm);
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

bool serialAvailable(int count) {
  return Serial.available() >= count;

  /*=================LOCAL TEST=================*/
  /* Comment the lines above, and uncomment the */
  /* lines below to enable the local test.      */
  /* Also, do the same in serialRead().         */
  /*============================================*/
  // return true;
}

int serialRead() {
  return Serial.read();

  /*=================LOCAL TEST=================*/
  /* Comment the lines above, and uncomment the */
  /* lines below to enable the local test.      */
  /* Also, do the same in serialAvailable().    */
  /*============================================*/
  // // Simulate gamepad commands by reading keyboard keys entered into the buffer.
  // if (Serial.available() > 0) {
  //   int serialCmd = Serial.read();
  //   Serial.println(serialCmd);

  //   if (serialCmd == 'f') {        // forward
  //     simulatedSerialData[1] = min(simulatedSerialData[1] + 10, 254);
  //     simulatedSerialData[2] = min(simulatedSerialData[2] + 10, 254);
  //   } else if (serialCmd == 'g') {        // forward small
  //     simulatedSerialData[1] = min(simulatedSerialData[1] + 2, 254);
  //     simulatedSerialData[2] = min(simulatedSerialData[2] + 2, 254);
  //   } else if (serialCmd == 'h') {        // forward full
  //     simulatedSerialData[1] = 254;
  //     simulatedSerialData[2] = 254;
  //   } else if (serialCmd == 'r') { // reverse
  //     simulatedSerialData[1] = max(simulatedSerialData[1] - 10, 0);
  //     simulatedSerialData[2] = max(simulatedSerialData[2] - 10, 0);
  //   } else if (serialCmd == 't') { // reverse small
  //     simulatedSerialData[1] = max(simulatedSerialData[1] - 2, 0);
  //     simulatedSerialData[2] = max(simulatedSerialData[2] - 2, 0);
  //   } else if (serialCmd == 'y') { // reverse full
  //     simulatedSerialData[1] = 0;
  //     simulatedSerialData[2] = 0;
  //   } else if (serialCmd == 's') { // stop wheels
  //     simulatedSerialData[1] = 127;
  //     simulatedSerialData[2] = 127;

  //   } else if (serialCmd == 'u') {  // up
  //     int arm = min(simulatedSerialData[3] + 10, 254);
  //     simulatedSerialData[3] = arm;
  //   } else if (serialCmd == 'i') {  // up small
  //     int arm = min(simulatedSerialData[3] + 1, 254);
  //     if (arm >= 123 && arm <= 127) arm = 128;
  //     simulatedSerialData[3] = arm;
  //   } else if (serialCmd == 'o') {  // up full
  //     simulatedSerialData[3] = 254;
  //   } else if (serialCmd == 'j') {  // down
  //     simulatedSerialData[3] = max(simulatedSerialData[3] - 10, 0);
  //   } else if (serialCmd == 'k') {  // down small
  //     simulatedSerialData[3] = max(simulatedSerialData[3] - 1, 0);
  //   } else if (serialCmd == 'l') {  // down full
  //     simulatedSerialData[3] = 0;
  //   } else if (serialCmd == 'm') {  // stop arm
  //     simulatedSerialData[3] = 127;
  // }

  // int value = simulatedSerialData[simulatedSerialCurrIndex];
  // ++simulatedSerialCurrIndex;
  // if (simulatedSerialCurrIndex > 3) {
  //   simulatedSerialCurrIndex = 0;
  // }
  // return value;
}
