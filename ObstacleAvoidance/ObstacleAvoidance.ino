#include "DualVNH5019MotorShield.h"
#include "PinChangeInterrupt.h"
#include "PID_v1.h"

#define LEFT_ENCODER 3 //left motor encoder A to pin 3
#define RIGHT_ENCODER 5 //right motor encoder A to pin 5
#define inputB 10 //right motor speed input
#define inputA 9; //left motor speed input

DualVNH5019MotorShield md;

double leftEncoderValue = 0;
double rightEncoderValue = 0;
double difference;
double Setpoint, Input, Output;
double turnLeftEncoderValue, turnRightEncoderValue, startLeftEncoderValue, startRightEncoderValue;

double Kp = 0.5, Ki = 0.0, Kd = 0.0;
//562.25 square wave = one revolution
PID myPID(&leftEncoderValue, &Output, &rightEncoderValue, Kp, Ki, Kd, DIRECT);
//PID(&input, &output, &setpoint, Kp, Ki, Kd, Direction)
//Parameters: Input - the variable we are trying to control
//          : Output - the variable that willl be adjusted by PID
//          : Setpoint - the value we want the input to maintain
//          : Direction - either DIRECT or REVERSE

// # Connection:
// #       Pin 1 VCC (URM V3.2) -> VCC (Arduino)
// #       Pin 2 GND (URM V3.2) -> GND (Arduino)
// #       Pin 4 PWM (URM V3.2) -> Pin 3 (Arduino)
// #       Pin 6 COMP/TRIG (URM V3.2) -> Pin 5 (Arduino)
// #
int URPWM = 6; // PWM Output 0－25000US，Every 50US represent 1cm
int URTRIG = 11; // PWM trigger pin

unsigned int Distance = 0;
uint8_t EnPwmCmd[4] = {0x44, 0x02, 0xbb, 0x01}; // distance measure command

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //md = new DualVNH5019MotorShield();
  md.init();
  pinMode (LEFT_ENCODER, INPUT); //set digital pin 3 as input
  pinMode (RIGHT_ENCODER, INPUT); //set digital pin 5 as input
  attachPCINT(digitalPinToPCINT(LEFT_ENCODER), leftEncoderInc, HIGH);
  attachPCINT(digitalPinToPCINT(RIGHT_ENCODER), rightEncoderInc, HIGH);
  myPID.SetOutputLimits(-50, 50);
  myPID.SetMode(AUTOMATIC);

//  PWM_Mode_Setup();

}

void loop() {
  // put your main code here, to run repeatedly:

//  PWM_Mode();

  md.setSpeeds(250 + Output, 250 - Output);

  myPID.Compute();

  Serial.print("Left:");
  Serial.print(leftEncoderValue);
  Serial.print(", Right:");
  Serial.print(rightEncoderValue);
  Serial.print(", Diff:");
  Serial.println(Output);

///  delay(3000);
  if((leftEncoderValue > 5050.00) ||(rightEncoderValue > 5050.00)) {
    turnRight();
    printAll();
    goPastObstacle();
    printAll();
    turnLeft();
    printAll();
    goPastObstacle();
    printAll();
    turnLeft();
    printAll();
    goPastObstacle();
    printAll();
    turnRight();
    printAll();
  }
  

//  if (Distance <= 13) {
//    md.setBrakes(400, 400);
//    delay(500);
//    turnRight();
//    goPastObstacle();
//    turnLeft();
//    goPastObstacle();
//    turnLeft();
//    goPastObstacle();
//    turnRight();
//  }
  //  if((leftEncoderValue > 10050.00) ||(rightEncoderValue > 10050.00)) {
  //    md.setBrakes(400,400);
  //    shutdown();
  //  }
}
void turnRight()
{
  turnLeftEncoderValue = leftEncoderValue;
  turnRightEncoderValue = rightEncoderValue;

  while ((leftEncoderValue - turnLeftEncoderValue <= 790.00) && (rightEncoderValue - turnRightEncoderValue <= 790.00))
  {
    md.setSpeeds(200 + Output, -200 + Output);
  }
  md.setBrakes(400, 400);
  delay(500);
}

void turnLeft()
{
  turnLeftEncoderValue = leftEncoderValue;
  turnRightEncoderValue = rightEncoderValue;

  while ((leftEncoderValue - turnLeftEncoderValue <= 800.00) && (rightEncoderValue - turnRightEncoderValue <= 800.00))
  {
    md.setSpeeds(-200 + Output, 200 + Output);
  }
  md.setBrakes(400, 400);
  delay(500);
}

void goPastObstacle()
{
  startLeftEncoderValue = leftEncoderValue;
  startRightEncoderValue = rightEncoderValue;
  while ((leftEncoderValue - startLeftEncoderValue <= 1432.00) && (startRightEncoderValue - turnRightEncoderValue <= 1432.00))
  {
    md.setSpeeds(200 + Output, 200 + Output);
  }
  md.setBrakes(400, 400);
  delay(500);
}

void PWM_Mode_Setup()
{
  pinMode(URTRIG, OUTPUT);                    // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH);                 // Set to HIGH

  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command

  for (int i = 0; i < 4; i++)
  {
    Serial.write(EnPwmCmd[i]);
  }
}

void PWM_Mode()
{ // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses

  unsigned long DistanceMeasured = pulseIn(URPWM, LOW);

  if (DistanceMeasured >= 10200)
  { // the reading is invalid.
    Serial.println("Invalid");
  }
  else
  {
    Distance = DistanceMeasured / 50;       // every 50us low level stands for 1cm
    Serial.print("Distance=");
    Serial.print(Distance);
    Serial.println("cm");
  }

}

void leftEncoderInc(void) {
  leftEncoderValue++;
}


void rightEncoderInc(void) {
  rightEncoderValue++;
}
void printAll(){
  Serial.print("Left:");
  Serial.print(leftEncoderValue);
  Serial.print(", Right:");
  Serial.print(rightEncoderValue);
  Serial.print(", Diff:");
  Serial.println(Output);
}
void shutdown()
{
  // optionally do stop motors dim the LED's etc.
  // Serial.print("stopped");  // or other warning
  while (1);
}
