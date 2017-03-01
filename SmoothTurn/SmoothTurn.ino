#include "DualVNH5019MotorShield.h"
#include "PinChangeInterrupt.h"
#include "PID_v1.h"

#include "SharpIR.h"
#include "RunningMedian.h"

#define ir A5
#define model 1080

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

//IR



boolean done=false;


SharpIR sharp(ir, 1, 93, model);

// ir: the pin where your sensor is attached
// 25: the number of readings the library will make before calculating a mean distance
// 93: the difference between two consecutive measurements to be taken as valid
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)


RunningMedian samples = RunningMedian(7);

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

  pinMode (ir, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  PWM_Mode();

  md.setSpeeds(250 + Output, 250 - Output);

  myPID.Compute();

  Serial.print("Left:");
  Serial.print(leftEncoderValue);
  Serial.print(", Right:");
  Serial.print(rightEncoderValue);
  Serial.print(", Diff:");
  Serial.println(Output);

///  delay(3000);
//  if((leftEncoderValue > 2050.00) ||(rightEncoderValue > 2050.00)) {
//    turnRight();
//    printAll();
//    goPastObstacle();
//    printAll();
//    turnLeft();
//    printAll();
//    goPastObstacle();
//    printAll();
//    turnLeft();
//    printAll();
//    goPastObstacle();
//    printAll();
//    turnRight();
//    printAll();
//    goPastObstacle();
//    shutdown();
//  }
  
  
  if (readIR() <= 13) {
    turnRight();

    turnCircle();
    turnRight();
//    goBack();
//    md.setBrakes(400,400);
//    delay(1000);
//    turnRight();
//    md.setBrakes(400,400);
//    delay(1000);
//    turnRight();
    md.setBrakes(400,400);
    shutdown();


//    md.setBrakes(400, 400);
//    delay(50);
//    turnRight();
//    goPastObstacle();
//    turnLeft();
//    goPastObstacleLonger();
//    turnLeft();
//    goPastObstacle();
//    turnRight();
  }
//  if((leftEncoderValue > 10050.00) ||(rightEncoderValue > 10050.00)) {
//    md.setBrakes(400,400);
//    shutdown();
//  }
}

void turnCircle()
{
  turnLeftEncoderValue = leftEncoderValue;
  turnRightEncoderValue = rightEncoderValue;

  while ((leftEncoderValue - turnLeftEncoderValue <= 11500.00) && (rightEncoderValue - turnRightEncoderValue <= 11500.00))
  {
    md.setSpeeds(150, 300);
    Serial.print("Left:");
    Serial.print(leftEncoderValue);
    Serial.print(", Right:");
    Serial.print(rightEncoderValue);
    Serial.print(", Diff:");
    Serial.println(Output);
    delay(500);
  }
  md.setBrakes(400, 400);
  delay(50);
}

void goBack() {
  
  do {
//    PWM_Mode();
    md.setSpeeds(250 + Output, 250 - Output);
    Serial.print("Left:");
    Serial.print(leftEncoderValue);
    Serial.print(", Right:");
    Serial.print(rightEncoderValue);
    Serial.print(", Diff:");
    Serial.println(Output);
//    delay(500);
  } while(readIR() > 13);
  md.setBrakes(400, 400);
  delay(100);
}

void turnRight()
{
  turnLeftEncoderValue = leftEncoderValue;
  turnRightEncoderValue = rightEncoderValue;

  while ((leftEncoderValue - turnLeftEncoderValue <= 820.00) && (rightEncoderValue - turnRightEncoderValue <= 820.00))
  {
    md.setSpeeds(200 + Output, -(200 - Output));
  }
//  md.setBrakes(200, 200);
//  delay(50); 
}

void turnLeft()
{
  turnLeftEncoderValue = leftEncoderValue;
  turnRightEncoderValue = rightEncoderValue;

  while ((leftEncoderValue - turnLeftEncoderValue <= 790.00) && (rightEncoderValue - turnRightEncoderValue <= 790.00))
  {
    md.setSpeeds(-(200 + Output), 200 - Output);
  }
  md.setBrakes(200, 200);
  delay(500);
}

void goPastObstacle()
{
  startLeftEncoderValue = leftEncoderValue;
  startRightEncoderValue = rightEncoderValue;
  while ((leftEncoderValue - startLeftEncoderValue <= 1432.00) && (startRightEncoderValue - turnRightEncoderValue <= 1432.00))
  {
    md.setSpeeds(250 + Output, 250 - Output);
  }
  md.setBrakes(400, 400);
  shutdown();
}

void goPastObstacleLonger()
{
  startLeftEncoderValue = leftEncoderValue;
  startRightEncoderValue = rightEncoderValue;
  while ((leftEncoderValue - startLeftEncoderValue <= 2864.00) && (startRightEncoderValue - turnRightEncoderValue <= 2864.00))
  {
    md.setSpeeds(250 + Output, 250 - Output);
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

int readIR() {
  int dis=sharp.distance();  // this returns the distance to the object you're measuring
//  samples.add(dis);
//  long m = samples.getMedian();
  
//  Serial.print("Mean distance: ");  // returns it to the serial monitor
//  Serial.println(dis);
//  Serial.println(m);

  return dis;
}

void shutdown()
{
  // optionally do stop motors dim the LED's etc.
  // Serial.print("stopped");  // or other warning
  while (1);
}
