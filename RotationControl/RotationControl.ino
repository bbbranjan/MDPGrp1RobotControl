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
//double Kp=1.7, Ki=0, Kd=0;
//562.25 square wave = one revolution
PID myPID(&leftEncoderValue, &Output, &rightEncoderValue, 0.50, 0, 0, DIRECT);
//PID(&input, &output, &setpoint, Kp, Ki, Kd, Direction)
//Parameters: Input - the variable we are trying to control
//          : Output - the variable that willl be adjusted by PID
//          : Setpoint - the value we want the input to maintain
//          : Direction - either DIRECT or REVERSE
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //md = new DualVNH5019MotorShield();
  md.init();
  pinMode (LEFT_ENCODER, INPUT); //set digital pin 3 as input
  pinMode (RIGHT_ENCODER, INPUT); //set digital pin 5 as input
  attachPCINT(digitalPinToPCINT(LEFT_ENCODER), leftEncoderInc, HIGH);
  attachPCINT(digitalPinToPCINT(RIGHT_ENCODER), rightEncoderInc, HIGH);
  myPID.SetOutputLimits(-50,50);
  myPID.SetMode(AUTOMATIC);

}

void loop() {
  // put your main code here, to run repeatedly:
//  md.setSpeeds(-(210+Output),200-Output);
//  
//  myPID.Compute();
//  
//  Serial.print("Left:");
//  Serial.print(leftEncoderValue);
//  Serial.print(", Right:");
//  Serial.print(rightEncoderValue);
//  Serial.print(", Diff:");
//  Serial.println(Output);
//
//  if((leftEncoderValue >= 785.00) || (rightEncoderValue >= 785.00)) {
//    md.setBrakes(400,400);
//    shutdown();
//  }

  rotateLeft(90);
}
void leftEncoderInc(void){
  leftEncoderValue++;
  }


void rightEncoderInc(void){
  rightEncoderValue++;
}

int rotateLeft(double angle) {
  
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  double target_Tick = 0;
  if (angle <= 90) target_Tick = angle * 8.65; //8.96
  else if (angle <=180 ) target_Tick = angle * 9.1;    //tune 180
  else if (angle <=360 ) target_Tick = angle * 8.95;
  else target_Tick = angle * 8.9;

  while (leftEncoderValue < target_Tick ) {
    myPID.Compute();
    md.setSpeeds(-(200+Output), (210-Output));
  }
  //md.setBrakes(385, 400);
  md.setBrakes(400,379);
  shutdown();
}

void shutdown()
{
 // optionally do stop motors dim the LED's etc. 
 // Serial.print("stopped");  // or other warning 
 while(1);
}
