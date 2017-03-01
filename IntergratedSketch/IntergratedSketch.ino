#include <DualVNH5019MotorShield.h>
#include <PinChangeInterrupt.h>
#include <PID_v1.h>
#include <SharpIR.h>
#include <RunningMedian.h>

#define LEFT_ENCODER 3
#define RIGHT_ENCODER 5
#define RF_IR A2
#define LF_IR A3
#define LS_IR A4

DualVNH5019MotorShield md;

double leftEncoderValue=0;
double rightEncoderValue=0;
double difference;
double Setpoint, Input, Output;
double turnLeftEncoderTarget, turnRightEncoderTarget,
       startLeftEncoderValue, startRightEncoderValue;

double Kp=0.5, Ki=0.0, Kd=0.0; 
// 562.25 squave wave = one revolution
PID myPID(&leftEncoderValue, &Output, &rightEncoderValue, Kp, Ki, Kd, DIRECT);
/*PID(&input, &output, &setpoint, Kp, Ki, Kd, Direction
Parameters: input - the variable we are trying to control
            output - the variable that will be adjusted by PID
            setpoint - the value we want the input to maintain
            direction - either DIREDCT or REVERSE
*/       
void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
