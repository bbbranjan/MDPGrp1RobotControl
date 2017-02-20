#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

// define some constants
const float pi = 3.14;

int ActPos = A0;    // select the input pin for feedback signal
int DesPos = A1;    // select the input pin for control signal

byte PWMOutput;
long Error[10];
long Accumulator;
long PID;
int PTerm;
int ITerm;
int DTerm;
byte Divider;

void GetError(void)
{
  byte i = 0;
  // read analogs
  word ActualPosition = analogRead(ActPos);  
// comment out to speed up PID loop
  Serial.print("ActPos= ");
  Serial.println(ActualPosition,DEC);

  word DesiredPosition = analogRead(DesPos);
// comment out to speed up PID loop
  Serial.print("DesPos= ");
  Serial.println(DesiredPosition,DEC);

  // shift error values
  for(i=9;i>0;i--)
    Error[i] = Error[i-1];
  // load new error into top array spot  
  Error[0] = (long)DesiredPosition-(long)ActualPosition;
// comment out to speed up PID loop
  Serial.print("Error= ");
  Serial.println(Error[0],DEC);

}

/* CalculatePID():
Error[0] is used for latest error, Error[9] with the DTERM
*/
void CalculatePID() {
  // Set constants here
  PTerm = 200;
  ITerm = 2.5;
  DTerm = 0;
  Divider = 1.0;
  
  // Calculate the PID  
  PID = Error[0]*PTerm;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  PID += ITerm*Accumulator; // add integral gain and error accumulation

  PID = PID>>Divider; // scale PID down with divider

  if(PID>=400)
    PID = 400;
  if(PID<=-400)
    PID = -400;
}

void loop() // run over and over
{
//     GetError();       // Get position error
//     CalculatePID();   // Calculate the PID output from the error
//     md.setM1Speed(PWMOutput);  // Set motor speed
//     delay(200);
}

void setup() {
  // initialize the serial communication:
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  md.setM1Speed(200);
  md.setM2Speed(-200);
  delay(2000);
//  GetError();       // Get position error
//  CalculatePID();   // Calculate the PID output from the error
//  md.setM1Speed(PWMOutput);  // Set motor speed
//  delay(200);
}

//void loop() {
//  // send the value of analog input 0:
//  Serial.println(analogRead(A0));
//  // wait a bit for the analog-to-digital converter
//  // to stabilize after the last reading:
//  delay(2);
//}
