#include <DualVNH5019MotorShield.h>
#include <PinChangeInterrupt.h>
#include <PID_v1.h>
#include <SharpIRNormal.h>
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
double Output = 0;
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

/* 
  Ultrasonic Connecttion:
  Pin 1 VCC (URM v3.2) -> VCC (Arduino)
  Pin 2 GND (URM V3.2) -> GND (Arduino)
  Pin 4 PWM (URM V3.2) -. Pin 6 (Arduino)
  Pin 6 COMP/TRIG (URM V3.2) -> Pin 11 (Arduino)
  
*/
int URPWM = 6; // PWM Output 0-25000US, Eevry 50US represent 1cm
int URTRIG = 11; // //PWM trigger pin

unsigned int Distance = 0;
uint8_t EnPwmCmd[4] = {0x44, 0x02, 0xbb, 0x01}; // distance measure command
String mainMessage, message1, message2, message3;
//loop function
char piCommand_buffer[10], readChar, instruction;
int i, arg;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  md.init(); //initialise the motor drivers
  pinMode(LEFT_ENCODER, INPUT); //set digital pin 3 as input
  pinMode(RIGHT_ENCODER, INPUT); // set digital pin 5 as input
  attachPCINT(digitalPinToPCINT(LEFT_ENCODER), leftEncoderInc, HIGH);
  attachPCINT(digitalPinToPCINT(RIGHT_ENCODER), rightEncoderInc, HIGH);
  myPID.SetOutputLimits(-50, 50);
  myPID.SetMode(AUTOMATIC);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  i = 0, arg = 0;

  while (1){
    if (Serial.available()){
      readChar = Serial.read();
      piCommand_buffer[i] = readChar;
      i++;
      
      if (readChar == '|'){
        i = 1;
        break;
      }
    }
  }  

  instruction = piCommand_buffer[0];

  while (piCommand_buffer[i] != '|') {
    arg *= 10; 
    arg = arg + (piCommand_buffer[i] - 48); 
    i++;
  }

  switch(instruction){
     case 'W':
      moveForward();
      message1 = "W";
      message2 = "done";
      mainMessage = message1 + message2 ;
      Serial.println(mainMessage);      
      break;
  }
}

void moveForward(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;

  while(leftEncoderValue != 4498.00){
    md.setSpeeds(200+Output, 200-Output);
  }
  md.setBrakes(400,400);
}

void moveBackward(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;

  while(leftEncoderValue != 4498.00){
    md.setSpeeds(200+Output, 200-Output);
  }
  md.setBrakes(400,400);
}

void turnLeft(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  
  while((leftEncoderValue  <= 800.00) && 
        (rightEncoderValue <= 800.00)){
          md.setSpeeds(-(200+Output), 200-Output);
        }
        md.setBrakes(400,400);
}

void turnRight(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;

  while((leftEncoderValue <= 800.00) && 
        (rightEncoderValue <= 800.00)){
          md.setSpeeds(200+Output, -(200-Output));
        }
        md.setBrakes(400,400);
  
}

void leftEncoderInc(){
  leftEncoderValue++;
}
 void rightEncoderInc(){
  rightEncoderValue++;
 }

