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
#define RS_IR A5
#define FC_IR A1
#define model 1080

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
//int URPWM = 6; // PWM Output 0-25000US, Eevry 50US represent 1cm
//int URTRIG = 11; // //PWM trigger pin
//
//unsigned int Distance = 0;
//uint8_t EnPwmCmd[4] = {0x44, 0x02, 0xbb, 0x01}; // distance measure command

String mainMessage, message1, message2, message3;
//loop function
char piCommand_buffer[10], readChar, instruction;
int i, arg;

SharpIR sharp_rf(RF_IR, 25, 93, model);
SharpIR sharp_lf(LF_IR, 25, 93, model);
SharpIR sharp_ls(LS_IR, 25, 93, model);
SharpIR sharp_rs(RS_IR, 25, 93, model);
SharpIR sharp_fc(FC_IR, 25, 93, model);

RunningMedian samples = RunningMedian(7);

double DIST_BETWEEN_SENSOR = 13.7;

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

  pinMode(RF_IR, INPUT);
  pinMode(LF_IR, INPUT);
  pinMode(LS_IR, INPUT);
  pinMode(RS_IR, INPUT);
  pinMode(FC_IR, INPUT);

//  PWM_Mode_Setup();  
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int buffer_size = sizeof(piCommand_buffer) / sizeof(*piCommand_buffer);
  
  i = 0, arg = 0;

  while (1){
    if (Serial.available()){
      readChar = Serial.read();
      piCommand_buffer[i] = readChar;
      i++;
      
      if (readChar == '#') {
        return;
      }
      
      if (readChar == '|' || i >= buffer_size) {
        i = 1;
        break;
      }
    }
  }  

  instruction = piCommand_buffer[0];

  while (piCommand_buffer[i] != '|' && i < buffer_size) {
    arg *= 10; 
    arg = arg + (piCommand_buffer[i] - 48); 
    i++;
  }
//  Serial.println(arg);
  switch(instruction){
    
      case 'F':
        Serial.print("p");
        moveForward();
        message1 = "F";
        message2 = "done";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);      
      break;
      
      case 'B':
        Serial.print("p");
        moveBackward();
        message1 = "B";
        message2 = "done";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);      
      break;
      
      case 'L':
        Serial.print("p");
        rotateLeft(90);
        message1 = "L";
        message2 = "done";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);      
      break;
      
      case 'R':
        Serial.print("p");
        rotateRight(90);
        message1 = "R";
        message2 = "done";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);      
      break;
      
      case 'S':
        Serial.print("p");
        sense();
        message1 = "S";
        message2 = "done";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);      
      break;

      case 'C':
        Serial.print("p");
        alignAngle();
        message1 = "C";
        message2 = "done";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);
  }
}

void moveForward(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  //3.5 for lounge
  while(leftEncoderValue <= (int)(562.25*arg/(3.1*3.1416))|| rightEncoderValue <= (int)(562.25*arg/(3.15*3.1416))) {
    md.setSpeeds(250+Output, 270-Output);
    myPID.Compute();
//    Serial.print("Left:");
//    Serial.print(leftEncoderValue);
//    Serial.print(", Right:");
//    Serial.print(rightEncoderValue);
//    Serial.print(", Diff:");
//    Serial.println(Output);
  }
  md.setBrakes(400,400);
}

void moveBackward(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;

  while(leftEncoderValue <= (int)(562.25*arg/(3.1*3.1416)) || rightEncoderValue <= (int)(562.25*arg/(3.15*3.1416))){
    md.setSpeeds(-(250+Output), -(270-Output));
    myPID.Compute();
//    Serial.print("Left:"); 
//    Serial.print(leftEncoderValue);
//    Serial.print(", Right:");
//    Serial.print(rightEncoderValue);
//    Serial.print(", Diff:");
//    Serial.println(Output);
  }
  md.setBrakes(400,400);
}

void turnLeft(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  
  while((leftEncoderValue  <= 800.00) && 
        (rightEncoderValue <= 800.00)){
          md.setSpeeds(-(200+Output), 212-Output);
          myPID.Compute();
        }
    md.setBrakes(300,300);
}

void turnRight(){
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;

  while((leftEncoderValue <= 800.00) && (rightEncoderValue <= 800.00)){
    md.setSpeeds(200+Output, -(212-Output));
    myPID.Compute();
  }
  md.setBrakes(400,400);
}

void sense(){
  
//  Serial.print("LS_IR: ");
  Serial.print(ir_sense(sharp_ls));
  Serial.print(":");
//  Serial.print("LF_IR: ");
  Serial.print(ir_sense(sharp_lf));
  Serial.print(":"); 
//  Serial.print("RF_IR: ");
  Serial.print(ir_sense(sharp_rf));
  Serial.print(":"); 
//  Serial.print("RS_IR: ") 
  Serial.print(ir_sense(sharp_rs));
  Serial.print(":");
//  PWM_Mode();
  Serial.print(ir_sense(sharp_fc));
  Serial.print(":");
  
}

int ir_sense(SharpIR sharp) {
  int dis=sharp.distance();  // this returns the distance to the object you're measuring
//  samples.add(dis);
//  long m = samples.getMedian();
  
  // returns it to the serial monitor
//  Serial.print(dis);
//  Serial.println(" cm");
//  Serial.println(m);
  return(dis);
}
int calObsAwayRFIR(int dis){
  if(abs(dis) < 10) return 1;
  else if(abs(dis) < 23) return 2;
  else if(abs(dis) < 36) return 3;
  else return -1;
}

int calObsAwayLFIR(int dis){
  if(abs(dis) < 10) return 1;
  else if(abs(dis) <21) return 2;
  else if(abs(dis) <35) return 3;
  else return -1;
}

int calObsAwayLSIR(int dis){
  if(abs(dis) < 10) return 1;
  else if(abs(dis) < 20) return 2;
  else if(abs(dis) < 30) return 3;
  else return -1;
}

int calObsAwayRSIR(int dis) {
  if(abs(dis) < 10) return 1;
  else if(abs(dis)<20) return 2;
  else if(abs(dis)<33) return 3;
  else return -1;
}

int calObsAwayFCIR(int dis) {
  if(abs(dis) < 10) return 1;
  else if(abs(dis)<20) return 2;
  else if(abs(dis)<33) return 3;
  else return -1;
}

//void PWM_Mode_Setup()
//{ 
//  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
//  digitalWrite(URTRIG,HIGH);                  // Set to HIGH
//  
//  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command
//  
//  for(int i=0;i<4;i++)
//  {
////      Serial.write(EnPwmCmd[i]);
//  } 
//}
// 
//void PWM_Mode()
//{                              // a low pull on pin COMP/TRIG  triggering a sensor reading
//  digitalWrite(URTRIG, LOW);
//  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses
//   
//  unsigned long DistanceMeasured=pulseIn(URPWM,LOW);
//   
//  if(DistanceMeasured>=10200)
//  {              // the reading is invalid.
//    Serial.println("Invalid");    
//  }
//  else
//  {
//    Distance=DistanceMeasured/50;           // every 50us low level stands for 1cm
////    Serial.print("Ultrasonic Distance: ");
//    Serial.print(Distance-2);
////    Serial.println("cm");
//  }
//}

void leftEncoderInc(){
  leftEncoderValue++;
}
 void rightEncoderInc(){
  rightEncoderValue++;
}

void alignAngle() {

  double rad2deg = 180/3.14159; 
  
  int sensor_R_dis = ir_sense(sharp_rf);
  int sensor_L_dis = ir_sense(sharp_lf);

  int sensorDiff = abs(sensor_R_dis - sensor_L_dis);

  int sensorMeanDiff = 0;

  while (sensorDiff > 0){
    
    double sensorMeanDiff = sensorDiff / 2;
    double sinTheta = sensorMeanDiff / DIST_BETWEEN_SENSOR;

    double thetaAngle = (asin(sinTheta) * rad2deg);
    
    if (sensor_L_dis > sensor_R_dis){
      rotateRight(thetaAngle);
    }
    else if (sensor_R_dis > sensor_L_dis){   
      rotateLeft(thetaAngle);
    }
    
    sensor_R_dis = ir_sense(sharp_rf);
    sensor_L_dis = ir_sense(sharp_lf);
    
    sensorDiff = abs(sensor_R_dis - sensor_L_dis);
  }
}

int rotateRight(double angle) {
  
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  double target_Tick = 0;
  if (angle <= 90) target_Tick = angle * 8.65; //8.96
  else if (angle <=180 ) target_Tick = angle * 9.1;    //tune 180
  else if (angle <=360 ) target_Tick = angle * 8.95;
  else target_Tick = angle * 8.9;

  while (leftEncoderValue < target_Tick ) {
    myPID.Compute();
    md.setSpeeds((200+Output), -(210-Output));
  }
  //md.setBrakes(385, 400);
  md.setBrakes(400,400);
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
  md.setBrakes(400,400);
}
void shutdown()
{
  // optionally do stop motors dim the LED's etc.
  // Serial.print("stopped");  // or other warning
  while (1);
}
