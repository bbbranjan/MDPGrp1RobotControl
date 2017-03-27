
#include <DualVNH5019MotorShield.h>
#include <PinChangeInterrupt.h>
#include <PID_v1.h>
#include <SharpIR.h>
#include <RunningMedian.h>

#define LEFT_ENCODER 3
#define RIGHT_ENCODER 5
#define LS_IR A4
#define LF_IR A3
#define RF_IR A2
#define RS_IR A5
#define FC_IR A1
#define model 1080
#define LFRF_OFFSET 4.7
#define FC_OFFSET 2.0
#define LSRS_OFFSET 3.8
DualVNH5019MotorShield md;

double leftEncoderValue=0;
double rightEncoderValue=0;
double difference;
double Output = 0;
double turnLeftEncoderTarget, turnRightEncoderTarget,
       startLeftEncoderValue, startRightEncoderValue;

double Kp=0.5, Ki=0.0, Kd=0.0; 
// 562.25 squave wave = one revolution
//PID myPID(&leftEncoderValue, &Output, &rightEncoderValue, Kp, Ki, Kd, DIRECT);
PID myPID(&rightEncoderValue, &Output, &leftEncoderValue, Kp, Ki, Kd, DIRECT);
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

SharpIR sharp_ls(LS_IR, (float)25.0, (float)93.0, model);
SharpIR sharp_lf(LF_IR, (float)25.0, (float)93.0, model);
SharpIR sharp_rf(RF_IR, (float)25.0, (float)93.0, model);
SharpIR sharp_rs(RS_IR, (float)25.0, (float)93.0, model);
SharpIR sharp_fc(FC_IR, (float)25.0, (float)93.0, model);

RunningMedian samples_ls = RunningMedian(7);
RunningMedian samples_lf = RunningMedian(7);
RunningMedian samples_rf = RunningMedian(7);
RunningMedian samples_rs = RunningMedian(7);
RunningMedian samples_fc = RunningMedian(7);

double DIST_BETWEEN_SENSOR = 18.0;

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
  int fc_init, fc_fin;
  switch(instruction) {
    
      case 'F':
        delay(100);
        Serial.print("p");
        //fc_init = ir_sense(sharp_fc);
        moveForward(arg);
        //fc_fin = ir_sense(sharp_fc);
//        while(abs(fc_fin - fc_init) < 2) {
//          moveForward(arg);
//          delay(300);
//          fc_fin = ir_sense(sharp_fc);
//        }
        message1 = "F";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);      
      break;
      
      case 'B':
        delay(100);
        Serial.print("p");
        moveBackward(arg);
        message1 = "B";
        message2 = "done";
        mainMessage = message1 + message2;
        //delay(300);
        Serial.println(mainMessage);      
      break;
      
      case 'L':
        delay(100);
        Serial.print("p");
        rotateLeft(90);
        message1 = "L";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);      
      break;
      
      case 'R':
        delay(100);
        Serial.print("p");
        rotateRight(90);
        message1 = "R";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);      
      break;
      
      case 'S':
        delay(100);
        Serial.print("p");
        sense();
        message1 = "S";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);      
      break;

      case 'C':
        delay(100);
        Serial.print("p");
        alignAngle();
        message1 = "C";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);
      break;

      case 'D':
        delay(100);
        Serial.print("p");
        adjustDistance();
        message1 = "D";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);
      break;

      case 'W':
        delay(100);
        Serial.print("p");
        moveCloserToWall();
        message1 = "W";
        message2 = "done";
        mainMessage = message1 + message2 ;
        //delay(300);
        Serial.println(mainMessage);
      break;

      default:
        message1 = "N";
        message2 = "none";
        mainMessage = message1 + message2 ;
        Serial.println(mainMessage);
  }
}

void moveForward(double dist) {

  int fwd_L_encoder = leftEncoderValue;
  int fwd_R_encoder = rightEncoderValue;
//  leftEncoderValue = 0, rightEncoderValue = 0;
//  Output = 0;
  //3.3 for lounge
  //3.18 for HWLab 3
  double fwd_dist;
  if(dist <= 20) {
    fwd_dist = (562.25*dist)/(3.3*3.1416);
  }
  else if(dist < 40) {
    fwd_dist = (562.25*dist)/(3.25*3.1416);
  }
  else if(dist < 60) {
    fwd_dist = (562.25*dist)/(3.2*3.1416);
  }
  else if(dist < 80) {
    fwd_dist = (562.25*dist)/(3.15*3.1416);
  }
  else if(dist < 100) {
    fwd_dist = (562.25*dist)/(3.1*3.1416);
  }
  else {
    fwd_dist = (562.25*dist)/(3.05*3.1416);
  }
  while((leftEncoderValue <= fwd_L_encoder + fwd_dist)|| (rightEncoderValue <= fwd_R_encoder + fwd_dist)) {
    md.setSpeeds(350-Output,350+Output);
    //md.setSpeeds(200+Output,242-Output);
//    md.setSpeeds(200+Output,242-Output);
//    md.setSpeeds(202+Output,233-Output);
    myPID.Compute();
//    Serial.print("Left:");
//    Serial.print(leftEncoderValue);
//    Serial.print(", Right:");
//    Serial.print(rightEncoderValue);
//    Serial.print(", Diff:");
//    Serial.println(Output);
  }
  md.setBrakes(-400,-400);
}

void moveBackward(double dist){

  int bwd_L_encoder = leftEncoderValue;
  int bwd_R_encoder = rightEncoderValue;
//  leftEncoderValue = 0, rightEncoderValue = 0;
//  Output = 0;
//  double bwd_dist = (562.25*dist)/(3.3*3.1416);

  double bwd_dist;
  if(dist <= 20) {
    bwd_dist = (562.25*dist)/(3.3*3.1416);
  }
  else if(dist < 40) {
    bwd_dist = (562.25*dist)/(3.25*3.1416);
  }
  else if(dist < 60) {
    bwd_dist = (562.25*dist)/(3.2*3.1416);
  }
  else if(dist < 80) {
    bwd_dist = (562.25*dist)/(3.15*3.1416);
  }
  else if(dist < 100) {
    bwd_dist = (562.25*dist)/(3.1*3.1416);
  }
  else {
    bwd_dist = (562.25*dist)/(3.05*3.1416);
  }
  
  while((leftEncoderValue <= bwd_L_encoder + bwd_dist) || (rightEncoderValue <= bwd_R_encoder + bwd_dist)){
    md.setSpeeds(-(200+Output),-(242-Output));
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

//void turnLeft(){
//  leftEncoderValue = 0, rightEncoderValue = 0;
//  Output = 0;
//  
//  while((leftEncoderValue  <= 800.00) && 
//        (rightEncoderValue <= 800.00)){
//          md.setSpeeds(-(200+Output), 212-Output);
//          myPID.Compute();
//        }
//    md.setBrakes(400,400);
//}
//
//void turnRight(){
//  leftEncoderValue = 0, rightEncoderValue = 0;
//  Output = 0;
//
//  while((leftEncoderValue <= 800.00) && (rightEncoderValue <= 800.00)){
//    md.setSpeeds(200+Output, -(212-Output));
//    myPID.Compute();
//  }
//  md.setBrakes(400,400);
//}

void sense(){
  
//  Serial.print("LS_IR: ");
//  samples_ls.add(ir_sense(sharp_ls));
//  long m_ls = samples_ls.getMedian();
  Serial.print(ir_sense(sharp_ls)-LSRS_OFFSET);
  Serial.print(":");
//  Serial.print("LF_IR: ");
//  samples_lf.add(ir_sense(sharp_lf));
//  long m_lf= samples_lf.getMedian();
  Serial.print(ir_sense(sharp_lf)-LFRF_OFFSET);
  Serial.print(":"); 
//  Serial.print("RF_IR: ");
//  samples_rf.add(ir_sense(sharp_rf));
//  long m_rf = samples_rf.getMedian();
  Serial.print(ir_sense(sharp_rf)-LFRF_OFFSET);
  Serial.print(":"); 
//  Serial.print("RS_IR: ") 
//  samples_rs.add(ir_sense(sharp_rs));
//  long m_rs = samples_rs.getMedian();
  Serial.print(ir_sense(sharp_rs)-LSRS_OFFSET);
  Serial.print(":");
//  Serial.print("FC_IR: ")
//  samples_fc.add(ir_sense(sharp_fc));
//  long m_fc = samples_fc.getMedian();
  Serial.print(ir_sense(sharp_fc) - FC_OFFSET);
  Serial.print(":");
  
}

float ir_sense(SharpIR sharp) {
  float dis=sharp.distance();  // this returns the distance to the object you're measuring
  return (dis);
}

void leftEncoderInc(){
  leftEncoderValue++;
}
 void rightEncoderInc(){
  rightEncoderValue++;
}

void alignAngle() {

  int cal_L_encoder = leftEncoderValue;
  int cal_R_encoder = rightEncoderValue;

  double rad2deg = 180/3.14159; 
  
  float sensor_R_dis;
  float sensor_L_dis;
  
  float sensorDiff;

//  boolean isTooClose = false;

//  sensor_R_dis = ir_sense(sharp_rf)-LFRF_OFFSET;
//  sensor_L_dis = ir_sense(sharp_lf)-LFRF_OFFSET;
//  while((sensor_R_dis < 5.1) || (sensor_L_dis < 5.1)) {
//    moveBackward(1);
//    delay(300);
//    sensor_R_dis = ir_sense(sharp_rf)-LSRS_OFFSET;
//    sensor_L_dis = ir_sense(sharp_lf)-LSRS_OFFSET;
//  }
//  delay(100);
//
////  if(sensor_R_dis < 10 || sensor_L_dis < 10) {
////    isTooClose = true;
////    moveBackward(2);
////    delay(500);
////  }

  moveCloserToWall();

  adjustDistance();
  
  sensor_R_dis = ir_sense(sharp_rf)-LSRS_OFFSET;
  sensor_L_dis = ir_sense(sharp_lf)-LSRS_OFFSET;
  
  sensorDiff = abs(sensor_R_dis - sensor_L_dis);

  while (sensorDiff > 0.6) {

    double sensorMeanDiff = ((double)sensorDiff) / 2;
    double sinTheta = sensorMeanDiff / DIST_BETWEEN_SENSOR;
  
    double thetaAngle = (asin(sinTheta) * rad2deg);
//    Serial.print(thetaAngle);
    if (thetaAngle > 90) {
      leftEncoderValue = cal_L_encoder;
      rightEncoderValue = cal_R_encoder;
      return;
    }
    if (sensor_L_dis > sensor_R_dis){
      rotateRight(thetaAngle);
    }
    else if (sensor_R_dis > sensor_L_dis){   
      rotateLeft(thetaAngle);
    }
    delay(100);
    sensor_R_dis = ir_sense(sharp_rf)-LSRS_OFFSET;
    sensor_L_dis = ir_sense(sharp_lf)-LSRS_OFFSET;
    
    sensorDiff = abs(sensor_R_dis - sensor_L_dis);
  }

//  if(isTooClose) {
//    moveForward(4);
//    delay(100);
//  }

  leftEncoderValue = cal_L_encoder;
  rightEncoderValue = cal_R_encoder;
  
}

int rotateRight(double angle) {
  
  int right_L_encoder = leftEncoderValue;
  int right_R_encoder = rightEncoderValue;
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  double target_Tick = 0;
  if (angle <= 90) target_Tick = angle * 9.02; //8.96
  else if (angle <=180 ) target_Tick = angle * 9.1;    //tune 180
  else if (angle <=360 ) target_Tick = angle * 8.95;
  else target_Tick = angle * 8.96;

  while (leftEncoderValue < target_Tick ) {
    myPID.Compute();
    md.setSpeeds((200+Output), -(200-Output));
  }
  //md.setBrakes(385, 400);
  md.setBrakes(400,400);
  leftEncoderValue = right_L_encoder;
  rightEncoderValue = right_R_encoder;
}
int rotateLeft(double angle) {
  
  const int left_L_encoder = leftEncoderValue;
  const int left_R_encoder = rightEncoderValue;
  leftEncoderValue = 0, rightEncoderValue = 0;
  Output = 0;
  double target_Tick = 0;
  if (angle <= 90) target_Tick = angle * 9.02; //8.96
  else if (angle <=180 ) target_Tick = angle * 9.1;    //tune 180
  else if (angle <=360 ) target_Tick = angle * 8.95;
  else target_Tick = angle * 8.9;

  while (leftEncoderValue < target_Tick ) {
    myPID.Compute();
    md.setSpeeds(-(200+Output), (200-Output));
  }
  //md.setBrakes(385, 400);
  md.setBrakes(400,400);
  leftEncoderValue = left_L_encoder;
  rightEncoderValue = left_R_encoder;
}
void adjustDistance() {

  moveFwdDistanceCalibration();
  
  int ad_L_encoder = leftEncoderValue;
  int ad_R_encoder = rightEncoderValue;
  double sensor_R_dis = ir_sense(sharp_rf)-LFRF_OFFSET;
  double sensor_L_dis = ir_sense(sharp_lf)-LFRF_OFFSET;
  double sensor_C_dis = ir_sense(sharp_fc)-FC_OFFSET;
  while((sensor_R_dis < 5.1) || (sensor_L_dis < 5.1) || (sensor_C_dis < 5.1)) {
    moveBackward(0.2);
    delay(100);
    sensor_R_dis = ir_sense(sharp_rf)-LFRF_OFFSET;
    sensor_L_dis = ir_sense(sharp_lf)-LFRF_OFFSET;
    sensor_C_dis = ir_sense(sharp_fc)-FC_OFFSET;
  }
  leftEncoderValue = ad_L_encoder;
  rightEncoderValue = ad_R_encoder;
}
void moveCloserToWall() {
  int mw_L_encoder = leftEncoderValue;
  int mw_R_encoder = rightEncoderValue;
  
  double sensor_R_dis = ir_sense(sharp_rf)-LFRF_OFFSET;
  double sensor_L_dis = ir_sense(sharp_lf)-LFRF_OFFSET;
  double sensor_C_dis = ir_sense(sharp_fc)-FC_OFFSET;
  while((sensor_R_dis > 6.5) || (sensor_L_dis > 6.5) || (sensor_C_dis > 6.5)) {
    moveForward(0.2);
    delay(100);
    sensor_R_dis = ir_sense(sharp_rf)-LFRF_OFFSET;
    sensor_L_dis = ir_sense(sharp_lf)-LFRF_OFFSET;
    sensor_C_dis = ir_sense(sharp_fc)-FC_OFFSET;
  }
  leftEncoderValue = mw_L_encoder;
  rightEncoderValue = mw_R_encoder;
}
void moveFwdDistanceCalibration() {
  int mFwdDC_L_encoder = leftEncoderValue;
  int mFwdDC_R_encoder = rightEncoderValue;
  
  double sensor_R_dis = ir_sense(sharp_rf)-LFRF_OFFSET;
  double sensor_L_dis = ir_sense(sharp_lf)-LFRF_OFFSET;
  double sensor_C_dis = ir_sense(sharp_fc)-FC_OFFSET;
  SharpIR sharp_calib_sensor = sharp_fc;
  double sensor_calib, sensor_offset;
  if(sensor_R_dis < sensor_L_dis) {
    if(sensor_C_dis < sensor_R_dis) {
      sharp_calib_sensor = sharp_fc;
      sensor_calib = sensor_C_dis;
      sensor_offset = FC_OFFSET;
    }
    else {
      sharp_calib_sensor = sharp_rf;
      sensor_calib = sensor_R_dis;
      sensor_offset = LFRF_OFFSET;
    }
  }
  else {
    if(sensor_C_dis < sensor_L_dis) {
      sharp_calib_sensor = sharp_fc;
      sensor_calib = sensor_C_dis;
      sensor_offset = FC_OFFSET;
    }
    else {
      sharp_calib_sensor = sharp_lf;
      sensor_calib = sensor_L_dis;
      sensor_offset = LFRF_OFFSET;
    }
  }
  while(sensor_calib > 6.5) {
    moveForward(0.2);
    delay(100);
    sensor_calib = ir_sense(sharp_calib_sensor)-sensor_offset;
  }
  leftEncoderValue = mFwdDC_L_encoder;
  rightEncoderValue = mFwdDC_R_encoder;
}
void shutdown()
{
  // optionally do stop motors dim the LED's etc.
  // Serial.print("stopped");  // or other warning
  while (1);
}

