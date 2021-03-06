#include "SharpIR.h"
#include "RunningMedian.h"

#define ir A5
#define model 1080

boolean done=false;


SharpIR sharp(ir, 50, 93, model);

// ir: the pin where your sensor is attached
// 25: the number of readings the library will make before calculating a mean distance
// 93: the difference between two consecutive measurements to be taken as valid
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)


RunningMedian samples = RunningMedian(7);
void setup(){
  
  Serial.begin(9600);
  pinMode (ir, INPUT);
  
}

void loop(){

  delay(2000);    // it gives you time to open the serial monitor after you upload the sketch
  
 while (1){  // it only runs the loop once
  

  unsigned long pepe1=millis();  // takes the time before the loop on the library begins
  
  
  int dis=sharp.distance();  // this returns the distance to the object you're measuring
  samples.add(dis);
  long m = samples.getMedian();
  
  Serial.print("Mean distance: ");  // returns it to the serial monitor
  Serial.print(dis);
  Serial.println(" cm");
  Serial.println(m);
//  unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
//  Serial.print("Time taken (ms): ");
//  Serial.println(pepe2);  
  
  delay(400);
  //done=true;
  
}

}
  

