#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
const float pi = 3.14;
void stopIfFault()
  {
    if (md.getM1Fault())
      {
        Serial.println("M1 fault");
        while(1);
      }
    if (md.getM2Fault())
      { 
        Serial.println("M2 fault");
        while(1);
      } 
  }
void setup()
  {
    Serial.begin(115200);
    Serial.println("Dual VNH5019 Motor Shield");
    md.init();
//    md.setM1Speed(200);
//    md.setM2Speed(-200);
  }
void loop()
  {
    for (int i = 250; i <= 300; i++)
{
      md.setM1Speed(i*sin((i-250)*pi/25));
      md.setM2Speed(-i*sin((i-250)*pi/25));
      stopIfFault();
      if (i%200 == 0) {
        Serial.print("M1 current: ");
        Serial.println(md.getM1CurrentMilliamps());
        Serial.print("M2 current: ");
        Serial.println(md.getM2CurrentMilliamps());
      }
      delay(100);
      Serial.println(analogRead(A0));
    }
//    for (int i = 400; i <= 400; i--)
//      {
//        
//        stopIfFault();
//          if (i%200 == 100)
//          {
//            Serial.print("M2 current: ");
//            Serial.println(md.getM2CurrentMilliamps());
//          }
//        delay(2);
//      }

  }
