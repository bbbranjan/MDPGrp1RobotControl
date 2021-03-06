#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
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
    md.setM1Speed(50);
    md.setM2Speed(-50);
  }
void loop()
  {
//    for (int i = 0; i <= 500; i++)
//    {
//      md.setM1Speed(i);
//      md.setM2Speed(-i);
//      stopIfFault();
//        if (i%200 == 0)
//          {
//            Serial.print("M1 current: ");
//            Serial.println(md.getM1CurrentMilliamps());
//            Serial.print("M2 current: ");
//            Serial.println(md.getM2CurrentMilliamps());
//          }
//    delay(100);
//    }
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
