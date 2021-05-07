//-----------------------------------------------------------------------------
// Duckling.cpp - Eende-kuiken (volgt de moeder)
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"

//---------------------------------------------------------------------------------------
// LppSensorDucklingSetup -
//---------------------------------------------------------------------------------------
// 120..240 degrees view (-60 to 60, forward looking)
// Divided into 8 segments to keep track of mother
// when there are other, closer objects in view.
//---------------------------------------------------------------------------------------
void LppSensorDucklingSetup()
{
   Lpp.SensorSetup(0, 120, 15);  // 120->135
   Lpp.SensorSetup(1, 135, 15);
   Lpp.SensorSetup(2, 150, 15);
   Lpp.SensorSetup(3, 165, 15);
   Lpp.SensorSetup(4, 180, 15);
   Lpp.SensorSetup(5, 195, 15);
   Lpp.SensorSetup(6, 210, 15);
   Lpp.SensorSetup(7, 225, 15);  // 225->240
}

//-----------------------------------------------------------------------------
// MissieDuckling -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissieDuckling(TState &S)
{  static int MDistance = 9999, MDegrees32 = 0;

   S.Update("Duckling");

   switch (S.State) {
      case 0 : {  // LIDAR-STARTEN
         if (S.NewState) {
            LppSensorDucklingSetup();     // reconfigure Lpp
            Lpp.Start();
         }

         if (S.StateTime() > 2000) {      // Wacht op start lidar
            S.State++;
         }
      }
      break;


      case 1 : {  // Follow mother
         if (S.NewState) {
            // find mother
            MDistance = 9999;
            MDegrees32 = 0;
            for (int i=0; i< 7; i++) {
               if (Distance > Lpp.Sensor[i].Distance) {
                  MDistance   = Lpp.Sensor[i].Distance;
                  MDegrees32  = Lpp.Sensor[i].Degrees32;
               }
            }
            CSerial.printf("Mother initial at %d mm, %d degrees, sensor: %d \n", MDistance, MDegrees32, (MDegrees32 - 120) / 15 / 32);
         }

         // ** follow mother **
         // Use law of cosines to find point closest to previous mother location.
         // Wee use robot-relative location and both the robot (duckling) and mother could be moving.
         // The algorithm assumes the relative movement of mother is less than the distance
         // to any other obstacle.
      }
      break;


      default : {
         CSerial.printf("Error: ongeldige state in MissieTemplate (%d)\n", S.State);
         return true;  // error => mission end
      }
   }
   return false;  // mission nog niet gereed
}

