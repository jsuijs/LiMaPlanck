//-----------------------------------------------------------------------------
// Duckling.cpp - Eende-kuiken (volgt de moeder)
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

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
// MissionDuckling -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionDuckling(TState &S)
{  static int MDistance = 9999, MDegrees_q5 = 0;

   S.Update("Duckling");

   switch (S.State) {
      case 0 : {  // LIDAR-START
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
            MDegrees_q5 = 0;
            for (int i=0; i< 7; i++) {
               if (MDistance > Lpp.Sensor[i].Distance) {
                  MDistance   = Lpp.Sensor[i].Distance;
                  MDegrees_q5  = Lpp.Sensor[i].Degrees32;
               }
            }
            CSerial.printf("Mother initial at %d mm, %d degrees, sensor: %d \n", MDistance, MDegrees_q5/32, (MDegrees_q5/32 - 120) / 15);
         }

         // ** follow mother **
         // Use law of cosines to find point closest to previous mother location.
         // Wee use robot-relative location and both the robot (duckling) and mother could be moving.
         // The algorithm assumes the relative movement of mother is less than the distance
         // to any other obstacle.

         int NewDelta  = 99999;
         int NewDistance   = -1;
         int NewDegrees_q5 = -1;
         for (int i=0; i<8; i++) {
            // law of cosine:  c^2 = a^2 + b^2 - 2ab * cos(gamma)
            double C2 =    Lpp.Sensor[i].Distance * Lpp.Sensor[i].Distance +
                           MDistance * MDistance -
                           2 * Lpp.Sensor[i].Distance * MDistance *
                           cos( (Lpp.Sensor[i].Degrees32 - MDegrees_q5) / (32 * 57.3) );
            int C = sqrt(C2);
            CSerial.printf("Sensor: %d, Distance: %d, Degrees: %d, Delta: %d\n",
                  i, Lpp.Sensor[i].Distance, Lpp.Sensor[i].Degrees32/32, C);

            // Store smallest distance
            if (NewDelta > C2) {
               NewDelta       = C2;
               NewDistance    = Lpp.Sensor[i].Distance;
               NewDegrees_q5  = Lpp.Sensor[i].Degrees32;
            }
         }
         MDistance   = NewDistance;
         MDegrees_q5 = NewDegrees_q5;
         CSerial.printf("Mother at %d mm, %d degrees, sensor: %d \n", MDistance, MDegrees_q5/32, (MDegrees_q5/32 - 120) / 15);
      }
      break;


      default : {
         CSerial.printf("Error: ongeldige state in MissieTemplate (%d)\n", S.State);
         return true;  // error => mission end
      }
   }
   return false;  // mission nog niet gereed
}

