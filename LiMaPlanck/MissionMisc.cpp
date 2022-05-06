//-----------------------------------------------------------------------------
// MissionMisc.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

////-----------------------------------------------------------------------------
//// MissionTemplate -
////-----------------------------------------------------------------------------
////-----------------------------------------------------------------------------
//bool MissionTemplate(TState &S)
//{
//   S.Update(__FUNCTION__, Flags.IsSet(11));
//
//   switch (S.State) {
//      case 0 : {  //
//         if (S.NewState) {
//            Driver.XY(S.Param1, 0, S.Param1, 0);  // X, Y, Speed, EndSpeed - all in mm(/sec)
//         }
//
//         if (Driver.IsDone()) S.State += 10; // To next state if driver is done
//      }
//      break;
//
//      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
//   }
//   return false;  // mission nog niet gereed
//}

//-----------------------------------------------------------------------------
// ***** MissionGripperTest - state machine *****************************
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionGripperTest(TState & S) // Eerste Servo Test
{
  S.Update(__FUNCTION__, Flags.IsSet(11));

  switch (S.State) {

    case 0 : {          // Grijper openen (als deze nog niet open is)
        if (S.NewState) {
          Driver.Pwm(0, 0);
          printf("Grijper - opstart\n");
        }

        if (ServoSlope(myservo, SERVO_OPEN, 20)) S.State += 10;
      }
      break;

    case 10 : {         // Grijper sluiten
        if (S.NewState) {
          printf("Grijper Servo naar 2300\n");
        }

        if (ServoSlope(myservo, SERVO_CLOSE, 20)) S.State += 10;
      }
      break;

    case 20 : {  // Wachttijd grijper gesloten
        if (S.NewState) {
          printf("Grijper wacht\n");
        }

        if (S.StateTime() > 3000) S.State += 10;
      }
      break;

    case 30 : {         // Grijper openen
        if (S.NewState) {
          printf("Servo naar SERVO_OPEN\n");
        }

        if (ServoSlope(myservo, SERVO_OPEN, 20)) S.State += 10;
      }
      break;

    case 40 : {  // Wachttijd grijper open
        if (S.NewState) {
          printf("Grijper wacht\n");
        }

        if (S.StateTime() > 3000) S.State = 10;
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
  }
  return false;  // mission nog niet gereed
}
//*************************************

//-----------------------------------------------------------------------------
// MissionWheelSizeCalibrate - Heenrijden, draaien en terugrijden.
//-----------------------------------------------------------------------------
// Ten behoeve van calibratie odometrie
//
// input:
//    S.Param0 = te rijden afstand
//    S.Param1 = 1 (CCW) of -1 (CW)
//-----------------------------------------------------------------------------
bool MissionWheelSizeCalibrate(TState &S)
{
  S.Update(__FUNCTION__, Flags.IsSet(11));

  switch (S.State) {

    case 0 : {  // naar 3000mm testrit
        if (S.NewState) {
          Position.Reset();
          Driver.XY(S.Param0, 0 , 200, 0 );
        }

        if (Driver.IsDone()) S.State += 10;
      }
      break;

    case 10 : { // Draai naar -180 of +180, afhankelijk van MissionHandler.ParamGet(0)
        if (S.NewState) {
          Driver.Rotate(180 * S.Param1); // 180 graden links of rechts
        }
        if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

    case 20 : {  // terug naar startpunt (0.0) voor volgende sessie
        if (S.NewState) Driver.XY(0, 0 , 200, 0 );

        if (Driver.IsDone()) return true;
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
  }
  return false;  // mission nog niet gereed
}

//-----------------------------------------------------------------------------
// MissionRandomRijden -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionRandomRijden(TState &S)
{
  S.Update(__FUNCTION__, Flags.IsSet(11));

  int Lidar_Blik_V  = Lpp.Sensor[S_VOOR].Distance;

  // Alleen LV/RV afstanden kleiner dan Lid_Max hebben invloed op rijrichting
  const short int Lid_Max = 500;          // Lidar Meetwaarde beperken
  int Lidar_Blik_LV = min(Lpp.Sensor[S_LINKS_VOOR].Distance, Lid_Max);
  int Lidar_Blik_RV = min(Lpp.Sensor[S_RECHTS_VOOR].Distance,  Lid_Max);

  switch (S.State) {

    //----------------------------------------------------------------------
    case 0 : { // LIDAR-STARTEN
        if (S.NewState) {
          Driver.Pwm(0, 0);
          Lpp.Start();
        }

        if (S.StateTime() > 2000) {    // Wacht op start lidar
          S.State = 10;                // Random rijden in bak
        }
      }
      break;

    //----------------------------------------------------------------------
    case 10 : { // Random rijden

        if (S.NewState) {
        }

        if (Lidar_Blik_V < 240) {
          int randNumber = random(2);  // Random: 0 or 1
          printf("Random: %d\n", randNumber);
          if (randNumber == 1) {
            Driver.Rotate(-90);        // draai naar recht
            S.State = 20;              // wacht tot draai klaar is.
            break;
          }
          else {
            Driver.Rotate(-90);        // draai naar links
            S.State = 20;              // wacht tot draai klaar is.
            break;
          }
        }

        if ((Lidar_Blik_LV < 200) || (Lidar_Blik_RV > Lidar_Blik_LV)) {
          Driver.SpeedLR(150, 90);     // Rechts afdraaien
          break;
        }

        if ((Lidar_Blik_RV < 200) || (Lidar_Blik_LV > Lidar_Blik_RV)) {
          Driver.SpeedLR(90, 150);     //Links afdraaien
          break;
        }

        Driver.SpeedLR(150, 150);      // default rechtuit
      }
      break;

    //----------------------------------------------------------------------
    case 20 : { // Wacht tot Driver (draai) klaar is

        if (Driver.IsDone()) S.State = 10;
      }
      break;

    default : return S.InvalidState(__FUNCTION__);    // Report invalid state & end mission
  }
  return false;  // mission nog niet gereed
}
