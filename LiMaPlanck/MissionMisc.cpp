//-----------------------------------------------------------------------------
// MissionMisc.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

//-----------------------------------------------------------------------------
// ***** MissionGripperTest - state machine *****************************
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionGripperTest(TState & S) // Eerste Servo Test
{
  S.Update(__FUNCTION__, Flags.IsSet(11));

  switch (S.State) {

    case 0 : {  // Grijper open
        if (S.NewState) {
          Driver.Pwm(0, 0);
          printf("Wachttijd OpStart : %d\n", S.State);
        }

        if (ServoSlope(myservo, 550, 20)) S.State ++; //grijper open
      }
      break;
    case 1 : {                        // Start loop open-sluiten
        if (S.NewState) {
          printf("Servo 550 in state %d\n", S.State);
        }

        if (S.StateTime() > 3000) S.State++; // Wacht
      }
      break;

    case 2 : {              // Grijper naar sluiten
        if (S.NewState) {
          printf("Servo 550 in state %d\n", S.State);
        }

        if (ServoSlope(myservo, 2300, 20)) S.State ++;
      }
      break;

    case 3 : {  // Wachttijd grijper gesloten
        if (S.NewState) {
          printf("Servo 2300 in state %d\n", S.State);
        }

        if (S.StateTime() > 3000) S.State++; // Wacht
      }
      break;

    case 4 : {  // Grijper naar open
        if (S.NewState) {
          printf("Servo 2300 in state %d\n", S.State);
        }

        if (ServoSlope(myservo, 550, 20)) S.State = 1;  //++;
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
  }
  return false;  // mission nog niet gereed
}
//*************************************



//-----------------------------------------------------------------------------
// MissionOdoTest - state machine  >> 3000 heen - 180gr draaien - 3000 terug <<
//-----------------------------------------------------------------------------
// Ten behoeve van calibratie odometrie
//
// input: S.Param1 = 1 (CCW) of -1 (CW)
//
//-----------------------------------------------------------------------------
bool MissionOdoTest(TState &S)
{

  S.Update(__FUNCTION__, Flags.IsSet(11));

  switch (S.State) {

    case 0 : {  // naar 3000mm testrit
        if (S.NewState) {
          Position.Reset();
          Driver.XY(3000, 0 , 200, 0 );
        }

        if (Driver.IsDone()) S.State += 10;
      }
      break;

    case 10 : { // Draai naar -180 of +180, afhankelijk van MissionHandler.ParamGet(0)
        if (S.NewState) {
          Driver.Rotate(180 * S.Param1); // 180 graden links of rechts
        }
        if (Driver.IsDone()) S.State = 4; // Naar de volgende state als de beweging klaar is
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
// RandomRijdenTakt -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionRandomRijden(TState &S)
{
  S.Update(__FUNCTION__, Flags.IsSet(11));

  //   int Lidar_A = Lpp.Sensor[0].Distance;         // Afstand achterzijde
  //   int Lidar_grV = Lpp.Sensor[1].Degrees32 / 32; // Scannen(1) Hoek van 90 graden + 180 gr tot 270 graden
  //   int Lidar_90V = Lpp.Sensor[2].Distance;       // Scannen(2) Korste Afstand van 135 graden + 90 gr tot 225 graden
//  int Lidar_Blik_L  = Lpp.Sensor[3].Distance;  // Scannen(3) Korste Afstand van 70 graden + 40 gr tot 110 graden
  int Lidar_Blik_LV = Lpp.Sensor[4].Distance;  // Scannen(4) Korste Afstand van 110 graden + 40 gr tot 150 graden
  int Lidar_Blik_V  = Lpp.Sensor[5].Distance;  // Scannen(5) Korste Afstand van 150 graden + 60 gr tot 210 graden
  int Lidar_Blik_RV = Lpp.Sensor[6].Distance;  // Scannen(6) Korste Afstand van 210 graden + 40 gr tot 250 graden
//  int Lidar_Blik_R  = Lpp.Sensor[7].Distance;  // Scannen(7) Korste Afstand van 250 graden + 40 gr tot 290 graden

  switch (S.State) {

    //----------------------------------------------------------------------
    //      *** MaxonRobot - Slalom=8 - Rijden LockDown Challenge !!! 02-09-2020
    //----------------------------------------------------------------------
    case 0 : { // LIDAR-STARTEN 8 - Rijden LockDown Challenge
        if (S.NewState) {
          Driver.Pwm(0, 0);
          Lpp.Start();
        }

        if (S.StateTime() > 2000) {   //Wacht op start lidar
          S.State = 601;              //Random rijden in bak
        }
      }
      break;

    //----------------------------------------------------------------------
    //      *** Maxon2 Robot - Random rijden
    //----------------------------------------------------------------------
    case 601 : { // Random rijden

        if (S.NewState) {
        }

        int Lid_Max = 500;          // Lidar Meetwaarde beperken
        //if (Lidar_grV > Lid_Max)   Lidar_grV = Lid_Max;        // 90><180 gr valse meting
        //if (Lidar_90V > Lid_Max)   Lidar_90V = Lid_Max;        // 135><225 gr valse meting
//        if (Lidar_Blik_L  > Lid_Max)  Lidar_Blik_L  = Lid_Max;   // 70><110 gr valse meting
        if (Lidar_Blik_LV > Lid_Max)  Lidar_Blik_LV = Lid_Max;   // 110><150 gr valse meting
        if (Lidar_Blik_V  > Lid_Max)  Lidar_Blik_V  = Lid_Max;   // 150><210 gr valse meting
        if (Lidar_Blik_RV > Lid_Max)  Lidar_Blik_RV = Lid_Max;   // 210><250 gr valse meting
//        if (Lidar_Blik_R  > Lid_Max)  Lidar_Blik_R  = Lid_Max;   // 250><290 gr valse meting

        if (Lidar_Blik_V < 240) {
          // print a random number from 0 to 6
          int randNumber = random(2);
          CSerial.println(randNumber);
          if (randNumber == 1) {
            S.State = 603;      // 180 gr R/Om draaien
          }
          else {
            S.State = 604;      // 180 gr L/Om draaien
          }
        }
        if ((  Lidar_Blik_LV < 200) && (Lidar_Blik_V < 250) && (Lidar_Blik_RV < 200)) {
          S.State = 602;      // 180 gr R/Om draaien
        }
        if ((Lidar_Blik_LV < 200) || (Lidar_Blik_RV > Lidar_Blik_LV)) {
          Driver.SpeedLR(150, 90);   // Rechts afdraaien
          break;
        }
        if ((Lidar_Blik_RV < 200) || (Lidar_Blik_LV > Lidar_Blik_RV)) {
          Driver.SpeedLR(90, 150);   //Links afdraaien
          break;
        }
        Driver.SpeedLR(150, 150);   // default rechtuit
      }
      break;

    //----------------------------------------------------------------------
    case 602 : { // 180 gr RECHTSOM draaien - Random rijden

        if (S.NewState) {
          Driver.Rotate(-180);
        }
        if (Driver.IsDone()) S.State = 601;
      }
      break;

    //----------------------------------------------------------------------
    case 603 : { // 90 gr RECHTSOM draaien - Random rijden

        if (S.NewState) {
          Driver.Rotate(-90);
        }
        if (Driver.IsDone()) S.State = 601;
      }
      break;

    //----------------------------------------------------------------------
    case 604 : { // 90 gr LINKSOM draaien - Random rijden

        if (S.NewState) {
          Driver.Rotate(90);
        }
        if (Driver.IsDone()) S.State = 601;
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
  }
  return false;  // mission nog niet gereed
}
