//-----------------------------------------------------------------------------
// MIssionStartvector.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"
//-----------------------------------------------------------------------------
const int lidar_Offset = 180;
const int lidar_sign = -1;

//-----------------------------------------------------------------------------
static int VolgLinksVW()          // Wandvolgen VoorWaards
{
   return (350 - Lpp.Sensor[S_VIER].Distance) / 3;
}

//-----------------------------------------------------------------------------
static int VolgLinksAW()          // Wandvolgen AchterWaards
{
   return (315 - Lpp.Sensor[S_DRIE].Distance) / 3;
}

//-----------------------------------------------------------------------------
static void MapSensorSetup(int Sensor, int Start, int Width)
{
   Lpp.SensorSetup(Sensor, (Start * lidar_sign) + lidar_Offset - Width, Width);
}

//-----------------------------------------------------------------------------
//    ***** STARTVECTOR UITVOEREN   9-11-2021 (av) ******
//-----------------------------------------------------------------------------
//lppstart
bool MissionStartVector1(TState &S)
{
   static int AWScannen;      // X * Heen en weer rijden om Achterwand te scannen

  //Lpp0;    Lidar Achter   -15 <> +15 graden
  //Lpp1;    Lidar Voor     135 <> 125 graden
  //Lpp3;    Lidar L-achter  45 <> 65  graden
  //Lpp4;    Lidar L-voor   125 <> 145 graden
  //Lpp6;    Lidar L-voor   210 <> 250 graden
  //Lpp7;    Lidar Rechts   250 <> 290 graden

   S.Update(__FUNCTION__, Flags.IsSet(11));

   printf("Lpp0_3_4_7  %d %d %d %d\n",
               Lpp.Sensor[S_NUL].Distance,  Lpp.Sensor[S_DRIE].Distance,
               Lpp.Sensor[S_VIER].Distance, Lpp.Sensor[S_ZEVEN]. Distance); //Distance

   switch (S.State) {

      case 0 : {  // LIDAR-STARTEN
         if (S.NewState) {
            Position.Reset();
            AWScannen = 1;

            MapSensorSetup(S_NUL,   -15, 30);  // Sensor 0  achterwaarts (-15 + 30 = +15 graden)   **SuperSlalom + StartVector**
            MapSensorSetup(S_EEN,   165, 30);  // Sensor 1C, vanaf 165 graden, (+165 + 30 = 195 graden) **StartVector**
            MapSensorSetup(S_DRIE,   40, 20);  // Sensor 3B, vanaf 35 graden, segment van 40 graden -   **StartVector**
            MapSensorSetup(S_VIER,  125, 20);  // Sensor 4B, vanaf 125 graden, segment van 20 graden -  **StartVector**
            MapSensorSetup(S_ZEVEN, 260, 20);  // Sensor 7B, vanaf 250 graden, segment van 20 graden     **StartVector**
            Lpp.Start();

            printf("Wachttijd Op Start : %d\n", S.State);
         }

         if (S.StateTime() > 3000) {                      // Wacht op start lidar
            if (ServoSlope(myservo, 550, 20)) S.State ++; // Grijper open
         }
      }
      break;

      case 1 : {       // VectorScan starten langste afstand zoeken >> -B-
         if (S.NewState) {
            printf("case 1: Pre-VectorScan Starten \n");
            Driver.SpeedLR(-30, 30);   // Langzaam Rechtsom draaien
         }

         if (Lpp.Sensor[S_ZEVEN].Distance > 1500) {    // Lange afstand >> -B-
            Driver.Stop();
            Buzzer.Beep(300, 2);                  // (30 lengte signaal, 2 aantal signalen) AV

            printf("Pre-VectorScan Starten : Hoek-klem - controle \n");
            if ((Lpp.Sensor[S_EEN]. Distance <= 300) && (Lpp.Sensor[S_NUL]. Distance <= 300)) { // Klem in hoek?
               S.State = 10;
            }
            else {
               S.State = 20;
            }
         }
      }
      break;

      case 10 : {       // Afstand Hoek-wanden corrigeren
         if (S.NewState) {
            printf("case 2: Afstand Hoek-wanden corigeren in -A- \n");
            Driver.Rotate(-25);                      // 20 graden Rechtsom
         }

         if (Driver.IsDone()) {
            Driver.Stop();
            if ((Lpp.Sensor[S_EEN]. Distance <= 300) && (Lpp.Sensor[S_NUL]. Distance <= 300)) { // Klem in hoek?
               S.State = 15;
            }
            else {
               S.State = 20;
            }
         }
      }
      break;

      case 15 : {       // Afstand Hoek-wanden corrigeren
         if (S.NewState) {
            printf("case 2: Afstand Hoek-wanden corigeren in -A- \n");
            Driver.Rotate(40);                       // 40 graden Linkom
         }

         if (Driver.IsDone()) {
            Driver.Stop();
            if ((Lpp.Sensor[S_EEN]. Distance <= 300) && (Lpp.Sensor[S_NUL]. Distance <= 300)) { // Klem in hoek?
               S.State = 10;
            }
            else {
               S.State = 20;
            }
         }
      }
      break;

      case 20 : {       // Afstand wanden controleren
         if (S.NewState) {
            printf("case 2: Afstand wanden controleren in -A- \n");
         }

         if (Lpp.Sensor[S_NUL].Distance < 300) {   // wandafstand controleren

            S.State = 30;// wand scan voorwaards
            break;
         }

         if (Lpp.Sensor[S_EEN].Distance < 300) {   // wandafstand controleren
            S.State = 40;                       // wand scan achterwaards
            break;
         }

         printf("Let op: default\n");
//         Driver.SpeedLR(0, 0);               // Stoppen
         S.State = 30;
      }
      break;

      case 30 : {    // Rij voorwaards, uitlijnen op achterwand
         Driver.SpeedLR(70, (70 - VolgLinksVW()));
         printf("Case3-Lpp[S_VIER]-Volg-L-VW  %d %d\n", Lpp.Sensor[S_VIER]. Distance, VolgLinksVW()); //Distance -Factor

         if ((Lpp.Sensor[S_EEN]. Distance <= 300) && (Lpp.Sensor[S_NUL]. Distance > 300)) {                  // Wand Y-richting gezien
            S.State = 40;     // wand scan achterwaards
         }
      }
      break;

      case 40 : {   // Rij achterwaards, uitlijnen op achterwand
         Driver.SpeedLR(-70, (-70 + VolgLinksAW()));
         printf("Case4-Lpp[S_DRIE]-Volg-L-AW  %d %d\n", Lpp.Sensor[S_DRIE]. Distance, VolgLinksAW()); //Distance -Factor

         if ((Lpp.Sensor[S_NUL]. Distance <= 300) && (Lpp.Sensor[S_EEN]. Distance > 300)) {                    // Wand X-richting gezien

            AWScannen++;   // heen-en-weer teller ophogen
            if (AWScannen >= 4){
               // voldoende maal heen-en-weer gescand
               S.State = 50;
            }
            else {
               S.State = 30;  // wand scan voorwaards
            }
         }
      }
      break;

      case 50 : {      // Afstand tot (X)achterwand
         if (S.NewState) {
            printf("case 5: Afstand tot (X)Rechter wand \n");
            Driver.SpeedLR(50, 50);                    // stukje Voorw. rijden
         }

         Driver.SpeedLR(50, (50 - VolgLinksVW()));    // Voorwaards uitlijnen

         if (Lpp.Sensor[S_NUL].Distance >= 600) {     // Midden vak -A- TE VERVANGEN DOOR EEN VARIABLE = H&W-TT-Blikken
            Driver.Stop();
            S.State = 60;
         }
      }
      break;

      case 60 : {        // 90 graden Rechtsom
         if (S.NewState) {
            Driver.Rotate(-91);                      // 90 graden Rechtsom
            printf("case 4: 90 graden Rechtsom \n");
         }

         if (Driver.IsDone()) S.State = 70;
      }
      break;

      case 70 : {      // Controle Afstand tot achterwand = 300 mm
         if (S.NewState) {
            printf("case 5: Afstand tot (A)achterwand \n");
         }

         if (Lpp.Sensor[S_NUL].Distance < 295) {
           Driver.SpeedLR(15, 15);                    // stukje Vooruit rijden
         }
         else {
            if (Lpp.Sensor[S_NUL].Distance > 300) {
               Driver.SpeedLR(-15, -15);              // stukje Achteruit rijden
            }
            else
            {
               Driver.Stop();
               Position.Reset();
               return true;
            }
         }
      }
      break;

      default : return S.InvalidState(__FUNCTION__);  // Report invalid state & end mission
   }
   return false;                                       // Nog niet klaar.
}

//********* Einde StartVECTOR ****