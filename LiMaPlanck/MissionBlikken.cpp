//-----------------------------------------------------------------------------
// MissionBlikken.cpp - (c) Aloys V.
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

static bool OldSensorSetupCan(int Nr, int StartAngle, int StepAngle)
{
   int NewStartAngle = 180 - (StartAngle + StepAngle);
   printf("OldSensorSetupCan %d [%d -> %d] %d\n",
      Nr, StartAngle, NewStartAngle, StepAngle);
   return Lpp.SensorSetupCan(Nr, NewStartAngle, StepAngle);
}

//-----------------------------------------------------------------------------
// MissionBlikken -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionBlikken(TState &S) //  **
{
   static TState MissionSubS;  // State of sub-mission   ++  StartVector
   static int BlikNummer;      // 6-Blikken ophalen
   static int MissieFase;          // Rit nr 1=B en 2=C

   S.Update(__FUNCTION__, Flags.IsSet(11));  //   ++  StartVector

   switch (S.State) {

      case 0 : {  // LIDAR-STARTEN = "Mission AV1 Blikken Ophalen
         if (S.NewState) {
            Position.Reset();
            //Lpp.Start();
            OldSensorSetupCan(2, 120, 120);  // Sensor 2, vanaf 120 graden, segment van 120 graden  **blikken**

            MissieFase = 1;                     // is zoeken in vak -A <> B-
            printf("Case 0-Naar-Wachttijd AV-OpStart : %d\n", S.State);
         }

         if (S.StateTime() > 1000) {      // Wacht op start lidar
            printf("Na-Wachttijd naar case1 AV-VectorOpStart : %d\n", S.State);
            if (ServoSlope(myservo, 550, 20)) S.State = 1;//++; //grijper open
         }
      }
      break;

 //  ==STARTVECTOR==

      case 1 : {  // MissionStartVector1 uitvoeren
         if (S.NewState) {
            printf("case 1 Starten van MissionSubs \n");
            MissionSubS.Reset();    // geef aan dat we opnieuw starten
         }

         if (MissionStartVector1(MissionSubS)) {
            printf("na StartVector = case 1 naar case 2 \n");
            S.State++; // Naar de volgende state als de missie klaar is.
         }
      }
      break;

 //  ==EINDE-STARTVECTOR==
      case 2 : {
         if (S.NewState) {
            if (MissieFase == 1) {             // Blikken zoeken in vak A<>B
               S.State = 20;
            } else {
               S.State = 100;                 // Blikken zoeken in vak -C-
            }
         }
      }
      break;

      case 100 : {
         if (S.NewState) Driver.XY(1500, 0 , 200, 0 );   // Naar midden parcour -C- rijden

         if (Driver.IsDone()) S.State = 101;
      }
      break;

      case 101 : {  // Pos.1500.0 > 90 gr draaien naar -C- vak
         if (S.NewState) Driver.RotateHeading(90);

         if (Driver.IsDone()) {
            S.State = 20;
         }
      }
      break;

      case 105 : {  // Achteruit Rijden naar 1500.0 voor zoeken in vak -C- Of EINDE opdracht
         if (S.NewState) {
            //printf("case 105:Naar 1500.0 pos. \n");

            Driver.XY(1500, 0 , -200, 0 );   // Naar midden parcour A<>B rijden
         }

         if (Driver.IsDone()) S.State =106;
      }
      break;

      case 106 : {  // Selecteren naar vak -C- Of naar EINDE opdracht

         if (MissieFase == 2) {
             S.State = 101;       //Blikken zoeken in vak -C-
         }

         if (MissieFase == 3) {
            S.State = 107;        // Einde Opdracht Blikzoeken
         }
      }
      break;

      case 107 : {  // = EINDE OPDRACHT BLIKKEN Achteruit Rijden naar 0.0
         if (S.NewState) {
            //printf("case 107:Naar 0.0 pos. Einde opdracht \n");

            Driver.XY(0, -150 , -200, 0 );   // Naar midden parcour A<>B rijden
         }

         if (Driver.IsDone()) return S.Done(__FUNCTION__);   // Klaar!!!
      }
      break;

      case 20 : { // Blikken zoeken
         if (S.NewState) {
            printf("case 20: Naar blikken zoeken \n");
            Driver.SpeedLR(140, 140);   // default rechtuit
         }

         if (Position.XPos >= 2800) {    // 300 3100 Einde vak -B-
            Driver.SpeedLR(0, 0);           // Motoren Stoppen
            MissieFase = 2;                     // Gaan zoeken in vak -C-
            S.State = 105;
         }

         if (Position.YPos >= 1600) {    // Einde vak -C-
            Driver.SpeedLR(0, 0);            // Motoren Stoppen
            MissieFase = 3;                      // Klaar in vak -C-
            S.State = 105;
         }

         printf("Lidar S2 Distance %d, Degrees: %d\n",
               Lpp.Sensor[2].Distance, Lpp.Sensor[2].Degrees32 / 32);

         if (Lpp.Sensor[2].Distance < 600) {
            Driver.SpeedLR(0, 0);            // Motoren Stoppen
            S.State = 300;
         }
      }
      break;

      case 300 : { // BLIK GEZIEN-Draai naar blik .... graden
         if (S.NewState) {
            int BlikHoekCorrectie = Lpp.Sensor[2].Degrees32 / 32;
            if (BlikHoekCorrectie > 0) {
               BlikHoekCorrectie += 2;
            }
            else {
               BlikHoekCorrectie -= 4;
            }
            printf("HoekCor: %d\n", (BlikHoekCorrectie));
            Driver.Rotate(BlikHoekCorrectie); // Berekende graden links of rechts
         }
         if (Driver.IsDone()) {
            Driver.SpeedLR(0, 0);       // Rijden naar blik
            S.State += 10;
         }
      }
      break;

      case 310 : {  // Naar blik rijden
         if (S.NewState) Driver.SpeedLR(80, 80);

         if (Lpp.Sensor[2].Distance < 130) {
            Driver.SpeedLR(0, 0);       // Stop motoren
            printf("blik gevonden: \n");
            S.State += 10; // Naar de volgende state als de beweging klaar is
         }
      }
      break;

      case 320 : {  // Grijper sluiten

         if (ServoSlope(myservo, 2300, 20)) {// Wacht tot servo gesloten is, kies dan volgende stap

            if (MissieFase == 1) S.State = 500;  // Blikken terug brengen uit -B-

            if (MissieFase == 2) S.State += 10;  // Blikken terug brengen uit -C-
         }
      }
      break;

      case 330 : {  // Vanuit C naar midden parcour A<>B rijden
         if (S.NewState) Driver.XY(1500, 0 , -200, 0 );

         if (Driver.IsDone()) S.State += 10;
      }
      break;

      case 340 : {  // Draaien voor achterwaards terug naar -A- vak
         if (S.NewState) Driver.RotateHeading(0);

         if (Driver.IsDone()) S.State = 500;
      }
      break;

      // --------------------------------
      // Rij terug naar A en zet blik weg
      // --------------------------------

      case 500 : {   // Rij naar blik-afzet positie (vanuit gebied A-B)
         if (S.NewState) {

            int X = 0, Y = 0;
            switch (BlikNummer) {                  // (BlikNummer)
               case 0 : X = -50; Y = 440; break;   // 470 Blikken in -A- vorm
               case 1 : X = -50; Y =  10; break;   // 50 70
               case 2 : X = 100; Y = 360; break;   // 400 90-400
               case 3 : X = 100; Y = 210; break;   // 250 90-270
               case 4 : X = 300; Y = 210; break;   // 250 230-270
               case 5 : X = 100; Y =  60; break;   // 100 90-140
            }
            Driver.XY(X, Y, -200, 0);       // X, Y, Speed, EndSpeed - alles in mm(/sec)
         }

         if (Driver.IsDone()) S.State +=10;
      }
      break;

      case 510 : {   // Pos.0.0 > 180 gr draaien naar achter wand
        if (S.NewState) Driver.RotateHeading(180);

        if (Driver.IsDone()) S.State +=10;
      }
      break;

      case 520 : {   // Grijper openen
         if (ServoSlope(myservo, 550, 20)) S.State +=10;
      }
      break;

      case 530 : {   // Pos.0.0 > 90 gr draaien richting 0.0
         if (S.NewState) Driver.Rotate(90);

         if (Driver.IsDone()) S.State +=10;
      }
      break;

      case 540 : {   // Rijden naar 0.0 voor volgende sessie
         if (S.NewState) Driver.XY(150, 0 , 150, 0 );  //terug naar startpunt + 150

         if (Driver.IsDone()) S.State += 10;
      }
      break;

      case 550 : {   // Pos.0.0 > 90 gr draaien naar rijrichting
         if (S.NewState) {
            BlikNummer ++;
            printf("Case 10 Missie %d, BlikNummer: %d\n", MissieFase,  BlikNummer);
            Driver.RotateHeading(0);      // Draaien naar rijrichting
         }

         if (Driver.IsDone()) {
            if (BlikNummer >= 6) {        // 4<>7=6 Blikken = einde missie
               S.State += 10;
            }
            else {
               S.State = 2;
            }
         }
      }
      break;

      case 560 : {  // Einde blikzoeken
         if (S.NewState) {
            printf("Case 12 Missie: %d, Blik NR: %d\n", MissieFase, BlikNummer);
            Driver.XY(150, 0 , -100, 0 ); //terug naar startpunt + 100
         }

         if (Driver.IsDone()) return S.Done(__FUNCTION__);   // Report done & end mission
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
   }
   return false;  // mission nog niet gereed
}

