//-----------------------------------------------------------------------------
// ProgrammTakt.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//    ***** MissionSuperSlalom   19-09-2021 (JS) ****** Super Slalom H&W
//-----------------------------------------------------------------------------
//lppstart
//passagesetup 180 5 16
//passagefind 800 2
//int PassageStart; // start of detected passage
//int PassageLen;   // width (in ArrayDegrees) of passage
//printf("PassageLen Adres %d\n", Passage);  // << Dit is het adres waar de class in het geheugen staat. Ga je niks mee doen...
//-----------------------------------------------------------------------------
bool MissionSuperSlalom(TState &S)
{
  S.Update(__FUNCTION__);
  static int Pos1X;
  static int Pos1Y = 500;  // 600;
  static int Pos2X;
  //static int Pos2Y;

  switch (S.State) {

    case 0 : {    // Start lidar & open servo (indien nodig)
        if (S.NewState) {
          Position.Reset();
          Lpp.Start();
          Passage.Setup(90, 5, 16);         // LINKS   opening zoeken
        }

        if (S.StateTime() > 3000) {      // Wacht op start lidar
          if (ServoSlope(myservo, 550, 20)) S.State += 10; // Grijper open
        }
      }
      break;

    case 10 : {   // R1= Opening -1- zoeken
        if (S.NewState) Driver.SpeedLR(150, 150);   // default rechtuit

        if (Lpp.Sensor[S_NUL].Distance > 650) {     // Rijden tot eerste blik (voldoende afstand tot achterwand)

          int Deg = Passage.Find(800, 5);          // 2 -800 meetlengte opening 2 blikken

          printf("HoekOpening %d, Passage Start %d, Len %d\n",
             Deg, Passage.PassageStart, Passage.PassageLen);

          if (Deg < - 3 ) {                       // -0- Opening aan de linkerzijde
             Driver.SpeedLR(0, 0);                 // Stoppen
             Pos1X = (Position.XPos + 100);        // merkpunt opening -1R- Y0
             S.State += 10;
          }
        }
      }
      break;

    case 20 : {   // Bocht 90 gr CCW
        if (S.NewState) Driver.Rotate(90);   // 90 graden linksom

        if (Driver.IsDone()) S.State += 10;
      }
      break;

    case 30 : {   // Door eerste opening rijden
        if (S.NewState) Driver.SpeedLR(120, 120);       // Voorwaards

        if (Position.YPos >= Pos1Y) {     // 600/700 Einde opening -1-
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          //Pos2Y = Position.YPos;          // merkpunt opening -1L-
          S.State += 10;
        }
      }
      break;

    case 40 : {   // 90 gr CW draaien = voorwaards
        if (S.NewState) Driver.RotateHeading(0);  // vooruit rijden

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 50 : {   // R2= Tweede opening zoeken
        if (S.NewState) {
          //Passage.Setup(90, 5, 16);     // LINKS   opening zoeken
          //Passage.Setup(180, 5, 16);    // VOOR    opening zoeken
          Passage.Setup(270, 5, 16);      // RECHTS  opening zoeken
          Driver.SpeedLR(150, 150);       // Voorwaards
        }

        if (S.StateTime() > 2000) {      // Wacht tot volgende blik

          int Deg = Passage.Find(800, 8);     // 2 -800 meetlengte opening 2 blikken

          printf("HoekOpening %d\n", Deg);
          printf("PassageStart %d\n", Passage.PassageStart);
          printf("PassageLen %d\n", Passage.PassageLen);
          if (Deg < 3600) {
            if (Deg > 2 ) {               // -0- Opening aan de Rechterzijde
              Driver.SpeedLR(0, 0);         // Stoppen
              Pos2X = (Position.XPos + 120);        // merkpunt opening -2L- Y600
              S.State += 10;
            }
          }
        }
      }
      break;

    case 60 : {   //
        if (S.NewState) Driver.RotateHeading(-90);  // 90 gr Rechts draaien

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 70 : {   //
        if (S.NewState) Driver.SpeedLR(150, 150);       // Door opening Twee rijden

        if (Position.YPos <= 0) {         // terug naar Y0=startlijn (600)
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          S.State += 10;
        }
    }
    break;

    case 80 : {   //
        if (S.NewState) Driver.RotateHeading(0);  // 90 gr CCW vooruit rijden

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 90 : {   // R3=  Derde opening = vak -B-
        if (S.NewState) {
          printf("case 80 :Naar x 2900. 0 pos. \n");
          Driver.XY(2900, 0 , 150, 0 );   // X=3000 naar eindpunt vak -B-
        }
        if (Driver.IsDone()) {
          S.State += 10;
        }
      }
      break;

    case 100 : {  // 90 gr CCW draaien in vak -B-
        if (S.NewState) Driver.RotateHeading(90);

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 110 : {  // Door opening 3 == vak -B-
        if (S.NewState) Driver.SpeedLR(150, 150);       // Voorwaards
        if (Position.YPos >= Pos1Y) {     // 600/700 Einde opening in vak -B-
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          S.State += 10;
        }
      }
      break;

    case 120 : {  // 90 gr CCW draaien Terugrit
        if (S.NewState) Driver.RotateHeading(180);

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 130 : {  // R4= Tweede opening zoeken = terugweg
        if (S.NewState) Driver.SpeedLR(150, 150);       // Voorwaards
        if (Position.XPos <= Pos2X) {      // Einde Terugweg Opening twee
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          S.State += 10;
        }
      }
      break;

    case 140 : {  // 90 gr CCW draaien Terugrit
        if (S.NewState) Driver.RotateHeading(-90);

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 150 : {  // Door opening -2- terugweg
        if (S.NewState) Driver.SpeedLR(150, 150);       // Voorwaards
        if (Position.YPos <= 0) {         // Einde opening -2- terugweg  (600)
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          //Pos2Y = Position.YPos;          // merkpunt opening -1L-
          S.State += 10;
        }
      }
      break;

    case 160 : {  //
        if (S.NewState) Driver.RotateHeading(180);  // 90 gr Rechts draaien

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);           // Stoppen
          S.State += 10;
        }
      }
      break;

    case 170 : {  // R5= Eerste opening zoeken = terugweg
        if (S.NewState) Driver.SpeedLR(150, 150);       // Voorwaards

        if (Position.XPos <= Pos1X) {      // Einde Terugweg Opening -EEN-
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          S.State += 10;
        }
      }
      break;

    case 180 : {  //
        if (S.NewState) Driver.RotateHeading(90);  // 90 gr Rechts draaien naar eerste opening

        if (Driver.IsDone()) {
          Driver.SpeedLR(0, 0);             // Stoppen
          S.State += 10;
        }
      }
      break;

    case 190 : {  // Door opening 3 == vak -B-
        if (S.NewState) Driver.SpeedLR(150, 150);       // Voorwaards
        if (Position.YPos >= 600) {       // Einde opening in vak -B-
          Driver.SpeedLR(0, 0);           // Motoren Stoppen
          S.State += 10;
        }
      }
      break;

    case 200 : {  // 90 gr CCW draaien Terugrit
        if (S.NewState) Driver.RotateHeading(180);

        if (Driver.IsDone()) {
          S.State += 10;
        }
      }
      break;

    case 210 : {  // R6= Rijden naar x 0. 600  Eindpunt
        if (S.NewState) Driver.XY(0, 600 , 150, 0 );   // Y=600 naar eindpunt

        if (Driver.IsDone()) {
          //S.State = 999;
          return true;
        }
      }
      break;

    default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
  }
  return false;  // Nog niet klaar.
}


