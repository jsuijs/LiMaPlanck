//-----------------------------------------------------------------------------
// ProgrammTakt.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

// prototypes

bool __attribute__ ((weak)) MissionAloys1(TState &S) { S = S; return true; }
bool __attribute__ ((weak)) MissionServo1(TState &S) { S = S; return true; }
TState MissonS;  // Mission statemachine

//-----------------------------------------------------------------------------
// ProgrammaTakt - program-selection & call the program (mission)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ProgrammaTakt()
{  static TState Program;

   int ch = PfKeyGet();
   if (ch) {
      // knop ingedrukt

      printf("Key: %d\n", ch);
      if (ch == -1) {
         Program.Reset();           // reset, stop lopend programma / programma 'stilstaan'.
      } else {
         if (Program.State == 0) {  // andere pfkeys werken alleen als we stil staan
            Program.State = ch;
         }
      }
   }

   Program.Update("Program", Flags.IsSet(10));
   if (Program.NewState) {
      LppSensorDefaultSetup();      // re-load Lidar default configuration
      MissonS.Reset();              // reset mission statemachine
   }

   // Call active program 1 trough 12
   switch(Program.State) {

      case 0 : { // Program: stand-still
         if (Program.NewState) {
            Driver.Pwm(0, 0); // only on entry, so CLI-commands can be used in this state.
            Lpp.Stop();
         }
      }
      break;

      case 1 : { // Programma: rijden1
//         if (MissionRijden1(Program)) Program.State = 0;
      }
      break;

    case 2 : {  // Programma: Grijper open/toe
        if (MissionGripperTest(MissonS)) Program.State = 0;  //**Grijper open/toe
      }
      break;

      case 3 : {  // Programma:
         if (MissionDuckling(MissonS)) Program.State = 0;
      }
      break;

      case 4 : { // Programma:
        if (MissionAloys1(MissonS)) Program.State = 0;
      }
      break;

      case 7 : { // Programma:
//         if (MissionTest(MissonS)) Program.State = 0;
      }
      break;

      case 8 : { // Programma: MissieBlikken
//         if (MissieBlikken(MissonS)) Program.State = 0;
      }
      break;

//      case 9 : { // Programma: Heen en Weer
//         MissonS.Param1 = 200;  // speed
//         if (MissieHeenEnWeer(MissonS)) Program.State = 0;
//      }
//      break;

      case 10 : { // Programma: ttijd
         MissonS.Param1 = 300;  // speed
//         if (MissieTTijd(MissonS)) Program.State = 0;
      }
      break;

      case 11 : { // Programma:
        if (MissionRandomRijden(MissonS)) Program.State = 0;
      }
      break;

      case 12 : { // Programma: test1
         Driver.Rotate(90);
      }
      break;
    case 102 : { // Programma: MissionOdoTest CW
        MissonS.Param1 = -1;               // set CW
        if (MissionOdoTest(MissonS)) Program.State = 0;
      }
      break;

    case 103 : { // Programma: MissionOdoTest CCW
        MissonS.Param1 = 1;                // set CCW
        if (MissionOdoTest(MissonS)) Program.State = 0;
      }
      break;

      default : {
         printf("ProgrammaTakt: ongeldig programma %d\n", Program.State);
         Program.Reset();
      }
      break;
   } // einde van switch
}

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
