//-----------------------------------------------------------------------------
// ProgrammTakt.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

// prototypes

bool __attribute__ ((weak)) MissionAloys1(TState &S) { S = S; return true; }
TState SubS;            // Sub-mission statemachine

//-----------------------------------------------------------------------------
// ProgrammaTakt - program-selection & call the program (mission)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ProgrammaTakt()
{
   int ch = PfKeyGet();
   if (ch) printf("Key: %d\n", ch);

   // One shot call to PfKey commands
   switch(ch) {

      case 0 : {  // Default - no key
      }
      break;

      case -1 : { // Stop
         MissionControl.Reset(); // stop mission
         Driver.Pwm(0, 0);       // stop motors
      }
      break;

      case 2 : {  // Programma: Grijper open/toe
        MissionControl.Start(MissionGripperTest);
      }
      break;

      case 3 : {  // Programma:
         MissionControl.Start(MissionDuckling);
      }
      break;

      case 4 : { // Programma:
        MissionControl.Start(MissionAloys1);
      }
      break;

      case 7 : { // Programma:
         MissionControl.Start(MissionSuperSlalom);
      }
      break;

      case 8 : { // Programma: MissieBlikken
         MissionControl.Start(MissionBlikken);
      }
      break;

//      case 9 : { // Programma: Heen en Weer
//         MissionControl.S.Param1 = 200;  // speed
//         MissionControl.Start(MissieHeenEnWeer);
//      }
//      break;

      case 10 : { // Programma: ttijd
         MissionControl.S.Param1 = 300;  // speed
//         MissionControl.Start(MissieTTijd);
      }
      break;

      case 11 : { // Programma:
        MissionControl.Start(MissionRandomRijden);
      }
      break;

      case 12 : { // Programma: test1
         Driver.Rotate(90);
      }
      break;

      case 101 : { // Programma: MissionStartVector1
         MissionControl.Start(MissionStartVector1);
      }
      break;

      case 102 : { // Programma: MissionWheelSizeCalibrate CW
         MissionControl.S.Param1 = 2000;              // distance to drive
         MissionControl.S.Param2 = -1;                // set CW
         MissionControl.Start(MissionWheelSizeCalibrate);
      }
      break;

      case 103 : { // Programma: WheelSizeCalibrate CCW
         MissionControl.S.Param1 = 2000;              // distance to drive
         MissionControl.S.Param2 = 1;                 // set CCW
         MissionControl.Start(MissionWheelSizeCalibrate);
      }
      break;

      default : {
         printf("ProgrammaTakt: onbekende PfKey %d\n", ch);
      }
      break;
   } // einde van switch

   // Execute mission
   if (MissionControl.Takt()) {
      // Mission done.
      Driver.Pwm(0, 0); // only once, so CLI-commands can be used
   }
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
