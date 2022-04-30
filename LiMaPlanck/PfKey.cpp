//-----------------------------------------------------------------------------
// PfKey.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

//-----------------------------------------------------------------------------
// PfKey - execute function key (one shot)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void PfKey(int ch)
{
   printf("Key: %d\n", ch);
   switch(ch) {

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
        //MissionControl.Start(MissionAloys1);
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

      default : {
         printf("ProgrammaTakt: onbekende PfKey %d\n", ch);
      }
      break;
   } // einde van switch
}
