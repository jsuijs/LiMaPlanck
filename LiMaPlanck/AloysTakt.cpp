//-----------------------------------------------------------------------------
// ProgrammTakt.cpp
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

//-----------------------------------------------------------------------------
// ***** MissionServo1 - state machine *****************************
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionServo1(TState & S) // Eerste Servo Test
{
   S.Update(__FUNCTION__, Flags.IsSet(11));

   switch (S.State) {

      case 0 : {  // Grijper open
         if (ServoSlope(myservo, SERVO_OPEN, 20)) S.State ++;
      }
      break;

      case 1 : {  // Wacht
         if (S.StateTime() > 3000) S.State++;
      }
      break;

      case 2 : {  // Grijper sluiten
         if (ServoSlope(myservo, SERVO_CLOSE, 20)) S.State ++;
      }
      break;

      case 3 : {  // Wacht
         if (S.StateTime() > 2000) S.State = 0;
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
   }
   return false;  // mission nog niet gereed
}
//*************************************
