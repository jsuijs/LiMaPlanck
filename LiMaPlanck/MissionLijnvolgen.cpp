//-----------------------------------------------------------------------------
// MissionLijnvolgen.cpp -
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

PID LinePid(0.007, 0.0009, 0.0, false);
int LineSpeedSetpoint = 100;

//-----------------------------------------------------------------------------
// MissionLijnVolgen -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionLijnVolgen(TState &S)
{
   S.Update(__FUNCTION__, Flags.IsSet(11));

   switch (S.State) {
      case 0 : {  // LIDAR-START
         if (S.NewState) {
            LinePid.SetOutputLimits(-1000, 1000);
         }

         S.State++;
      }
      break;


      case 1 : {  // Follow line
         if (S.NewState) {
            // Nothing
         }
         LinePid.Compute(LinePosition, 0);
         Driver.Pwm((LineSpeedSetpoint - LinePid.Output), (LineSpeedSetpoint + LinePid.Output));   // Lijnvolgen
         printf("MLV %d -> %d\n", LinePosition, (int)LinePid.Output);
      }
      break;


      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
   }
   return false;  // mission nog niet gereed
}

