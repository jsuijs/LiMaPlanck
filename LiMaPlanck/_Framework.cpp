//-----------------------------------------------------------------------------
// _Framework.cpp
//-----------------------------------------------------------------------------
// Alle generieke files.
//-----------------------------------------------------------------------------
#define FRAMEWORK_CODE

#include "RobotSettings.h"
#include "Libs/PID_v1.h"
#include "Libs/MyRobot.h"  // this includes code with FRAMEWORK_CODE defined.
#include "Project.h"

#include "Libs/PID_v1.cpp"
#include "Libs/MyRobot.cpp"
#include "Libs/MotorController.cpp"
#include "Libs/RcDispatch.cpp"

#include "Libs/Utilities.cpp"
#include "Libs/Adafruit_ADS1015.cpp"