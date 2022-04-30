//-----------------------------------------------------------------------------
// MyRobot.h - include voor diverse .cpp files.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#ifndef MYROBOT_H
#define MYROBOT_H

#include <Arduino.h>
#include <Servo.h>

#define TICKS_360_GRADEN   (360L * 256 * 256 / ODO_HEADING)
#define GRAD2RAD(x)        ((float)(x) / 57.2957795)
#define MAIN_TAKT_RATE     (1000 / MAIN_TAKT_INTERVAL)   // Hz
#define WIEL_BASIS         ((ODO_TICK_TO_METRIC * 917L) / ODO_HEADING)
#define RAD2GRAD(x)        ((float)(x) * 57.2957795)   // uitkomst is float, deze kan evt zonder verlies geschaald worden naar hogere resulotie
#define ACT_SPEED_MM_SEC(ActSpeed) ((ActSpeed * (long)ODO_TICK_TO_METRIC)) / (4 * MAIN_TAKT_INTERVAL);
#define ABS(x)             ( (x>=0) ? x : -x )
#define NORM_Q8            (360L * 256)

template <typename T> inline
T ABSOLUTE(const T& v) { return v < 0 ? -v : v; }

#define FRAME_END          0xC0  // indicates end of packet
#define FRAME_START        0xC1  // indicates start of packet

#include "Libs/Flags.h"          // contains code...
#include "Libs/State.h"          // contains code...
#include "Libs/MissionControl.h" // contains code...
#include "Libs/Position.h"       // contains code...

#include "Libs/Commands.h"       // contains code...
#include "Libs/Buzzer.h"         // contains code...
#include "Libs/RC5.h"            // contains code...

#include "Libs/LppMaster.h"      // contains code...
#include "Libs/PassageFinder.h"  // contains code...
#include "Libs/Apa102.h"         // contains code...

bool ServoSlope(Servo &S, int Setpoint, int Step);

//-----------------------------------------------------------------------------
// Motors.cpp
void SetupMotors();
void Motors(int PwmL, int PwmR);

//-----------------------------------------------------------------------------
// MotorController.cpp
void MotorController(int SetpointL, int SetpointR);

// Utilities.cpp
int EenparigVertragen( int Afstand, int SetSpeed, int EndSpeed, int Vertraging);
long Clip(long input, long min, long max);
void Slope(int &SlopeInOut, int Setpoint, int Step);
long NormHoek(long hoek, long Norm);
void Cartesian2Polar(long &hoek, int &afstand, int x, int y);
void I2cClearBus(int PinSda, int PinScl);

// ProgrammaTakt.cpp
void ProgrammaTakt();

// RcDispatch.cpp
void RcDispatch(int &RcData);
int  PfKeyGet();
void PfKey(int InKey);


#include "Libs/Drive.h"          // contains code...

#endif
