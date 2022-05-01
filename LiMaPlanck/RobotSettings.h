//-----------------------------------------------------------------------------
// RobotSettings.h - Robot-specific settings
//-----------------------------------------------------------------------------
// Put all settings for your robot in this file
// (Put 'normal' includes inProject.h)

#ifndef ROBOTSETTINGS_H
#define ROBOTSETTINGS_H

#define ROBOT_JOEP
//#define ROBOT_ALOYS
//#define ROBOT_KAREL

#define CSerial Serial2 // define Console-serial.
#define MyPrintf printf

// Choose i2c for Lidar Preprocessor
#include <Wire.h>
extern  TwoWire Wire2;
#define LppWire Wire2

// IO mapping
const int BUZZER_PIN = PA4;

//-----------------------------------------------------------------------------
// TB6612 Motor configuration
const int TB6612_PWML      = PA8;   // Pin 27
const int TB6612_PWMR      = PA11;  // pin 24
const int TB6612_DIRL_A1   = PB15;  // pin 28
const int TB6612_DIRL_A2   = PA10;  // pin 25  jumper pin 23 PA12
const int TB6612_DIRR_B1   = PB14;  // pin 29
const int TB6612_DIRR_B2   = PA9;   // pin 26  jumper pin 31 PB12

//-----------------------------------------------------------------------------
// Timing, odometrie etc.

const int MAIN_TAKT_INTERVAL  = 20;    // miliseconds
const int MAX_SLOPE           = 10;    // in mm/sec/MAIN_TAKT_INTERVAL

// Motor PID (MPID) parameters
const int MPID_I_MAX          = 100;   // integrator limit
const int MPID_OFFSET         =  10;   // motor offset
const float MPID_SP_GAIN      = 1.0;   // setpoint gain
const float MPID_P_GAIN       = 1.0;
const float MPID_I_GAIN       =   1;
const float MPID_D_GAIN       =  -1;

// P-gain for heading & limit of heading-correction
const int PID_Kp              = 1000;  // hoekerr gain / 4096
const int PID_OUT_CLIP        =   12;  // correctie clip (degrees)

const int ROTATE_CLIP_Q8      =  2560; // degrees / tick
const float ROTATE_P_GAIN     =  0.08;
const float ROTATE_D_GAIN     =  0.05;

// ---------------------------------------------------------
// settings per user - Activate at the top of this file
// ---------------------------------------------------------

//=============js===============================
#ifdef ROBOT_JOEP

   // Encoder parameters (Position.h)
   const float F_ODO_TICK_TO_METRIC = 0.124023;    // Distance per tick in mm (left wheel).
   const float F_ODO_TICK_L_R       = 1.000195;    // Wheel-size correction. (Right wheel vs left one, 4096 = equal size)
   const float F_ODO_HEADING        = 0.039581;    // Odo-ticks to heading, larger => more degrees per tick
                                                   // ODO_HEADING = (ODO_TICK_TO_METRIC * 57.2957795) / WIELBASIS (in mm)
                                                   // If the robots rotates too much, increase this number

   // Define RC5-codes of (amongst others) function-keys
   const int RC_STOP       = 0x3775; // STOP robot - stop button (Function-key -1)

   const int RC_F01        = 0x3770; // mark
   const int RC_F02        = 0x2745; // audio
   const int RC_F03        = 0x2741; // title menu
   const int RC_F04        = 0x2749; // sub title
   const int RC_F05        = 0x2744; // input
   const int RC_F06        = 0x2740; // tv
   const int RC_F07        = 0x2746; // hdd
   const int RC_F08        = 0x274a; // on line
   const int RC_F09        = 0x3769; // zoom
   const int RC_F10        = 0x274c; // - volume
   const int RC_F11        = 0x3751; // + volume
   const int RC_F12        = 0x3750; // store

   const int LPP_OFFSET    = -3;
   const int LPP_REVERSE   = 1;

   const int SERVO_CLOSE   = 2200;
   const int SERVO_OPEN    =  550;
#endif // ROBOT_JOEP


//=============kd===============================
#ifdef ROBOT_KAREL

   // Encoder parameters (Position.h)
   const float F_ODO_TICK_TO_METRIC =  508/ 4096.0;   // Distance per tick (left wheel). 4096 means 1 tick => 1 mm
   const float F_ODO_TICK_L_R       = 4096.8/4096.0;  // Wheel-size correction. (Right wheel vs left one, 4096 = equal size)
   const float F_ODO_HEADING        = 2594/ 65536.0;  // Odo-ticks to heading, larger => more degrees per tick
                                                      // ODO_HEADING = (ODO_TICK_TO_METRIC * 917) / WIELBASIS (in mm)
                                                      // If the robots rotates too much, increase this number

   // Define RC5-codes of (amongst others) function-keys
   const int RC_STOP       = 0x3775; // STOP robot - stop button (Function-key -1)

   const int RC_F01        = 0x3770; // mark
   const int RC_F02        = 0x2745; // audio
   const int RC_F03        = 0x2741; // title menu
   const int RC_F04        = 0x2749; // sub title
   const int RC_F05        = 0x2744; // input
   const int RC_F06        = 0x2740; // tv
   const int RC_F07        = 0x2746; // hdd
   const int RC_F08        = 0x274a; // on line
   const int RC_F09        = 0x3769; // zoom
   const int RC_F10        = 0x274c; // - volume
   const int RC_F11        = 0x3751; // + volume
   const int RC_F12        = 0x3750; // store

   const int LPP_OFFSET    = -3;
   const int LPP_REVERSE   = 1;

   const int SERVO_CLOSE   = 2200;
   const int SERVO_OPEN    =  550;
#endif // ROBOT_KAREL

//=============av===============================
#ifdef ROBOT_ALOYS

   // Encoder parameters (Position.h)        // Testen met Joep = Bergenop Zoom 11-09-2021
   const float F_ODO_TICK_TO_METRIC =  508/ 4096.0;   //496; av508 Distance per tick (left wheel). 4096 means 1 tick => 1 mm
   const float F_ODO_TICK_L_R       = 4109/ 4096.0;   //96; boz av4109  // Wheel-size correction. (Right wheel vs left one, 4096 = equal size)

   const float F_ODO_HEADING        = 2592/ 65536.0;  //av2592; boz 2574 // Odo-ticks to heading, larger => more degrees per tick
                                                      // ODO_HEADING = (ODO_TICK_TO_METRIC * 917) / WIELBASIS (in mm)
                                                      // If the robots rotates too much, increase this number

   // Define RC5-codes of (amongst others) function-keys
   const int RC_STOP       = 0x300c; // 0/1-STOP robot

   const int RC_F01        = 0x3001; // Knop 1
   const int RC_F02        = 0x3002; // Knop 2
   const int RC_F03        = 0x3003; // Knop 3
   const int RC_F04        = 0x3004; // Knop 4
   const int RC_F05        = 0x3005; // Knop 5
   const int RC_F06        = 0x3006; // Knop 6
   const int RC_F07        = 0x3007; // Knop 7
   const int RC_F08        = 0x3008; // Knop 8
   const int RC_F09        = 0x3009; // Knop 9
   const int RC_F10        = 0x300a; // DrKn << >>
   const int RC_F11        = 0x300d; // Mute = knop 13!!
   const int RC_F12        = 0x300b; // Knop CH+P/C
	const int RC_F14        = 0x3018; // Knop Knop -A-

   const int LPP_OFFSET    = -4;
   const int LPP_REVERSE   = 1;

   const int SERVO_CLOSE   = 2300;
   const int SERVO_OPEN    =  500;
#endif // ROBOT_ALOYS

#endif
