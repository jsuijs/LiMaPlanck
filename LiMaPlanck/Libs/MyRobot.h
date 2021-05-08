// file: MyRobot.h - include voor diverse .cpp files.
#ifndef MYROBOT_H
#define MYROBOT_H

//-----------------------------------------------------------------------------
// Position.cpp
//-----------------------------------------------------------------------------
#include "Arduino.h"

#include "Libs/LppMaster.h"   // contains code...
extern int LidarArray_L100;
extern int LidarArray_L80 ;
extern int LidarArray_L60 ;
extern int LidarArray_L40 ;
extern int LidarArray_L20 ;
extern int LidarArray_V   ;
extern int LidarArray_R20 ;
extern int LidarArray_R40 ;
extern int LidarArray_R60 ;
extern int LidarArray_R80 ;
extern int LidarArray_R100;

#include "Libs/Commands.h"    // contains code...

#define TICKS_360_GRADEN   (360L * 256 * 256 / ODO_HEADING)
#define GRAD2RAD(x)        ((float)(x) / 57.2957795)
#define MAIN_TAKT_RATE     (1000 / MAIN_TAKT_INTERVAL)   // Hz
#define WIEL_BASIS         ((ODO_TICK_TO_METRIC * 917L) / ODO_HEADING)
#define RAD2GRAD(x)        ((float)(x) * 57.2957795)   // uitkomst is float, deze kan evt zonder verlies geschaald worden naar hogere resulotie
#define ACT_SPEED_MM_SEC(ActSpeed) ((ActSpeed * (long)ODO_TICK_TO_METRIC)) / (4 * MAIN_TAKT_INTERVAL);
#define ABS(x)             ( (x>=0) ? x : -x )
#define NORM_Q8            (360L * 256)

#define FRAME_END             0xC0    /* indicates end of packet */
#define FRAME_START           0xC1    /* indicates start of packet */

template <typename T> inline
T ABSOLUTE(const T& v) { return v < 0 ? -v : v; }

class TPosition
{
   public:

      TPosition();
      void init() { Reset(); }
      void Takt();
      void Reset();

      void OdoGet(int &OdoL_out, int &OdoR_out, int &OdoT_out) ;
      void Print();

      int  ActSpeedL, ActSpeedR;    // in odo_ticks per MAIN_TAKT_INTERVAL
      int  XPos;  // in mm
      int  YPos;
      int  Hoek;  // in graden

      long HoekHires() { return VarRobotDegrees_q8; }

   private:

      // de robot positie.
      long int VarRobotXPos_q10;    // in 1/1024 mm (ca 1 um)
      long int VarRobotYPos_q10;    // in 1/1024 mm (ca 1 um)
      long int VarRobotDegrees_q8;  // in 360*256 /circel (graden*256)

      long int OdoL_ticks;          // afstand in odo_ticks  - LET OP - alleen voor hoek; var maakt sprong van '360 graden' ticks.
      long int OdoR_ticks;          // afstand in odo_ticks  - LET OP - alleen voor hoek;

      long int OdoL_q10;            // afstand in mm * 1024 (ongeveer um)
      long int OdoR_q10;            // afstand in mm * 1024 (ongeveer um)
      long int OdoT_q10;            // afstand in mm * 1024 (ongeveer um) (gemiddelde van L+R, absolute waarde!)

      // private methods
      void Update();
};

extern TPosition Position;

//-----------------------------------------------------------------------------
// Drive.cpp
//-----------------------------------------------------------------------------

// Constante per bewegingstype (DriveMode) die we ondersteunen.
enum TDiveMode { UNDEFINED, M_PWM, M_SPEED_LR, M_SPEED_ROTATION, M_SPEED_HEADING, M_XY, M_ROTATE, M_ARC, M_STOP };

class TDrive
{
   public:

      TDrive();
      void init();
      void Takt();
      bool IsDone();

      // bewegingen
      void Pwm(int PwmL, int PwmR);
      void SpeedLR(int SpeedL, int SpeedR);
      void SpeedRotation(int Speed, int Rotation_q8);
      void SpeedHeading(int Speed, int Heading);
      void XY(int X, int Y, int Speed, int EndSpeed);
      void RotateHeading(int Heading);
      void Rotate(int Degrees);
      void ArcHeading(int Heading, int Radius, int Speed, int EndSpeed);
      void Arc(int Degrees, int Radius, int Speed, int EndSpeed);
      void Stop();

      int SollSpeedL, SollSpeedR; // Snelheid (in mm/sec) die we nastreven, verandering begrensd door MaxSlope

   private:

      TDiveMode DriveMode;    // actief type aansturing
      int Param1;             // Paramers van actieve aansturing type
      int Param2;
      int Param3;
      int Param4;

      bool IsDoneFlag;        // Movement is gereed
      bool NewMovement;

      int MaxSlope;

      void SpeedLRTakt(bool FirstCall, int SpeedL, int SpeedR, int MaxSlopeP);
      bool SpeedRotationTakt(bool FirstCall, int InSpeed, int InRotation_q8);
      bool SpeedHeadingTakt(bool FirstCall, int InSpeed, int InHeading);
      bool XYTakt(bool FirstCall, int TargetX, int TargetY, int Speed, int EndSpeed);
      bool RotateRelTakt(bool FirstCall, int DeltaDegrees);
      bool ArcRelTakt(bool FirstCall, int DeltaDegrees, int Radius, int Speed, int EndSpeed);
      bool StopTakt(bool FirstCall);
};

extern TDrive Driver;

//-----------------------------------------------------------------------------
class TState
{
   public:

      TState() { Reset(); }

      void Update(const char *InName, bool Verbose=true) {
         NewState = false;
         if (PrevState != State) {
            if (Verbose) CSerial.printf("%s state %d -> %d\n", InName, PrevState, State);

            PrevState      = State;
            NewState       = true;
            StateStartTime = millis();
         }
      }

      void Reset() {
         State  = 0;
         PrevState = -1;
      }

      int StateTime() {
         return millis() - StateStartTime;
      }

      int  State;
      bool NewState;

      int  Param1;   // user param

   private:
      int PrevState;
      int StateStartTime;
};


//-----------------------------------------------------------------------------
class TBuzzer
{
   public:

      //-----------------------------------------------------------------------
      // Construct - store pin & set pin to output
      //-----------------------------------------------------------------------
      //-----------------------------------------------------------------------
      TBuzzer(int Pin)
      {
         BeepState      = 0;
         BeepDuration   = 0;
         BuzzerPin      = Pin;
         pinMode(BuzzerPin, OUTPUT);
      }

      //-----------------------------------------------------------------------
      // Beep - sound nr of beeps.
      //-----------------------------------------------------------------------
      // duration = time length for sound and silence (in ms)
      //-----------------------------------------------------------------------
      void Beep(int duration, int number_of_beeps = 1)
      {
         BeepState      = 2 * number_of_beeps-1;
         BeepDuration   = duration;
         BeepCountDown  = BeepDuration;
      }

      void BeepWait(int duration, int number_of_beeps = 1)
      {
         Beep(duration, number_of_beeps);
         Wait();
      }

      //-----------------------------------------------------------------------
      // Call this at 1 kHz rate
      //-----------------------------------------------------------------------
      //-----------------------------------------------------------------------
      void Takt()
      {
         if (BeepCountDown > 0) {
            // buzzy beeping
            BeepCountDown --;
            if (BeepCountDown<=0) {
               // this pulse done
               if (BeepState) {
                  // more pulses to go
                  BeepState --;
                  BeepCountDown = BeepDuration;
               }
            }
         }
         if (BeepState & 1) {
            // odd BeepState generate sound
            Toggle = !Toggle;    // create 500 Hz square wave
         } else {
            Toggle = false;
         }
         digitalWrite(BuzzerPin, Toggle);  // drive buzzer
      }

      void Wait()
      {
         for(;BeepState;) { /* wait for BeepState to become 0*/ }
      }

   private :
      int BuzzerPin;
      volatile int BeepState, BeepDuration;
      int BeepCountDown, Toggle;
};

//-----------------------------------------------------------------------------
class TFlags
{
   public:

      TFlags(int NrFlags) {
         SetIx(NrFlags-1); // 32 means 0..31
         NrFlagWords = WordIx + 1;
         FlagWords   = (int *)malloc(NrFlagWords * sizeof(int));
         for (int i=0; i<NrFlagWords; i++) FlagWords[i] = 0;;
      }

      bool IsSet(int Nr) {
         if (!SetIx(Nr)) return false;
         return ((FlagWords[WordIx] & (1<<BitIx)) != 0);
      }

      void Set(int Nr, bool Value) {
         if (!SetIx(Nr)) {
            CSerial.printf("ERROR setting flag %d\n", Nr);
            return;
         }
         if (Value) {
            FlagWords[WordIx] |= (1<<BitIx);
         } else {
            FlagWords[WordIx] &= 0xFFFFFFFF ^ (1<<BitIx);
         }
         CSerial.printf("Flag %d set to %d (%d %d %d)\n", Nr, Value, WordIx, BitIx, NrFlagWords);
      }

      void Dump() {
         CSerial.printf("NrFlagWords: %d\n", NrFlagWords);
         for (int i=0; i<NrFlagWords; i++) CSerial.printf("%08x ", FlagWords[i]);
         CSerial.printf("\n");
      }

   private:
      int *FlagWords;
      int NrFlagWords;  // WordIx + 1
      int WordIx, BitIx;

      bool SetIx(int Nr) {  // return true if Nr is valid
         WordIx = Nr / 32;
         BitIx  = Nr - 32 * WordIx;
         if (WordIx >= NrFlagWords) return false;  // out of range
         if (!FlagWords)            return false;  // no vars malloc'd
         return true;
      }
};

extern TFlags Flags;

//-----------------------------------------------------------------------------
// Encoders
void InitStmEncoders();
void ReadStmEncodersDelta(int &Left, int &Right);

extern volatile int EncoderLTeller, EncoderRTeller;  // aantal flanken

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

// ProgrammaTakt.cpp
void ProgrammaTakt();

// RcDispatch.cpp
void RcDispatch(int &RcData);
int  PfKeyGet();
void PfKeySet(int InKey);

#endif
