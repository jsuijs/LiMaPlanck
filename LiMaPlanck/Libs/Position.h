//-----------------------------------------------------------------------------
// Position.h - Keep track of robot position & STM32 encoder routines...
//-----------------------------------------------------------------------------
// Note: #define FRAMEWORK_CODE enables acutual code, in adition to prototypes
//-----------------------------------------------------------------------------
#include "math.h"
#include "stdio.h"

class TPosition
{
   public:
      TPosition();
      void init() { Reset(); }
      void Takt();
      void Reset();
      void Set(float X, float Y, float Degrees);

      void OdoGet(int &OdoL_out, int &OdoR_out, int &OdoT_out) ;
      void Print();

      int  ActSpeedL, ActSpeedR;    // in odo_ticks per MAIN_TAKT_INTERVAL
      int  XPos;  // in mm
      int  YPos;
      int  Hoek;  // in graden

      long HoekHires() { return fVarRobotDegrees * 256; }

   private:
      // de robot positie.
      float fVarRobotXPos;       // in mm
      float fVarRobotYPos;       // in mm
      float fVarRobotDegrees;    // in 360 /cirkel

      float fOdoL;               // afstand in mm
      float fOdoR;               // afstand in mm
      float fOdoT;               // afstand in mm (gemiddelde van L+R, absolute waarde!)

      void Update();
};

extern TPosition Position;

//-----------------------------------------------------------------------------
// STM32 Encoders
void InitStmEncoders();
void ReadStmEncodersDelta(int &Left, int &Right);

#ifdef FRAMEWORK_CODE

//-----------------------------------------------------------------------------
// TPosition::TPosition - constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
TPosition::TPosition()
   {
      // call init when CSerial is up.
   }

//-----------------------------------------------------------------------------
// TPosition::OdoGet - Get OdoL, R (in mm), OdoT (absolute waarde, in mm)
//-----------------------------------------------------------------------------
// Gebruik deze routine voor besturing waarbij de x/y/hoek niet
// bruikbaar (of handig) is.
//-----------------------------------------------------------------------------
void TPosition::OdoGet(int &OdoL_out, int &OdoR_out, int &OdoT_out)
   {
      OdoL_out = fOdoL;
      OdoR_out = fOdoR;
      OdoT_out = fOdoT;
   }

//------------------------------------------------------------------------
// TPosition::Reset - Zet positie op 0, 0, 0
//------------------------------------------------------------------------
//------------------------------------------------------------------------
void TPosition::Reset()
   {
      Set(0,0,0);
   }

//------------------------------------------------------------------------
// TPosition::Set - Zet positie op gegeven waarde.
//------------------------------------------------------------------------
//------------------------------------------------------------------------
void TPosition::Set(float X, float Y, float Degrees)
   {
      printf("SetRobotPosition to %d %d %d\n", (int) X, (int) Y, (int) Degrees);
      fVarRobotXPos     = X;
      fVarRobotYPos     = Y;
      fVarRobotDegrees  = Degrees;

      // wis odo hulp vars:
      fOdoT       = 0;
      fOdoL       = 0;
      fOdoR       = 0;

      Update();   // update variabelen
   }

//-----------------------------------------------------------------------------
// TPosition::Print -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TPosition::Print()
   {
      Update();
      printf("RobotPosition X: %d, Y: %d, Hoek: %d, ActSpeed %d / %d\n", XPos, YPos, Hoek,ActSpeedL, ActSpeedR);
   }

//-----------------------------------------------------------------------------
// TPosition::Takt - Lees encoders & werk RobotPositie bij
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TPosition::Takt()
   {
      // Haal encoder gegevens op
      ReadStmEncodersDelta(ActSpeedL, ActSpeedR);

      if ((ActSpeedL !=0) || (ActSpeedR != 0)) { // als we verplaatst zijn

         // Update heading (eerste helft, incl. correctie voor verschil in wiel-grootte)
         float HalfHeadingDelta = (ActSpeedR * F_ODO_TICK_L_R - ActSpeedL) * F_ODO_HEADING / 2;
         fVarRobotDegrees += HalfHeadingDelta;

         // bereken afgelegde weg in mm
         float fDeltaT = (ActSpeedL + ActSpeedR) * F_ODO_TICK_TO_METRIC / 2;
         fOdoL += ActSpeedL * F_ODO_TICK_TO_METRIC;
         fOdoR += ActSpeedR * F_ODO_TICK_TO_METRIC;
         fOdoT += ABS(fDeltaT); // Absolute waarde : totaal afgelegde weg

         // update X/Y
         float RadHeading = GRAD2RAD(fVarRobotDegrees);
         fVarRobotXPos += fDeltaT * cos(RadHeading);
         fVarRobotYPos += fDeltaT * sin(RadHeading);

         // resterende heading update
         fVarRobotDegrees += HalfHeadingDelta;

         Update();
//       printf("SpeedL: %d, SpeedR: %d, Lticks: %ld, Rticks: %ld, DeltaL: %ld, DeltaR: %ld, XPos: %d YPos: %d Hoek: %d\n",
//          ActSpeedL, ActSpeedR, OdoL_ticks, OdoR_ticks, DeltaL, DeltaR, XPos, YPos, Hoek);
      }
   }

//------------------------------------------------------------------------
//------------------------------------------------------------------------
// Hieronder de PRIVATE procedures
//------------------------------------------------------------------------
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// TPosition::Update - update public var's
//------------------------------------------------------------------------
//------------------------------------------------------------------------
void TPosition::Update()
   {
      //--------------------
      // Normaliseer hoek
      while (fVarRobotDegrees >  (180))  fVarRobotDegrees -= 360;  // > 180 graden
      while (fVarRobotDegrees <= (-180)) fVarRobotDegrees += 360;  // =< -180 graden (<=, niet < omdat anders zowel 180 als -180 geldig zouden zijn)

      //--------------------
      // update public var's
      XPos  = fVarRobotXPos;     // RobotXPos in mm
      YPos  = fVarRobotYPos;     // RobotYPos in mm
      Hoek  = fVarRobotDegrees;  // RobotHoekPos in graden

      if (Flags.IsSet(20)) {
         // Note: float in format string doesn't work on
         // stm32duino and prinf seems to buffer until
         // newline, so three print-statements with CSerial
         // are required to print the position message:
         CSerial.printf("%cPOSITION %d %d ", FRAME_START, XPos, YPos);
         CSerial.print(GRAD2RAD(fVarRobotDegrees));
         CSerial.printf("%c\n", FRAME_END);
      }
   }

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Encoders.ino - Dit bestand bevat de low level routines voor de encoder.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#define MAQUEENPLUS_PIN_ENCODER_L_A    PA6   //PA6
#define MAQUEENPLUS_PIN_ENCODER_L_B    PA7   //PA7
#define MAQUEENPLUS_TIMER_ENCL         TIM3  // 16 bits timer/counter

#define MAQUEENPLUS_PIN_ENCODER_R_A    PA1    //PA0
#define MAQUEENPLUS_PIN_ENCODER_R_B    PA0    //PA1
#define MAQUEENPLUS_TIMER_ENCR         TIM2  // 16 bits timer/counter

HardwareTimer TimerEncL(MAQUEENPLUS_TIMER_ENCL);
HardwareTimer TimerEncR(MAQUEENPLUS_TIMER_ENCR);

void InitStmEncoders()
{
   //-----------------------------------------------------------------------
   // Init hardware quadrature encoders & corresponding IO
   //-----------------------------------------------------------------------
   pinMode(MAQUEENPLUS_PIN_ENCODER_L_A, INPUT_PULLUP);
   pinMode(MAQUEENPLUS_PIN_ENCODER_L_B, INPUT_PULLUP);
   pinMode(MAQUEENPLUS_PIN_ENCODER_R_A, INPUT_PULLUP);
   pinMode(MAQUEENPLUS_PIN_ENCODER_R_B, INPUT_PULLUP);

   // setup hardware quadrature encoders (requires STM32Cube functions)
   TIM_HandleTypeDef    Encoder_Handle;
   TIM_Encoder_InitTypeDef sEncoderConfig;

   /* Initialize TIM* peripheral as follow:
       + Period = 65535
       + Prescaler = 0
       + ClockDivision = 0
       + Counter direction = Up

      https://community.st.com/s/question/0D50X00009XkfbN/quadrature-encoder
      STM32Cube_FW_F4_V1.16.0\Projects\STM32469I_EVAL\Examples\TIM\TIM_Encoder\Src\main.c
   */

   Encoder_Handle.Init.Period             = 65535;
   Encoder_Handle.Init.Prescaler          = 0;
   Encoder_Handle.Init.ClockDivision      = 0;
   Encoder_Handle.Init.CounterMode        = TIM_COUNTERMODE_UP;
   Encoder_Handle.Init.RepetitionCounter  = 0;
   Encoder_Handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_DISABLE;

   sEncoderConfig.EncoderMode             = TIM_ENCODERMODE_TI12;

   sEncoderConfig.IC1Polarity             = TIM_ICPOLARITY_RISING;
   sEncoderConfig.IC1Selection            = TIM_ICSELECTION_DIRECTTI;
   sEncoderConfig.IC1Prescaler            = TIM_ICPSC_DIV1;
   sEncoderConfig.IC1Filter               = 0;

   sEncoderConfig.IC2Polarity             = TIM_ICPOLARITY_RISING;
   sEncoderConfig.IC2Selection            = TIM_ICSELECTION_DIRECTTI;
   sEncoderConfig.IC2Prescaler            = TIM_ICPSC_DIV1;
   sEncoderConfig.IC2Filter               = 0;

   Encoder_Handle.Instance = MAQUEENPLUS_TIMER_ENCL;
   if(HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK) printf("TIM2 init error");
   HAL_TIM_Encoder_Start(&Encoder_Handle, TIM_CHANNEL_ALL);

   Encoder_Handle.Instance = MAQUEENPLUS_TIMER_ENCR;
   if(HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK) printf("TIM3 init error");
   HAL_TIM_Encoder_Start(&Encoder_Handle, TIM_CHANNEL_ALL);

//   systick_attach_callback(&encoder1_read);
}

void ReadStmEncodersDelta(int &Left, int &Right)
{  static int RawEncoderLeft, RawEncoderRight;

   // save prev values
   int PrevEncoderLeft  = RawEncoderLeft;
   int PrevEncoderRight = RawEncoderRight;

   // read raw
   RawEncoderLeft  = -TimerEncL.getCount();  // flip sign here if required
   RawEncoderRight = TimerEncR.getCount();   // flip sign here if required

   // difference (cast to short int required to properly handle wrap around of 16-bit counters)
   Left  = (short int)(RawEncoderLeft  - PrevEncoderLeft);
   Right = (short int)(RawEncoderRight - PrevEncoderRight);
}

#endif