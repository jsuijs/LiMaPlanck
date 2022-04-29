//-----------------------------------------------------------------------------
// Position.cpp
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#include "math.h"
#include "stdio.h"

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
      printf("ResetRobotPosition\n");
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

         // Corrigeer voor verschil in wiel-grootte & reken om naar graden
         fVarRobotDegrees += (ActSpeedR * ODO_TICK_L_R / 4096.0 - ActSpeedL) * (ODO_HEADING / 65536.0);   // ODO_HEADING is _Q16

         // reken afgelegde weg om naar mm
         float fDeltaL = ActSpeedL * (ODO_TICK_TO_METRIC / 4096.0);  // ODO_TICK_TO_METRIC is mm*4096
         float fDeltaR = ActSpeedR * (ODO_TICK_TO_METRIC / 4096.0);
         float fDeltaT = (fDeltaL + fDeltaR) / 2; 						   // /2 (ivm sum L+R)

         if (fDeltaT > 0) { 	// Absolute waarde / totaal afgelegde weg (o.a. voor compas)
            fOdoT += fDeltaT;
         } else {
            fOdoT -= fDeltaT;
         }
         fOdoL += fDeltaL;
         fOdoR += fDeltaR;

         // update X/Y positie
         float RadHeading = GRAD2RAD(fVarRobotDegrees);
         fVarRobotXPos += fDeltaT * cos(RadHeading);
         fVarRobotYPos += fDeltaT * sin(RadHeading);

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
