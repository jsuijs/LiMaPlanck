//-----------------------------------------------------------------------------
// LppLedRing.h  - Glue between LppMaster & a led ring
//-----------------------------------------------------------------------------
// The LedRing is an 360 degrees array of leds.
// This file contains code to use this ring. Probably not a core library
// of LiMaPlanck, but nevertheless - to some degree - reusable code...
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Degrees2RingIndex -
//-----------------------------------------------------------------------------
// Please specify:
//    const int LEDRING_OFFSET      = 180;   // change to set 1 and -1 degrees at proper led
//    const int LEDRING_CLOCKWISE   = false; // change to set -90 and 90 degrees at the proper led
//-----------------------------------------------------------------------------
int Degrees2RingIndex(int Degrees)
{
   // Settings
   int NrLeds = Leds.NrLeds();

   // Calculation
   int Index = ((Degrees + LEDRING_OFFSET) * NrLeds) / 360;
   if (LEDRING_CLOCKWISE) Index *= -1;

   while (Index < 0)       Index += NrLeds;
   while (Index >= NrLeds) Index -= NrLeds;

   return Index;
}

//-----------------------------------------------------------------------------
// LedEyes - show 'eyes' on a ledring (or ledstrip)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void LedEyes(int Center, int InColor)
{   TColor CHi;

      if (InColor == 0) CHi = CRGB_BLUE;
      if (InColor == 1) CHi = CRGB_RED;
      TColor CLo = CHi;
      CLo.Dim(16);

      Leds.Clear();
      Leds.Brightness = 8;
      Leds.RGB(Center - 4, CLo);
      Leds.RGB(Center - 3, CHi);
      Leds.RGB(Center - 2, CLo);

      Leds.RGB(Center + 2, CLo);
      Leds.RGB(Center + 3, CHi);
      Leds.RGB(Center + 4, CLo);
      Leds.Commit();
}

//-----------------------------------------------------------------------------
// LedTakt - for now, only to show a pattern...
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void LedTakt()
{  static int State;
   static int Center;
   static int SpeedCounter;

return;

   if (SpeedCounter > 0) {
      SpeedCounter --;
      return;
   }
   SpeedCounter = 1;

   switch (State) {
      case 0 : {
         Leds.Clear();
         Leds.Brightness = 8;
         Leds.Commit();
         Center = 20;
         State = 10;
      }
      break;

      case 10 : {
         LedEyes(Center, 1);
         Center ++;
         if (Center > 40) State += 10;
      }
      break;

      case 20 : {
         LedEyes(Center, 1);
         Center --;
         if (Center < 20) State = 10;
      }
      break;
   }
}

//-----------------------------------------------------------------------------
// ShowLppSensor -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ShowLppSensor(int Nr)
{

   if ((Nr < -1) || (Nr > 7)) return;

   TLppSetupData R = Lpp.ReadPrintSensorCfg(Nr, true);

//   printf("Mode: %d, Count: %d, Start: %d, Step: %d, Act: %d, %d\n",
//         R.Mode, R.StepCount, R.StartAngle, R.StepAngle, Lpp.Sensor[Nr].Degrees32 / 32, Lpp.Sensor[Nr].Distance);

   Leds.Clear();
   Leds.Brightness = 8;

   // Mark sensor-range blue
   TColor C = CRGB_BLUE;
   C.Dim(16);
   for (int i=R.StartAngle; i<(R.StartAngle + R.StepAngle); i++) {
      Leds.RGB(Degrees2RingIndex(i), C);
   }

   // mark sensor detection heading (based on cached reading)
   C = CRGB_RED;
   C.Dim(16);
   if (Lpp.Sensor[Nr].Distance < 9999) {
      Leds.RGB(Degrees2RingIndex(Lpp.Sensor[Nr].Degrees32 / 32), C);
   }

   Leds.Commit();
}
