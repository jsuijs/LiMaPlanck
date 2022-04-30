//-----------------------------------------------------------------------------
// Buzzer.h  - take a guess.
//-----------------------------------------------------------------------------
// Note: #define FRAMEWORK_CODE enables acutual code, in adition to prototypes
//-----------------------------------------------------------------------------

class TBuzzer
{
   public:
      TBuzzer(int Pin);
      void Takt();

      void Beep(int duration, int number_of_beeps = 1);
      void BeepWait(int duration, int number_of_beeps = 1);
      void Wait();

   private :
      int BuzzerPin;
      volatile int BeepState, BeepDuration;
      int BeepCountDown, Toggle;
};

extern TBuzzer Buzzer;

#ifdef FRAMEWORK_CODE

//-----------------------------------------------------------------------
// Constructor - store pin & set pin to output
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
TBuzzer::TBuzzer(int Pin)
   {
      BeepState      = 0;
      BeepDuration   = 0;
      BuzzerPin      = Pin;
      pinMode(BuzzerPin, OUTPUT);
   }

//-----------------------------------------------------------------------
// TBuzzer::Takt - drive beep pattern.
//-----------------------------------------------------------------------
// Call at 1 kHz rate.
//
// e.g.
//    TBuzzer Buzzer(BUZZER_PIN);
//    void HAL_SYSTICK_Callback(void) { Buzzer.Takt(); }
//
//-----------------------------------------------------------------------
void TBuzzer::Takt()
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

//-----------------------------------------------------------------------
// Beep - initiate sounding of beeps.
//-----------------------------------------------------------------------
// duration = time length for sound and silence (in ms)
//-----------------------------------------------------------------------
void TBuzzer::Beep(int duration, int number_of_beeps)
   {
      BeepState      = 2 * number_of_beeps-1;
      BeepDuration   = duration;
      BeepCountDown  = BeepDuration;
   }

//-----------------------------------------------------------------------
// TBuzzer - sound nr of beeps and wait for it.
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
void TBuzzer::BeepWait(int duration, int number_of_beeps)
   {
      Beep(duration, number_of_beeps);
      Wait();
   }

//-----------------------------------------------------------------------
// TBuzzer::Wait - wait for beeps to finish
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
void TBuzzer::Wait()
   {
      for(;BeepState;) { /* wait for BeepState to become 0*/ }
   }

#endif
