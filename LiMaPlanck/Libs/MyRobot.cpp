//-----------------------------------------------------------------------------
// MyRobot.cpp
//-----------------------------------------------------------------------------
// The actual code of lots of (smaller) framework modules.
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// TState::TState -
//-----------------------------------------------------------------------------
// Small class to store the state of a Finite State Machine and do some
// housekeeping.
//-----------------------------------------------------------------------------
TState::TState()
   {
      Reset();
   }

//-----------------------------------------------------------------------------
//  TState::Reset - Back to state 0
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TState::Reset()
   {
      State  = 0;
      PrevState = -1;
   }

//-----------------------------------------------------------------------------
// TState::Update - housekeeping, call this at the beginning of each takt.
//-----------------------------------------------------------------------------
// * detect changes in State (from previous call)
// * report state changes when Verbose is true
// * update NewState
//-----------------------------------------------------------------------------
void TState::Update(const char *InName, bool Verbose)
   {
      NewState = false;
      if (PrevState != State) {
         if (Verbose) CSerial.printf("%s state %d -> %d\n", InName, PrevState, State);

         PrevState      = State;
         NewState       = true;
         StateStartTime = millis();
      }
   }

//-----------------------------------------------------------------------------
// TState::InvalidState - Report invalid state
//-----------------------------------------------------------------------------
// Returns true, so the error can be handled with a single line statement:
//
//    default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
//
//-----------------------------------------------------------------------------
bool TState::InvalidState(const char *FName)
   {
      CSerial.printf("Error: invalied state in %s (%d)\n", FName, State);
      return true;
   }

//-----------------------------------------------------------------------------
// TState::StateTime - return # of ms since we entered this state.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int TState::StateTime()
   {
      return millis() - StateStartTime;
   }

//-----------------------------------------------------------------------
// Construct - store pin & set pin to output
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
// TBuzzer::Wai - wait for beeps to finish
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
void TBuzzer::Wait()
   {
      for(;BeepState;) { /* wait for BeepState to become 0*/ }
   }

//-----------------------------------------------------------------------------
// ServoSlope - Slope that operates on Servo class
//-----------------------------------------------------------------------------
// Returns: done (true when setpoint is reached)
//-----------------------------------------------------------------------------
bool ServoSlope(Servo &S, int Setpoint, int Step)
{
   int Current;
   if (Setpoint > 200) {
      // Value in micros
      Current = S.readMicroseconds();
   } else {
      // Value in degrees
      Current = S.read();
   }
   Slope(Current, Setpoint, Step);  // this updates Current
   S.write(Current);                // supports both degrees and micros
   return (Current == Setpoint);
}


//-----------------------------------------------------------------------------
// _write - link printf to CSerial
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
extern "C" int _write(int file, char *ptr, int len)
{
    int i;
    file = file;
    len  = len;
    for (i = 0; *ptr; i++)
    {
        CSerial.print(*ptr++);
    }
    return i;
}