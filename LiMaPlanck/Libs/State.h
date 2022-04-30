//-----------------------------------------------------------------------------
// State.h  - support for Finit State Machines.
//-----------------------------------------------------------------------------
// Note: #define FRAMEWORK_CODE enables acutual code, in adition to prototypes
//-----------------------------------------------------------------------------

class TState
{
   public:
      TState();
      void Reset();

      void Update(const char *FName, bool Verbose=true);
      bool Done(const char *FName);
      bool InvalidState(const char *FName);
      int StateTime();

      int  State;
      bool NewState;

      int  Param1;   // user param
      int  Param2;   // user param

   private:
      int PrevState;
      int StateStartTime;
};

#ifdef FRAMEWORK_CODE

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
void TState::Update(const char *FName, bool Verbose)
   {
      int Now = millis();
      if (Now == StateStartTime) printf("Waarschuwing: S.Update wordt waarschijnlijk dubbel aangeroepen! (%s)\n", FName);
      NewState = false;
      if (PrevState != State) {
         if (Verbose) printf("%s state %d -> %d\n", FName, PrevState, State);

         PrevState      = State;
         NewState       = true;
         StateStartTime = Now;
      }
   }

//-----------------------------------------------------------------------------
// TState::Done - Report invalid state
//-----------------------------------------------------------------------------
// Returns true, so the error can be handled with a single line statement:
//
//    return S.Done(__FUNCTION__);   // Report done & end mission
//
//-----------------------------------------------------------------------------
bool TState::Done(const char *FName)
   {
      printf("%s state %d -> Done\n", FName, State);
      return true;
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
      printf("Error: invalid state in %s (%d)\n", FName, State);
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

#endif
