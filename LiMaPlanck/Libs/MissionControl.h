//-----------------------------------------------------------------------------
// MissionControl.h  - Handle missions
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
class TMissionControl
{
   public:
      TMissionControl();
      void Start(bool (*InMission)(TState &S));

      void Reset();
      bool IsActive() { return CurrentMission != NULL; }
      bool Takt();

      TState S;  // Mission statemachine

   private:
      bool (*CurrentMission)(TState &S);
};

extern TMissionControl MissionControl;

#ifdef FRAMEWORK_CODE


//-----------------------------------------------------------------------------
// TMissionControl - Constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
TMissionControl::TMissionControl()
   {
      CurrentMission = NULL;
   }

//-----------------------------------------------------------------------------
// TMissionControl::Reset - Abort mission (if running)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TMissionControl::Reset()
   {
      if (IsActive()) {
         printf("MissionControl.Reset: Abort mission.\n");
         CurrentMission = NULL;
      } else {
         printf("MissionControl.Reset: No mission active.\n");
      }
   }

//-----------------------------------------------------------------------------
// TMissionControl::Start - Start new mission
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TMissionControl::Start(bool (*InMission)(TState &S))
   {
      if (IsActive()) {
         printf("MissionControl.Start ERROR: a mission is already running.\n");
         return;
      }
      printf("MissionControl: Mission start.\n");
      S.Reset();  // reset mission statemachine
      CurrentMission = InMission;
   }

//-----------------------------------------------------------------------------
// TMissionControl::Takt -
//-----------------------------------------------------------------------------
// return true when done.
//-----------------------------------------------------------------------------
bool TMissionControl::Takt()
   {
      if (!IsActive()) return true;

      bool r = CurrentMission(S);
      if (r) {
         printf("MissionControl: Mission done.\n");
         CurrentMission = NULL;
      }
      return r;
   }

#endif
