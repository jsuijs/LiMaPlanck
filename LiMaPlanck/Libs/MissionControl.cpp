//-----------------------------------------------------------------------------
// MissionControl.cpp  - Handle missions
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// TMissionControl - constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
TMissionControl::TMissionControl()
   {
      CurrentMission = NULL;
   }

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TMissionControl::Reset()
   {
      if (CurrentMission == NULL) {
         printf("MissionControl.Reset: No mission active.\n");
      } else {
         printf("MissionControl.Reset: Abort mission.\n");
         CurrentMission = NULL;
      }
   }

//-----------------------------------------------------------------------------
// TMissionControl::Start - Start new mission
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TMissionControl::Start(bool (*InMission)(TState &S))
   {
      if (CurrentMission != NULL) {
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
      if (CurrentMission == NULL) return true;

      bool r = CurrentMission(S);
      if (r) {
         printf("MissionControl: Mission done.\n");
         CurrentMission = NULL;
      }
      return r;
   }
