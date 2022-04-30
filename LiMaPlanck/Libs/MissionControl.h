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