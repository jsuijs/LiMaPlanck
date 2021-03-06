//-----------------------------------------------------------------------------
// MyRobot.cpp
//-----------------------------------------------------------------------------
// The actual code of (smaller) framework modules.
//-----------------------------------------------------------------------------

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
// MissionTakt - drive the program (mission)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void MissionTakt()
{
   if (!MissionControl.IsDone()) {
      // Execute mission
      if (MissionControl.Takt()) {
         // Mission done.
         Driver.Pwm(0, 0); // Stop motors (and only once, so CLI-commands can be used)
      }
   }
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
    for (i = 0; (i<len) && (*ptr); i++)
    {
        CSerial.print(*ptr++);
    }
    return i;
}