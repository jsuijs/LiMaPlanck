//-----------------------------------------------------------------------------
// MotorController.cpp - the speed-controller of the robot.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

class TMc   // local MotorControl class.
{
   public:
      void Compute(int Setpoint, int ActSpeed)
      {
         int Setpoint_Ticks = (Setpoint * (long) MAIN_TAKT_INTERVAL) / (F_ODO_TICK_TO_METRIC * 1024);

         if (Setpoint_Ticks == 0) {
            // Stop
            IError = 0;
            Power  = 0;
            return;
         }

         //  Moving: PID motor speed control
         int PError = Setpoint_Ticks - ActSpeed;         // P error, in odo-pulsen per takt-inteval
         int DError = ActSpeed - PrevSpeed;              // D error
         PrevSpeed  = ActSpeed;
         IError += PError;                               // I error

         IError = Clip(IError, -MPID_I_MAX, MPID_I_MAX); // Integrator anti windup

         // Calculate PWM using PID + proportional forward
         Power =  MPID_OFFSET       +                    // forward offset
                  Setpoint_Ticks    * MPID_SP_GAIN +     // forward gain (proportional)
                  PError            * MPID_P_GAIN  +     // P
                  IError            * MPID_I_GAIN  +     // I
                  DError            * MPID_D_GAIN  ;     // D
      }
      int Power;                                         // output variable

   private:
      int PrevSpeed; // last speed (in mm/sec)
      int IError;    // integraded error (I-component of PID)
};

TMc McLeft, McRight;

//-----------------------------------------------------------------------------
// SpeedController - control motorspeed
//-----------------------------------------------------------------------------
// Parameters: TargetSpeed (in mm/sec)
//-----------------------------------------------------------------------------
void MotorController(int SetpointL, int SetpointR)
{
   McLeft.Compute( SetpointL, Position.ActSpeedL);
   McRight.Compute(SetpointR, Position.ActSpeedR);

   Motors(McLeft.Power, McRight.Power);
}
