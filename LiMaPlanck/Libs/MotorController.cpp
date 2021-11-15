//-----------------------------------------------------------------------------
// MotorController.cpp - the speed-controller of the robot.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// MotorController (Mc) data structure with PID var's
struct tMc
{
   int PrevSpeedL, PrevSpeedR; // last speed (in mm/sec)
   int IErrorL, IErrorR;       // integraded error (I-component of PID)
} Mc;

//-----------------------------------------------------------------------------
// SpeedController - control motorspeed
//-----------------------------------------------------------------------------
// Parameters: SpeedSoll* (in mm/sec)
//-----------------------------------------------------------------------------
void MotorController(int SetpointL, int SetpointR)
{
   int PError = 999;
   int DError = 999;
   int PowerL, PowerR;

   // convert setpoints from mm/s to ticks/interval
   volatile int SetpointL_Ticks = (SetpointL * 4L * MAIN_TAKT_INTERVAL) / ODO_TICK_TO_METRIC;
   volatile int SetpointR_Ticks = (SetpointR * 4L * MAIN_TAKT_INTERVAL) / ODO_TICK_TO_METRIC;

  //printf("!! %d %d %d %d\n", SetpointL, SetpointL_Ticks, MAIN_TAKT_INTERVAL, ODO_TICK_TO_METRIC);

   //----------------
   // Left controller
   //----------------
   if (SetpointL_Ticks != 0) {
      //  PID motor speed control:
      PError = SetpointL_Ticks - Position.ActSpeedL;           // P error, in odo-pulsen per takt-inteval
      DError = Position.ActSpeedL - Mc.PrevSpeedL;             // D error
      Mc.PrevSpeedL = Position.ActSpeedL;
      Mc.IErrorL += PError;                                    // I error

      if (Mc.IErrorL >  MPID_I_MAX) Mc.IErrorL =  MPID_I_MAX;  //anti windup
      if (Mc.IErrorL < -MPID_I_MAX) Mc.IErrorL = -MPID_I_MAX;

      // Calculate PWM
      PowerL = MPID_OFFSET       +                             // PID controller
               SetpointL_Ticks   * MPID_SP_GAIN +
               PError            * MPID_P_GAIN  +
               Mc.IErrorL        * MPID_I_GAIN  +
               DError            * MPID_D_GAIN  ;
   } else {
      Mc.IErrorL = 0;
      PowerL     = 0;
   }

   //-----------------
   // Right controller
   //-----------------
   if (SetpointR_Ticks != 0) {
      //  PID motor speed control:
      PError = SetpointR_Ticks - Position.ActSpeedR;           // P error, in odo-pulses per takt-inteval
      DError = Position.ActSpeedR - Mc.PrevSpeedR;             // D error
      Mc.PrevSpeedR = Position.ActSpeedR;
      Mc.IErrorR += PError;                                    // I error

      if (Mc.IErrorR >  MPID_I_MAX) Mc.IErrorR =  MPID_I_MAX;  //anti windup
      if (Mc.IErrorR < -MPID_I_MAX) Mc.IErrorR = -MPID_I_MAX;

      // Calculate PWM
      PowerR = MPID_OFFSET       +                             // PID controller
               SetpointR_Ticks   * MPID_SP_GAIN +
               PError            * MPID_P_GAIN  +
               Mc.IErrorR        * MPID_I_GAIN  +
               DError            * MPID_D_GAIN  ;
   } else {
      Mc.IErrorR = 0;
      PowerR     = 0;
   }

   Motors(PowerL, PowerR);
//   printf("PID Soll: %d / %d, Ist: %d / %d, I: %d / %d, Power: %d / %d\n",
//         SetpointL_Ticks, SetpointR_Ticks, Position.ActSpeedL, Position.ActSpeedR, Mc.IErrorL, Mc.IErrorR, PowerL, PowerR);
}

