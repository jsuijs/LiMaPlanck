//-----------------------------------------------------------------------------
// MissionAutoWSCal.cpp - Automatic (multiple) WheelSizeCall measurements
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"

const int S_BACK = 0;
const int S_RIGHT = 7;

class TMeasure
{  public:

   void Start(int InNrSamples)
   {
      NrSamples = InNrSamples;
      CountDown = InNrSamples;

      RightDistSum   = 0;
      RightDegSum    = 0;
      BackDistSum    = 0;
      BackDegSum     = 0;

      LppRotationCount = Lpp.Status.RotationCount;
      printf("Measure: start %d samples on rotation %d\n", NrSamples, LppRotationCount);
   }

   bool Takt()
   {
      if (LppRotationCount == Lpp.Status.RotationCount) return false;   // wait for next sample

      LppRotationCount = Lpp.Status.RotationCount;

      //printf("%d Sample %d %d - %d %d\n",
      //      LppRotationCount,
      //      Lpp.Sensor[S_RIGHT].Distance, Lpp.Sensor[S_BACK].Distance,
      //      Lpp.Sensor[S_RIGHT].Degrees32, Lpp.Sensor[S_BACK].Degrees32);

      RightDistSum   += Lpp.Sensor[S_RIGHT].Distance;
      RightDegSum    += (Lpp.Sensor[S_RIGHT].Degrees32 +  90*32);   // map to front
      BackDistSum    += Lpp.Sensor[S_BACK].Distance;
      if (Lpp.Sensor[S_BACK].Degrees32 > 0) {
         BackDegSum     += (Lpp.Sensor[S_BACK].Degrees32 - 180*32);   // map to front
      } else {
         BackDegSum     += (Lpp.Sensor[S_BACK].Degrees32 + 180*32);   // map to front
      }

      CountDown--;
      if (CountDown > 0) return false; // not yet done

      printf("TMeasure result: %d %d %f\n", RightDistance(), BackDistance(), Heading());
      printf("TMeasure debug Right Deg: %f - Back Deg: %f\n",
            RightDegSum / (float) (NrSamples * 32), BackDegSum   / (float) (NrSamples * 32));

      return true;   // done
   }

   int RightDistance()
   {
      if (!NrSamples) return 0;
      return (RightDistSum + NrSamples/2) / NrSamples;
   }
   int BackDistance()
   {
      if (!NrSamples) return 0;
      return (BackDistSum  + NrSamples/2) / NrSamples;
   }
   float Heading()
   {
      if (!NrSamples) return 0;
      return (RightDegSum + BackDegSum) / (float) (NrSamples * 32 * -2);
   }

   private :
      int NrSamples, CountDown, LppRotationCount;
      int RightDistSum, RightDegSum, BackDistSum, BackDegSum;

};

TMeasure Measure;

//-----------------------------------------------------------------------------
//  MissionAutoWSCal
//-----------------------------------------------------------------------------
bool MissionAutoWSCal(TState &S)
{  static int StartDistanceRight;
   static int RunCounter;

   S.Update(__FUNCTION__, Flags.IsSet(11));

   Lpp.ReadStatus(); // get RotationCount.

   switch (S.State) {

      case 0 : {     // LIDAR-START (if required)
         if (S.NewState) {
            Lpp.SensorSetup(S_BACK,   135, 90);
            Lpp.SensorSetup(S_RIGHT, -135, 90);

            if (Lpp.Status.RotationTime != 0) { // is lidar running?
               S.State += 10; // goto next state without waiting
            }
            Lpp.Start();      // issue start command in any case (lidar might be running while LppMaster not in state 2...)
         }

         if (S.StateTime() > 3000) { // Wacht op start lidar
            RunCounter = 0;
            S.State += 10;
         }
      }
      break;

      case 10 : {       // Measure & set position
         if (S.NewState) {
            printf("AutoWSCal run %d\n", RunCounter);
            Measure.Start(20);
         }

         if (Measure.Takt()) {
            // measurement ready
            Position.Set(Measure.BackDistance(), Measure.RightDistance(), Measure.Heading());
            S.State += 10;
         }
      }
      break;

      case 20 : {       // A bit forward...
         if (S.NewState) Driver.XY(500, 300, 200, 0);

         if (Driver.IsDone()) S.State += 10;
      }
      break;

      case 30 : {       // Back...
         if (S.NewState) Driver.XY(300, 300, -200, 0);

         if (Driver.IsDone()) S.State += 10;
      }
      break;

      case 40 : {       // And turn straight - to get into at the desired starting-point
         if (S.NewState) Driver.RotateHeading(0);

         if (Driver.IsDone()) S.State += 10;
      }
      break;

      case 50 : {       // Measure right distance before test
         if (S.NewState) Measure.Start(10);

         if (Measure.Takt()) {
            // measurement ready
            StartDistanceRight = Measure.RightDistance();
            S.State += 10;
         }
      }
      break;

      case 60 : {       // Do the test
         if (S.NewState) {
            SubS.Param1 = 2000; // distance to drive
            SubS.Param2 = 1;    // set CW
            SubS.Reset();       // Start mission at the beginning
         }

         if (MissionWheelSizeCalibrate(SubS)) S.State += 10;
      }
      break;

      case 70 : {       // Turn straight
         if (S.NewState) Driver.RotateHeading(0);

         if (Driver.IsDone()) S.State += 10;
      }
      break;

      case 80 : {       // Measure right distance after test
         if (S.NewState) Measure.Start(10);

         if (Measure.Takt()) {
            // measurement ready
            printf("WSCal-result Pre: %d, Post: %d, Y: %d\n",
                  StartDistanceRight, Measure.RightDistance(), Position.YPos);

            // Check if we're done or need another run.
            RunCounter --;
            if (RunCounter < 20) {
               S.State = 10;
            } else {
               return true;
            }
         }
      }
      break;

      default : return S.InvalidState(__FUNCTION__);  // Report invalid state & end mission
   }
   return false;                                       // Nog niet klaar.
}
