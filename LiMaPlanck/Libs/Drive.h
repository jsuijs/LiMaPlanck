//-----------------------------------------------------------------------------
// Drive.h
//-----------------------------------------------------------------------------
//
// Per 'bewegingstype' heeft in deze class 2 methodes (functies). Neem
// bijvoorbeeld de beweging 'SpeedLR' (M_SPEED_LR), waarme je de gewenste
// snelheid van de linkse en rechtse motor kunt aangeven.
// De eerste methode is
//
//    void SpeedLR(int SpeedL, int SpeedR);
//
// Een aanroep naar deze public methode wordt een eventuele actieve beweging
// afgebroken en deze beweging wordt gestart. Of eigenlijk: alle parameters
// worden ingesteld voor deze beweging.
// De beweging zelf wordt uitgevoerd door regelmatig de methode Takt()
// aan te roepen. Deze kiest, op basis van de ingestelde parameters, de
// juiste uitvoeringsroutine. In dit geval:
//
//    void SpeedLRTakt(bool NewMovement, int SpeedL, int SpeedR, int MaxSlopeP);
//
// Dit is een private methode, die je dus niet zelf aanroept. Dat doet de Takt()
// methode. Maar... hier gebeurt het echte werk, op basis van de parameters die
// je via SpeedLR() hebt ingesteld.
//
// Tot slot de methode
//
//    bool IsDone();
//
// Deze methode geeft aan als een beweging is afgerond. Bijvoorbeeld bij de
// beweging 'XY', die de robot naar een bepaald punt laat rijden, geeft deze
// methode 'true' terug zodra dit punt bereikt is.
// Een beweging als 'SpeedLR' kent geen eindpunt. De methode IsDone() geeft
// dan nooit 'true' terug. In dit geval bepaal je op een andere manier wanneer
// je een volgende beweging start.
//-----------------------------------------------------------------------------

class TDrive
{
   enum TDiveMode { UNDEFINED, M_PWM, M_SPEED_LR, M_SPEED_ROTATION, M_SPEED_HEADING, M_XY, M_ROTATE, M_ARC, M_STOP };

   public:
      TDrive();
      void init();
      void Takt();
      bool IsDone();

      // bewegingen
      void Pwm(int PwmL, int PwmR);
      void SpeedLR(int SpeedL, int SpeedR);
      void SpeedRotation(int Speed, int Rotation_q8);
      void SpeedHeading(int Speed, int Heading);

      void Rotate(int Degrees, int RotateClip = ROTATE_CLIP_Q8);
      void RotateHeading(int Heading, int RotateClip = ROTATE_CLIP_Q8);
      void RotatePoint(int X, int Y, int RotateClip = ROTATE_CLIP_Q8);

      void Arc(int Degrees, int Radius, int Speed, int EndSpeed);
      void ArcHeading(int Heading, int Radius, int Speed, int EndSpeed);

      void XY(int X, int Y, int Speed, int EndSpeed);

      void Stop();

   private:
      TDiveMode DriveMode;    // actief type aansturing
      int Param1;             // Paramers van actieve aansturing type
      int Param2;
      int Param3;
      int Param4;

      bool IsDoneFlag;        // Movement is gereed
      bool NewMovement;

      int SollSpeedL, SollSpeedR; // Snelheid (in mm/sec) die we nastreven, verandering begrensd door MaxSlope

      int MaxSlope;

      void SpeedLRTakt(bool FirstCall, int SpeedL, int SpeedR, int MaxSlopeP);
      bool SpeedRotationTakt(bool FirstCall, int InSpeed, int InRotation_q8);
      bool SpeedHeadingTakt(bool FirstCall, int InSpeed, int InHeading);
      bool XYTakt(bool FirstCall, int TargetX, int TargetY, int Speed, int EndSpeed);
      bool RotateRelTakt(bool FirstCall, int DeltaDegrees, int RotateClip_q8);
      bool ArcRelTakt(bool FirstCall, int DeltaDegrees, int Radius, int Speed, int EndSpeed);
      bool StopTakt(bool FirstCall);
};

extern TDrive Driver;

#ifdef FRAMEWORK_CODE

//-----------------------------------------------------------------------------
// TDrive - constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
TDrive::TDrive()
   {
      // call init when CSerial is up.
   }

//-----------------------------------------------------------------------------
// TDrive::init - manual called constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TDrive::init()
   {
      // reset : PWM mode, output 0.
      Pwm(0, 0);
      MaxSlope = MAX_SLOPE;
   }

//-----------------------------------------------------------------------------
// Takt -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TDrive::Takt()
   {  static bool PrevIsDone = false;
      bool FirstCall = false;

      if (NewMovement) {
         // gewijzigde drive mode => diverse init's
         if (Flags.IsSet(1)) printf("Drive - new movement\n");
         FirstCall = true;

         SollSpeedL = ACT_SPEED_MM_SEC(Position.ActSpeedL);
         SollSpeedR = ACT_SPEED_MM_SEC(Position.ActSpeedR);
         NewMovement = false;
         IsDoneFlag  = false;
      }

      if (IsDoneFlag) {
         // Done en geen nieuwe beweging
         if (PrevIsDone == false) Motors(0,0);  // call once
         PrevIsDone = true;
         return;
      }
      PrevIsDone = false;

      //printf("Drive.Takt %d %d %d\n", DriveMode, Param1, Param2);
      switch(DriveMode) {
         case M_PWM : {
            Motors(Param1, Param2);
            break;
         }
         case M_SPEED_LR : {
            SpeedLRTakt(FirstCall, Param1, Param2, MaxSlope);
            break;
         }
         case M_SPEED_ROTATION : {
            SpeedRotationTakt(FirstCall, Param1, Param2);
            break;
         }
         case M_SPEED_HEADING : {
            SpeedHeadingTakt(FirstCall, Param1, Param2);
            break;
         }
         case M_XY : {
            IsDoneFlag = XYTakt(FirstCall, Param1, Param2, Param3, Param4);   // New, X, Y, Speed, EndSpeed
            break;
         }
         case M_ROTATE : {
            IsDoneFlag = RotateRelTakt(FirstCall, Param1, Param2);
            break;
         }
         case M_ARC : {
            IsDoneFlag = ArcRelTakt(FirstCall, Param1, Param2, Param3, Param4);
            break;
         }
         case M_STOP : {
            IsDoneFlag = StopTakt(FirstCall);
            break;
         }
         default :{
            printf("Drive.Takt(): onbekende drivemode (%d)\n", DriveMode);
            TDrive();   // reset class
            break;
         }
      }
   }

//-----------------------------------------------------------------------------
// IsDone - return true als beweging klaar is.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool TDrive::IsDone()
   {
      return IsDoneFlag;
   }

//-----------------------------------------------------------------------------
// TDrive::Pwm - rij met gegeven pwm waarden (L, R)
//-----------------------------------------------------------------------------
// Pwm* is pwm waarde + rijrichting, range 255...-255
//
// Updatable:  Yes
// Indefinite: Yes
//-----------------------------------------------------------------------------
void TDrive::Pwm(int PwmL, int PwmR)
   {
      if (DriveMode != M_PWM) { // Updatable
         NewMovement = true;
         IsDoneFlag = false;
      }

      DriveMode = M_PWM;
      Param1 = PwmL;
      Param2 = PwmR;
   }

//-----------------------------------------------------------------------------
// SpeedLR - rij met gegeven snelheid (L, R)
//-----------------------------------------------------------------------------
//
// Updatable:  Yes
// Indefinite: Yes
//-----------------------------------------------------------------------------
void TDrive::SpeedLR(int SpeedL, int SpeedR)
   {
      if (DriveMode != M_SPEED_LR) { // Updatable
         NewMovement = true;
         IsDoneFlag = false;
      }

      DriveMode = M_SPEED_LR;
      Param1 = SpeedL;
      Param2 = SpeedR;
   }

//-----------------------------------------------------------------------------
// SpeedRotation - rij met gegeven snelheid - voorwaards & rotatie
//-----------------------------------------------------------------------------
//
// Updatable:  Yes
// Indefinite: Yes
//-----------------------------------------------------------------------------
void TDrive::SpeedRotation(int Speed, int Rotation_q8)
   {
      if (DriveMode != M_SPEED_ROTATION) { // Updatable
         NewMovement = true;
         IsDoneFlag = false;
      }

      // Limit Rotation value to 50% of max rotation allowed by heading control loop
      const int RotLimit_q8 = ROTATE_CLIP_Q8 * MAIN_TAKT_RATE / 2;
      bool Limit = false;
      if (Rotation_q8 > RotLimit_q8) {
         Rotation_q8 = RotLimit_q8;
         Limit = true;
      }
      if (Rotation_q8 < -RotLimit_q8) {
         Rotation_q8 = -RotLimit_q8;
         Limit = true;
      }
      if (Limit) printf("Drive.SpeedRotation warning: Rotation limited to %d\n", Rotation_q8);

      DriveMode = M_SPEED_ROTATION;
      Param1 = Speed;
      Param2 = Rotation_q8;
   }

//-----------------------------------------------------------------------------
// SpeedHeading - rij met gegeven snelheid in gegeven richting
//-----------------------------------------------------------------------------
//
// Updatable:  Yes
// Indefinite: Yes
//-----------------------------------------------------------------------------
void TDrive::SpeedHeading(int Speed, int Heading)
   {
      if (DriveMode != M_SPEED_HEADING) { // Updatable
         NewMovement = true;
         IsDoneFlag = false;
      }

      DriveMode = M_SPEED_HEADING;
      Param1 = Speed;
      Param2 = Heading;
   }

//-----------------------------------------------------------------------------
// XY - Rij naar gegeven punt
//-----------------------------------------------------------------------------
//
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::XY(int X, int Y, int Speed, int EndSpeed)
   {
      if (Flags.IsSet(1)) printf("Drive.XY  x: %d, y: %d, Speed: %d, EndSpeed: %d\n",
            X, Y, Speed, EndSpeed);

      NewMovement = true;
      IsDoneFlag = false;

      DriveMode = M_XY;
      Param1 = X;
      Param2 = Y;
      Param3 = Speed;
      Param4 = EndSpeed;
   }

//-----------------------------------------------------------------------------
// Rotate - draai graden (relatief).
//-----------------------------------------------------------------------------
// Draai de stilstaande robot naar opgegeven richting.
// - 'Degrees' is het aantal graden, positief = tegen de klok in. Waarde mag
//    groter zijn dan +/- 360 graden, de robot draait dan meer dan een hele ronde.
//
// Gebruikte constantes: ROTATE_P_GAIN, ROTATE_D_GAIN, ROTATE_CLIP
//
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::Rotate(int Degrees, int RotateClip)
   {
      if (Flags.IsSet(1)) printf("Drive.Rotate Degrees: %d, Clip: %d\n", Degrees, RotateClip);

      NewMovement = true;
      IsDoneFlag = false;

      DriveMode = M_ROTATE;
      Param1 = Degrees;    // Aantal te draaien graden (relatief).
      Param2 = RotateClip; // Draaisnelheid (degrees * 16 / tick)
   }

//-----------------------------------------------------------------------------
// RotateHeading - draai naar absolute heading (graden).
//-----------------------------------------------------------------------------
// Draai de stilstaande robot naar opgegeven richting.
// - 'Heading' is de absolute hoek, in graden.
//
// Gebruikte constantes: ROTATE_P_GAIN, ROTATE_D_GAIN, ROTATE_CLIP
//
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::RotateHeading(int Heading, int RotateClip)
   {
      if (Flags.IsSet(1)) printf("Drive.RotateHeading Heading: %d, Clip: %d\n", Heading, RotateClip);

      NewMovement = true;
      IsDoneFlag = false;

      DriveMode = M_ROTATE;
      Param1 = NormHoek(Heading - Position.Hoek, 360); // Aantal te draaien graden (relatief).
      Param2 = RotateClip; // Draaisnelheid (degrees * 16 / tick)
   }

//-----------------------------------------------------------------------------
// RotatePoint - draai naar punt (X,Y).
//-----------------------------------------------------------------------------
// Draai de stilstaande robot naar opgegeven punt.
//
// Gebruikte constantes: ROTATE_P_GAIN, ROTATE_D_GAIN, ROTATE_CLIP
//
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::RotatePoint(int TargetX, int TargetY, int RotateClip)
   {
      if (Flags.IsSet(1)) printf("Drive.RotatePoint X:%d, Y: %d, Clip: %d\n", TargetX, TargetY, RotateClip);

      // Reken Heading uit van huidige positie naar opgegeven punt.
      long TargetHeading;
      int  TargetDistance;

      Cartesian2Polar(TargetHeading, TargetDistance, TargetX - Position.XPos, TargetY - Position.YPos);
      if (Flags.IsSet(6)) printf ("RotatePoint TargetVector %d mm, %d graden (%d %d %d %d))\n", TargetDistance, (int)(TargetHeading/256),
            TargetX, Position.XPos, TargetY, Position.YPos);

      RotateHeading(TargetHeading / 256, RotateClip);
   }

//-----------------------------------------------------------------------------
// Stop - Slow down & stop using speed curve.
//-----------------------------------------------------------------------------
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::Stop()
   {
      if (Flags.IsSet(1)) printf("Drive.Stop\n");

      NewMovement = true;
      IsDoneFlag = false;

      DriveMode = M_STOP;
   }

//-----------------------------------------------------------------------------
// Arc - Rij boog van Degrees (graden).
//-----------------------------------------------------------------------------
// Rij een boog tot de opgegeven richting.
// - 'Degrees' is de relatieve hoek, in graden.
// - Radius is straal van de draaicirkel in mm
//
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::Arc(int Degrees, int Radius, int Speed, int EndSpeed)
   {
      if (Flags.IsSet(1)) printf("Drive.Arc Degrees: %d, Radius: %d, Speed: %d, EndSpeed: %d\n",
            Degrees, Radius, Speed, EndSpeed);

      NewMovement = true;
      IsDoneFlag = false;

      DriveMode = M_ARC;
      Param1 = Degrees;
      Param2 = Radius;
      Param3 = Speed;
      Param4 = EndSpeed;
   }

//-----------------------------------------------------------------------------
// ArcHeading - Rij boog naar heading (graden).
//-----------------------------------------------------------------------------
// Rij een boog tot de opgegeven richting.
// - 'Heading' is de absolute hoek, in graden.
// - Radius is straal van de draaicirkel in mm
//
// Updatable:  No
// Indefinite: No
//-----------------------------------------------------------------------------
void TDrive::ArcHeading(int Heading, int Radius, int Speed, int EndSpeed)
   {
      if (Flags.IsSet(1)) printf("Drive.ArcHeading Heading: %d, Radius: %d, Speed: %d, EndSpeed: %d\n",
            Heading, Radius, Speed, EndSpeed);

      NewMovement = true;
      IsDoneFlag = false;

      DriveMode = M_ARC;
      Param1 = NormHoek(Heading - Position.Hoek, 360);   // DeltaDegrees
      Param2 = Radius;
      Param3 = Speed;
      Param4 = EndSpeed;
   }

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Private functions
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SpeedLRTakt -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TDrive::SpeedLRTakt(bool FirstCall, int SpeedL, int SpeedR, int MaxSlopeP)
   {
      if (FirstCall) {
         if (Flags.IsSet(1)) printf("SpeedLRTakt FirstCall %d %d %d\n", SpeedL, SpeedR, MaxSlopeP);
      } else {
         if (Flags.IsSet(2)) printf("SpeedLRTakt %d %d %d\n", SpeedL, SpeedR, MaxSlopeP);
      }

      // Update SpeedSetpoints (SollSpeed*)
      Slope(SollSpeedL, SpeedL, MaxSlopeP);
      Slope(SollSpeedR, SpeedR, MaxSlopeP);

      MotorController(SollSpeedL, SollSpeedR);
   }

//-----------------------------------------------------------------------------
// RotateRelTakt -
//-----------------------------------------------------------------------------
// Parameter: DeltaDegrees - gewenste aantal graden draaien
// return: true when done
//-----------------------------------------------------------------------------
bool TDrive::RotateRelTakt(bool FirstCall, int DeltaDegrees, int RotateClip_q8)
   {  static int RestHoek_q8, PrevRestHoek_q8;
      static int VorigeHoek_q8;
      static char StilStand;

      if (FirstCall) {
         RestHoek_q8    = DeltaDegrees * 256;      // doel
         VorigeHoek_q8  = Position.HoekHires(); // start
         StilStand      = 0;                    // tbv eind-detectie
      }

      int DezeHoek_q8   = Position.HoekHires();
      int Delta_q8      = NormHoek(DezeHoek_q8 - VorigeHoek_q8, (360*256));
      RestHoek_q8      -= Delta_q8;
      VorigeHoek_q8     = DezeHoek_q8;

      int Clipped_q8 = Clip(RestHoek_q8, RotateClip_q8, -RotateClip_q8);
      int SpeedR = Clipped_q8 * (float) ROTATE_P_GAIN + (RestHoek_q8 - PrevRestHoek_q8) * (float) ROTATE_D_GAIN;
      int SpeedL = -SpeedR;

      if (FirstCall) {
         if (Flags.IsSet(1)) {
            printf("RotateTakt FirstCall InDegrees: %d DezeHoek: %d, RestHoek: %d, SpeedL: %d, Clipped: %d, Delta: %d\n",
                  DeltaDegrees, DezeHoek_q8/256, RestHoek_q8/256, SpeedL, Clipped_q8/256, Delta_q8/256);
         }
      } else {
         if (Flags.IsSet(3)) {
            printf("RotateTakt InDegrees: %d DezeHoek: %d, RestHoek: %d, SpeedL: %d, Clipped: %d, Delta: %d\n",
                  DeltaDegrees, DezeHoek_q8/256, RestHoek_q8/256, SpeedL, Clipped_q8/256, Delta_q8/256);
         }
      }

      if (ABS(RestHoek_q8) > ABS(Clipped_q8)) {
         // clipped
         SpeedLRTakt(FirstCall, SpeedL, SpeedR, MAX_SLOPE);
      } else {
         // Niet geclipped, dus we zijn er blijkbaar bijna => vertraging in SpeedLR uitschakelen.
         SpeedLRTakt(FirstCall, SpeedL, SpeedR, MAX_SLOPE * 99);
      }

      if (RestHoek_q8 == PrevRestHoek_q8) {
         // stilstand
         StilStand ++;
         if (StilStand > 10) {
            return true; // klaar als we 10 ticks niet bewogen hebben.
         }
      } else {
         StilStand = 0;
      }

      PrevRestHoek_q8 = RestHoek_q8;
      return false; // not done yet
   }

//-----------------------------------------------------------------------------
// SpeedRotationTakt -
//-----------------------------------------------------------------------------
// InSpeed in mm/sec
// InRotation in 256*graden/sec
//
// De snelheidswijziging verloopt via een slope
//-----------------------------------------------------------------------------
bool TDrive::SpeedRotationTakt(bool FirstCall, int InSpeed, int InRotation_q8)
   {  static int HeadingSp_q8 = 0;

      // slope input speed
      if (FirstCall) {
         HeadingSp_q8 = Position.HoekHires();
         if (Flags.IsSet(1)) printf("SpeedRotationTakt FirstCall HeadingSp: %d, InSpeedSp: %d, InRotation: %d\n", HeadingSp_q8/256, InSpeed, InRotation_q8);
      } else {
         if (Flags.IsSet(7)) printf("SpeedRotationTakt HeadingSp: %d, InSpeedSp: %d, InRotation: %d\n", HeadingSp_q8/256, InSpeed, InRotation_q8);
      }

      HeadingSp_q8 += InRotation_q8 / MAIN_TAKT_RATE;
      HeadingSp_q8 = NormHoek(HeadingSp_q8, NORM_Q8);
      SpeedHeadingTakt(FirstCall, InSpeed, HeadingSp_q8 / 256);

      return false; // never done
   }

//-----------------------------------------------------------------------------
// SpeedHeadingTakt -
//-----------------------------------------------------------------------------
// InSpeed in mm/sec
// InHeading in graden
//
// De snelheidswijziging verloopt via een slope
//-----------------------------------------------------------------------------
bool TDrive::SpeedHeadingTakt(bool FirstCall, int InSpeed, int InHeading)
   {  int SetSpeedL, SetSpeedR;
      static int SpeedSp = 0;

      // slope input speed
      if (FirstCall) {
         int l = ACT_SPEED_MM_SEC(Position.ActSpeedL);
         int r = ACT_SPEED_MM_SEC(Position.ActSpeedR);
         SpeedSp = (l+r)/2;
         if (Flags.IsSet(1)) printf("SpeedHeadingTakt FirstCall SpeedL: %d/%d SpeedR: %d/%d (mm/sec / raw), SpeedSp: %d, InHeading: %d\n", l, Position.ActSpeedL, r, Position.ActSpeedR, SpeedSp, InHeading);
      } else {
         Slope(SpeedSp, InSpeed, MAX_SLOPE);
         if (Flags.IsSet(4)) printf("SpeedHeadingTakt InSpeed: %d, SpeedSp: %d, InHeading: %d\n", InSpeed, SpeedSp, InHeading);
      }

      // Hoekfout bepalen (i.c.m. richting)
      long CurrentHoek = Position.HoekHires();
      long HoekError   = NormHoek(CurrentHoek - InHeading * 256L, (360L * 256));

      // Eigenlijk hoort SpeedSp niet (langere tijd) 0 te zijn.
      // Als deze toch 0 is en de hoekerror is klein, dan onderdrukt
      // de volgende regel het corrigeren van een kleine hoekfout.
      if ((SpeedSp == 0) && (ABS(HoekError) < 256)) {
         HoekError = 0;
      }

      // simpele P regelaar voor richting  (zie ook UmArcTakt)
      long Correctie = (HoekError * PID_Kp) / 4096;

      long ClippedCorrectie = Clip(Correctie, PID_OUT_CLIP * 256L, PID_OUT_CLIP * -256L);

      SetSpeedL = SpeedSp + (ClippedCorrectie * WIEL_BASIS) / (1024);
      SetSpeedR = SpeedSp - (ClippedCorrectie * WIEL_BASIS) / (1024);

      if (Flags.IsSet(2)) printf("%ld %ld %ld %ld %d %d\n",
            CurrentHoek, HoekError, Correctie, ClippedCorrectie, SetSpeedL, SetSpeedR);

      SpeedLRTakt(FirstCall, SetSpeedL, SetSpeedR, MAX_SLOPE);

      return false; // never done
   }

//-----------------------------------------------------------------------------
// XYTakt - Rij naar punt (X, Y)
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
bool TDrive::XYTakt(bool FirstCall, int TargetX, int TargetY, int Speed, int EndSpeed)
   {  static long SaveHeading; // rijrichting (voor de laatste paar cm)
      static int LastTargetDistance;
      static int State = 0;
      long TargetHeading;
      int  TargetDistance;

      if (FirstCall) {
         if (Flags.IsSet(1)) printf ("XYTakt x: %d, y: %d, Speed: %d, EndSpeed: %d\n", TargetX, TargetY, Speed, EndSpeed);
         State = 0;
      }

      // doel bepalen (steeds opnieuw).
      // hoek & afstand bepalen, resultaat in graden*256 (360/cirkel) en mm
      Cartesian2Polar(TargetHeading, TargetDistance, TargetX - Position.XPos, TargetY - Position.YPos);
      if (Flags.IsSet(6)) printf ("XYTakt TargetVector %d mm, %d graden (%d %d %d %d))\n", TargetDistance, (int)(TargetHeading/256),
            TargetX, Position.XPos, TargetY, Position.YPos);

      //   printf("run plaats: ( %d, %d ), doel: ( %d, %d ), TargetDistance: %d\n", RobotXPos()/10, RobotYPos()/10, TargetX, TargetY, TargetDistance);

      if (TargetDistance < 5) {
         if (Flags.IsSet(1)) printf("XYTakt done < 5mm\n");
         return true; // stap is afgerond als we minder dan 10mm afstand tot doel hebben
      }

      switch(State) {
         case 0: {   // eerste itteratie: wel afstand bepalen & bewaren, maar geen conclusie mogelijk.
            LastTargetDistance = TargetDistance;
            SaveHeading = TargetHeading;
            State ++;
            break;
         }
         case 1: {   // wacht op afstands-afname van meer dan 70 mm (nodig in geval we moeten draaien)
            if ((TargetDistance + 70) < LastTargetDistance) {
               LastTargetDistance = TargetDistance;
               State ++;
            }
            break;
         }
         case 2: {   // wacht tot we het doel bereikt hebben
            // afstands-toename van meer dan 2 cm
            if (TargetDistance > (LastTargetDistance + 20)) {
               if (Flags.IsSet(1)) printf("XYTakt done increment %d %d", TargetDistance, LastTargetDistance);

               return 1; // okay => stap is afgerond
            }
            // LastTargetDistance bevat kleinste afstand tot nu toe
            if (LastTargetDistance > TargetDistance) {
               LastTargetDistance = TargetDistance;
            }
            break;
         }
      }

      // op korte afstand, Heading gebruiken in plaats van echte richting.
      // (dit om te voorkomen dat de robot een slinger maakt vlak bij of voorbij het punt)
      long HeadingOut;
      if (TargetDistance > 50) {
         // grote afstand => TargetHeading gebruiken
         HeadingOut = TargetHeading; // graden*256
         SaveHeading = TargetHeading;
      } else {
         // we zijn op minder dan 50mm afstand
         HeadingOut = SaveHeading;
      }

      if (Speed < 0) {
         // negative speed => heading +180
         HeadingOut = NormHoek(HeadingOut + 180*256L, 360*256L);
      }

      Speed = EenparigVertragen(TargetDistance, Speed, EndSpeed, MAX_SLOPE * MAIN_TAKT_RATE*2);

      SpeedHeadingTakt(FirstCall, Speed, HeadingOut/256); // Bij FirstCall neem SpeedHeadingTakt de huidige snelheid als startpunt

//      printf("XYTakt State: %d, HeadingOut: %d, distance: %d, SpeedOut: %d\n", State, (int)(HeadingOut / 256), TargetDistance, Speed);

      return false; // not done yet
   }

//-----------------------------------------------------------------------------
// ArcRelTakt - Rij boog van DeltaDegrees graden
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool TDrive::ArcRelTakt(bool FirstCall, int DeltaDegrees, int Radius, int Speed, int EndSpeed)
   {  static int   StartDegrees_q8, StartOdoT;
      static float TurnRate_q8;
      static int   TurnedDegrees, LastDegrees;

      int OdoL, OdoR, OdoT;  // afgelegde weg in mm
      Position.OdoGet(OdoL, OdoR, OdoT);

      if (FirstCall) {
         // bepaal hoeveel graden we moeten draaien per afgelegde mm (Turnrate)
         TurnRate_q8 = 360 * 256 / 2 / 3.14159 / Radius; // TurnRate in graden_q8 / mm
         StartOdoT = OdoT;
         StartDegrees_q8 = Position.HoekHires();

         TurnedDegrees  = 0;
         LastDegrees    = Position.Hoek;

         if (DeltaDegrees < 0) TurnRate_q8 = -TurnRate_q8;

         if (Flags.IsSet(1)) printf("ArcTakt First Turnrate*100: %d (%d %d %d %d)\n", (int)(TurnRate_q8 * 100), DeltaDegrees, Radius, OdoT, Position.Hoek);
      }

      // Update rotation & check
      TurnedDegrees += NormHoek(Position.Hoek - LastDegrees, 360);
      LastDegrees    = Position.Hoek;
      int RemainingDegrees = DeltaDegrees - TurnedDegrees;
      if (DeltaDegrees < 0) RemainingDegrees *= -1;
      int RestantWeg = 2 * 3.14159 * Radius * RemainingDegrees / 360;

      if (RestantWeg <= 1)  return true;   // done

      long TargetHoek_q8 = TurnRate_q8 * (OdoT - StartOdoT);  // deze hoek willen we nu hebben (graden).
      long HoekError_q8  = NormHoek(Position.HoekHires() - StartDegrees_q8 - TargetHoek_q8, NORM_Q8);

      // bepaal maximale snelheid op gegeven afstand van doel, en verschil tussen L en R o.b.v. radius
      int SpeedL = EenparigVertragen(RestantWeg, Speed, EndSpeed, MAX_SLOPE * MAIN_TAKT_RATE); // max speed
      int SpeedR = SpeedL;

      // feed-forward for rotation
      int _Delta = SpeedL * (WIEL_BASIS / 2) / Radius;
      SpeedL += _Delta;
      SpeedR -= _Delta;

      // simpele P regelaar voor richting
      int Correctie = (HoekError_q8 * PID_Kp) / 4096;
      int ClippedCorrectie = Clip(Correctie, PID_OUT_CLIP * 256, PID_OUT_CLIP * -256);
      SpeedL += (ClippedCorrectie * WIEL_BASIS) / (1024);
      SpeedR -= (ClippedCorrectie * WIEL_BASIS) / (1024);

      // stuur de motoren
      SpeedLRTakt(FirstCall, SpeedL, SpeedR, MAX_SLOPE);

      if (Flags.IsSet(5)) printf("ArcTakt3 : RestWeg: %d, TargetHoek: %d, HoekError: %d, SpeedL/R: %d %d\n",
            RestantWeg, (int)(TargetHoek_q8 / 256), (int)(HoekError_q8/256), SpeedL, SpeedR);

      return false; // not done yet
   }

//-----------------------------------------------------------------------------
// StopTakt -
//-----------------------------------------------------------------------------
// De snelheidswijziging verloopt via een slope
//-----------------------------------------------------------------------------
bool TDrive::StopTakt(bool FirstCall)
   {  static int SavedHeading = 0;

      // save heading
      if (FirstCall) {
         SavedHeading = Position.Hoek;
      }

      SpeedHeadingTakt(FirstCall, 0, SavedHeading);

      if ((SollSpeedL + SollSpeedR) != 0) return false;  // we staan nog niet stil.

      return true;   // done (we staan stil, of toch bijna, de beide setpoints zijn samen 0.

   }

#endif
