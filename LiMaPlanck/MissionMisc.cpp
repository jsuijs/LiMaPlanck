//-----------------------------------------------------------------------------
// MissionMisc.cpp - Diverse missies
//-----------------------------------------------------------------------------
#include "RobotSettings.h"
#include "Libs/MyRobot.h"
#include "Project.h"


//-----------------------------------------------------------------------------
// Rijden1Takt - pwm trapje, print pwm + toerental
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool Rijden1Takt(bool Init)
{
   static int Step;
   static bool Oplopend;
   static int Slow;

   if (Init) {
      Step = 0;
      Oplopend = true;
      Slow = 0;
   }

   if (Slow == 0) {
      if (Oplopend) {
         Step +=2;
      } else {
         Step -=2;
      }
      Slow = 10;
   } else {
      Slow --;
   }

    if (Step >  255) Oplopend = false;
    if (Step < -255) Oplopend = true;

    Driver.Pwm(Step, Step);
    printf("Pwm: %d, Speed: %d / %d\n", Step, Position.ActSpeedL, Position.ActSpeedR);
    Position.Print();

    if (Oplopend && (Step == 0)) {
      return true; // Klaar
   }

   return false;  // Nog niet klaar.
}

#define UMB_MARK_AFSTAND 800    // mm
#define UMB_MARK_SPEED 300    // mm/sec

//-----------------------------------------------------------------------------
// MissionTest - state machine
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissionTest(TState &S)
{  static int x = 0;

   S.Update("Test", Flags.IsSet(11));

   switch (S.State) {

      case 0 : {  // LIDAR-STARTEN
         if (S.NewState) {
            Lpp.Start();
         }

         if (S.StateTime() > 2000) {      // Wacht op start lidar
            S.State++;
         }
      }
      break;

      case 1 : {
         // blijf hier tot op stop wordt gedrukt.
         if (x !=  Lpp.Sensor[3].Distance) {
            x =  Lpp.Sensor[3].Distance;
            printf("Lidar S2 Distance %d, Degrees: %d\n",
               Lpp.Sensor[2].Distance, Lpp.Sensor[2].Degrees32/32);
         }

      }
      break;


      default : {
         printf("Error: ongeldige state in MissionTest (%d)\n", S.State);
         return true;  // error => mission end
      }
   }
   return false;  // mission nog niet gereed
}

static int VolgRechts()
{
//@@   return (400 - LidarArray_R40)/50;  // wand volgen
return 0;
}

//-----------------------------------------------------------------------------
// MissieTTijd - state machine
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#define TT_FRONT_DETECT 400
bool MissieTTijd(TState &S)
{  int x;

   S.Update("TTijd", Flags.IsSet(11));

   switch (S.State) {

      case 0 : {  // LIDAR-STARTEN
         if (S.NewState) {
            Driver.Pwm(0, 0);
            Position.Reset();
            Lpp.Start();
         }

         if (S.StateTime() > 2000) S.State += 10; //Wacht op start lidar
      }
      break;

      case 10 : { // Volg wand naar vak B

         Driver.SpeedHeading(S.Param1, VolgRechts());  // Volg rechtse wand

//@@         if (LidarArray_L40 < TT_FRONT_DETECT) S.State += 10; // Naar de volgende state als we de wand voor ons zien
      }
      break;

      case 20 : { // Stop

         if (S.NewState) Driver.Stop();

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 30 : { // 1e draai in vak B

         if (S.NewState) Driver.Rotate(90); // 90 graden linksom

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 40 : { // Volg wand in vak B

         Driver.SpeedHeading(S.Param1, VolgRechts() + 90);  // Volg rechtse wand

//@@         if (LidarArray_L40 < TT_FRONT_DETECT) S.State += 10; // Naar de volgende state als we de wand voor ons zien
      }
      break;

      case 50 : { // Stop

         if (S.NewState) Driver.Stop();

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 60 : { // 2e draai in vak B

         if (S.NewState) Driver.Rotate(90);

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 70 : { // Volg wand uit vak B

         Driver.SpeedHeading(S.Param1, VolgRechts() + 180);  // Volg rechtse wand

//@@         if (LidarArray_R40 > 400) S.State += 10; // Naar de volgende state als we rechts geen wand meer zien
      }
      break;

      case 80 : { // Nog klein stukje rechtdoor

         if (S.NewState) Driver.XY(Position.XPos-450, Position.YPos, S.Param1, 0);

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 90 : { // draai richting C

         if (S.NewState) Driver.Rotate(-90); // 90 graden rechtsom

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 100 : { // Volg wand naar vak C
         x = VolgRechts();  // wand volgen
         x = Clip(x, -5, 5);
         Driver.SpeedHeading(S.Param1, x + 90);  // Volg rechtse wand
//@@         printf("LidarArray_R40: %d, x: %d\n", LidarArray_R40, x);

//@@         if (LidarArray_L40 < TT_FRONT_DETECT) S.State += 10; // Naar volgende state als we de wand voor ons zien
      }
      break;

      case 110 : { // Stop

         if (S.NewState) Driver.Stop();

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 120 : { // 1e draai in vak C

         if (S.NewState) Driver.Rotate(90); // 90 graden naar links

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 130 : { // Volg wand in vak C

         Driver.SpeedHeading(S.Param1, VolgRechts() + 180);  // Volg rechtse wand

//@@         if (LidarArray_L40 < TT_FRONT_DETECT) S.State += 10; // Naar volgende state als we de wand voor ons zien
      }
      break;

      case 140 : { // Stop

         if (S.NewState) Driver.Stop();

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 150 : { // 2e draai in vak C

         if (S.NewState) Driver.Rotate(90); // 90 graden naar links

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 160 : { // Volg wand uit vak C

         Driver.SpeedHeading(S.Param1, VolgRechts() + 270);  // Volg rechtse wand

//@@         if (LidarArray_R40 > 400) S.State += 10; // Naar volgende state als we rechts geen wand meer zien
      }
      break;

      case 170 : { // Nog een klein stukje rechtdoor

         if (S.NewState) Driver.XY(Position.XPos, Position.YPos-450, S.Param1, 0); // Heading (in graden)

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 180 : { // Draai richting vak A

         if (S.NewState) Driver.Rotate(-90); // 90 graden naar links

         if (Driver.IsDone()) S.State += 10; // Naar de volgende state als de beweging klaar is
      }
      break;

      case 190 : { // Volg wand naar vak A
         Driver.SpeedHeading(S.Param1, VolgRechts() + 180);  // Volg rechtse wand

//@@         if (LidarArray_L40 < TT_FRONT_DETECT) S.State += 10;  // als we de wand voor ons zien
      }
      break;

      case 200 : { // Stop (via curve)
         // Deze state zorgt dat we netjes afremmen voordat we de melden dat de missie klaar is.
         //
         // Het hoofd-programma zet pwm op 0 als de missie is afgerond
         // en dat geeft een 'noodstop' als we nog rijden.
         //
         if (S.NewState) Driver.Stop();

         if (Driver.IsDone()) return true; // Missie klaar als we netjes gestopt zijn.
      }
      break;

      default : {
         printf("Error: ongeldige state in MissieTTijd (%d)\n", S.State);
         return true;  // error => mission end
      }
   }
   return false;  // mission nog niet gereed
}

//-----------------------------------------------------------------------------
// MissieBlikken -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissieBlikken(TState &S)
{  static int x = 0;
   static int BlikNummer;      // Blikken ophalen
   static int BlikPosY;        // plaats waar blik wordt weggezet

   S.Update("Mission Blikken");

   switch (S.State) {

      case 0 : {  // LIDAR-STARTEN
         if (S.NewState) {
            Position.Reset();
            Lpp.Start();
            BlikNummer = 1;
            BlikPosY = 520;    // voor 1e blik
         }

         if (S.StateTime() > 2000) {                        // Geef de lidar wat tijd
            if (ServoSlope(myservo, SERVO_OPEN, 20)) S.State ++;   // wacht daarna op servo
         }
       }
       break;

      case 1 : { // default: rij rechtuit
         if (S.NewState) Driver.SpeedLR(100, 100);

         if (x !=  Lpp.Sensor[3].Distance) {
            x =  Lpp.Sensor[3].Distance;
            printf("Lidar S2 Distance %d, Degrees: %d\n",
                   Lpp.Sensor[2].Distance, Lpp.Sensor[2].Degrees32 / 32);
         }

         if (Lpp.Sensor[2].Distance < 500) S.State ++;   // Blik in bereik
      }
      break;

      case 2 : {  // Draai naar blik .... graden
         if (S.NewState) {
            int BlikHoekCorrectie = (180 - Lpp.Sensor[2].Degrees32 / 32);
            if (BlikHoekCorrectie > 0) {
               BlikHoekCorrectie = (BlikHoekCorrectie + 2);
            }
            else {
               BlikHoekCorrectie = (BlikHoekCorrectie - 4);
            }
            printf("HoekCor: %d\n", BlikHoekCorrectie);
            Driver.Rotate(BlikHoekCorrectie); // Berekende graden links of rechts
         }

         if (Driver.IsDone()) S.State ++;
      }
      break;

      case 3 : {  // Naar blik rijden
         if (S.NewState) Driver.SpeedLR(60, 60);     // Rijden naar blik

         if (Lpp.Sensor[2].Distance < 130) {
            printf("blik gevonden: \n");
            Driver.SpeedLR(0, 0);       // Stop motoren
            S.State ++; // Naar de volgende state als de beweging klaar is
         }
      }
      break;

      case 4 : {  // Grijper sluiten
         if (ServoSlope(myservo, SERVO_CLOSE, 20)) S.State = 5;
      }
      break;

      case 5 : {  // Achterwaards rijden naar plaats om blik weg te zetten.
         if (S.NewState) Driver.XY(0, BlikPosY , -100, 0 );

         if (Driver.IsDone()) S.State ++;
      }
      break;

      case 6 : {  // Pos.0.0 > 180 gr draaien naar achterwand
         if (S.NewState) Driver.RotateHeading(180);

         if (Driver.IsDone()) S.State ++;
      }
      break;

      case 7 : {  // Grijper openen
         if (ServoSlope(myservo, SERVO_OPEN, 20)) S.State ++;
      }
      break;

      case 8 : {  // Pos.0.0 > 90 gr draaien richting 0.0
        if (S.NewState) Driver.Rotate(90);

        if (Driver.IsDone()) S.State ++;
      }
      break;

      case 9 : {  // terug naar startpunt (0.0) voor volgende sessie
         if (S.NewState) Driver.XY(0, 0 , 60, 0 );

         if (Driver.IsDone()) S.State ++;
      }
      break;

      case 10 : {  // Kijk of we klaar zijn
         if (S.NewState) {

            BlikPosY -= 130;     // plaats van volgend blik
            BlikNummer ++;       // nummer van volgend blik

            if (BlikNummer >= 5) {
               printf("5 blikken: klaar\n");
               return true;
            }

            Driver.Rotate(90);   // draai naar rijrichting
         }

         if (Driver.IsDone()) S.State = 1; // volgend blik
      }
      break;

      //****
      default : {
         printf("Error: ongeldige state in MissieBlikken (%d)\n", S.State);
         return true;  // error => mission end
      }
   }
   return false;  // mission nog niet gereed
}

int AfstBediening;

//-----------------------------------------------------------------------------
// RandomRijdenTakt -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool MissieRandomRijden(TState &S)
{
   S.Update("RandomRijden", Flags.IsSet(11));

//   int Lidar_A = Lpp.Sensor[0].Distance;         // Afstand achterzijde
//   int Lidar_grV = Lpp.Sensor[1].Degrees32 / 32; // Scannen(1) Hoek van 90 graden + 180 gr tot 270 graden
//   int Lidar_90V = Lpp.Sensor[2].Distance;       // Scannen(2) Korste Afstand van 135 graden + 90 gr tot 225 graden
   int Lidar_Blik_L  = Lpp.Sensor[3].Distance;  // Scannen(3) Korste Afstand van 70 graden + 40 gr tot 110 graden
   int Lidar_Blik_LV = Lpp.Sensor[4].Distance;  // Scannen(4) Korste Afstand van 110 graden + 40 gr tot 150 graden
   int Lidar_Blik_V  = Lpp.Sensor[5].Distance;  // Scannen(5) Korste Afstand van 150 graden + 60 gr tot 210 graden
   int Lidar_Blik_RV = Lpp.Sensor[6].Distance;  // Scannen(6) Korste Afstand van 210 graden + 40 gr tot 250 graden
   int Lidar_Blik_R  = Lpp.Sensor[7].Distance;  // Scannen(7) Korste Afstand van 250 graden + 40 gr tot 290 graden

   switch (S.State) {

      //----------------------------------------------------------------------
      //      *** MaxonRobot - Slalom=8 - Rijden LockDown Challenge !!! 02-09-2020
      //----------------------------------------------------------------------
      case 0 : { // LIDAR-STARTEN 8 - Rijden LockDown Challenge
         if (S.NewState) {
            Driver.Pwm(0, 0);
            Lpp.Start();
         }

         if (S.StateTime() > 2000) {   //Wacht op start lidar
            S.State = 601;              //Random rijden in bak
         }
      }
      break;

      //----------------------------------------------------------------------
      //      *** Maxon2 Robot - Random rijden
      //----------------------------------------------------------------------
      case 601 : { // Random rijden

         if (S.NewState) {
         }

         int Lid_Max = 500;          // Lidar Meetwaarde beperken
         //if (Lidar_grV > Lid_Max)   Lidar_grV = Lid_Max;        // 90><180 gr valse meting
         //if (Lidar_90V > Lid_Max)   Lidar_90V = Lid_Max;        // 135><225 gr valse meting
         if (Lidar_Blik_L  > Lid_Max)  Lidar_Blik_L  = Lid_Max;   // 70><110 gr valse meting
         if (Lidar_Blik_LV > Lid_Max)  Lidar_Blik_LV = Lid_Max;   // 110><150 gr valse meting
         if (Lidar_Blik_V  > Lid_Max)  Lidar_Blik_V  = Lid_Max;   // 150><210 gr valse meting
         if (Lidar_Blik_RV > Lid_Max)  Lidar_Blik_RV = Lid_Max;   // 210><250 gr valse meting
         if (Lidar_Blik_R  > Lid_Max)  Lidar_Blik_R  = Lid_Max;   // 250><290 gr valse meting

         if (Lidar_Blik_V < 240) {
            // print a random number from 0 to 6
            int randNumber = random(2);
            CSerial.println(randNumber);
            if (randNumber == 1) {
               S.State = 603;      // 180 gr R/Om draaien
            }
            else {
               S.State = 604;      // 180 gr L/Om draaien
            }
         }
         if ((  Lidar_Blik_LV < 200) && (Lidar_Blik_V < 250) && (Lidar_Blik_RV < 200)) {
            S.State = 602;      // 180 gr R/Om draaien
         }
         if ((Lidar_Blik_LV < 200) || (Lidar_Blik_RV > Lidar_Blik_LV)) {
            Driver.SpeedLR(150, 90);   // Rechts afdraaien
            break;
         }
         if ((Lidar_Blik_RV < 200) || (Lidar_Blik_LV > Lidar_Blik_RV)) {
            Driver.SpeedLR(90, 150);   //Links afdraaien
            break;
         }
         Driver.SpeedLR(150, 150);   // default rechtuit
      }
      break;

      //----------------------------------------------------------------------
      case 602 : { // 180 gr RECHTSOM draaien - Random rijden

         if (S.NewState) {
            Driver.Rotate(-180);
         }
         if (Driver.IsDone()) S.State = 601;
      }
      break;

      //----------------------------------------------------------------------
      case 603 : { // 90 gr RECHTSOM draaien - Random rijden

         if (S.NewState) {
            Driver.Rotate(-90);
         }
         if (Driver.IsDone()) S.State = 601;
      }
      break;

     //----------------------------------------------------------------------
      case 604 : { // 90 gr LINKSOM draaien - Random rijden

         if (S.NewState) {
            Driver.Rotate(90);
         }
         if (Driver.IsDone()) S.State = 601;
      }
      break;

      default : return S.InvalidState(__FUNCTION__);   // Report invalid state & end mission
   }
   return false;  // mission nog niet gereed
}
