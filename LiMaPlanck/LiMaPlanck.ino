//-----------------------------------------------------------------------------
// LiMaPlanck.ino
//-----------------------------------------------------------------------------

#define MAIN
#include "RobotSettings.h"          // project-instellingen
#include "Libs/MyRobot.h"           // super-include
#include "Libs/Motors_TB6612.cpp"   // Kies je motor-sturing
#include "Project.h"

void Execute();   // prototype

// global instances
TPosition      Position;
TDrive         Driver;
TCommand       Command(Execute);
TLpp           Lpp;
HardwareSerial Serial2 (PA3, PA2);  // rx, tx
TwoWire        Wire2(PB11, PB10);   // sda, scl
TFlags         Flags(32);

Servo myservo;  // create servo object to control a servo

//int LidarArray_L100;
//int LidarArray_L80 ;
//int LidarArray_L60 ;
int LidarArray_L40 ;
//int LidarArray_L20 ;
int LidarArray_V   ;
//int LidarArray_R20 ;
int LidarArray_R40 ;
//int LidarArray_R60 ;
//int LidarArray_R80 ;
//int LidarArray_R100;

//---------------------------------------------------------------------------------------
// RC5 stuff start
#include "Libs/RC5.h"

int Rc5Data;  // Set on receive, feel free to set to zero when done.
int IR_PIN = PB4;
//int RC5_INTERRUPT = 0;
RC5 rc5(IR_PIN);

void Rc5Isr()
{ static unsigned int   PrevMessage;
  unsigned int Message;
  if (rc5.read(&Message)) {
    if (Message != PrevMessage) {
      Rc5Data     = Message;
      PrevMessage = Message;
    }
  }
}
// Rc5 stuff done (but do not forget to attach Rc5Isr() to IrPin).
//---------------------------------------------------------------------------------------

// Create buzzer instance & call it each ms from SYSTICK
TBuzzer Buzzer(BUZZER_PIN);
void HAL_SYSTICK_Callback(void) { Buzzer.Takt(); }

//---------------------------------------------------------------------------------------
// setup -
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void setup() {
   // start serial
   CSerial.begin(115200);
   CSerial.printf("Starten\n");

   Flags.Set(20, true);    // Position print (each update)
   Position.init();        // delayed constructor
   Driver.init();          // delayed constructor

   SetupMotors();
   InitStmEncoders();

   pinMode(PB1, OUTPUT);    //Led op Maple-Mini

   // Link PinChange interrupt to RC5 reader.
   attachInterrupt(IR_PIN, Rc5Isr, CHANGE);

   I2cClearBus(PB11, PB10); // SDA, SCL
   LppWire.begin();
   if (Lpp.begin()) {

      Lpp.SetOffsetDegrees(184);    // Align lidar with robotlib coordinate system; 180 Degrees = forward.
      Lpp.SetReverse(0);            // Angle to the left is positive.

      Lpp.ArraySetup(70, 20, 11);   // Setup array with 11 segments of 20 degrees

      LppSensorDefaultSetup();      // Separate function, so we can reload later.

      // Lees en print status (ter informatie)
      Lpp.ReadStatus();
      Lpp.PrintStatus();
   } else {
      CSerial.printf("LPP I2C error.\n");
      Buzzer.BeepWait(200, 3);
   }

   Flags.Set(1, true);     // Drive FirstCall - all movements (sub takt)
   //Flags.Set(2, true);     // 2 Driver.SpeedLRTakt
   //Flags.Set(3, true);     // 3 Driver.RotateRelTakt
   //Flags.Set(4, true);     // 4 Driver.SpeedHeadingTakt
   //Flags.Set(5, true);     // 5 Driver.ArcRelTakt
   //Flags.Set(6, true);     // 6 Driver.XYTakt
   Flags.Set(7, true);     // 7 Driver.SpeedRotationTakt


   //Flags.Set(9, true);   // Lpp array dump
   Flags.Set(10, true);    // ProgrammaTakt programma-keuze
   Flags.Set(11, true);    // ProgrammaTakt Missie-takt
   Flags.Set(12, true);    // PassageFinder

   Buzzer.Beep(30, 2);
   CSerial.printf("Opstarten gereed.\n");

   myservo.attach(PB5);    // attaches the servo on pin 17 to the servo object
   myservo.write(550);     // default: gripper open
}

//---------------------------------------------------------------------------------------
// loop -
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void loop() {
   static int NextMainTakt;
   static int NextSecTakt;
   static int PrevMs;

   RcDispatch(Rc5Data);

   // eens per miliseconde
   int ms = millis();
   if (ms != PrevMs) {  // miliseconde takt
      PrevMs = ms;  // bewaar huidige tijd
      BlinkTakt();
   }
   // Main takt interval
   ms = millis();
   if ((ms - NextMainTakt) > 0) {
      NextMainTakt = ms + MAIN_TAKT_INTERVAL;  // zet tijd voor volgende interval
      // hier de periodieke acties voor deze interval
      ReadLpp();
      Position.Takt();  // Lees & verwerk encoder data
      ProgrammaTakt();  // Voer (stapje van) geselecteerde programma uit
      Driver.Takt();    // stuur motoren aan
   }

   // Seconde interval
   ms = millis();
   if ((ms - NextSecTakt) > 0) {
      NextSecTakt = ms + 1000;  // zet tijd voor volgende interval
      // hier de periodieke acties voor deze interval

      if (Flags.IsSet(5)) {
         Position.Print();
      }

      //int Batterij = analogRead(BATTERIJ_PIN);
      //int Spanning = (int) (145L * Batterij / 960);  // 14.8 volt geeft waarde 964
      //CSerial.printf("Batterij: %d (V * 10) (%d)\n", Spanning, Batterij);
      //CSerial.printf("Lijn: %d (%d %d %d)\n", Lijn, digitalRead(5), digitalRead(6), digitalRead(7));
   }

   Command.Takt(CSerial);  // Console command interpreter
}

//---------------------------------------------------------------------------------------
// LppSensorDefaultSetup -
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void LppSensorDefaultSetup()
{
   Lpp.SensorSetup(0, -15, 30);  // Sensor 0  achterwaarts (-15 + 30 = +15 graden)
   Lpp.SensorSetup(1, 90, 180);  // Sensor 1, vanaf 90 graden, (+90 + 180 = 270 graden)
   Lpp.SensorSetupCan(2, 135, 90);  // Sensor 2, vanaf 135 graden, segment van 90 graden
   Lpp.SensorSetup(3, 70, 40);   // Sensor 3, vanaf 70 graden, segment van 40 graden LockDown 5=8-Slalom
   Lpp.SensorSetup(4, 110, 40);  // Sensor 4, vanaf 110 graden, segment van 40 graden
   Lpp.SensorSetup(5, 150, 60);  // Sensor 5, vanaf 150 graden, segment van 60 graden
   Lpp.SensorSetup(6, 210, 40);  // Sensor 4, vanaf 210 graden, segment van 40 graden
   Lpp.SensorSetup(7, 250, 40);  // Sensor 7, vanaf 250 graden, segment van 40 graden
}

//---------------------------------------------------------------------------------------
// ReadLpp - get state & put it in specific vars
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void ReadLpp()
{
   Lpp.ReadArray();  // lees lidar array data
   Lpp.ReadSensors();// lees lidar sensor data

   if (Flags.IsSet(9)) {
      for (int i=0; i<16; i++) {
         CSerial.printf("%d ", Lpp.Array[i]);
      }
      CSerial.printf("\n");
   }
   LidarArray_L40    = Lpp.Array[ 3].Distance;
   LidarArray_V      = Lpp.Array[ 5].Distance;
   LidarArray_R40    = Lpp.Array[ 7].Distance;
}

//-----------------------------------------------------------------------------
// BlinkTakt -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void BlinkTakt()
{  static int Count;

   if (Count > 100) {
      digitalWrite(PB1, ! digitalRead(PB1));
      Count = 0;
   }
   Count ++;
}

//-----------------------------------------------------------------------------
// Execute - execute commando
//-----------------------------------------------------------------------------
// Called via CmdTakt() when a command is received from the serial port.
//-----------------------------------------------------------------------------
void Execute(int Param[])
{
   if (Command.Match("?",              0)) Command.Help("ArduinoPlanck command parser.");

   // Drive commands
   if (Command.Match("DrivePwm",       2)) Driver.Pwm(Param[0], Param[1]);
   if (Command.Match("DriveLR",        2)) Driver.SpeedLR(Param[0], Param[1]);
   if (Command.Match("DriveSR",        2)) Driver.SpeedRotation(Param[0], Param[1]);
   if (Command.Match("DriveSH",        2)) Driver.SpeedHeading(Param[0], Param[1]);
   if (Command.Match("DriveXY",        4)) Driver.XY(Param[0], Param[1], Param[2], Param[3]);
   if (Command.Match("DriveRotateH",   1)) Driver.RotateHeading(Param[0]);
   if (Command.Match("DriveRotate",    1)) Driver.Rotate(Param[0]);
   if (Command.Match("DriveRotateH",   2)) Driver.RotateHeading(Param[0], Param[1]);
   if (Command.Match("DriveRotate",    2)) Driver.Rotate(Param[0], Param[1]);
   if (Command.Match("DriveArcH",      4)) Driver.ArcHeading(Param[0], Param[1], Param[2], Param[3]);
   if (Command.Match("DriveArc",       4)) Driver.Arc(Param[0], Param[1], Param[2], Param[3]);

   if (Command.Match("Stop",           0)) Driver.Stop();

   if (Command.Match("LppStatus",      0)) { Lpp.ReadStatus(); Lpp.PrintStatus(); }
   if (Command.Match("LppStart",       0)) Lpp.Start();
   if (Command.Match("LppStop",        0)) Lpp.Stop();

   if (Command.Match("PfKey",          1)) PfKeySet(Param[0]);
   if (Command.Match("Position",       0)) Position.Print();
   if (Command.Match("PositionReset",  0)) Position.Reset();

   if (Command.Match("Flag",           1)) CSerial.printf("Flag %d is %d\n", Param[0], Flags.IsSet(Param[0]));
   if (Command.Match("Flag",           2)) Flags.Set(Param[0], Param[1]);
   if (Command.Match("FlagDump",       0)) Flags.Dump();

   if (Command.Match("Servo",          1)) myservo.write(Param[0]);

   if (Command.Match("PassageSetup",   3)) Passage.Setup(Param[0], Param[1], Param[2]);
   if (Command.Match("PassageFind",    2)) printf("PassageFind %d degrees\n", Passage.Find(Param[0], Param[1]));
}
