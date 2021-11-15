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
   printf("Starten\n");

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

      Lpp.SetOffsetDegrees(4);      // Align lidar with robotlib coordinate system; 180 Degrees = forward.
      Lpp.SetReverse(1);            // Angle to the left is positive.

      //Lpp.ArraySetup(70, 20, 11);   // Setup array with 11 segments of 20 degrees

      LppSensorDefaultSetup();      // Separate function, so we can reload later.

      // Lees en print status (ter informatie)
      Lpp.ReadStatus();
      Lpp.PrintStatus();
   } else {
      printf("LPP I2C error.\n");
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
   printf("Opstarten gereed.\n");

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
      //printf("Batterij: %d (V * 10) (%d)\n", Spanning, Batterij);
      //printf("Lijn: %d (%d %d %d)\n", Lijn, digitalRead(5), digitalRead(6), digitalRead(7));
   }

   Command.Takt(CSerial);  // Console command interpreter
}

//---------------------------------------------------------------------------------------
// LppSensorDefaultSetup -
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void LppSensorDefaultSetup()
{
   Lpp.SensorSetup(0,  165, 30);    // Achterwaarts 180 +/- 15
   Lpp.SensorSetup(1, -110, 40);    // Rechts, -90 +/- 20
   Lpp.SensorSetup(2,  -70, 40);    // Rechts-voor -50 +/- 20
   Lpp.SensorSetup(3,  -30, 60);    // Voor, 0 +/- 30
   Lpp.SensorSetup(4,   30, 40);    // Links-voor, 50 +/- 20
   Lpp.SensorSetup(5,   70, 40);    // Links   90 +/- 20
   Lpp.SensorSetupCan(6, -45, 90);  // Blikdetectie voor, 0 +/- 45
//   Lpp.SensorSetup(7, 180-250, 40);  // Vrije sensor, stel in als benodigd bij de missie
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
      Lpp.PrintArray();
      Lpp.PrintSensors();
   }
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
   if (Command.Match("LppSetupS",      3)) Lpp.SensorSetup(Param[0], Param[1], Param[2]);  // Sensor #, start & width
   if (Command.Match("LppPrintS",      0)) Lpp.PrintSensors();

   if (Command.Match("PassageSetup",   3)) Passage.Setup(Param[0], Param[1], Param[2]);
   if (Command.Match("PassageFind",    2)) printf("PassageFind %d degrees\n", Passage.Find(Param[0], Param[1]));

   if (Command.Match("PfKey",          1)) PfKeySet(Param[0]);
   if (Command.Match("Position",       0)) Position.Print();
   if (Command.Match("PositionReset",  0)) Position.Reset();

   if (Command.Match("Flag",           1)) printf("Flag %d is %d\n", Param[0], Flags.IsSet(Param[0]));
   if (Command.Match("Flag",           2)) Flags.Set(Param[0], Param[1]);
   if (Command.Match("FlagDump",       0)) Flags.Dump();

   if (Command.Match("Servo",          1)) myservo.write(Param[0]);

}
