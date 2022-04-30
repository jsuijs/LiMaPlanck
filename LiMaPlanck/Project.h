//-----------------------------------------------------------------------------
// Project.h - Project-specific includes.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#ifndef PROJECT_H
#define PROJECT_H

#include <Servo.h>
extern Servo myservo;

void LppSensorDefaultSetup();

bool MissionGripperTest(TState & S);
bool MissionWheelSizeCalibrate(TState &S);
bool MissionRandomRijden(TState &S);
bool MissionSuperSlalom(TState &S);

bool MissionAloys1(TState &S);
bool MissionDuckling(TState &S);
bool MissionLijnVolgen(TState &S);        //LijnSensor 5xBPW40
bool MissionSlalom1(TState &S);
bool MissionTTijdOpening1(TState &S);      // T-Tijd met opening zoeken in vak -C-
bool MissionStartVector1(TState &S);
bool MissionBlikken(TState &S);

extern TState SubS;  // Sub-mission statemachine
void ReadLijnsensor();

// conversie van oude nummers (in letters) naar nieuw
// (ten behoeve van conversie; !! let op ook range van Degrees32 is gewijzig !!)
const int S_NUL   = 0;
const int S_EEN   = 7;
const int S_TWEE  = 6;
const int S_DRIE  = 5;
const int S_VIER  = 4;
const int S_VIJF  = 3;
const int S_ZES   = 2;
const int S_ZEVEN = 1;

// Nieuwe namen voor sensoren
const int S_ACHTER      = 0;
const int S_RECHTS_VOOR = 2;
const int S_VOOR        = 3;
const int S_LINKS_VOOR  = 4;

void LedTakt();

#endif
