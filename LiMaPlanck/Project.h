//-----------------------------------------------------------------------------
// Project.h - Project-specific includes.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#ifndef PROJECT_H
#define PROJECT_H

#include <Servo.h>
extern Servo myservo;

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

void ReadLijnsensor();

// Nieuwe namen voor sensoren
const int S_ACHTER      = 0;
const int S_RECHTS_VOOR = 2;
const int S_VOOR        = 3;
const int S_LINKS_VOOR  = 4;

void LedTakt();

#endif
