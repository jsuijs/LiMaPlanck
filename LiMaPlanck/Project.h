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
bool MissionUmbMark(TState &S);
bool MissionOdoTest(TState &S);
bool MissionRandomRijden(TState &S);
bool MissionSuperSlalom(TState &S);

bool MissionAloys1(TState &S);
bool MissionDuckling(TState &S);
bool MissionLijnVolgen(TState &S);        //LijnSensor 5xBPW40
bool MissionSlalom1(TState &S);
bool MissionTTijdOpening1(TState &S);      // T-Tijd met opening zoeken in vak -C-
bool MissionVectorStart1(TState &S);

extern TState MissonS;  // Mission statemachine
void ReadLijnsensor();

#endif
