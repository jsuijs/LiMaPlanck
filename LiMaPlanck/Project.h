//-----------------------------------------------------------------------------
// Project.h - Project-specific includes.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#ifndef PROJECT_H
#define PROJECT_H

#include <Servo.h>
extern Servo myservo;

void LppSensorDefaultSetup();

bool MissionAloys1(TState &S);
bool MissionDuckling(TState &S);

bool MissionRijden1(TState &S);
bool MissieTTijd(TState &S);
//bool MissieHeenEnWeer(TState &S);
bool MissieRandomRijden(TState &S);
bool MissieBlikken(TState &S);
bool MissionTest(TState &S);

#endif
