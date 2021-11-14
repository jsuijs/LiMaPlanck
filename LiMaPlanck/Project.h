//-----------------------------------------------------------------------------
// Project.h - Project-specific includes.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#ifndef PROJECT_H
#define PROJECT_H

#include <Servo.h>
extern Servo myservo;

void LppSensorDefaultSetup();

extern int LidarArray_L40 ;
extern int LidarArray_V   ;
extern int LidarArray_R40 ;

bool MissionAloys1(TState &S);
bool MissionDuckling(TState &S);

bool Rijden1Takt(bool Init);
bool MissieUmbMark1(TState &S);
bool MissieTTijd(TState &S);
bool MissieHeenEnWeer(TState &S);
bool MissieRandomRijden(TState &S);
bool MissieBlikken(TState &S);
bool MissionTest(TState &S);

#endif
