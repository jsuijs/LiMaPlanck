
#include "Libs/Adafruit_ADS1015.h"

void ReadLineSensor();
extern int LinePosition;

#ifdef FRAMEWORK_CODE

Adafruit_ADS1015 AdsL(ADS1015_ADDRESS);
Adafruit_ADS1015 AdsR(ADS1015_ADDRESS+2);

int LineValues[8];
int LinePosition= 0;

void ReadLineSensor()
{  static bool First = true;
   const int Weights[] = { 8, 4, 2, 1, -1, -2, -4, -8 };

   if (First) {
      First = false;
      printf("Try to read linesensor / ADS1015. This will block on failure...\n");


   }

   //int Start = micros();
   LinePosition = 0;
   for (int i=0; i<8; i++) {
      if (i < 4) {
         LineValues[i] = AdsR.readADC_SingleEnded(i);
      } else {
         LineValues[i] = AdsL.readADC_SingleEnded(i-4);
      }
      //printf("i: %d, v: %d\n", i, LineValues[i]);
      LinePosition += (LineValues[i] * Weights[i]);
   }

   //printf("LinePos: %d\n", LinePosition);
}


#endif // FRAMEWORK_CODE
