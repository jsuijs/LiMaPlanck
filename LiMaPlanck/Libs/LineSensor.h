
#include "Libs/Adafruit_ADS1015.h"

void LineSensorRead();

#ifdef FRAMEWORK_CODE

Adafruit_ADS1015 AdsL(ADS1015_ADDRESS);     /* Use thi for the 12-bit version */
Adafruit_ADS1015 AdsR(ADS1015_ADDRESS+2);     /* Use thi for the 12-bit version */

int LineValues[8];
int LinePosition= 0;

void LineSensorRead()
{  static bool First = true;
   const int Weights[] = { 8, 4, 2, 1, -1, -2, -4, -8 };

   if (First) {
      printf("Try to read linesensor / ADS1015. This will block on failure...\n");
      First = false;
   }

   LinePosition = 0;
   for (int i=0; i<8; i++) {
      if (i < 4) {
         LineValues[i] = AdsR.readADC_SingleEnded(i);
      } else {
         LineValues[i] = AdsL.readADC_SingleEnded(i-4);
      }
      printf("i: %d, v: %d\n", i, LineValues[i]);
      LinePosition += (LineValues[i] * Weights[i]);
   }

   printf("LinePos: %d\n", LinePosition);
}


#endif // FRAMEWORK_CODE
