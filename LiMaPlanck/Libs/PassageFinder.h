
//-----------------------------------------------------------------------------
// TPassageFinder - use LppArray to find passage.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
class TPassageFinder {
   public:

      int PassageStart; // start of detected passage
      int PassageLen;   // width (in ArrayDegrees) of passage
      int ArrayDegrees;
      int ArrayCount;

      TPassageFinder() {
         ArrayDegrees  = 5;
         ArrayCount    = 16;
         PassageStart  = 0;
         PassageLen    = 0;
      }

      void Setup(int InCenter, int InDegrees, int InCount);
      int Find(int NormDistance, int MinSegments);
};

extern TPassageFinder Passage;

// --- C++ ---
#ifdef MAIN

//-----------------------------------------------------------------------------
// TPassageFinder::Setup - Configure Lpp Array for passage detection
//-----------------------------------------------------------------------------
// Center - 180 is staight ahead
// Degrees - degrees per segment
//
// Settings are saved in ArrayDegrees & ArrayCount
//-----------------------------------------------------------------------------
void TPassageFinder::Setup(int InCenter, int InDegrees, int InCount)
   {
      printf("Passage.Setup Center: %d, Degrees %d, Count: %d\n",
         InCenter, InDegrees, InCount);

      ArrayDegrees = InDegrees;
      ArrayCount   = InCount;

      int StartDeg = InCenter - (ArrayCount / 2) * ArrayDegrees;
      Lpp.ArraySetup(StartDeg, ArrayDegrees, ArrayCount);
   }

//-----------------------------------------------------------------------------
// TPassageFinder::Find - try to find passage
//-----------------------------------------------------------------------------
// NormDistance - distance in mm to wall, distances beyond this are considered
//                passages.
// MinSegments  - Minimal width of passage. Roughly corresponds to
//                   (MinSegments * ArrayDegrees) -2
//                   to
//                   (MinSegments * ArrayDegrees) + 2
//
// This assumes the passage is in a wall perpendicular to the center
// measurement. NormDistance is distance at center.
//
// Return: Degrees, relative to array center. 3600 when no passage is found
//-----------------------------------------------------------------------------
int TPassageFinder::Find(int NormDistance, int MinSegments)
   {
      int State      = 0;
      int NewStart   = 0;
      PassageLen     = 0; // no passage (clear previous passage)

      for (int i=0; i<16; i++) {
         int Deg = (ArrayDegrees * (ArrayCount-1)) / 2 - i * ArrayDegrees;
         int Dist1 = Lpp.Array[i].Distance;
         int Dist2 = Dist1 * cos(GRAD2RAD(Deg));
         if (Flags.IsSet(12)) printf("PF %d %d %d %d\n", i, Deg, Dist1, Dist2);

         switch(State) {
            case 0 : {  // wait for edge
               if (Dist2 < NormDistance) {
                  if (Flags.IsSet(12)) printf("PF State 0 -> 10 @ %d\n", i);
                  State = 10;
               }
            }
            break;

            case 10 : {  // wait for passage start
               if (Dist2 > NormDistance) {
                  if (Flags.IsSet(12)) printf("PF State 10 -> 20 @ %d\n", i);
                  NewStart = i;
                  State = 20;
               }
            }
            break;

            case 20 : {  // wait for passage end
               if (Dist2 < NormDistance) {
                  int NewLen = i - NewStart;
                  if ((NewLen >= MinSegments) && (NewLen > PassageLen)) {
                     PassageLen = NewLen;
                     PassageStart = NewStart;
                     if (Flags.IsSet(12)) printf("PF End Start: %d, i: %d NewLen %d, store\n", NewStart, i, NewLen);
                  } else {
                     if (Flags.IsSet(12)) printf("PF End Start: %d, i: %d NewLen %d, pass\n", NewStart, i, NewLen);
                  }
                  State = 10;
               }
            }
            break;
         }
      }

      if (PassageLen == 0) {
         printf("Passage.Find - No Passage found\n");
         return 3600;
      }

      int PassageDegrees = (ArrayDegrees * (ArrayCount-1)) / 2 - ((PassageStart * 2 + PassageLen) * ArrayDegrees) / 2;

      printf("Passage.Find result: %d Degrees - Start: %d, Len: %d, D1: %d, D2: %d\n",
         PassageDegrees, PassageStart, PassageLen, Lpp.Array[PassageStart - 1].Distance, Lpp.Array[PassageStart + PassageLen].Distance);

      return PassageDegrees;
   }

TPassageFinder Passage;

#endif // MAIN
