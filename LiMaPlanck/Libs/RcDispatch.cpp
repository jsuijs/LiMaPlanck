//-----------------------------------------------------------------------------
// RcDispach.cpp
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// RcDispatch - Map RC5 codes
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void RcDispatch(int &RcData)
{
   if (!RcData) return; // geen nieuwe code ontvangen

   RcData &= 0xF7FF;   // mask toggle bit
   printf("Rc5: 0x%04x\n", RcData);
   int NewCode = RcData;
   RcData = 0; // wis verwerkte code (RcData is referentie)

   switch(NewCode) {

      // Special keys
      case 0x3761    : Position.Reset();   break; // menu -> reset position

      // PF-keys
      case RC_STOP   : PfKey(-1);       break; // STOP (special, but passed to ProgrammaTakt)

      case RC_F01    : PfKey( 1);       break;
      case RC_F02    : PfKey( 2);       break;
      case RC_F03    : PfKey( 3);       break;
      case RC_F04    : PfKey( 4);       break;
      case RC_F05    : PfKey( 5);       break;
      case RC_F06    : PfKey( 6);       break;
      case RC_F07    : PfKey( 7);       break;
      case RC_F08    : PfKey( 8);       break;
      case RC_F09    : PfKey( 9);       break;
      case RC_F10    : PfKey(10);       break;
      case RC_F11    : PfKey(11);       break;
      case RC_F12    : PfKey(12);       break;
#ifdef RC_F13
      case RC_F13    : PfKey(13);       break;
#endif
#ifdef RC_F14
      case RC_F14    : PfKey(14);       break;
#endif
#ifdef RC_F15
      case RC_F15    : PfKey(15);       break;
#endif
#ifdef RC_F16
      case RC_F16    : PfKey(16);       break;
#endif
   }
}
