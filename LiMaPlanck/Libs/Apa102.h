//-----------------------------------------------------------------------------
// Apa102.h  - Control a string of APA102 RGB LEDs
//-----------------------------------------------------------------------------
// Derived from RobotLib
// Note: #define MAIN enables acutual code, in adition to prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// From Robotlib: ColorLed.h
#ifndef __COLORLED_H
#define __COLORLED_H

class TColor
{
public:
   TColor()                   : Color(0)        {};
   TColor(uint32_t InColor)   : Color(InColor)  {};
   TColor(uint8_t Red, uint8_t Green, uint8_t Blue) :
      Dummy(0), R(Red), G(Green), B(Blue)  {};

   void RGB(uint8_t Red, uint8_t Green, uint8_t Blue);
   void HSV(uint8_t Hue, uint8_t Saturation=255, uint8_t Value=255);

   union {
      uint32_t Color;
      struct {
         uint8_t Dummy;
         uint8_t R;
         uint8_t G;
         uint8_t B;
      };
   };
};

const TColor CRGB_RED   (0xFF, 0x00, 0x00);
const TColor CRGB_YELLOW(0xFF, 0xB0, 0x00);   // Yellow/Orange
const TColor CRGB_GREEN (0x00, 0xFF, 0x00);
const TColor CRGB_BLUE  (0x00, 0x80, 0xFF);   // Azure
const TColor CRGB_PURPLE(0xC0, 0x00, 0xFF);   // Magenta/violet
const TColor CRGB_WHITE (0x80, 0x80, 0x80);
const TColor CRGB_BLACK (0x00, 0x00, 0x00);   // Off

class TColorLed
{
public:

   TColorLed(int NrLeds);
   virtual void Commit() = 0;

   void RGB(int Nr, TColor RGB);
   void RGB(int Nr, uint8_t Red, uint8_t Green, uint8_t Blue);
   void HSV(int Nr, uint8_t Hue, uint8_t Saturation=255, uint8_t Value=255);

   void Clear();
   int  NrLeds() { return _NrLeds; } ;
   uint8_t  Brightness;

protected:
   int      _NrLeds;
   TColor  *Leds;
};

//-----------------------------------------------------------------------------
// From Robotlib: Apa102.h

class TApa102 : public TColorLed
{
public:

   TApa102(int NrLeds, int ClockPin, int DataPin);
   void Init();
   void Commit();

private:

   // SPI interface (software)
   void SPI_writeApa(TColor D);
   void SPI_write(uint8_t    c);

   // SPI IO routines
   int ClockPin, DataPin;
   void SetData(bool Value)   { digitalWrite(DataPin,  Value); }
   void ClockHigh()           { digitalWrite(ClockPin, true ); }
   void ClockLow()            { digitalWrite(ClockPin, false); }
};

#endif //__COLORLED_H

#ifdef MAIN
//-----------------------------------------------------------------------------
// From Robotlib: ColorLed.cpp

//-----------------------------------------------------------------------------
// TColorLed::TColorLed - constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
TColorLed::TColorLed(int InNrLeds) : _NrLeds(InNrLeds)
   {

      Leds = (TColor *)malloc(sizeof(TColor) * (_NrLeds));  // one extra to be sure
      if (Leds == NULL) {
         _NrLeds = 0;
      }

      Brightness = 255;
      Clear();
   }

//-----------------------------------------------------------------------------
// TColorLed::Clear - clear all LEDs
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TColorLed::Clear()
   {
      for (int i=0; i<_NrLeds; i++) Leds[i].Color = 0;
   }

//-----------------------------------------------------------------------------
// TColorLed::RGB - set LED[Nr] to RGB
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TColorLed::RGB(int Nr, uint8_t R, uint8_t G, uint8_t B)
   {
      if ((Nr < 0) || (Nr >= _NrLeds)) return;
      Leds[Nr].RGB(R, G, B);
   }

//-----------------------------------------------------------------------------
// TColorLed::RGB - set LED[Nr] to RGB
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TColorLed::RGB(int Nr, TColor RGB)
   {
      if ((Nr < 0) || (Nr >= _NrLeds)) return;
      Leds[Nr] = RGB;
   }

//-----------------------------------------------------------------------------
// TColorLed::HSV - set LED[Nr] to HSV
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TColorLed::HSV(int Nr, uint8_t Hue, uint8_t Saturation, uint8_t Value)
   {
      if ((Nr < 0) || (Nr >= _NrLeds)) return;
      Leds[Nr].HSV(Hue, Saturation, Value);
   }

//-----------------------------------------------------------------------------
// support routines for ColorHsvToRgb
static float fract(float x) { return x - int(x); }
static float mix(float b, float t) { return 1.0 + (b - 1.0) * t; }
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// TColor::HSV - convert 8 bit H, S and V to TColor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TColor::HSV(uint8_t Hue, uint8_t Saturation, uint8_t Value)
   {
      float h = Hue        / 255.0;
      float s = Saturation / 255.0;

      R = Value * mix(constrain(ABSOLUTE(fract(h + 1.0      ) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
      G = Value * mix(constrain(ABSOLUTE(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
      B = Value * mix(constrain(ABSOLUTE(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
      Dummy = 0;
   }

//-----------------------------------------------------------------------------
// TColor::HSV - convert 8 bit H, S and V to TColor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TColor::RGB(uint8_t Red, uint8_t Green, uint8_t Blue)
   {
      R = Red;
      G = Green;
      B = Blue;
   }

//-----------------------------------------------------------------------------
// From Robotlib: Apa102.cpp

//-----------------------------------------------------------------------------
// TApa102::TApa102 - constructor
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
TApa102::TApa102(int InNrLeds, int InClockPin, int InDataPin) :
   TColorLed(InNrLeds), ClockPin(InClockPin), DataPin(InDataPin)
   {
   }

//-----------------------------------------------------------------------------
// TApa102::Init - init LED bus, setup pins for software SPI
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TApa102::Init()
   {
      pinMode(DataPin,  OUTPUT);
      pinMode(ClockPin, OUTPUT);
      ClockLow(); // initial state of Clock is low
   }

//-----------------------------------------------------------------------------
// TApa102::Commit -
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void TApa102::Commit()
   {
      SPI_writeApa(0);  // Start Frame

      // for each LED
      for (int i=0; i<_NrLeds; i++) {
         TColor D = Leds[i];
         D.Dummy = 0xE0 | (Brightness >> 3);   // Pre-amble & Brightness
         SPI_writeApa(D);
      }

      SPI_writeApa(0);   // Reset frame - Only needed for SK9822, has no effect on APA102

      // End frame: 8+8*(leds >> 4) clock cycles (at least one pulse for every 2 leds)
      for (int i=0; i<=_NrLeds; i+=16) {
         SPI_write(0);  // 8 more clock cycles
      }
   }

//-----------------------------------------------------------------------------
// TApa102::SPI_writeApa - write 32 bits, re-order R en B bytes for Apa102.
//-----------------------------------------------------------------------------
// Software based SPI implementation.
//-----------------------------------------------------------------------------
void TApa102::SPI_writeApa(TColor D)
   {
      SPI_write(D.Dummy);  // Brightness
      SPI_write(D.B);
      SPI_write(D.G);
      SPI_write(D.R);
   }

//-----------------------------------------------------------------------------
// TApa102::SPI_write - write 8 bits.
//-----------------------------------------------------------------------------
// Software based SPI implementation.
//-----------------------------------------------------------------------------
void TApa102::SPI_write(uint8_t c)
   {
      // Assumed state before call: Clock low, Data high
      for (int i=0; i<8; i++) {

         SetData(c&0x80);  // Set Data to next bit
         ClockHigh();      // Data-sample available

         c = c<<1;

         ClockLow();
      }
      // State after call: Clock low, Data high
   }

#endif   // MAIN
