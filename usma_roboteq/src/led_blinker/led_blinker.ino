#include "LPD8806.h"
#include "SPI.h" // Comment out this line if using Trinket or Gemma
#ifdef __AVR_ATtiny85__
 #include <avr/power.h>
#endif

// Example to control LPD8806-based RGB LED Modules in a strip

/*****************************************************************************/

// Number of RGB LEDs in strand:
int nLEDs = 32;

// Chose 2 pins for output; can be any valid output pins:
int dataPin  = 2;
int clockPin = 3;

// First parameter is the number of LEDs in the strand.  The LED strips
// are 32 LEDs per meter but you can extend or cut the strip.  Next two
// parameters are SPI data and clock pins:
LPD8806 strip = LPD8806(nLEDs, dataPin, clockPin);

// You can optionally use hardware SPI for faster writes, just leave out
// the data and clock pin parameters.  But this does limit use to very
// specific pins on the Arduino.  For "classic" Arduinos (Uno, Duemilanove,
// etc.), data = pin 11, clock = pin 13.  For Arduino Mega, data = pin 51,
// clock = pin 52.  For 32u4 Breakout Board+ and Teensy, data = pin B2,
// clock = pin B1.  For Leonardo, this can ONLY be done on the ICSP pins.
//LPD8806 strip = LPD8806(nLEDs);

void setup() {
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  // Start up the LED strip
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();
}


void loop() {

  // Send a simple pixel chase in...
  colorSolid(strip.Color(127,   0,   0), 50); // Red
  delay(3000);
  for (int i=0; i<10; i++) {
    
  colorBlink(strip.Color(127,   0,   0), 1000); // Red
  }

}

void rainbow(uint8_t wait) {
  int i, j;
   
  for (j=0; j < 384; j++) {     // 3 cycles of all 384 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel( (i + j) % 384));
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  
  for (j=0; j < 384 * 5; j++) {     // 5 cycles of all 384 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 384-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 384 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 384 / strip.numPixels()) + j) % 384) );
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}



// Chase one dot down the full strip.
void colorSolid(uint32_t c, uint8_t wait) {
    strip.setPixelColor(0, c); // Set new pixel 'on'
    strip.setPixelColor(1, c);
    strip.setPixelColor(2, c);
    strip.setPixelColor(3, c);
    strip.setPixelColor(4, c);
    strip.setPixelColor(5, c);
    strip.setPixelColor(6, c);
    strip.setPixelColor(7, c);
    strip.setPixelColor(8, c);
    strip.setPixelColor(9, c); // Set new pixel 'on'
    strip.setPixelColor(10, c);
    strip.setPixelColor(11, c);
    strip.setPixelColor(12, c);
    strip.setPixelColor(13, c);
    strip.setPixelColor(13, c);
    strip.setPixelColor(14, c);
    strip.setPixelColor(15, c);
    strip.setPixelColor(16, c);
    strip.setPixelColor(17, c); // Set new pixel 'on'
    strip.setPixelColor(18, c);
    strip.setPixelColor(19, c);
    strip.setPixelColor(20, c);
    strip.setPixelColor(21, c);
    strip.setPixelColor(22, c);
    strip.setPixelColor(23, c);
    strip.setPixelColor(24, c);
    strip.setPixelColor(25, c);
    strip.setPixelColor(26, c); // Set new pixel 'on'
    strip.setPixelColor(27, c);
    strip.setPixelColor(28, c);
    strip.setPixelColor(29, c);
    strip.setPixelColor(30, c);
    strip.setPixelColor(31, c);
    strip.setPixelColor(32, c);

    strip.show();     // Refresh LED states
  
}

void colorBlink(uint32_t c, uint8_t wait) {
  int i;

  // Start by turning all pixels off:
  //for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);
  
  // Then display one pixel at a time:
  //for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(0, c); // Set new pixel 'on'
    strip.setPixelColor(1, c);
    strip.setPixelColor(2, c);
    strip.setPixelColor(3, c);
    strip.setPixelColor(4, c);
    strip.setPixelColor(5, c);
    strip.setPixelColor(6, c);
    strip.setPixelColor(7, c);
    strip.setPixelColor(8, c);
    strip.setPixelColor(9, c); // Set new pixel 'on'
    strip.setPixelColor(10, c);
    strip.setPixelColor(11, c);
    strip.setPixelColor(12, c);
    strip.setPixelColor(13, c);
    strip.setPixelColor(13, c);
    strip.setPixelColor(14, c);
    strip.setPixelColor(15, c);
    strip.setPixelColor(16, c);
    strip.setPixelColor(17, c); // Set new pixel 'on'
    strip.setPixelColor(18, c);
    strip.setPixelColor(19, c);
    strip.setPixelColor(20, c);
    strip.setPixelColor(21, c);
    strip.setPixelColor(22, c);
    strip.setPixelColor(23, c);
    strip.setPixelColor(24, c);
    strip.setPixelColor(25, c);
    strip.setPixelColor(26, c); // Set new pixel 'on'
    strip.setPixelColor(27, c);
    strip.setPixelColor(28, c);
    strip.setPixelColor(29, c);
    strip.setPixelColor(30, c);
    strip.setPixelColor(31, c);
    strip.setPixelColor(32, c);

    strip.show();     // Refresh LED states
    delay(wait);
    strip.setPixelColor(0, 0); // Erase pixel, but don't refresh!
    strip.setPixelColor(1, 0);
    strip.setPixelColor(2, 0);
    strip.setPixelColor(3, 0);
    strip.setPixelColor(4, 0);
    strip.setPixelColor(5, 0);
    strip.setPixelColor(6, 0);
    strip.setPixelColor(7, 0);
    strip.setPixelColor(8, 0);
    strip.setPixelColor(9, 0);
    strip.setPixelColor(10, 0);
    strip.setPixelColor(11, 0); // Erase pixel, but don't refresh!
    strip.setPixelColor(12, 0);
    strip.setPixelColor(13, 0);
    strip.setPixelColor(14, 0);
    strip.setPixelColor(15, 0);
    strip.setPixelColor(16, 0);
    strip.setPixelColor(17, 0);
    strip.setPixelColor(18, 0);
    strip.setPixelColor(19, 0);
    strip.setPixelColor(20, 0);
    strip.setPixelColor(21, 0);
    strip.setPixelColor(22, 0);
    strip.setPixelColor(23, 0);
    strip.setPixelColor(24, 0);
    strip.setPixelColor(25, 0);
    strip.setPixelColor(26, 0);
    strip.setPixelColor(27, 0);
    strip.setPixelColor(28, 0);
    strip.setPixelColor(29, 0);
    strip.setPixelColor(30, 0);
    strip.setPixelColor(31, 0);
    strip.setPixelColor(32, 0);
    strip.show();   
    delay(wait);

  //}

  strip.show(); // Refresh to turn off last pixel
}




/* Helper functions */

//Input a value 0 to 384 to get a color value.
//The colours are a transition r - g -b - back to r

uint32_t Wheel(uint16_t WheelPos)
{
  byte r, g, b;
  switch(WheelPos / 128)
  {
    case 0:
      r = 127 - WheelPos % 128;   //Red down
      g = WheelPos % 128;      // Green up
      b = 0;                  //blue off
      break; 
    case 1:
      g = 127 - WheelPos % 128;  //green down
      b = WheelPos % 128;      //blue up
      r = 0;                  //red off
      break; 
    case 2:
      b = 127 - WheelPos % 128;  //blue down 
      r = WheelPos % 128;      //red up
      g = 0;                  //green off
      break; 
  }
  return(strip.Color(r,g,b));
}
