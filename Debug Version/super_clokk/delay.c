// delay.c (for 18.432MHz clock)
// Rick Shear
// Derived from code from mrobbins@mit.edu (http://www.nerdkits.com)

#include <inttypes.h>
#include "delay.h"

// delay_us(...):
// delays a given number of microseconds
inline void delay_us(uint16_t us) {
  //_delay_us(us);
  uint16_t i;
  for(i=0; i<us; i++) {
    NOP;	
    NOP;
    NOP;	
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
  }
}

// delay_ms(...):
// delays a given number of milliseconds
void delay_ms(uint16_t ms) {
  //_delay_ms(ms);
  uint16_t i;
  for(i=0; i<ms; i++)
    delay_us(1000);
}
