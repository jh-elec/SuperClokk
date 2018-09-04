// ht1632c
// Basic HT1632C driver library for
// JY-MCU 3208 Display
// Rick Shear


#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <inttypes.h>

#include "ht1632c.h"

void ht1632c_init(uint8_t bright)  // Initialization for HT1632C
{
	csreg	|= (1 << cspin);
	csport	|= (1 << cspin);
	
	wrreg	|= (1 << wrpin);
	dtreg	|= (1 << dtpin);
		
	uint8_t cnfg[] =
	{
		0x01,	// System aktivieren
		0x03,	// LED�s einschalten
		0x08,	// Blinken ausschalten
		0x18,	// 1632C Master Mode
		0x20,	// N-MOS Open Drain und 32 Reihen x 8 Spalten
		bright,	// Helligkeit
	};
	
	for ( uint8_t i = 0 ; i < sizeof(cnfg) / sizeof(cnfg[0]) ; i++ )
	{
		ht1632cSendCmd( cnfg[i] );
	}
}

void ht1632c_chipselect()
{
	csport &= ~(1<< cspin);
}

void ht1632c_chipfree()
{
	csport |= (1<< cspin);
}

void ht1632c_send_to_display (uint8_t data_to_send, uint8_t dtlen)
{
    while (dtlen) 
	{
		wrport &= ~(1 << wrpin);
		if (data_to_send & dtlen)
		{
			dtport |= (1 << dtpin);
		}
		else
		{
			dtport &= ~(1 << dtpin);
		}
		wrport |= (1 << wrpin);
		dtlen >>= 1;
    }
}

void ht1632c_send_page( void )
{
  	uint8_t addr;
	ht1632c_chipselect();
	ht1632c_send_to_display( 0b101 , 1<<2 );
	ht1632c_send_to_display( 0 , 1<<6 );
	for( addr = 0 ; addr < 64 ; addr++ )
	{
		ht1632c_send_to_display( ht1632_shadowram[addr] , 1<<3 );
	}
	ht1632c_chipfree();
}

void ht1632cSendCmd(uint8_t command)
{
    ht1632c_chipselect();  // Select chip
    ht1632c_send_to_display(0b100 ,1<<2);  // Send 3 bits to identify as a command
    ht1632c_send_to_display(command, 1<<7);  // send the command data
    ht1632c_send_to_display(1, 1);     // send a final bit to complete the sequence (doesn't matter if 0 or 1)
    ht1632c_chipfree(); // Release chip
}

void ht1632c_send_data(uint8_t address, uint8_t data)
{
    ht1632c_chipselect();  // Select chip
    ht1632c_send_to_display(0b101, 1<<2);  // send 3 bits to identify as data write
    ht1632c_send_to_display(address, 1<<6); // Send memory address
    ht1632c_send_to_display(data, 1<<3); // send nibble of data
    ht1632c_chipfree(); // Release chip
}

uint8_t ht1632c_get(uint8_t x, uint8_t y)
{
    uint8_t bitval;
    int addr;

    if ((x < 0) || (y < 0) || (x > (COLS-1)) || (y > (ROWS-1))) 
	{
      return 0;
    }
 
    bitval = 8>>(x&3);  // compute which bit will need set (Will equal 1, 2, 4, or 8)
    addr = ((x>>3)<<4)+(y<<1)+((x&4)>>2);  // compute which memory word this is in
  
    if((ht1632_shadowram[addr] & bitval) != 0) 
	{
      return 1;
    } 
	else 
	{
      return 0;
    }
 }

void ht1632c_set(uint8_t x, uint8_t y, uint8_t val)
{
    uint8_t bitval;
    int addr;

    if ((x < 0) || (y < 0) || (x > 31) || (y > 7)) 
	{
      return;
    }
    bitval = 8>>(x&3);  // compute which bit will need set (Will equal 1, 2, 4, or 8)
    addr = ((x>>3)<<4)+(y<<1)+((x&4)>>2);  // compute which memory word this is in
  
    if (val) // Modify the shadow memory
	{  
      ht1632_shadowram[addr] |= bitval;
    }
    else 
	{
      ht1632_shadowram[addr] &= ~bitval;
    }
 
}
