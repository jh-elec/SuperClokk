/*				***SUPER_CLOKK_2015***
*
*		- DCF77 Signal Receive Clock.
*		- LED MATRIX ( 8 x 8 x 4 ).
*		- VIBRATION MOTOR for Alarm.
*		- SOUND for Alarm.
*		- Actual Temperature (from BMP180) Range : -40 ... +85 °C 
*		- Actual Pressure    (from BMP180) Range : 300 ... 1100hPa 
*		  HARDWARE DESCRRIPTION.:
*			
*			->
*		 
*		 
*		Power @ 5,2 VDC.: 
*							Bright 1  = 15 mA  
*							Bright 16 = 115 mA
*/				



/* setting for the extern Crystal */
#define F_CPU							16000000UL

/* minutes for signal timeout */
#define DCF77_TIMEOUT_TIME				5

/* speed for scroll the info text @ display */
#define INFO_SCROLL_SPEED				20

/* speed for scroll the "Save" information @ display */
#define SCROLL_SAVE_SPEED				10

/* speed for scroll the "temperature" & "date" & "pressure" @ display */
#define TEMP_PRESS_DATE_SCROLL_SPEED	20

#define BLINK_INTERVALL_MS				400

#define ALERTS							8

/* switches */
#define SWITCH_MENUE_PRESSED		(!(PINC & (1<<PC2))) 
#define SWITCH_MENUE_RELEASED		(PINC & (1<<PC2))

#define SWITCH_EXIT_PRESSED			(!(PINC & (1<<PC3)))	
#define SWITCH_EXIT_RELEASED		(PINC & (1<<PC3))

#define SWITCH_SET_DOWN_PRESSED		(!(PINC & (1<<PC4)))
#define SWITCH_SET_DOWN_RELEASED	(PINC & (1<<PC4))

#define SWITCH_SET_UP_PRESSED		(!(PINC & (1<<PC5)))
#define SWITCH_SET_UP_RELEASED		(PINC & (1<<PC5))

#define SWITCH_ENTER_PRESSED		(!(PINC & (1<<PC6)))
#define SWITCH_ENTER_RELEASED		(PINC & (1<<PC6))

#define ENCODE_A					(!(PIND & (1<<PD7)))
#define ENCODE_B					(!(PIND & (1<<PD6)))			
#define ENCODE_SWITCH				(!(PIND & (1<<PD5)))

/* switches for testboard (jy-mcu) */
#define SWITCH_ONE_PRESSED			(!(PIND & (1<<PD7)))
#define SWITCH_ONE_RELEASED			((PIND & (1<<PD7))

#define SWITCH_TWO_PRESSED			(!(PIND & (1<<PD6)))			
#define SWITCH_TWO_RELEASED			(PIND & (1<<PD6))

#define SWITCH_THREE_PRESSED		(!(PIND & (1<<PD5)))
#define SWITCH_THREE_RELEASED		(PIND & (1<<PD5))

/*-----------------------------------------------------------------*/

/* alarm signals */
#define VIBRATION_MOTOR_ON			PORTB |=  (1<<PB3)
#define VIBRATION_MOTOR_OFF			PORTB &= ~(1<<PB3)
#define VIBRATION_MOTOR_TOGGLE		PORTB ^=  (1<<PB3)

#define ALARM_SOUND_ON				PORTB |=  (1<<PB4)
#define ALARM_SOUND_OFF				PORTB &= ~(1<<PB4)

/*-----------------------------------------------------------------*/

/* live the CPU? */
#define HEARTBEAT_LED_ON			PORTD &=  ~(1<<PD4)
#define HEARTBEAT_LED_OFF			PORTD |=   (1<<PD4)
#define HEARTBEAT_LED_TOGGLE		PORTD ^=   (1<<PD4)

/*-----------------------------------------------------------------*/

/* power supply good? */
#define LD39050_POWER_GOOD			(!(PINA & (1<<PA6))	

/*-----------------------------------------------------------------*/			

/* control pins for the Matrix Driver (HT1632C) */
#define HT1632C_CS					PORTA |=  (1<<PA5)
#define HT1632C_RD					PORTA |=  (1<<PA4)
#define HT1632C_WR					PORTA |=  (1<<PA3)
#define HT1632C_DATA				PORTA |=  (1<<PA2)
#define HT1632C_SYNC				PORTA |=  (1<<PA1)
#define HT1632C_OSC					PORTA |=  (1<<PA0)

/*-----------------------------------------------------------------*/

/* all leds off */
#define MATRIX_LEDS_OFF				ht1632cSendCmd(0x02)

/*-----------------------------------------------------------------*/



/* header files */
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdlib.h>


#include "Super_Clokk.h"
#include "delay.h"
#include "i2cmaster.h"
#include "dcf.h"
#include "ht1632c.h"
#include "5x8font.h"
#include "RX8564.h"
#include "BMP180.h"
#include "build_info.h"
#include "convert.h"
#include "bytes_inf.h"
										
 typedef enum
 {
	 ALERT_ONE,
	 ALERT_TWO,
	 ALERT_THREE,
	 ALERT_FOUR,
	 ALERT_FIVE,
	 ALERT_SIX,
	 ALERT_SEVEN,
	 ALERT_EIGHT,
	 
	 ALERT_MAX
 }alert_e;
 
rx8564_t rx8564; 
 
typedef struct
{	
	uint8_t		byte8[ MAX_8_BYTE_ENTRYS ];	
}var_t; var_t eep EEMEM; var_t ram;

typedef struct  
{	
	uint16_t motorEnable		:1;
	uint16_t soundEnable		:1;
	uint16_t alertEnable		:1;
	uint16_t menueOpen			:1;
	uint16_t menueExit			:1;
	uint16_t isDimm				:1;
	uint16_t isInit				:1;
}flag_t; flag_t flag;

typedef struct  
{	
	uint16_t	byte16[ MAX_BYTE16_ENTRYS ];
}error_t; error_t err; error_t erreep EEMEM;

typedef struct  
{
	alert_e name;	
	uint8_t enable;
	uint8_t ring;
}alert_t; alert_t alert;

typedef struct  
{	
	uint8_t menue	:1;
	uint8_t exit	:1;
	uint8_t down	:1;
	uint8_t up		:1;
	uint8_t enter	:1;
}button_t;

typedef union	
{
	button_t pressed;
	uint8_t all;
}button_u; volatile button_u button;

typedef struct  
{
	uint8_t		syncMinuteCntDCF77,
				syncHourCntDCF77,
				autoChangeSec,
				autoChangeMin,
				alertCycles,
				alertNewHour,
				autoChangeOld, 
				autoChangeNew,
				key_state,
				key_rpt,
				key_press,
				cmpNewHour,
				cmpOldHour;
	
	uint16_t   blinkCnt , 
			   menueTimeout;
	
}system_t; volatile system_t sys;


uint8_t reboot(void)
{
	scroll_display( "Reboot now.." , INFO_SCROLL_SPEED );
	
	wdt_enable( WDTO_15MS ); 
	while(1){};	
	
// 	void (*swReset)(void) = 0;
// 	swReset();
	
	return 0;
}

void scrollDate					( rx8564_t *d, uint8_t speed)
{	
	char date[60] = "";
	
	/* BCD to ASCII */
	date[0] = (d->day >> 4)  + 0x30; // ten
	date[1] = (d->day & 0x0f) + 0x30; // one
	date[2] = '.'; // point
	date[3] = (d->month >> 4)  + 0x30; // ten
	date[4] = (d->month & 0x0f) + 0x30; // one 
	date[5] = '.'; // point
	date[6] = '2'; // 2
	date[7] = '0'; // 0
	date[8] = (d->year >> 4)  + 0x30; // ten
	date[9] = (d->year & 0x0f) + 0x30; // one
	date[10] = '.'; // space
		
	switch( time.stime )
	{
		/* Winterzeit */
		case 0:
		{
			strcat( &date[11] , "MEZ  ");
		}break;	
			
		/* Sommerzeit */	
		case 1:
		{
			strcat( &date[11] , "MESZ ");
		}break;
	
	}// end switch

	switch (d->dayName)
	{
		case 0:
		{
			strcat( &date[16] ,"Sonntag");
		}break;
		
		case 1:
		{
			strcat( &date[16] ,"Montag");
		}break;
		
			
		case 3:
		{
			strcat( &date[16] ,"Mittwoch");
		}break;		
		
		case 4:
		{
			strcat( &date[16] ,"Donnerstag");
		}break;
		
		case 5:
		{
			strcat( &date[16] ,"Freitag");
		}break;

		case 6:
		{
			strcat( &date[16] ,"Samstag");
		}break;
		
	}// end switch
	
	char tmp[] = "-----";
	strcat( date , "     KW: " );
	strcat( date , itoa( rtcGetWeek( rtcBcdToDec( d->day ) , rtcBcdToDec( d->month ) , rtcBcdToDec( d->year ) ) , tmp , 10 ) );
	
	/* shift the new data over the display */
	scroll_display(date, speed);
	
	clearDisplay( true , false );

}

#ifdef _DEBUG
void ScrollDebugMsg( Dcf77Debug_t *Object )
{
	char Msg[350] = "";
	char Tmp[10] = "";
	
	strcpy( Msg , "<Start " );
	strcat( Msg , ultoa( Object->Average[DEBUG_DCF77_START_TIME].Minimum , Tmp , 10 ));
	strcat( Msg , "-" );
	strcat( Msg , ultoa( Object->Average[DEBUG_DCF77_START_TIME].Maximum , Tmp , 10 ));
	strcat( Msg , "-" );
	strcat( Msg , "n=" );
	strcat( Msg , itoa( Object->Average[DEBUG_DCF77_START_TIME].nBits , Tmp , 10 ) );
	strcat( Msg , "<" );
	
	strcat( Msg , " >Low " );
	strcat( Msg , ultoa( Object->Average[DEBUG_DCF77_LOW_TIME].Minimum , Tmp , 10 ));
	strcat( Msg , "-" );
	strcat( Msg , ultoa( Object->Average[DEBUG_DCF77_LOW_TIME].Maximum , Tmp , 10 ));
	strcat( Msg , "-" );
	strcat( Msg , "n=" );
	strcat( Msg , itoa( Object->Average[DEBUG_DCF77_LOW_TIME].nBits , Tmp , 10 ) );
	strcat( Msg , "<" );

	strcat( Msg , " >High " );
	strcat( Msg , ultoa( Object->Average[DEBUG_DCF77_HIGH_TIME].Minimum , Tmp , 10 ));
	strcat( Msg , "-" );
	strcat( Msg , ultoa( Object->Average[DEBUG_DCF77_HIGH_TIME].Maximum , Tmp , 10 ));	
	strcat( Msg , "-" );
	strcat( Msg , "n=" );
	strcat( Msg , itoa( Object->Average[DEBUG_DCF77_HIGH_TIME].nBits , Tmp , 10 ) );
	strcat( Msg , "<" );
		
	/* shift the new data over the display */
	scroll_display( Msg , 40 );
	
	clearDisplay( true , false );	
}
#endif

void putChar_ud					(char c, uint8_t offset, uint8_t up) 
{
	char buff[7];

	getFont(c, buff);
	
	uint8_t width = buff[1];
	uint8_t y, x, s,n;
	
	if(up)
	{
		for( y = 0 ; y < ROWS ; y++ ) 
		{
			n = 0;
			s = ( 5 - y );
			while( s < ROWS )
			{
				for( x = 0 ; x < width ; x++ ) 
				{
					if( ( offset + x ) < COLS ) 
					{
						if( ( buff[ 2 + x ] & ( 1<<( ( ROWS - 1 ) -n ) ) ) != 0 ) 
						{
							ht1632c_set(offset + x,s,1);
						} 
						else 
						{
							ht1632c_set(offset + x,s,0);
						}
					}
				}
				delay_ms(4);
				n++;
				s++;
			}
		}
	} 
	else
	{
		for( y = 0 ; y < ROWS ; y++ ) 
		{
			n = ( ROWS - 1 ) -y;
			s = 0;
			while( s <= y )
			{
				for( x = 0 ; x < width ; x++ ) 
				{
					if( ( offset + x ) < COLS )
					{
						if( ( buff[ 2 + x ] & ( 1<<( ( ROWS - 1 ) -n ) ) ) != 0 ) 
						{
							ht1632c_set(offset + x,s,1);
						} 
						else 
						{
							ht1632c_set(offset + x,s,0);
						}
					}
				}
				delay_ms(4);
				n++;
				s++;
			}
		}
	}
	
	// blank the next column to the right
	for( y = 0 ; y < ROWS ; y++ )  
	{
		ht1632c_set(offset+width,y, 0);
	}
}

void putTimertcBcdToDec			(uint8_t hou, uint8_t min)
{
	uint8_t hour	= rtcBcdToDec( hou );
	uint8_t minute	= rtcBcdToDec( min );
	
	putChar( ( ( hour / 10   ) + '0' ) , 1		); // show hour (ten)
	putChar( ( ( hour % 10   ) + '0' ) , 7		); // show hour (one)
	putChar( ( ( minute / 10 ) + '0' ) , 19		); // show minute (ten)
	putChar( ( ( minute % 10 ) + '0' ) , 25		); // show minute (one)		
}

void putTimeDec					( uint8_t hou , uint8_t min)
{
	putChar( ( ( hou / 10   ) + '0' ) , 1		); // show hour (ten)
	putChar( ( ( hou % 10   ) + '0' ) , 7		); // show hour (one)
	putChar( ( ( min / 10 ) + '0' ) , 19		); // show minute (ten)
	putChar( ( ( min % 10 ) + '0' ) , 25		); // show minute (one)	
}

void ledarray_flash				(uint16_t rate,uint8_t repeats) 
{
	uint8_t y, x,t;

	for(t=0;t<repeats;t++){
		for(y=0;y<ROWS;y++) {
			for(x=0;x<COLS;x++) {
				ht1632c_set(x,y, 1 - ht1632c_get(x,y));
			}
		}
		delay_ms(rate/4);

		for(y=0;y<ROWS;y++) {
			for(x=0;x<COLS;x++) {
				ht1632c_set(x,y, 1 - ht1632c_get(x,y));
			}
		}
		delay_ms(rate);
	}
}

uint8_t cnfgTime_				( uint8_t *buff );

uint8_t scroll_display			(const char *s,uint8_t speed) 
{
	ledarray_blank();
	
	uint8_t exit_ = 0;
	int8_t	offset = 0 , next_offset = 0;
	uint8_t is_started = 0;
	uint8_t y = 0;
	char	x = ' ';
	
	if( speed==0 )
	{
		speed = 90;
	}

	while( ( *s != 0x00 ) && ( exit_ != 1 ) )
	{		
		sys.autoChangeSec = 0;
	
		if ( button.pressed.exit )
		{
			button.all = 0;
			exit_ = 1;
		}
	
		if( is_started ) 
		{
			while( next_offset > ( COLS - 1 ) )
			{
				delay_ms( speed );
				ledarray_left_shift();
			
				if( next_offset > 0 ) 
				{
					offset -= 1;
					next_offset -= 1;
				}
				
				putChar( x , offset );
			}
		}	
		else 
		{
			offset = COLS-1;
		}
		
		x = (char)*s;
		s++;
		
		if( is_started )
		{
			offset = next_offset;
		}
		
		putChar(x, offset);
		next_offset = offset + getFontWidth(x)+1;
		is_started = 1;
	}
	
	while( ( next_offset >  (COLS - 1 ) ) && ( exit_ != 1 ) )
	{
		
		if ( button.pressed.exit )
		{
			button.all = 0;
			exit_ = 1;
		}
		
		delay_ms(speed);
		ledarray_left_shift();
		
		if( next_offset > 0 ) 
		{
			offset -= 1;
			next_offset -= 1;
		}
		
		putChar(x, offset);
	}
	
	delay_ms(speed);
	
	for( y = 0 ; ( y < COLS ) && ( exit_ != 1 ) ; y++ )
	{
		ledarray_left_shift();
		delay_ms(speed);
	}
	
	sys.menueTimeout	= 0;
	flag.menueExit	= 0;
	
	if ( exit_ )
	{
		return 1;
	}
	
	return 0;
}

void ledarray_right_shift		( void )
{
	// shift everything right one position
	uint8_t y, x;
	for(y=0; y<ROWS; y++) 
	{
		for(x=COLS-1; x>0; x--) 
		{
			ht1632c_set(x,y, ht1632c_get(x-1, y));
		}
		ht1632c_set(0,y,0);
	}
}

void ledarray_shift_down		( void )
{
	// shift everything down one position
	uint8_t y,x;
	for(y=(ROWS-1);y>0;y--){
		for(x=0;x<COLS;x++){
			ht1632c_set(x,y,ht1632c_get(x,y-1));
		}
	}
	// blank top line to prevent 'smearing'
	for(x=0;x<COLS;x++)
	{
		ht1632c_set(x,0,0);
	}
}

void ledarray_left_shift		( void ) 
{
	// shift everything one position left
	uint8_t y, x;
	for(y=0; y<ROWS; y++) 
	{
		for(x=0; x<COLS-1; x++) 
		{
			ht1632c_set(x,y, ht1632c_get(x+1,y));
		}
		ht1632c_set((COLS-1),y,0);
	}
}

void ledarray_shift_up			( void )
{
	// shift everything up one position
	uint8_t y,x;
	for(y=0;y<ROWS-1;y++)
	{
		for(x=0;x<COLS;x++)
		{
			ht1632c_set(x,y,ht1632c_get(x,y+1));
		}
	}
	// blank bottom line to prevent 'smearing'
	for(x=0;x<COLS;x++)
	{
		ht1632c_set(x,(ROWS-1),0);
	}
}

void ledarray_twinkle			( void ) 
{
	uint8_t y, x, ct;
	ledarray_blank();
	// set initital pattern (every other LED on)
	for(y=0;y<ROWS;y++) {
		for(x=y%2;x<COLS;x += 2) {
			ht1632c_set(x,y, 1 - ht1632c_get(x,y));
		}
	}
	//wait 50ms
	delay_ms(50);
	
	// toggle the pattern turning what was on off then back on again 10 times
	for(ct=0;ct<10;ct++){
		for(y=0;y<ROWS;y++) {
			for(x=0;x<COLS;x++) {
				ht1632c_set(x,y, 1 - ht1632c_get(x,y));
			}
		}
		delay_ms(50);
		for(y=0;y<ROWS;y++) {
			for(x=0;x<COLS;x++){
				ht1632c_set(x,y, 1 - ht1632c_get(x,y));
			}
		}
		delay_ms(50);
	}
}

uint8_t getFontWidth			( char c )
{
	char buff[7] = "";

	getFont( c , buff );
	
	return buff[1];
}

void getFont					( char match, char *buff )
{
	// copies the character "match" into the bufffer
	uint8_t y;
	PGM_P p;
	
	for( y = 0 ; y < FONT_SIZE ; y++)
	{
		memcpy_P( &p , &font[y] , sizeof( PGM_P ) );
		
		if(memcmp_P( &match , p , 1 ) == 0 )
		{
			memcpy_P(buff, p, 7);
			
			return;
		}
	}
	
	getFont('?', buff);
	
	return;
}

void putChar					( char c , uint8_t offset ) 
{
	char buff[7];
	getFont(c, buff);

	uint8_t width = buff[1];
	uint8_t y, x;
	for(y=0; y<ROWS; y++) 
	{
		for(x=0; x<width; x++) 
		{
			if((offset + x) < COLS) 
			{
				if( (buff[2+x] & (1<<((ROWS-1)-y))) != 0) 
				{
					ht1632c_set(offset + x,y,1);
				} 
				else 
				{
					ht1632c_set(offset + x,y,0);
				}
			}
		}
	}

	for(y=0; y<ROWS; y++) 
	{
		ht1632c_set(offset+width,y, 0);
	}
}

void putStr						( char *str , uint8_t offset )
{
	uint8_t width = 0;
	
	putChar( *str , offset );
	width = getFontWidth( *str++ ) + 1;
	
	while( *str )
	{
		putChar( *str , offset + width );
		width += getFontWidth( *str++ ) + 1;	
	}
}

void ledarray_testpattern		( uint8_t rate ) 
{
	uint8_t y, x;
	ledarray_blank();
	
	for(y=0;y<ROWS;y++) {
		for(x=0;x<COLS;x++) {
			ht1632c_set(x,y, 1 - ht1632c_get(x,y));
			delay_ms(rate);
		}
	}

	for(y=0;y<ROWS;y++) {
		for(x=0;x<COLS;x++) {
			ht1632c_set(x,y, 1 - ht1632c_get(x,y));
			delay_ms(rate);
		}
	}
}

void doublePointOff				( void )
{
	putChar(' ',15);
}

void doublePointOn				( void )
{
	putChar(':',15);
}

void switch_debounce			( void )
{
	static uint8_t ct0, ct1, rpt;
	uint8_t i;
	
	i =  sys.key_state ^ ~PINC;					// key changed ?
	ct0 = ~( ct0 & i );						// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);					// reset or count ct1
	i &= ct0 & ct1;							// count until roll over ?
	sys.key_state ^= i;							// then toggle debounced state
	sys.key_press |= sys.key_state & i;				// 0->1: key press detect
	
	if( (sys.key_state & (0b01111100)) == 0 )	// check repeat function
	rpt = 50;								// start delay
	if(--rpt == 0 )
	{
		rpt = 15;                            // repeat delay
		sys.key_rpt |= sys.key_state & (0b01111100);
	}
	
	if ( get_key_press( 1<<PC2 ) )
	{
		button.pressed.menue = 1;	// Menü
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}	
	
	if ( get_key_press( 1<<PC3 ) )
	{
		button.pressed.exit = 1;	// Exit
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}
	
	if ( get_key_press( 1<<PC4 ) )
	{
		button.pressed.down = 1;
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}
	
	if (get_key_rpt(1<<PC4))
	{
		button.pressed.down = 1;
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}
	
	if (get_key_rpt(1<<PC5))
	{
		button.pressed.up = 1;
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}
	
	if (get_key_press(1<<PC5))
	{
		button.pressed.up = 1;
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}
	
	if (get_key_press(1<<PC6))// Enter
	{
		button.pressed.enter = 1;
		sys.menueTimeout = 0;
		sys.blinkCnt = 0;
	}
	
}	

void blinkTime					( uint8_t hh , uint8_t mm , uint8_t pos )
{
	if ( ( sys.blinkCnt < BLINK_INTERVALL_MS ))
	{
		putTimeDec( hh , mm );
	}
	else if ( ( sys.blinkCnt > BLINK_INTERVALL_MS ) && ( sys.blinkCnt < BLINK_INTERVALL_MS * 2 ) )
	{
		switch( pos )
		{
			case 0:
			{
				putChar( '`' , 1 );
				putChar( '`' , 7 );
			}break;
			
			case 1:
			{
				putChar('`',19);
				putChar('`',25);
			}break;
		}

	}
	else if ( sys.blinkCnt > BLINK_INTERVALL_MS * 2 )
	{
		sys.blinkCnt = 0;
	}
}

void blinkChars( char *str , uint8_t pos )
{	
	if ( ( sys.blinkCnt < BLINK_INTERVALL_MS ))
	{
		putStr( str , pos );
	}
	else if ( ( sys.blinkCnt > BLINK_INTERVALL_MS ) && ( sys.blinkCnt < BLINK_INTERVALL_MS * 2 ) )
	{
		for (uint8_t i = 0 ; i < strlen(str) ; i++ )
		{
			putChar( '`' , pos + (i * 6) );
		}
	}
	else if ( sys.blinkCnt > BLINK_INTERVALL_MS * 2 )
	{
		sys.blinkCnt = 0;
	}	
}

void blink2Digit				( uint8_t val , uint8_t pos )
{
	if ( ( sys.blinkCnt < BLINK_INTERVALL_MS ))
	{
		uint8ToDisp( val , pos );
	}
	else if ( ( sys.blinkCnt > BLINK_INTERVALL_MS ) && ( sys.blinkCnt < BLINK_INTERVALL_MS * 2 ) )
	{
		putChar( '`' , pos );
		putChar( '`' , pos + 6 );
	}
	else if ( sys.blinkCnt > BLINK_INTERVALL_MS * 2 )
	{
		sys.blinkCnt = 0;
	}	
}

void ledarray_blank				( void ) 
{
	uint8_t j;
	for(j=0;j<((COLS*ROWS)/4);j++)
	{
		ht1632_shadowram[j]=0x00;
	}
	
}

void clearDisplay				( bool new, bool old )
{
	bool sync_state = 0, sync_state_old = 0;
	
	sync_state = new;
	sync_state_old = old;
	
	/*clear the old Charakter from Display */
	if (sync_state != sync_state_old)
	{
		ledarray_blank();
		sync_state = sync_state_old;
	}
}

void scrollTemperature			( uint8_t speed )
{
	char bufff[35] = "";

	cli();
	bmpGetTemperaure(cal , &cal.temperature);
	sei();
		
	/* signed " + " */
 	if (cal.temperature > 0){strcpy(bufff, "Aktuelle Temperatur : +");}
 
 	/* signed " - " */
 	if (cal.temperature < 0){strcpy(bufff, "Aktuelle Temperatur : -");; cal.temperature  *= (-1);}	
			
	sprintf(bufff, "Aktuelle Temperatur: %ld.%ld °C", cal.temperature / 10 , cal.temperature % 10 );
			
	scroll_display(bufff, speed);
	
	clearDisplay( true , false );	
}

void scrollPressure				( uint8_t speed )
{
	char buff[35]		= "";
	
	cli();
 	bmp180GetPressure( &cal , 3 );
	sei();
	
	cal.pressure = ( cal.pressure * 0.01 );		
		
	strcpy( buff , "Aktueller Luftdruck : ");
	itoa( cal.pressure , buff + strlen( buff ) , 10 );
	strcat( buff , " hPa");		

	scroll_display(buff, speed);
	
	clearDisplay( true , false );
}

uint8_t incDec					( uint8_t value, uint8_t valueMax )
 {	 
	if ( ++value > valueMax )
	{
		value = 0;
	}
	return value;
 }
 
uint8_t decDec					( uint8_t value, uint8_t valueMin )
{	
	if ( (int8_t)--value < 0 )
	{
		value = valueMin;
	}
	return value;
}

void uint8ToDisp				( uint8_t value , uint8_t offset)
{	
	putChar( ( value / 10 ) + '0' , offset);
	putChar( ( value % 10 ) + '0' , offset + 6 );
}

void uint16ToDisp				( uint16_t value , uint8_t offset )
{
	char temp[5];
	
	temp[0] = ((value / 10000) % 10) + '0';
	temp[1] = ((value / 1000)  % 10) + '0';
	temp[2] = ((value / 100)   % 10) + '0';
	temp[3] = ((value / 10)    % 10) + '0';
	temp[4] = (value  % 10)			 + '0';

	putChar(temp[0],offset + 0);
	putChar(temp[1],offset + 6);
	putChar(temp[2],offset + 12);
	putChar(temp[3],offset + 18);	
	putChar(temp[4],offset + 24);	
}

void setPwm						( uint8_t bright )
{	
	cli();
	ht1632cSendCmd(0x03);		// LED On
	ht1632cSendCmd(0x08);		// Blinking off
	ht1632cSendCmd(bright);	// Set PWM to 1/4 intensity (can be A0 Thru AF)
	sei();
}

void compareTime				( void )
{	
	/*
	*	Ca. jede 500ms wird die Uhrzeit abgeholt
	*/
	rtcGetData( &rx8564 );
	
	/*
	*	Sollte eine neue Stunde sein, wird dies vermerkt
	*/
	if ( sys.cmpOldHour != rx8564.hour )
	{
		if ( sys.cmpNewHour++ >= 5 )
		{
			sys.cmpOldHour = rx8564.hour;
			sys.alertNewHour = 1;
		}
	}
	
	/*
	*	Uhrzeit umrechnen und zwischen speichern
	*	Damit nicht unnötige Funktionsaufrufe von rtcBcdToDec() entstehen beim vergleichen
	*/
	uint8_t hour	= rtcBcdToDec(rx8564.hour);
	uint8_t minute	= rtcBcdToDec(rx8564.minute);
	
	/*
	*	Der aktuelle Alarm wird nach n Minuten wieder frei gegeben
	*	Das soll verhindern das der Alarm innerhalb einer abfrage mehrmals zuschlagen kann
	*/	
	#define RELAESE_ALARM_MIN			10
	
	static uint8_t	alarmReleaseMin		= 0;
	static uint8_t	alarmReleaseMinOld	= 0;
	static uint8_t	alarmReleaseMinCnt	= 0;
	static uint16_t lastAlarm			= 0;
	
	alarmReleaseMin = minute;
	if ( alarmReleaseMinOld != alarmReleaseMin )
	{
		alarmReleaseMinOld = alarmReleaseMin;
		
		if ( ++alarmReleaseMinCnt > RELAESE_ALARM_MIN )
		{
			alarmReleaseMinCnt	= 0;
			lastAlarm			= 0;
		}	
	}
	
	/*
	*	Aktuelle Uhrzeit mit den eingetellten Alarmzeiten überprüfen
	*/
	for ( uint8_t i = 0 ; i < ALERTS ; i++ )
	{
		if ( ( hour == ram.byte8[ ALERT_HOUR_1 + i ] ) &&  ( minute == ram.byte8[ ALERT_MINUTE_1 + i ] ) && ( alert.enable & 1<<i ) )
		{	
			if ( ( lastAlarm & 1<<i ) == 1<<i )
			{
				return;
			}
			alert.ring			= 1<<i;
			lastAlarm			|= 1UL<<i;
			flag.alertEnable = 1;
		}
	}
}

void alarmRoutine				( void )
{		
	/*
	*	Wurde das "Alarm Flag" gesetzt?
	*/
	if ( ( flag.alertEnable ) )
	{
		uint8_t alertNum = 0;
		char buff[7] = "";
		switch( alert.ring )
		{
			case 1<<0: alertNum = 1; break;
			case 1<<1: alertNum = 2; break;
			case 1<<2: alertNum = 3; break;
			case 1<<3: alertNum = 4; break;
			case 1<<4: alertNum = 5; break;
			case 1<<5: alertNum = 6; break;
			case 1<<6: alertNum = 7; break;
			case 1<<7: alertNum = 8; break;
		}
		
		sprintf( buff, "A.: %d!" , alertNum );
		clearDisplay( true , false );
		putStr( buff , 0 );
		
		ht1632cSendCmd(0x09); // "blinken" Kommando senden
		
		for ( uint8_t i = 0 ; i < sys.alertCycles ; i++ )
		{
			if ( flag.motorEnable )
			{
				VIBRATION_MOTOR_ON;
			}
			_delay_ms(1000);
			VIBRATION_MOTOR_OFF;
			
			for (uint8_t x = 0 ; x < 10 ; x++)
			{
				HEARTBEAT_LED_ON;
				
				if ( flag.motorEnable )
				{
					VIBRATION_MOTOR_ON;
				}

				if ( flag.soundEnable )
				{
					ALARM_SOUND_ON;
				}

				_delay_ms(50);
				HEARTBEAT_LED_OFF;
				VIBRATION_MOTOR_OFF;
				ALARM_SOUND_OFF;
				_delay_ms(250);
				
				if (SWITCH_MENUE_PRESSED || SWITCH_EXIT_PRESSED || SWITCH_SET_DOWN_PRESSED || SWITCH_SET_UP_PRESSED || SWITCH_ENTER_PRESSED)
				{					
					break;
				}
			}// end for
			
			if (SWITCH_MENUE_PRESSED || SWITCH_EXIT_PRESSED || SWITCH_SET_DOWN_PRESSED || SWITCH_SET_UP_PRESSED || SWITCH_ENTER_PRESSED)
			{
				break;	
			}		
		}// end for
	
	clearDisplay( true , false );				
	flag.alertEnable	= 0;
	flag.menueExit		= 0;	
	flag.isDimm			= false;			
	setPwm( ram.byte8[BRIGHT] );
		
	}// end if
}

void compareAutoDimm			( void )
{	
	enum
	{
		DIMM_OFF,
		DIMM_ON,
		CURRENT_TIME,
		DIMM_MAX	
	};
	
	uint16_t dimm[ DIMM_MAX - 1 ];
	
	
	dimm[CURRENT_TIME]	= ( ( rtcBcdToDec(rx8564.hour)  * 60	) + rtcBcdToDec(rx8564.minute) );
	dimm[DIMM_OFF]		= ( ram.byte8[DIMM_HOUR_OFF] * 60	) + ram.byte8[DIMM_MINUTE_OFF];
	dimm[DIMM_ON]		= ( ram.byte8[DIMM_HOUR_ON]	 * 60	) + ram.byte8[DIMM_MINUTE_ON];
	
	if ( dimm[DIMM_ON] > dimm[DIMM_OFF] )
	{
		if( (dimm[CURRENT_TIME] >= dimm[DIMM_ON] && dimm[CURRENT_TIME] > dimm[DIMM_OFF]) || dimm[CURRENT_TIME] < dimm[DIMM_OFF] )
		{
			if( !flag.isDimm )
			{
				setPwm(0xA0);
				flag.isDimm = true;
			}
		}
		else if( dimm[CURRENT_TIME] >= dimm[DIMM_OFF] && dimm[CURRENT_TIME] < dimm[DIMM_ON] && flag.isDimm )
		{
  			setPwm( ram.byte8[BRIGHT] );
  			flag.isDimm = false;
		}		
	}
	else if ( dimm[DIMM_OFF] > dimm[DIMM_ON] ) 
	{
		if( dimm[CURRENT_TIME] >= dimm[DIMM_ON] && dimm[CURRENT_TIME] < dimm[DIMM_OFF] )
		{
			if( !flag.isDimm )
			{
 				setPwm(0xA0);
				flag.isDimm = true;
			}
		}
		else if( dimm[CURRENT_TIME] >= dimm[DIMM_OFF] && flag.isDimm )
		{
			setPwm( ram.byte8[BRIGHT] );
 			flag.isDimm = false;
		}		
	}
}

uint8_t info					( void )
{
	char bufff[20] = "";
	char exit_ = 0;
	
	while ( ( ! ( button.pressed.enter ) ) && ( exit_ != 1) )
	{
		strcpy( bufff , "Ver.: ");
		strcat( bufff , buildVer() );
		exit_ = scroll_display( bufff , INFO_SCROLL_SPEED );
	}
	
	clearDisplay( true , false );
	
	button.all = 0;
	
	return 0;
}

uint8_t get_key_short			( uint8_t key_mask )
{
	cli();                                          // read key state and key press atomic !
	return get_key_press( ~sys.key_state & key_mask );
}

uint8_t get_key_press			( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= sys.key_press;                          // read key(s)
	sys.key_press ^= key_mask;                          // clear key(s)
	sei();
	return key_mask;
}

uint8_t get_key_rpt				( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= sys.key_rpt;                            // read key(s)
	sys.key_rpt ^= key_mask;                            // clear key(s)
	sei();
	return key_mask;
}

uint8_t cnfgTime_				( uint8_t *buff )
{     
	uint8_t state = 0;

    /*
    *      Werte müssen beim Funktionsaufruf mitgegeben werden..
    *      Anhand der ersten beiden Bytes von "buff" werden die
    *      maximal Größen von den Werten festgelegt
    */
    uint8_t maxValues[] = { buff[0] , buff[1] }; 

	buff[0] = buff[2];
	buff[1] = buff[3];
	
	doublePointOn();
    putTimeDec( buff[ 0 ] , buff[ 1 ] );

    for ( uint8_t i = 0 ; i < 2 && ( ! ( flag.menueExit ) ) ; i++ )
    {
		while( ! ( state ) )
		{
			if ( button.pressed.up )
			{
				button.all = 0;   
				buff[i] = incDec( buff[i] , maxValues[i] );
			}
            
			if ( button.pressed.down )
            {
				button.all	= 0;                        
				buff[i] = decDec( buff[i] , maxValues[i] );
            }
			
			if ( button.pressed.enter )
			{
				button.all	= 0;
				break;
			}
			
			if ( ( button.pressed.exit ) || ( flag.menueExit ) )
			{
				button.all		= 0;
				flag.menueExit	= 0;
				return 1;
			}     
        
			/*
			*      Hier wird durch blinken die aktuelle Position des
			*      virtuellen Cursors angezeigt..
			*/
			blinkTime( buff[0] , buff[1] , i );
		}     
	}


	flag.menueExit	= 0;
    sys.autoChangeSec	= 0;
    button.all      = 0;

    return 0;
}

uint8_t dcf77StartScan			( void )
{			
	uint8_t	dcfTmeOut = 0 , dcfTmeOutOld = 0;

	#ifdef _DEBUG
	for ( uint8_t i = 0 ; i < DEBUG_DCF77_MAX ; i++ )
	{
		DCF77Debug.Average[i].Maximum = 0;
		DCF77Debug.Average[i].Minimum = 0xFFFF;
	}
	DCF77Debug.Average[DEBUG_DCF77_START_TIME].nBits = 0;
	DCF77Debug.Average[DEBUG_DCF77_LOW_TIME].nBits = 0;
	DCF77Debug.Average[DEBUG_DCF77_HIGH_TIME].nBits = 0;
	#endif
	
	/*
	*	Bit		Beschreibung
	*	
	*	0	-	Bei welchem Scan wurde ein Timeout ausgelöst [BIT0]
	*	1	-	Bei welchen Scan wurde ein Timeout ausgelöst [BIT1]
	*	2	-	Bei welchem Scan wurde ein Timeout ausgelöst [BIT2]
	*	3	-	Hat der Benutzer den Scan abgebrochen?
	*/
	uint8_t state = 0;
				
	typedef struct
	{
		uint8_t day;
		uint8_t month;
		uint8_t hour;
		uint8_t wDay;
		uint8_t meszMes;
		uint16_t year;
	}dcfRec_t;
		
	dcfRec_t *dcfRec = malloc( ram.byte8[DCF77_NUM_OF_RECORDS] * sizeof(dcfRec_t) );
	if ( dcfRec == NULL )
	{
		return 1;
	}
	
	Dcf77Debug_t *Dcf77ScanDebug = malloc( ram.byte8[DCF77_NUM_OF_RECORDS] * sizeof(Dcf77Debug_t) );
	if ( Dcf77ScanDebug == NULL )
	{
		return 2;
	}
				
	syncDCF				= true;
	dcf77ScanIsActive	= true;
	
	MATRIX_LEDS_OFF;
	
	/*	DCF77
	*	Aufnahme beginnt
	*/
	HEARTBEAT_LED_ON;
	
	uint8_t i;
	
	for ( i = 0 ; ( i < ram.byte8[DCF77_NUM_OF_RECORDS] ) && ( ! ( state ) ) ; i++ )	
	{
		while ( ( dcfNewData == 0 ) && ( flag.alertEnable == 0 ) && ( state == 0 ) )
		{	
			if ( dcfTmeOutOld != rx8564.minute )
			{
				dcfTmeOutOld = rx8564.minute;
				dcfTmeOut++;
			}

			if ( checkDCF )
			{
				checkDCF = false;			
				dcf_check( &Dcf77ScanDebug[i] );
				HEARTBEAT_LED_TOGGLE;
			}
		
			if ( SWITCH_EXIT_PRESSED )
			{	
				_delay_ms(100);				
				dcfRec[i].day		= 50 + i;
				dcfRec[i].month		= 50 + i;
				dcfRec[i].hour		= 50 + i;
				dcfRec[i].wDay		= 50 + i;
				dcfRec[i].meszMes	= 50 + i;
				dcfRec[i].year		= 60 + i;
				state				|= 1<<0;				
				break;
			}
		
			if ( ( dcfTmeOut - 1 ) >= DCF77_TIMEOUT_TIME)
			{	
				dcfRec[i].day		= 50 + i;
				dcfRec[i].month		= 50 + i;
				dcfRec[i].hour		= 50 + i;
				dcfRec[i].wDay		= 50 + i;
				dcfRec[i].meszMes	= 50 + i;
				dcfRec[i].year		= 60 + i;
				dcfTmeOut			= 0;
				state				|= 1<<1;
				break;
			}
			
		}// end while

		/*
		*	Empfangene Informationen sichern
		*/
		if ( ( ( flag.alertEnable == 0 ) && ( state == 0 ) ) )
		{
			dcfRec[i].day		= time.Day;
			dcfRec[i].month		= time.Month;
			dcfRec[i].hour		= time.Hour;
			dcfRec[i].wDay		= time.wDay;
			dcfRec[i].meszMes	= time.stime;
			dcfRec[i].year		= time.Year;			
		}
				
		syncDCF				= true;
		dcf77ScanIsActive	= true;	
		dcfNewData			= 0;
		dcfTmeOut			= 0;
	}// end for

	if ( flag.isDimm )
	{
		setPwm(0xA0);	
	}
	else
	{
		setPwm( ram.byte8[BRIGHT] );
	}

	uint8_t cmp[ ram.byte8[DCF77_NUM_OF_RECORDS] ], cmpCnt = 0;
	for ( uint8_t i = 0 ; i < ram.byte8[DCF77_NUM_OF_RECORDS] ; i++ )
	{
		cmp[i] = 0b00111111;
		if ( time.Day == dcfRec[i].day )
		{
			cmp[i] &= ~(1<<0);	
		}
		if ( time.Month == dcfRec[i].month )
		{
			cmp[i] &= ~(1<<1);
		}		
		if ( time.Hour == dcfRec[i].hour )
		{
			cmp[i] &= ~(1<<2);
		}		
		if ( time.wDay == dcfRec[i].wDay )
		{
			cmp[i] &= ~(1<<3);
		}	
		if ( time.stime == dcfRec[i].meszMes )
		{
			cmp[i] &= ~(1<<4);
		}	
		if ( time.Year == dcfRec[i].year )
		{
			cmp[i] &= ~(1<<5);
		}
		
		if ( ! ( cmp[i] ) )
		{
			/*
			*	DCF77_RECORD bestimmt den späteren Wert von
			*	'cmpCnt'
			*/
			cmpCnt++;
		}
	}
	
	dcf77ScanIsActive = false;
	
	if ( cmpCnt == ram.byte8[DCF77_NUM_OF_RECORDS] )
	{			
		/* @ rtc Sunday is = 0. @ dcf77 is sunday = 7 */
		if (time.wDay == 7)
		{
			time.wDay = Sonntag;
		}// end if
				
		/* write the received dcf77 time into the rtc */
		rtcSetTime(time.Hour, time.Min, time.Sec); // send time to RTC
					
		/* write the received dcf77 time into the rtc */
		rtcSetDate(time.Day, time.wDay, time.Month, time.Year); // send date to RTC

		/* "eeprom_update_byte" for a long living instead of "eeprom_write_byte" */
		eeprom_update_byte(&eep.byte8[MES_MESZ],time.stime);
		eeprom_busy_wait();
		//eeprom_update_word( ( uint16_t * )&erreep.byte16[SYNC_SUCCESS_CNT] , ++err.byte16[SYNC_SUCCESS_CNT] );
	}
	else
	{	
		if ( ! ( flag.alertEnable ) )
		{
			char errStr[ 33 + ( 5 * ram.byte8[DCF77_NUM_OF_RECORDS] ) ];
			char buff[5]	= "";
		
			/*	Error String Format
			*	"String" - "Daten Stream Fehlerkodes[0..n]"
			*	Der Fehlerkode besteht aus den vergleichen empfangener
			*	Streams
			*/
			strcpy( errStr , "Sync. Error - State.: " );
			strcat( errStr , decHex8( state , buff ) );
			strcat( errStr , " Cmp.: ");
			for ( uint8_t i = 0 ; i < ram.byte8[DCF77_NUM_OF_RECORDS] ; i++ )
			{
				strcat( errStr , " " );
				strcat( errStr , decHex8( cmp[i] , buff ) );
			}
			scroll_display( errStr , INFO_SCROLL_SPEED);
			
			#ifdef _DEBUG
			for ( uint8_t y = 0 ; y < i ; y++ )
			{
				char Tmp[] = { '<' , (1+y) + '0' , '.' , '.' , i + '0' , '>' , ' ' , '\0' };
				scroll_display( "Scan " , 35 );
				scroll_display( Tmp , 35);
				ScrollDebugMsg( &Dcf77ScanDebug[y] );
			}
			#endif
			
			eeprom_update_word( ( uint16_t * )&erreep.byte16[SYNC_ERROR_CNT] , ++err.byte16[SYNC_ERROR_CNT] );			
		}
	}
		
	/*
	*	Sollte während der Funktion eine Taste betätigt wurden sein,
	*	müssen wir diese vor verlassen noch bestätigen
	*/
 	button.all = 0;
	
	sys.syncHourCntDCF77	= 0;
	sys.syncMinuteCntDCF77	= 0;
		
	flag.isDimm = false;
	
	free( (dcfRec_t*) dcfRec );
	
	free( (Dcf77Debug_t*) Dcf77ScanDebug );
	
	/*	DCF77
	*	Aufnahme ist beendet
	*/
 	HEARTBEAT_LED_OFF;
	 	
	return state;
}

void defaultEEP					( void )
{
	/*	EEPROM
	*	Prüfen ob der EEPROM schon initalisiert wurden ist..
	*/
	eeprom_busy_wait();
	
	if ( eeprom_read_byte( &eep.byte8[IS_INIT_BYTE_HIGH] ) == 0x21 && eeprom_read_byte( &eep.byte8[IS_INIT_BYTE_LOW] ) == 0x08 )
	{
		flag.isInit = 1;
		return;
	}
	
	flag.isInit = 0;
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[BRIGHT]				,	0xAF	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[SOUND_ENABLE]			,	0x01	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[MOTOR_ENABLE]			,	0x01	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[ALERT]					,	0x00	);
	eeprom_busy_wait();	
	eeprom_write_byte(&eep.byte8[DIMM_HOUR_ON]			,	20	);
	eeprom_busy_wait();	
	eeprom_write_byte(&eep.byte8[DIMM_MINUTE_ON]		,	0	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[DIMM_HOUR_OFF]			,	12	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[DIMM_MINUTE_OFF]		,	0	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[SYNC_HOUR]				,	3	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[SYNC_MINUTE]			,	0	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[MES_MESZ]				,	0	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[AUTO_CHANGE_MINUTE]	,	5	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[AUTO_CHANGE_SECOUND]	,	0	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[ALERT_CYCLES]			,	3	);
	eeprom_busy_wait();
	eeprom_update_word(( uint16_t * )&erreep.byte16[SYNC_ERROR_CNT]	,	0	);
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[DCF77_NUM_OF_RECORDS]	,	2	);
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_HOUR_1 ]		, 9 );
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_MINUTE_1 ]		, 15 );
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_HOUR_2 ]		, 9 );
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_MINUTE_2 ]		, 35 );	
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_HOUR_3 ]		, 12 );
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_MINUTE_3 ]		, 35 );
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_HOUR_4 ]		, 13 );
	eeprom_busy_wait();
	eeprom_write_byte( &eep.byte8[ ALERT_MINUTE_4 ]		, 0 );
	eeprom_busy_wait();	
	eeprom_write_byte(&eep.byte8[IS_INIT_BYTE_HIGH]		, 0x21 );
	eeprom_busy_wait();
	eeprom_write_byte(&eep.byte8[IS_INIT_BYTE_LOW]		, 0x08 );
	eeprom_busy_wait();
}

void loadeep					( void )
{
	_delay_ms(500);
	
	defaultEEP();
	
	ram.byte8[BRIGHT] = eeprom_read_byte( &eep.byte8[BRIGHT] );
	eeprom_busy_wait();
	
	flag.soundEnable = eeprom_read_byte( &eep.byte8[SOUND_ENABLE]);
	eeprom_busy_wait();
	
	flag.motorEnable = eeprom_read_byte( &eep.byte8[MOTOR_ENABLE] );
	eeprom_busy_wait();
	
	alert.enable = eeprom_read_byte( &eep.byte8[ALERT] );
	eeprom_busy_wait();
	
	ram.byte8[DIMM_HOUR_ON] = eeprom_read_byte(&eep.byte8[DIMM_HOUR_ON]); 
	eeprom_busy_wait(); 
	
	ram.byte8[DIMM_MINUTE_ON] = eeprom_read_byte(&eep.byte8[DIMM_MINUTE_ON]); 
	eeprom_busy_wait(); 
	
	ram.byte8[DIMM_HOUR_OFF] = eeprom_read_byte(&eep.byte8[DIMM_HOUR_OFF]); 
	eeprom_busy_wait(); 
	
	ram.byte8[DIMM_MINUTE_OFF] = eeprom_read_byte(&eep.byte8[DIMM_MINUTE_OFF]);
	eeprom_busy_wait(); 

	ram.byte8[SYNC_HOUR] = eeprom_read_byte(&eep.byte8[SYNC_HOUR]);
	eeprom_busy_wait();
	
	ram.byte8[SYNC_MINUTE] = eeprom_read_byte(&eep.byte8[SYNC_MINUTE]);
	eeprom_busy_wait();
	
	time.stime = eeprom_read_byte(&eep.byte8[MES_MESZ]); 
	eeprom_busy_wait();

	ram.byte8[AUTO_CHANGE_SECOUND] = eeprom_read_byte(&eep.byte8[AUTO_CHANGE_SECOUND]);
	eeprom_busy_wait();
	
	ram.byte8[AUTO_CHANGE_MINUTE] = eeprom_read_byte(&eep.byte8[AUTO_CHANGE_MINUTE]);
	eeprom_busy_wait();
	
	sys.alertCycles	= eeprom_read_byte(&eep.byte8[ALERT_CYCLES]); 
	eeprom_busy_wait(); 	
		
	err.byte16[SYNC_ERROR_CNT] = eeprom_read_word( ( uint16_t * )&erreep.byte16[SYNC_ERROR_CNT]  );
	eeprom_busy_wait();
	
	err.byte16[SYNC_SUCCESS_CNT] = eeprom_read_word( ( uint16_t * )&erreep.byte16[SYNC_SUCCESS_CNT]  );
	eeprom_busy_wait();
	
	ram.byte8[DCF77_NUM_OF_RECORDS] = eeprom_read_byte( &eep.byte8[DCF77_NUM_OF_RECORDS] );
	eeprom_busy_wait();
	
	for ( uint8_t i = 0 ; i < ALERTS ; i++ )
	{
		ram.byte8[ ALERT_HOUR_1 + i ] = eeprom_read_byte( &eep.byte8[ALERT_HOUR_1 + i ] );
		eeprom_busy_wait();
		ram.byte8[ ALERT_MINUTE_1 + i ] = eeprom_read_byte( &eep.byte8[ALERT_MINUTE_1 + i ] );
		eeprom_busy_wait();
	}
}

#define _MENUE_EXIT_			0xFF
typedef struct
{
	char *name;
	uint8_t ( *fnc )( void );
	uint8_t menuePos;
}menue_t;

uint8_t openMenue			( menue_t *m , size_t size , uint8_t *sPara );
uint8_t menueAlarm			( void );
uint8_t menueAutoDimm		( void );
uint8_t menueAutoChange		( void );
uint8_t menueTime			( void );
uint8_t menueSync			( void );
uint8_t menueSyncTime		( void );
uint8_t menueSound			( void );
uint8_t menueMotor			( void );
uint8_t menueAlarmCycle		( void );
uint8_t menueDCF77Record	( void );
uint8_t menueBrightness		( void );
uint8_t menueDate			( void );

uint8_t menueCursor = 0;

menue_t menue_struct			[] =	
{
	{ "Time"	, menueTime			, 0				},
	{ "Date"	, menueDate			, 1				},
	{ "Alarm."	, menueAlarm			, 2				},
	{ "Sound"	, menueSound			, 3				},
	{ "Vibr."	, menueMotor			, 4				},		
	{ "Brig."	, menueBrightness	, 5				},
	{ "Dimm."	, menueAutoDimm		, 6				},
	{ "Disp"	, menueAutoChange	, 7				},	
	{ "Sync?"	, menueSync		, 8				},
	{ "Info."	, info					, 9				},
	{ "Reset"	, reboot				, 10			},
	{ "Exit"	, NULL					, _MENUE_EXIT_	},
};

menue_t menueAlert_struct		[] =	
{
	{"-A1"		, NULL			,	0		},
	{"-A2"		, NULL			,	1		},
	{"-A3"		, NULL			,	2		},
	{"-A4"		, NULL			,	3		},
	{"-A5"		, NULL			,	4		},
	{"-A6"		, NULL			,	5		},
	{"-A7"		, NULL			,	6		},
	{"-A8"		, NULL			,	7		},
	{"Cycl."	, NULL			,	8		},
	{"Exit"		, NULL			,	_MENUE_EXIT_		},
};

menue_t menueAutoDimm_struct	[] =	
{
	{"-On"		, NULL					,	0				},
	{"-Off"		, NULL					,	1				},
	{"Exit"		, NULL					,	_MENUE_EXIT_	},
};

menue_t menueOnOff_struct		[] =	
{
	{"On    "	,	NULL	,	0	},
	{"Off   "	,	NULL	,	1	},
};

menue_t menueSync_struct		[] =	
{
	{"-Go!"		, NULL					, 0				},
	{"-Time"	, NULL					, 1				},
	{"-nRX"		, NULL					, 2				},
	{"-Err."	, NULL					, 3				},
	{"Exit"		, NULL					, _MENUE_EXIT_	},
};


uint8_t openMenue			( menue_t *m , size_t size , uint8_t *sPara )
{
	uint8_t index = 0;
	
	if ( sPara != NULL)
	{
		index = sPara[0];
	}
	else
	{
		index = menueCursor;
	}
	

	flag.menueOpen	= 1;

	clearDisplay( true , false );
	
	while ( 1 )
	{	
		if ( ( flag.alertEnable ) || ( flag.menueExit ) )
		{
			sys.menueTimeout	= 0;
			flag.menueExit	= 0;
			clearDisplay( true , false );
			return _MENUE_EXIT_;	
		}
		else if ( button.pressed.up )
		{
			button.all = 0;
			clearDisplay( true , false );
			if ( index < ( size - 1 ) )
			{
				index++;
			}
		}
		else if ( button.pressed.down )
		{
			button.all = 0;
			clearDisplay( true, false );
			if ( index > 0 )
			{
				index--;
			}
		}
		else if ( button.pressed.exit )
		{
			button.all = 0;
			clearDisplay( true , false );
			return _MENUE_EXIT_;
		}
		
		putStr( m[index].name , 0 );
		
		if ( button.pressed.enter )
		{	
			button.all = 0;
			clearDisplay( true , false );
			menueCursor = 0;
			
			if ( m[index].fnc != NULL)
			{
				m[index].fnc();
				menueCursor = m[index].menuePos;
			}
			
			clearDisplay( true , false );
			
			sys.menueTimeout = 0;
			
			return index;
		}
	}
	
	flag.menueOpen		= 0;
	sys.autoChangeSec	= 0;
	sys.autoChangeMin	= 0;
	
	return 0;
}

uint8_t menueAlarm			( void )
{
	uint8_t ret	= 0;
	
Anfang:
	
	ret = openMenue( menueAlert_struct , sizeof(menueAlert_struct) / sizeof(menueAlert_struct[0]) , &ret );
	
	if ( ret == _MENUE_EXIT_ || ret == 9 )
	{
		return ret;
	}
	
	clearDisplay( true , false );
		
	uint8_t buff[] = { 23 , 59 , eeprom_read_byte( &eep.byte8[ALERT_HOUR_1 + ret] ) , eeprom_read_byte( &eep.byte8[ALERT_MINUTE_1 + ret] ) }; // Darf nicht verändert werden..

	if ( ret == 8 )
	{
		menueAlarmCycle();
		return 0;
	}

	uint8_t subExit = cnfgTime_( buff );
	
	if ( subExit )
	{
		return subExit;
	}

	if ( alert.enable & 1<<ret )
	{
		strcpy( menueOnOff_struct[0].name , "On  §");
		strcpy( menueOnOff_struct[1].name , "Off %");
	}
	else
	{
		strcpy( menueOnOff_struct[0].name , "On  %");
		strcpy( menueOnOff_struct[1].name , "Off §");
	}
		
	uint8_t choose	= 0;
	choose = openMenue( menueOnOff_struct , sizeof(menueOnOff_struct) / sizeof(menueOnOff_struct[0]) , NULL );
		
	switch( choose )
	{
		case 0:
		{
			alert.enable |= (1<<ret);
		}break;
			
		case 1:
		{
			alert.enable &= ~(1<<ret);
		}break;
		
		case _MENUE_EXIT_:
		{
			return _MENUE_EXIT_;
		}break;
	}
	
	/*	Alarmzeiten
	*	Aktuelle Zeiten im EEProm speichern
	*/	
	eeprom_update_byte( &eep.byte8[ ALERT_HOUR_1 + ret ]	, buff[0] );
	eeprom_busy_wait();
	eeprom_update_byte( &eep.byte8[ ALERT_MINUTE_1 + ret ]  , buff[1] );
	eeprom_busy_wait();
	eeprom_update_byte( &eep.byte8[ ALERT ] , alert.enable);
	eeprom_busy_wait();
	
	/*	Alarmzeiten
	*	Aktuelle Zeiten in den RAM kopieren
	*/
	ram.byte8[ ALERT_HOUR_1 + ret ]		= buff[0];
	ram.byte8[ ALERT_MINUTE_1 + ret ]	= buff[1];
	
	
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );
	
	goto Anfang;
	
	return ret;
}

uint8_t menueAutoDimm		( void )
{
	uint8_t ret	= 0;
	
Anfang:
	
	ret = openMenue( menueAutoDimm_struct , sizeof(menueAutoDimm_struct) / sizeof(menueAutoDimm_struct[0]) , &ret );
	
	if ( ret == _MENUE_EXIT_ )
	{
		HEARTBEAT_LED_TOGGLE;
		return _MENUE_EXIT_;
	}

	uint8_t buff[] = { 23 , 59 , eeprom_read_byte( &eep.byte8[ DIMM_HOUR_ON + ret ]) , eeprom_read_byte( &eep.byte8[ DIMM_MINUTE_ON + ret ]) }; // Darf nicht verändert werden..
	
	uint8_t exit = 0;
	exit = cnfgTime_( buff );
	
	if ( exit )
	{
		return exit;
	}
	
	eeprom_update_byte( &eep.byte8[ DIMM_HOUR_ON + ret ]   , buff[0] );
	eeprom_busy_wait();
	eeprom_update_byte( &eep.byte8[DIMM_MINUTE_ON + ret ] , buff[1] );
	eeprom_busy_wait();
	
	ram.byte8[DIMM_HOUR_ON + ret	]	= buff[0];
	ram.byte8[DIMM_MINUTE_ON + ret	]	= buff[1];
	
	clearDisplay( true , false );
	
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );	

	if ( ret != _MENUE_EXIT_ )
	{
		goto Anfang;
	}
	
	return 0;	
}

uint8_t menueAutoChange		( void )
{
	uint8_t ret = 0;
	
	uint8_t buff[] = { 59 , 59 , eeprom_read_byte( &eep.byte8[AUTO_CHANGE_MINUTE] ) , eeprom_read_byte( &eep.byte8[AUTO_CHANGE_SECOUND] ) }; // Darf nicht verändert werden..
	
	ret = cnfgTime_( buff );
	
	if ( ret )
	{
		return 1;
	}
	
	eeprom_update_byte( &eep.byte8[AUTO_CHANGE_MINUTE] , buff[0] );
	eeprom_busy_wait();
	eeprom_update_byte( &eep.byte8[AUTO_CHANGE_SECOUND] , buff[1] );
	eeprom_busy_wait();
	
	ram.byte8[AUTO_CHANGE_MINUTE]	= buff[0];
	ram.byte8[AUTO_CHANGE_SECOUND]	= buff[1];
	
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );
	
	return 0;
}

uint8_t menueTime			( void )
{
	uint8_t ret = 0;
	
	rtcGetData( &rx8564 );
	uint8_t buff[] = { 23 , 59 , rtcBcdToDec(rx8564.hour) , rtcBcdToDec(rx8564.minute) };
	
	ret = cnfgTime_( buff );
	
	if ( ret )
	{
		return 1;	
	}
	
	rtcSetTime( buff[0] , buff[1] , 0 );	
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );
	
	return 0;
}

uint8_t menueSync			( void )
{
	uint8_t ret = 0;
	
Anfang:

	ret = openMenue( menueSync_struct ,  sizeof(menueSync_struct) / sizeof(menueSync_struct[0]) , &ret );
	
	if ( ret == _MENUE_EXIT_ )
	{
		return ret;
	}
	
	switch( ret )
	{
		case 0:
		{
			dcf77StartScan();
		}break;
		
		case 1:
		{
			menueSyncTime();
		}break;
		
		case 2:
		{
			menueDCF77Record();
		}break;
		
		case 3:
		{
			char buff[17]="";
		
			itoa( err.byte16[SYNC_SUCCESS_CNT] , buff , 10 );
			strcat( buff , " - RX Ok" );
			scroll_display( buff , 30 );

			itoa( err.byte16[SYNC_ERROR_CNT] , buff , 10 );
			strcat( buff , " - RX Err." );
			scroll_display( buff , 30 );
			
			if ( SWITCH_MENUE_PRESSED )
			{
				err.byte16[SYNC_SUCCESS_CNT]	= 0;
				err.byte16[SYNC_ERROR_CNT]	= 0; 
				eeprom_write_word( ( uint16_t * )&erreep.byte16[SYNC_ERROR_CNT], 0 );
				eeprom_busy_wait();
				eeprom_write_word( ( uint16_t * )&erreep.byte16[SYNC_SUCCESS_CNT] , 0);
				eeprom_busy_wait();
				scroll_display( "Geloescht.." , INFO_SCROLL_SPEED );
			}
			
		}break;
		
		case _MENUE_EXIT_:
		{
			return _MENUE_EXIT_;
		}break;
	}
	
	goto Anfang;
	
	return 0;
}

uint8_t menueSyncTime		( void )
{
	uint8_t ret = 0;
	
	uint8_t buff[] = { 23 , 59 , eeprom_read_byte(&eep.byte8[SYNC_HOUR]) , eeprom_read_byte(&eep.byte8[SYNC_MINUTE]) };
	
	ret = cnfgTime_( buff );
	
	if ( ret )
	{
		return 1;
	}

	if ( buff[1] == 0 )
	{
		buff[1] = 1;
	}
	
	eeprom_update_byte(  &eep.byte8[SYNC_HOUR] , buff[0] );
	eeprom_busy_wait();
	eeprom_update_byte(  &eep.byte8[SYNC_MINUTE] , buff[1] );
	eeprom_busy_wait();
	
	ram.byte8[SYNC_HOUR] = buff[0];
	ram.byte8[SYNC_MINUTE] = buff[1];
	
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );

	return 0;
}

uint8_t menueSound			( void )
{
	clearDisplay(false,true);
	
	flag.soundEnable = eeprom_read_byte(&eep.byte8[SOUND_ENABLE]);
	
	uint8_t enabled = 0;
	
	enabled = flag.soundEnable;
	
	if ( enabled & 1<<0 )
	{
		strcpy( menueOnOff_struct[0].name , "On  §");
		strcpy( menueOnOff_struct[1].name , "Off %");
	}
	else
	{
		strcpy( menueOnOff_struct[0].name , "On  %");
		strcpy( menueOnOff_struct[1].name , "Off §");
	}
	
	uint8_t ret	= 0;

	ret = openMenue( menueOnOff_struct , sizeof(menueOnOff_struct) / sizeof(menueOnOff_struct[0]) , NULL );
	
	if ( ret == _MENUE_EXIT_ )
	{
		return ret;
	}
			
	switch( ret )
	{
		case 0:
		{
			flag.soundEnable |= 1<<0;
		}break;
		
		case 1:
		{
			flag.soundEnable &= ~(1<<0);
		}break;
	}
	
	eeprom_update_byte( &eep.byte8[SOUND_ENABLE] , flag.soundEnable );
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );
		
	return 0;
}

uint8_t menueMotor			( void )
{
	uint8_t enabled = 0;
	
	clearDisplay(false,true);
	flag.motorEnable = eeprom_read_byte(&eep.byte8[MOTOR_ENABLE]);
	enabled = flag.motorEnable;
	
	
	if ( enabled & 1<<0 )
	{
		strcpy( menueOnOff_struct[0].name , "On  §");
		strcpy( menueOnOff_struct[1].name , "Off %");
	}
	else
	{
		strcpy( menueOnOff_struct[0].name , "On  %");
		strcpy( menueOnOff_struct[1].name , "Off §");
	}
	
	uint8_t ret	= 0;

	ret = openMenue( menueOnOff_struct , sizeof(menueOnOff_struct) / sizeof(menueOnOff_struct[0]) , NULL );

	
	if ( ret == _MENUE_EXIT_ )
	{
		return ret;
	}
	
	switch( ret )
	{
		case 0:
		{
			flag.motorEnable |= 1<<0;
		}break;
		
		case 1:
		{
			flag.motorEnable &= ~(1<<0);
		}break;
	}
	
	eeprom_update_byte( &eep.byte8[MOTOR_ENABLE] , flag.motorEnable );	
	scroll_display( (const __flash char *)"Gespeichert..!" , SCROLL_SAVE_SPEED );
	
	return 0;	
}

uint8_t menueAlarmCycle		( void )
{
	clearDisplay(false,true);
	
	/* default setting */
	sys.alertCycles = eeprom_read_byte( &eep.byte8[ALERT_CYCLES] );
	
	putStr( "n.:" , 0 );
	
	while( 1 )
	{
		if (button.pressed.up)
		{
			button.all = 0;
			sys.alertCycles = incDec(sys.alertCycles, 99);
			uint8ToDisp(sys.alertCycles,21);
		}
		else if (button.pressed.down)
		{
			button.all = 0;
			sys.alertCycles = decDec(sys.alertCycles, 99);
			uint8ToDisp(sys.alertCycles,21);
		}
		else if (button.pressed.enter)
		{
			button.all = 0;
			break;
		}
		else if ( button.pressed.exit || flag.menueExit )
		{
			button.all = 0;
			flag.menueExit = 0;
			clearDisplay(false,true);
			return 1;
		}
	
		blink2Digit( sys.alertCycles , 21 );
				
	}// end while
	
	flag.menueExit = 0;
	
	eeprom_update_byte(&eep.byte8[ALERT_CYCLES], sys.alertCycles);
	scroll_display( (const __flash char *)"Gespeichert..!",SCROLL_SAVE_SPEED);

	return 0;
}

uint8_t menueDCF77Record	( void )
{
	clearDisplay(false,true);
		
	/* default setting */
	ram.byte8[DCF77_NUM_OF_RECORDS] = eeprom_read_byte( &eep.byte8[DCF77_NUM_OF_RECORDS] );
		
	putStr( "n.:" , 0 );
		
	while( 1 )
	{
		if ( button.pressed.up )
		{
			button.all = 0;
			ram.byte8[DCF77_NUM_OF_RECORDS] = incDec( ram.byte8[DCF77_NUM_OF_RECORDS] , 10 );
			uint8ToDisp( ram.byte8[DCF77_NUM_OF_RECORDS] , 21 );
		}
		else if (button.pressed.down)
		{
			button.all = 0;
			ram.byte8[DCF77_NUM_OF_RECORDS] = decDec( ram.byte8[DCF77_NUM_OF_RECORDS] , 10 );
			uint8ToDisp( ram.byte8[DCF77_NUM_OF_RECORDS] , 21 );
		}
		else if ( button.pressed.enter )
		{
			button.all = 0;
			break;
		}
		else if ( button.pressed.exit || flag.menueExit )
		{
			button.all = 0;
			flag.menueExit = 0;
			clearDisplay(false,true);
			return 1;
		}
			
		blink2Digit( ram.byte8[DCF77_NUM_OF_RECORDS] , 21 );
			
	}// end while
		
	flag.menueExit = 0;
		
	eeprom_update_byte( &eep.byte8[DCF77_NUM_OF_RECORDS] , ram.byte8[DCF77_NUM_OF_RECORDS] );
	scroll_display( (const __flash char *)"Gespeichert..!",SCROLL_SAVE_SPEED);

	return 0;
}

uint8_t menueBrightness		( void )
{
	uint8_t index = 0;
	uint8_t lastSelected = 0;
	clearDisplay(false,true);
	
	index = eeprom_read_byte( &eep.byte8[BRIGHT] ) - 0xA0;
	lastSelected = index;
	eeprom_busy_wait(); // eepROM ready?
	
	while( 1 )
	{
		if (button.pressed.up)
		{
			button.all = 0;
			clearDisplay(false,true);
			index = incDec( index , 15 );
			ht1632cSendCmd ( 0xA0 + index );
		}
		
		if (button.pressed.down)
		{
			button.all = 0;
			clearDisplay(false,true);
			index = decDec( index , 15 );
			ht1632cSendCmd ( 0xA0 + index );
		}
		
		if (button.pressed.enter)
		{
			button.all = 0;
			clearDisplay(false,true);
			ht1632cSendCmd (0xA0 + index );
			break;
		}
		
		if ( button.pressed.exit || flag.menueExit )
		{
			button.all = 0;
			flag.menueExit = 0;
			clearDisplay(false,true);
			ht1632cSendCmd ( 0xA0 + lastSelected );
			return 1;
		}
		
		if ( index < 10 )
		{
			putChar( '0' + index , 1 );
			
		}
		else if ( index > 9 )
		{
			putChar( '1' , 1 );
			putChar( '0' + ( index - 10 ) , 7 );
		}
		

		if ( index == lastSelected )
		{
			putChar('§',27);
		}
		else
		{
			putChar('%',27);
		}
		
	}// end while
	
	ram.byte8[BRIGHT] = 0xA0 + index;
	eeprom_update_byte( &eep.byte8[BRIGHT] , 0xA0 + index );
	scroll_display( (const __flash char *)"Helligkeit gespeichert!",SCROLL_SAVE_SPEED);
	
	sys.autoChangeSec		= 0;
	sys.syncMinuteCntDCF77	= 0;
	sys.syncHourCntDCF77	= 0;
	button.all			= 0;
	flag.menueExit		= 0;
	
	return 0;
}

uint8_t menueDate			( void )
{	
	enum 
	{
		DAY,
		MONTH,
		YEAR,

		DATE_MAX_ENTRYS	
	};
	
	enum 
	{
		DAY_NAME_SO,
		DAY_NAME_MO,
		DAY_NAME_DI,
		DAY_NAME_MI,
		DAY_NAME_DO,
		DAY_NAME_FR,
		DAY_NAME_SA,
		
		DAY_NAMES_ENTRYS
	};

	uint8_t date	[ DATE_MAX_ENTRYS ][2];
	
	/*
	*	Default Werte---------|---------Maximal Werte
	*/
	date[0][0] = time.Day;			date[0][1] = 31;
	date[1][0] = time.Month;		date[1][1] = 12;
	date[2][0] = time.Year - 2000;	date[2][1] = 99;
	
	static char *msg		[] =
	{
		"dd.:",
		"mm.:",
		"yy.:",
	};
	
	static char *timeFormat	[]=
	{
		"Mez",
		"Mesz",
	};
		
	static char *days		[ DAY_NAMES_ENTRYS ] =
	{
		"So",
		"Mo",
		"Di",
		"Mi",
		"Do",
		"Fr",
		"Sa",
	};
		
	uint8_t state = 0;
	button.all = 0;
	
	for ( uint8_t i = 0 ; i < 3 ; i++ )
	{
		putStr( msg[i] , 0 );
		while( ! ( state ) )
		{
			if (button.pressed.up)
			{
				button.all = 0;
				date[i][0] = incDec( date[i][0] , date[i][1] );
				uint8ToDisp( date[i][0] , 21 );
			}
			else if (button.pressed.down)
			{
				button.all = 0;
				date[i][0] = decDec( date[i][0] , date[i][1] );
				uint8ToDisp( date[i][0] , 21 );
			}
			else if (button.pressed.enter)
			{
				button.all = 0;
				break;
			}
			else if ( button.pressed.exit || flag.menueExit )
			{
				button.all		 = 0;
				state			|= 1<<0;
				flag.menueExit	 = 0;
				clearDisplay(false,true);
				break;
			}
		
			blink2Digit( date[i][0] , 21 );
		
		}// end while	
			
	clearDisplay(false,true);
	
	}// end for
	
	putStr( timeFormat[time.stime] , 0 );
	
	while( ! ( state ) )
	{
		if ( button.pressed.up )
		{
			button.all = 0;
			clearDisplay(false,true);
			if (time.stime++ >= 1)
			{
				time.stime = 1;
			}
			putStr( timeFormat[time.stime] , 0 );
		}
		else if ( button.pressed.down)
		{
			button.all = 0;
			clearDisplay(false,true);
			if (time.stime-- <= 0 )
			{
				time.stime = 0;
			}
			putStr( timeFormat[time.stime] , 0 );
		}
		if ( button.pressed.exit || flag.menueExit )
		{
			button.all		 = 0;
			state			|= 1<<0;
			flag.menueExit	 = 0;
			clearDisplay(false,true);
			break;
		}
		if (button.pressed.enter)
		{
			button.all = 0;
			break;
		}
						
		blinkChars( timeFormat[time.stime] , 0 );
		
	}// end while
	
	
	if ( ! ( state ) )
	{
		scroll_display("Enter > Name of Day", INFO_SCROLL_SPEED);
		clearDisplay( true , false );
	}
	
	putStr( "wD.:" , 0 );
	putStr( days[time.wDay] , 21 );	
	while( ! ( state ) )
	{
		if ( button.pressed.up )
		{
			button.all = 0;
			clearDisplay(false,true);
			time.wDay = incDec(time.wDay, 0x06);
			putStr( "wD.:" , 0 );
			putStr( days[time.wDay] , 21 );
		}
		if ( button.pressed.down )
		{
			button.all = 0;
			clearDisplay(false,true);
			time.wDay = decDec(time.wDay, 0x06);
			putStr( "wD.:" , 0 );
			putStr( days[time.wDay] , 21 );
		}
		if ( button.pressed.enter )
		{
			button.all	= 0;
			sys.autoChangeSec = 0;
			clearDisplay(false,true);
			break;
		}
		if ( button.pressed.exit || flag.menueExit )
		{
			button.all		 = 0;
			state			|= 1<<0;
			flag.menueExit	 = 0;
			clearDisplay(false,true);
			break;
		}
	
		blinkChars( days[time.wDay] , 21 );
			
		if ( button.pressed.exit )
		{
			button.all = 0;
			clearDisplay(false,true);
			state |= 1<<4;
			break;
		}
	}//end while
	
	rtcSetDate(	date[0][0]	,	time.wDay	,	date[1][0] , date[2][0] + 2000 );
	eeprom_update_byte( &eep.byte8[MES_MESZ] , time.stime );
	
	if ( ! ( state ) )
	{
		scroll_display( (const __flash char *)"Gespeichert!",SCROLL_SAVE_SPEED);
	}
	
	
	button.all = 0;
	
	return 0;
}


int main( void )
{	
	/*	DCF77 Abgleich
	*	Mit der Variable werden die verstrichenen Minuten gezählt 
	*	bis zur nächsten Synchronisation
	*/
	uint8_t oldSyncTime = 0;
	
	/*	Ausgänge
	*	Ausgänge konfigurieren
	*/
	DDRD |= ((1<<PD1) | (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7));
	DDRC |= (1<<PC0);
	DDRB |= ((1<<PB3) | (1<<PB4));
	
	/*	Eingänge / Ausgänge
	*	Anfangszustände der Pins festlegen
	*/
	PORTC |= ((1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6));
	PORTB |= (1<<PB2);
	PORTD |= ((1<<PD5));
		
	/*	Timer 0 
	*	Wird auf CompareMatch eingestellt
	*	Auslöseintervall.: 10ms
	*/
	TCCR0	|= ((1<<CS02) | (1<<CS00) | (1<<WGM01)); // Prescaler : 1024 
	TIMSK   |= ((1<<OCIE0) | (1<<OCIE2));
	OCR0	 = ((F_CPU / 1024 / 100 ) - 1 );
	
	/*	Timer 2 
	*	Wird auf CompareMatch eingestellt
	*	Auslöseintervall.: 1ms
	*/
	TCCR2  |= ((1<<CS22) | (1<<WGM21)); // Prescaler : 64
	OCR2   = ((F_CPU / 64 / 1000 ) - 1 ); 
	
    /*	Interrupts
	*	Globale Interrupts aktivieren
	*/
    sei();
	
	loadeep		();
	i2c_init	();
	bmp180Init	();

	/*	Uhrzeit
	*	Zum Systemstart das erste mal die Uhrzeit lesen
	*/
	rtcGetData(&rx8564);
	sys.cmpOldHour = rx8564.hour;
	
	ht1632c_init	( ram.byte8[BRIGHT]		);	
	scroll_display	(">>Start Super Clokk<<", 10);
	scroll_display	( buildVer() , 20		);
	clearDisplay	( true , false			);

	

	if ( !flag.isInit )
	{
		scroll_display( "Default Werte werden geladen.." , INFO_SCROLL_SPEED );
		for (uint8_t i = 0 ; i < 32 ; i++ )
		{
			putChar( '|' , i );
			_delay_ms(150);
		}
		for (uint8_t i = 32 ; i > 0 ; i-- )
		{
			putChar( ' ' , i );
			_delay_ms(50);
		}
		
		clearDisplay( true , false );
		flag.isInit = 1;
	}

 	syncDCF		= false;
  	dcfNewData	= 0;
			
	/*	Tasten Entprellen
	*	Sollte beim Systemstart eine Taste gedrückt wurden sein,
	*	löschen wir diese nach main()
	*/
	button.all = 0;
	
	/*	Status LED
	*	Status Leuchtdiode ausschalten
	*/
	HEARTBEAT_LED_OFF;
	
    while( 1 )
    {
		alarmRoutine();
		compareAutoDimm();
			
		if ( button.pressed.exit )
		{
			button.all = 0;
			#ifdef _DEBUG
			ScrollDebugMsg( &DCF77Debug );
			#endif 
			button.all = 0;
		}			
		else if ( button.pressed.down )
		{		
			button.all = 0;
			scrollDate( &rx8564 , TEMP_PRESS_DATE_SCROLL_SPEED);
			button.all = 0;
		}							
		else if ( button.pressed.up )
		{
			button.all = 0;
 			scrollTemperature(TEMP_PRESS_DATE_SCROLL_SPEED);
			button.all = 0;
		}
		else if ( button.pressed.enter )
		{
			button.all = 0;
 			scrollPressure(TEMP_PRESS_DATE_SCROLL_SPEED);
			button.all = 0;
		}	
		else if ( button.pressed.menue )
		{
			uint8_t index = 0;
			
			button.all = 0;
			
			while( ( index != _MENUE_EXIT_ ) && ( flag.alertEnable == 0 ) )
			{
				index = openMenue( menue_struct , sizeof(menue_struct) / sizeof(menue_struct[0]) , NULL );
			}
			
			button.all = 0;
		}
		
		doublePointOn();
		putTimertcBcdToDec( rx8564.hour , rx8564.minute );		

		sys.autoChangeNew = rx8564.second;
		if ( sys.autoChangeNew != sys.autoChangeOld)
		{
			sys.autoChangeOld = sys.autoChangeNew;
			if (sys.autoChangeSec < 59)
			{
				sys.autoChangeSec++;
			}
			else
			{
				sys.autoChangeSec = 0;
				if (sys.autoChangeMin < 59)
				{
					sys.autoChangeMin++;
				}
				else
				{
					sys.autoChangeMin = 0;
				}
			}
		}
		
		if ( ( sys.autoChangeSec >= ram.byte8[AUTO_CHANGE_SECOUND] ) && ( sys.autoChangeMin >= ram.byte8[AUTO_CHANGE_MINUTE] ) )
		{
			for (uint8_t x = 0 ; x < 8 ; x++)
			{
				sys.autoChangeMin = 0;
				sys.autoChangeSec = 0;
				button.all = 0;
				ledarray_shift_up();
				_delay_ms(100);
			}// end for
						
			scrollDate( &rx8564 , TEMP_PRESS_DATE_SCROLL_SPEED);
			scrollTemperature (TEMP_PRESS_DATE_SCROLL_SPEED);
			scrollPressure(TEMP_PRESS_DATE_SCROLL_SPEED);
			
			#ifdef _DEBUG
			//ScrollDebugMsg( &DCF77Debug );
			#endif 
		
		}// end if
				
		/*	Zeitsynchronisation
		*	Kann im "Menü" -> "Sync?" -> "-Cycl." eingestellt werden
		*	...
		*/			
		if ( ( sys.syncHourCntDCF77 >= ram.byte8[SYNC_HOUR] ) && ( sys.syncMinuteCntDCF77 >= ( ram.byte8[SYNC_MINUTE] ) ) )
		{			
			dcf77StartScan();
			
			#ifdef _DEBUG
			ScrollDebugMsg( &DCF77Debug );
			#endif
			
			oldSyncTime	= rx8564.minute;
		}	
		
		/*
		*	Zähler für die automatische Synchronisation
		*/
		if (oldSyncTime != rx8564.minute )
		{
			oldSyncTime = rx8564.minute;
			if (sys.syncMinuteCntDCF77 < 59)
			{
				sys.syncMinuteCntDCF77++;
			}
			else
			{
				sys.syncMinuteCntDCF77 = 0;
				sys.syncHourCntDCF77++;
			}
		}
	
    }// end While (main)
}// end main


/*
*	Aufrufintervall.: 10ms
*/
ISR( TIMER0_COMP_vect )
{			
	static uint8_t delay = 0;
	static uint16_t newHourDelay = 0;
		
	if ( ++delay >= 40 )
	{
		compareTime();	
		delay = 0x00;	
	}

	if ( ! ( dcf77ScanIsActive ) )
	{	
		if ( sys.menueTimeout++ >= 500 )
		{
			sys.menueTimeout	= 0;
			flag.menueExit		= 1;
		}
		
		ht1632c_send_page();		
		switch_debounce();
	}
	
	if ( sys.alertNewHour )
	{	
		if ( ++newHourDelay >= 0 )
		{
			if ( flag.soundEnable )
			{
				ALARM_SOUND_ON;
			}
			if ( ! ( dcf77ScanIsActive ) )
			{
				HEARTBEAT_LED_ON;
			}	
		}
			
		if ( newHourDelay >= 5 )
		{
			ALARM_SOUND_OFF;

			if ( ! ( dcf77ScanIsActive ) )
			{
				HEARTBEAT_LED_OFF;
			}
		}

			
		if ( newHourDelay >= 25 )
		{
			if ( flag.soundEnable )
			{
				ALARM_SOUND_ON;
			}
			
			if ( ! ( dcf77ScanIsActive ) )
			{
				HEARTBEAT_LED_ON;
			}
		}
			
		if ( newHourDelay >= 30 )
		{
			sys.alertNewHour	= 0;
			newHourDelay		= 0;
			ALARM_SOUND_OFF;
			
			if ( ! ( dcf77ScanIsActive ) )
			{
				HEARTBEAT_LED_OFF;
			}
		}
	}
}

/*
*	Aufrufintervall.: 1ms
*/
ISR( TIMER2_COMP_vect )
{	
	if ( dcf77ScanIsActive )
	{
		checkDCF = true;
	}

	sys.blinkCnt++;
}

