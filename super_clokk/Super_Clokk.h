/*
*
*	Super_Clokk "Prototypes"
*
*/

#include <stdint.h>
#include <stdbool.h>
#include "rx8564.h"




/* Prototypes */

void ledarray_flash(uint16_t rate,uint8_t repeats);

uint8_t scroll_display(const char *s,uint8_t speed);
void putChar(char c, uint8_t offset);
void ledarray_testpattern(uint8_t rate);

void getFont(char match, char *buff);
void display_Username(uint8_t name);
void ledarray_right_shift(void);
void ledarray_left_shift(void);
void ledarray_shift_down(void);
void ledarray_shift_up(void);
void ledarray_twinkle(void);
void ledarray_blank(void);
void check_snooze(void);
uint8_t getFontWidth(char c);

uint8_t get_key_press(uint8_t key_mask);
uint8_t get_key_short(uint8_t key_mask);
uint8_t get_key_rpt( uint8_t key_mask );


uint8_t menueSetDCF77Record( void );

/* scroll the actually date over the display */
/*	Parameter	: z.B 12,3,15,6(Sunday), 5
*	Returns		: none
*/
void scrollDate( rx8564_t *d , uint8_t speed );

/* scroll the actually messuared temperature over the display */
/*	Parameter	: z.B 21, 5
*	Returns		: none
*/
void scrollTemperature(uint8_t speed);

/* scroll the actually messuared pressure over the display */
/*	Parameter	: z.B 1005, 5
*	Returns		: none
*/
void scrollPressure(uint8_t speed);

/* display the actually time @ certain positon */
/*	Parameter	: BCD(Hour) , BCD(Minutes)
*	Returns		: none
*/
void write_time_to_display(uint8_t hours, uint8_t minutes);

/* display a integer value @ certain position */
/*	Parameter	: none
*	Returns		: none
*/
void uint8ToDisp( uint8_t value , uint8_t offset);

/* generate a blinking double point (for time display) */
/*	Parameter	: none
*	Returns		: none
*/
void blink_double_point(void);

/* reconfigure the ht1632c */
/*	Parameter	: Brightness (0xA0 ... 0xAF)
*	Returns		: none
*/
void cnfgMatrix(uint8_t bright);

/* debounce the switches */
/*	Parameter	: none
*	Returns		: none
*/
void switch_debounce(void);

/* show a double point (for time diplay) */
/*	Parameter	: none
*	Returns		: none
*/
void doublePointOn(void);

/* show none double point (for time display) */
/*	Parameter	: none
*	Returns		: none
*/
void doublePointOff(void);

/* User can enable or disable the Vibration Motor */
/*	Parameter	: none
*	Returns		: none
*/
void cnfgVibroMotor(void);

/* User can set the Brightness */
/*	Parameter	: none
*	Returns		: none
*/
uint8_t cnfgBrightness(void);

/* function for clear the display */
/*	Parameter	: true , false = clear display || true , true = not clear || false , false = not clear
*	Returns		: none
*/
void clearDisplay(bool new, bool old);

/* User can set the actually time */
/*	Parameter	: none
*	Returns		: none
*/
uint8_t cnfgTime(void);

/* makes sound and vibration */
/*	Parameter	: none
*	Returns		: none
*/
void alarm(void);

/* compare the alarm with (set_Alarm_one and set_Alarm_two ...) */
/*	Parameter	: none
*	Returns		: none
*/
void compare_time(void);

/* User can stop the time (max. "59 :  59") */
/*	Parameter	: none
*	Returns		: none
*/
void stop_watch(void);

/* User can set a Countdown (max. "59 :  59") */
/*	Parameter	: none
*	Returns		: none
*/
void countdown(void);

/* User can enable or disable the Sound */
/*	Parameter	: none
*	Returns		: none
*/
uint8_t set_Sound(void);

/* test the "Super_Clokk" */
/*	Parameter	: none
*	Returns		: none
*/
void test_mode(void);

/* only for compiler */
/*	Parameter	: none
*	Returns		: none
*/
void i2c_init(void);

/* user can set alarm one */
/*	Parameter	: none
*	Returns		: none
*/
void set_Alarm_one(void);

/* user can set alarm two */
/*	Parameter	: none
*	Returns		: none
*/
void set_Alarm_two(void);

/* user can set alarm three */
/*	Parameter	: none
*	Returns		: none
*/
void set_Alarm_three(void);

/* user can set alarm four */
/*	Parameter	: none
*	Returns		: none
*/
void set_Alarm_four(void);

/* decrement @ BCD steps */
/*	Parameter	: Decimal Value , Decimal Value max
*	Returns		: Decimal @ BCD
*/
uint8_t decDec(uint8_t  Value , uint8_t ValueMax);

/* increment @ BCD steps */
/*	Parameter	: Decimal Value , Decimal Value max
*	Returns		: Decimal @ BCD
*/
uint8_t incDec(uint8_t  Value , uint8_t ValueMax);

/* form BCD to Decimal */
/*	Parameter	: BCD Value (z.B 0x04)
*	Returns		: Decimal
*/
uint8_t rtcBcdToDec(uint8_t value);

/* scroll a info string over the display about the "Super_Clokk" */
/*	Parameter	: none
*	Returns		: none
*/
uint8_t info(void);

/* dimm the display @ AUTO_DIMM_ON and redimm the display @ AUTO_DIMM_OFF (see @ Super_Clokk.c)*/
/*	Parameter	: none
*	Returns		: none
*/
void check_auto_dimm(void);


/* user can change the auto dimm time for "on" and "off"*/
/*	Parameter	: none
*	Returns		: none
*/
void user_change_dimm(void);


/* user can change the sync rythem*/
/*	Parameter	: none
*	Returns		: none
*/
uint8_t menueSetSyncTime( void );

/* scan the dcf 77 signal and compare two scans*/
/*	Parameter	: none
*	Returns		: none
*/
uint8_t scan_dcf77(void);

/* check and reload the values from the eeprom*/
/*	Parameter	: none
*	Returns		: none
*/
void loadeep(void);

/* user can enter a time how ever change the display info (Date,Temperature,Pressure)*/
/*	Parameter	: none
*	Returns		: none
*/
void user_set_auto_change(void);

/* user can enter a count of alarm cycles*/
/*	Parameter	: none
*	Returns		: none
*/
uint8_t user_set_alarm_cycle(void);

/* check the hour sound function */
/*	Parameter	: none
*	Returns		: none
*/
void test_hour_sound(void);