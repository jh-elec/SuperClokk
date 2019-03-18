/* *****************************************************************************
 * File:        dcf.h
 * Project:     -
 * Author:      Nicolas Meyertöns
 * Version:     1.4 (26.01.2015)
 * Web:         http://pic-projekte.de/
 * ****************************************************************************/

#ifndef DCF_H
#define	DCF_H

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>



/*******************************************************************************
 * DCF77 Datenstruktur
 */

enum eDay
{
	Sonntag		= 0,
	Montag		= 1,
	Dienstag	= 2,
	Mittwoch	= 3,
	Donnerstag	= 4,
	Freitag		= 5,
	Samstag		= 6

};  

struct sTime
{
	uint8_t Day;
	uint8_t Month;
	uint16_t Year;
	uint8_t Hour;
	uint8_t Min;
	uint8_t Sec;
	enum eDay wDay;
	uint8_t stime;
	
}; typedef struct sTime tTime;
/*******************************************************************************
 * Hier musst du einstellen an welchem Port-Pin das Datensignal des DCF-Moduls
 * angeschlossen ist. Bitte vergiss nicht auf das zugehörige Trisbit zu setzen.
 */
 
#define DCF77_DATA  (!(PINB & (1<<PB2)))

/*******************************************************************************
 * Es folgen einige Makros (nicht ändern!)
 */

#define DCF_LOW     0   	// ~100 ms gemessen
#define DCF_HIGH    1   	// ~200 ms gemessen
#define DCF_START   3   	// Startbit gemessen >> 200ms
#define DCF_ZEIT    21  	// Start der Zeit- und Datumsinfo
#define DCF_MIN     21  	// Beginn Minuten (x7)
#define DCF_STD     29  	// Beginn Stunden (x6)
#define DCF_KT      36  	// Beginn Kalendertag (x6)
#define DCF_WT      42  	// Beginn Wochentag (x3)
#define DCF_M       45  	// Beginn Monat (x5)
#define DCF_J       50  	// Beginn Jahr (x8)

#define MESZ        true
#define MEZ         false

 /*******************************************************************************
 * Die folgenden Variablen müssen auch außerhalb der Datei dcf.c zugänglich sein
 * und werden aus diesem Grund mit dem Schlüsselwort extern versehen.
 */
 
 
 extern volatile bool		checkDCF;
 extern volatile bool		syncDCF;
 extern volatile tTime		time;
 extern volatile bool		dcf_Start;
 extern volatile bool		dcf77ScanIsActive;
 extern volatile uint8_t	dcfNewData;


#define _DEBUG

#ifdef _DEBUG

#warning __DEBUG_MODE_IS_ENABLED__

typedef struct
{
	enum DCF77
	{
		DEBUG_DCF77_START_TIME,
		DEBUG_DCF77_LOW_TIME,
		DEBUG_DCF77_HIGH_TIME,
		
		DEBUG_DCF77_MAX
	}DCF77_DEBUG_ENUM;
	

	
	struct
	{
		uint16_t Minimum;
		uint16_t Maximum;
		uint8_t	 nBits;
	}Average[3];
		
}Dcf77Debug_t;

 Dcf77Debug_t DCF77Debug;
 #endif
 
 /*******************************************************************************
 * Es folgt die Liste mit den Prototypen der implementierten Funktionen.
 */
 
bool dcf_running (void);
bool dcf_collect (void);
bool dcf_decode (void);
void dcf_check (void);

#endif	/* DCF_H */