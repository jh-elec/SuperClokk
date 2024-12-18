/* Generated by CodeDescriptor 1.5.0.0907 */
/*
* Project Name      -> Number Converter
* Version           -> 1.0.0.0927
* Author            -> Hm @ Workstadion.: QP-01-02
* Build Date        -> 27.09.2017 07:58:53
* Description       -> Convert Numbers in other Systems.
*
*
*
*/

#include <stdint.h>

#ifndef CONVERT_H
#define CONVERT_H


typedef char*(*decHex8Ptr) (uint8_t,  char *);
typedef char*(*decHex16Ptr)(uint16_t, char *);
typedef char*(*decHex32Ptr)(uint32_t, char *);

char *decHex8 (uint8_t  dec, char *b);
char *decHex16(uint16_t dec, char *b);
char *decHex32(uint32_t dec, char *b);

typedef char* (*decBcd8Ptr)(uint8_t, char *);
char *decBcd8(uint8_t dec, char *b);

typedef char* (*decBin8Ptr)(uint8_t, char *);
char *decBin8(uint8_t dec, char *b);

typedef struct
{
	decHex8Ptr  	decHex8;
	decHex16Ptr		decHex16;
	decHex32Ptr		decHex32;
	
	decBcd8Ptr  	decBcd8;
	decBin8Ptr  	decBin8;
}convert_t;

extern convert_t convert;

#endif
