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


#include <avr/io.h>
#include "convert.h"

char *decHex8(uint8_t dec, char *b)
{	
	/* hex char set */
	char hexchars[] = "0123456789ABCDEF";
  	b[0] = '0';	
   	b[1] = 'x'; 	 
    b[2] = hexchars[dec >>  4 & 0xF];          
    b[3] = hexchars[dec       & 0xF];
	b[4] = '\0';
	
    return b;
}

char *decHex16(uint16_t dec, char *b)
{	
	/* hex char set */
	char hexchars[] = "0123456789ABCDEF";
  	b[0] = '0';	
   	b[1] = 'x'; 	
    b[2] = hexchars[dec >> 12 & 0xF];  	
    b[3] = hexchars[dec >>  8 & 0xF];     
    b[4] = hexchars[dec >>  4 & 0xF];          
    b[5] = hexchars[dec       & 0xF];
	b[6] = '\0';
	
    return b;
}

char *decHex32(uint32_t dec, char *b)
{   
    /* hex char set */
    char hexchars[] = "0123456789ABCDEF";
    b[0] = '0';  
    b[1] = 'x';    
    b[2] = hexchars[dec >> 28 & 0xF];     
    b[3] = hexchars[dec >> 24 & 0xF];          
    b[4] = hexchars[dec >> 20 & 0xF];
    b[5] = hexchars[dec >> 16 & 0xF];      
    b[6] = hexchars[dec >> 12 & 0xF];     
    b[7] = hexchars[dec >>  8 & 0xF];          
    b[8] = hexchars[dec >>  4 & 0xF];
    b[9] = hexchars[dec       & 0xF];
	b[10] = '\0';
		     
    return b;
}

char *decBcd8(uint8_t dec, char *b)
{	
	/* hex sign */
	b[0] = '0';
	b[1] = 'x';

	b[2] = (dec / 10)+'0';
	b[3] = (dec % 10)+'0';
	
	b[4] = '\0';
	
	return b;
}

char *decBin8(uint8_t dec, char *b)
{
	unsigned char tempCnt = 0x02;
	unsigned char msk = 0x80;
	
	/* bin sign */
	b[0] = '0';
	b[1] = 'b';
	for(; tempCnt < 10 ; tempCnt++)
	{
		if(dec & msk)
			b[tempCnt] = '1';
		else
			b[tempCnt] = '0';
		
		msk>>=1;	
	}
	b[++tempCnt] = '\0';
	
	return b;
}

convert_t convert =
{
	.decHex8  = decHex8,
	.decHex16 = decHex16,
	.decHex32 = decHex32,
	
	.decBcd8 = decBcd8,
	.decBin8 = decBin8,
};