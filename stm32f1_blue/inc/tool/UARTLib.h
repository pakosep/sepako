#ifndef __UARTLIB_H
#define __UARTLIB_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define UART1_RXB		16			// size of circle buffor
#define UART2_RXB		16			// size of circle buffor
#define StrP122			0x0801E800		//122 Page see flash.c  1_Page=1024 byte
#define StrP119			0x0801DC00		//119 Page see flash.c	1_Page=1024 byte

typedef struct {
		uint16_t	rri;		// pozycja odczytu
		uint16_t  rwi;		// pozycja zapisu 
		uint16_t  rct;		// wskaźnik zajetosci
		char		 rbuf[UART1_RXB];
	} T_FIFO;

//extern volatile T_FIFO Fifo1, Fifo2;
extern volatile T_FIFO *p_Fifo1,*p_Fifo2;

void (*PutChar)(char a);

char *PSTR	(char *str);
char *strrev(char *str);
void reverse(char s[]);

void UART1_putc(char );
void UART2_putc(char );
void UART3_putc(char );

void UaPutC(char  p);
void UaPutS(char *s);
void UaPutK(const char *s);

void UaPutC2(char p);
void UaPutC3(char p);
void UART_getChar2( char *Char );

void sint2uart(s32 val);										
void uint2uart(u32 val);										
void unt2uart (u32 val ,u08 digit);
void int2uart (s32 val ,u08 digit, u08 k);	
void int2uarz (s32 val ,u08 digit, u08 k);
void num2uart (u32 val ,u08 digit, u08 k);	
void hex2uart (u32 val ,u08 digit);

char *sint2str(s32 val,char *s);
char *uint2str(u32 val,char *s);
char *unt2str (u32 val,char *s,u08 digit);
char *int2str (s32 val,char *s,u08 digit,u08 dot);
char *num2str (u32 num,char *s,u08 digit,u08 dot);
char *hex2str (u32 hex,char *s,u08 digit);

s32  str2int(char *);
int  str2heX(char *);
u32  str2hex(char *);

u32   UART_getNum(void);
u08 	UART_getDec(s32 *num);
u32   UART_getHex(void);
void  UART_getChar(char *);
char *UART_getStr (char *);
char  getDateTime_FAT(void);

void uart_putint( u32, u08);
void Text1(u08 addr);
void StrFF(u16 addr);

#ifdef __cplusplus

}
#endif
#endif //__UARTLIB_H
