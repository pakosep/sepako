
#include <string.h>
#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"
#include "inc/tool/delay.h"
#include "uc1608.h"
#include "config.h"
#include "inc/tool/UARTLib.h"

//=====================//
//   Lexmark printer   //
//  Resolution 160x64  //
//=====================//

static u08  UC1608_Buf [8][160];

/* Cache buffer in SRAM 84*48 bits or 504 uint8_ts */

void UC1608_chr(u08 w,u08 k,char p){
		p = p - 32;
		memcpy (UC1608_Buf[w]+k*6, Font1+p*5,5);
		*(UC1608_Buf[w]+k*6+5) = 0;
	}

void UC1608_str(u08 w, u08 k, char *s)	{
		u08 l=0;
		while (*s)		
		{
			UC1608_chr(w,k+l,*s);
			l++;
			s++;
		}
	}

void UC1608_Init(void){
		
		u08 n;
		u08 ini[146]={
		0xab ,0x92 ,0x27 ,0x44 ,0x24 ,0xa1 ,0x67 ,0x55 ,0x48 ,0x40 ,0x81 ,0x1b ,0x2f ,0x38 ,0x05 ,0x80 ,0x00 ,
		0x81 ,0x00 ,0x82 ,0x00 ,0x83 ,0x00 ,0x84 ,0x06 ,0x85 ,0x06 ,0x86 ,0x06 ,0x87 ,0x06 ,0x88 ,0x0b ,0x89 ,
		0x0b ,0x8a ,0x0b ,0x8b ,0x0b ,0x8c ,0x10 ,0x8d ,0x10 ,0x8e ,0x10 ,0x8f ,0x10 ,0x90 ,0x15 ,0x91 ,0x15 ,
		0x92 ,0x15 ,0x93 ,0x15 ,0x94 ,0x1a ,0x95 ,0x1a ,0x96 ,0x1a ,0x97 ,0x1a ,0x98 ,0x1f ,0x99 ,0x1f ,0x9a ,
		0x1f ,0x9b ,0x1f ,0x9c ,0x23 ,0x9d ,0x23 ,0x9e ,0x23 ,0x9f ,0x23 ,0xa0 ,0x27 ,0xa1 ,0x27 ,0xa2 ,0x27 ,
		0xa3 ,0x27 ,0xa4 ,0x2b ,0xa5 ,0x2b ,0xa6 ,0x2b ,0xa7 ,0x2b ,0xa8 ,0x2f ,0xa9 ,0x2f ,0xaa ,0x2f ,0xab ,
		0x2f ,0xac ,0x32 ,0xad ,0x32 ,0xae ,0x32 ,0xaf ,0x32 ,0xb0 ,0x35 ,0xb1 ,0x35 ,0xb2 ,0x35 ,0xb3 ,0x35 ,
		0xb4 ,0x38 ,0xb5 ,0x38 ,0xb6 ,0x38 ,0xb7 ,0x38 ,0xb8 ,0x3a ,0xb9 ,0x3a ,0xba ,0x3a ,0xbb ,0x3a ,0xbc ,
		0x3c ,0xbd ,0x3c ,0xbe ,0x3c ,0xbf ,0x3c ,0x38 ,0xc4 ,0xaf };
		
		
		LX_RES = 1;
		
		GPIOA->CRL = (GPIOA->CRL & 0x00000000) 	| 0x33333333;	// DAT0-DAT7
		GPIOB->CRL = (GPIOB->CRL & 0xffffff00) 	| 0x00000013;	// C/D,CLK
		GPIOB->CRH = (GPIOB->CRH & 0xffff00ff) 	| 0x00001100;	// CE,RES
		
		//GPIOA->ODR = (GPIOA->ODR & 0xff00 ) 		| 0x00ff 		;
		
		LX_RES = 1; Delay_ms(10);  LX_RES = 0; Delay_us(15); LX_RES = 1;	// hard reset
		
		//LX_CE  = 0;
		LX_CD  = 0;
		
		Delay_us(20);
		
		for(n=0;n<146;n++){		
			//GPIOA->ODR = ini[n];
			//GPIOA->ODR = ini2[n];
			GPIOA->BSRR = ini[n] | ((~ini[n])<<16);
			Delay_us(1);
			LX_CLK  = 1; Delay_us(1);
			LX_CLK  = 0; Delay_us(2);
		}		
		
		//LX_CD  = 1;
	}

void UC1608_Data(u08 dat){
		
		//GPIOA->ODR = dat ;
		GPIOA->BSRR = dat | ((~dat)<<16);
		/*Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);
		Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);
		Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);
		Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);*/
		
		LX_CLK  = 1;	LX_CLK  = 0; 			LX_CLK  = 1;	LX_CLK  = 0; 
		LX_CLK  = 1;	LX_CLK  = 0; 			LX_CLK  = 1;	LX_CLK  = 0; 
	}

void UC1608_Cmd(u08 dat){
		LX_CD  = 0;
		//GPIOA->ODR = dat ;
		GPIOA->BSRR = dat | ((~dat)<<16);
		LX_CLK  = 1;	LX_CLK  = 0; 		
		LX_CD  = 1;
	}

void UC1608_xy(u08 col, u08 row){
		LX_CD  = 0;
		u08 dat;
		
		dat = (row & 0x0f)| 0xb0  ;
		GPIOA->BSRR = dat | ((~dat)<<16);
		//GPIOA->ODR =  0xb0 | (row & 0x0f) ;		// page (row) address (0-7)
		LX_CLK  = 1; 	LX_CLK  = 0; 
		
		dat =  (col >> 4) | 0x10 ;
		GPIOA->BSRR = dat | ((~dat)<<16);
		//GPIOA->ODR =  0x10 | (col >> 4);			// column address MSB
		LX_CLK  = 1; 	LX_CLK  = 0; 
		
		dat =  (col & 0x0f) | 0x00 ;
		GPIOA->BSRR = dat | ((~dat)<<16);
		//GPIOA->ODR =  0x00 | (col & 0x0f);		// column address LSB
		LX_CLK  = 1; 	LX_CLK  = 0; 
		
		LX_CD  = 1;
	}

void UC1608_Clr(void){
		
		for(u08 w=0;w<8;w++){
			UC1608_xy(0, w);
			for(u08 n=0;n<160;n++) UC1608_Data(0xff);	
		}
	}

void UC1608_Refresh(void){
		UC1608_xy(0, 0);
		
		u08 * ptr_buf = UC1608_Buf[0] ;
		
		for (u16 n=0; n<8*160; n++){
			UC1608_Data(*ptr_buf++);
		}
		
	}