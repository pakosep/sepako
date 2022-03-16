/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			AsciiLib.h
** Descriptions:		None
**
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2010-11-2
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/

#ifndef __AsciiLib_H
#define __AsciiLib_H  

#ifndef VerPositionTFT
#define HerPositionTFT  	//Vertical Position use TFT
//#define HorPositionTFT  	//Horizontal Position use TFT
#endif
/* Includes ------------------------------------------------------------------*/
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define  ASCII_8X16_MS_Gothic
//#define  ASCII_8X16_System

/* Private function prototypes -----------------------------------------------*/
void GetASCIICode(unsigned char* pBuffer,unsigned char ASCII);

#endif 


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/



