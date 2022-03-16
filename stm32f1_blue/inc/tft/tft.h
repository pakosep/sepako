/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			glcd.h
** Descriptions:		STM32 FSMC TFT
**
**------------------------------------------------------------------------------------------------------
** Created by:			poweravr
** Created date:		2010-11-7
** Version:					1.0
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/

#ifndef __TFT_H 
#define __TFT_H

/* TFT color */

#define   BLACK        0x0000                    /*  0,   0,    0           */
#define   NAVY         0x000F                    /*  0,   0,    128         */
#define   DGREEN       0x03E0                    /*  0,   128,  0           */
#define   DCYAN        0x03EF                    /*  0,   128,  128         */
#define   MAROON       0x7800                    /*  128, 0,    0           */
#define   PURPLE       0x780F                    /*  128, 0,    128         */
#define   OLIVE        0x7BE0                    /*  128, 128,  0           */
#define   LGRAY        0xC618                    /*  192, 192,  192         */
#define   DGRAY        0x7BEF                    /*  128, 128,  128         */
#define   BLUE         0x001F                    /*  0,   0,    255         */
#define   GREEN        0x07E0                 	 /*  0,   255,  0           */
#define   CYAN         0x07FF                    /*  0,   255,  255         */
#define   RED          0xF800                    /*  255, 0,    0           */
#define   MAGENTA      0xF81F                    /*  255, 0,    255         */
#define   YELLOW       0xFFE0                    /*  255, 255,  0           */
#define   WHITE        0xFFFF                    /*  255, 255,  255         */

typedef struct {
	u16 x;
	u16 y;
	u08 c_char;
	u16 FGcol;
	u16 BGcol;
	u08 scale;
	u32	font_addr;
	u08	font_W;
	u08	font_H;
} TFONT;

extern u16 ColCon[];

#define TFT_REG         (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define TFT_RAM         (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */
/* Includes ------------------------------------------------------------------*/
#include "inc/stm32f10x.h"
/* Private define ------------------------------------------------------------*/
#define 	TFT_YSTP	240			// Y_max
#define 	TFT_XSTR	0				// X_min
/* Private function prototypes -----------------------------------------------*/
void TFT_Init(void);
void TFT_BackLight_Init(void);
void TFT_BackLight(uint8_t percent);
void TFT_Clear(uint16_t Color);	

uint16_t TFT_BGR2RGB(uint16_t color);
uint16_t TFT_GetPoint(uint16_t Xpos,uint16_t Ypos);
void TFT_DrawPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point);
void TFT_DrawLine(int x1, int y1, int x2, int y2, uint16_t bkColor);
void TFT_DrawVerLine(int x1, int y1, int y2, uint16_t bkColor);
void TFT_SetWindow(uint16_t xStart,uint16_t yStart,uint16_t xEnd,uint16_t yEnd);
void TFT_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY,uint16_t *pic);
void TFT_WriteScaledText(char * str, u16 x, u16 y, u16 xscale, u16 yscale,u16 Color, u16 bkColor);
void TFT_ScaledFont(unsigned char code, u16 x, u16 y, u16 xscale, u16 yscale,u16 Color, u16 bkColor);
void TFT_SetScaledPixel(u16 x,u16 y, u16 px,u16 py, u16 xscale,  u16 yscale, uint16_t Color);
void TFT_SetSclPoint(u16 X,u16 Y, u16 px,u16 py, u16 xscale,  u16 yscale,u16 ColoR);

void TFT_dy (u16 x ,u08 y1  ,u08 y2, u16 color, u08 width);
void TFT_sdy			(u16 x ,u08 y1  ,u08 y2, u16 ColoR, u08 width);
void TFT_sdy_grid	(u16 x ,u16 color);

void TFT_vLine_grid(u16 x1, u16 color);
void TFT_test(void);
void TFT_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);
void TFT_txt(char* str, u16 x, u16 y,u16 fgCol);

//strBlock = (u32)(0x800*strBlock + 0x08000000);
#define FLASH_PAGE(x)		((0x800ul)*(x) + (0x08000000ul))
//#define W_FONT_1408			0x0807d000		//250 Page flash memory
#define W_FONT_1608		FLASH_PAGE(249)	 	//249 Page flash memory
#define W_FONT_1408		FLASH_PAGE(250)		//250 Page flash memory

/* TFT Registers */
#ifdef __TFT_H
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R229           0xE5

#define RGB565CONVERT(red, green, blue) (u16) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))
#define RGB(red, green, blue) 					(u16) ((((red*63) >> 3) << 11) | (((green*63) >> 2) << 5) | ((blue*63) >> 3))
#endif

#endif 
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/





