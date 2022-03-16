// OLED 0.96"
/** 
  ******************************************************************************
  * @file    SSD1306.c 
  * @author  Waveshare Team
  * @version 
  * @date    13-October-2014
  * @brief   This file includes the OLED driver for SSD1306 display moudle
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"
#include "inc/tool/delay.h"
#include "config.h"
#include "inc/tool/UARTLib.h"

//#include "LIB_Config.h"
#include "SSD1306.h"
#include "Fonts.h"
#include "swiss16x.h"
#include "swiss24x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SSD1306_CMD    0
#define SSD1306_DAT    1

#define SSD1306_WIDTH    128
#define SSD1306_HEIGHT   64

#define SSD1306

/* Private macro -------------------------------------------------------------*/

#if !defined(SH1106) && !defined(SSD1306)
	#warning Please select first the target OLED device(SH1106 or SSD1306) in your application!
	#define SSD1306  //define SSD1306 by default	
#endif

#define __SET_COL_START_ADDR() 	do { \
									ssd1306_write_byte(0x00, SSD1306_CMD); \
									ssd1306_write_byte(0x10, SSD1306_CMD); \
								} while(false)

/* Private variables ---------------------------------------------------------*/
static uint8_t s_chDispalyBuffer[128][8];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Writes an byte to the display data ram or the command register
  *         
  * @param  chData: Data to be writen to the display data ram or the command register
  * @param chCmd:  
  *                           0: Writes to the command register
  *                           1: Writes to the display data ram
  * @retval None
**/



#define sSCK_oled   bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 4) 	// D0
#define sMOSI_oled  bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 3)  // D1
#define DC_oled			bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 12)	// DC
#define CS_oled			bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 11)	// CS
#define RST_oled		bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 15)	// RESET

void _sendData(uint8_t byte) {
	uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			
			sMOSI_oled = (byte & 0x80) ? 1 : 0;
			byte <<= 1;
			//Delay_us(0);
			sSCK_oled = 1; /* a slave latches input data bit */
			//if (sMISO_oled)			byte |= 0x01;
			//Delay_us(0);
			sSCK_oled = 0; /* a slave shifts out next output data bit */
			
		}
		//return(byte);
	}

static void ssd1306_write_byte(uint8_t chData, uint8_t chCmd) {
		
		CS_oled=0; 
		
		if (chCmd) {
			DC_oled=1;
		} else {
			DC_oled=0;
		}	
		//__SSD1306_WRITE_BYTE(chData);
		_sendData(chData);
		
		DC_oled=1;
		CS_oled=1; 
	}   	  

/**
  * @brief  SSd1306 initialization
  *         
  * @param  None
  *         
  * @retval None
**/
void ssd1306_init(void)
{

	GPIOA->CRH = (GPIOA->CRH & 0x0ff00fff) | 0x10011000;	// 
	GPIOB->CRL = (GPIOB->CRL & 0x0ff00fff) | 0x10011000;	// 
	
	RST_oled = 1;
	CS_oled=0;   //CS set
	DC_oled=1;   //D/C reset
	//__SSD1306_RES_SET();  //RES set
	
	
	RST_oled = 1;
	Delay_ms(10);
	RST_oled = 0;
	Delay_ms(10);
	RST_oled = 1;
	

	ssd1306_write_byte(0xAE, SSD1306_CMD);//--turn off oled panel
	ssd1306_write_byte(0x00, SSD1306_CMD);//---set low column address
	ssd1306_write_byte(0x10, SSD1306_CMD);//---set high column address
	ssd1306_write_byte(0x40, SSD1306_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	ssd1306_write_byte(0x81, SSD1306_CMD);//--set contrast control register
	ssd1306_write_byte(0xCF, SSD1306_CMD);// Set SEG Output Current Brightness
	ssd1306_write_byte(0xA1, SSD1306_CMD);//--Set SEG/Column Mapping     
	ssd1306_write_byte(0xC0, SSD1306_CMD);//Set COM/Row Scan Direction   
	ssd1306_write_byte(0xA6, SSD1306_CMD);//--set normal display
	ssd1306_write_byte(0xA8, SSD1306_CMD);//--set multiplex ratio(1 to 64)
	ssd1306_write_byte(0x3f, SSD1306_CMD);//--1/64 duty
	ssd1306_write_byte(0xD3, SSD1306_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	ssd1306_write_byte(0x00, SSD1306_CMD);//-not offset
	ssd1306_write_byte(0xd5, SSD1306_CMD);//--set display clock divide ratio/oscillator frequency
	ssd1306_write_byte(0x80, SSD1306_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	ssd1306_write_byte(0xD9, SSD1306_CMD);//--set pre-charge period
	ssd1306_write_byte(0xF1, SSD1306_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	ssd1306_write_byte(0xDA, SSD1306_CMD);//--set com pins hardware configuration
	ssd1306_write_byte(0x12, SSD1306_CMD);
	ssd1306_write_byte(0xDB, SSD1306_CMD);//--set vcomh
	ssd1306_write_byte(0x40, SSD1306_CMD);//Set VCOM Deselect Level
	ssd1306_write_byte(0x20, SSD1306_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	ssd1306_write_byte(0x02, SSD1306_CMD);//
	ssd1306_write_byte(0x8D, SSD1306_CMD);//--set Charge Pump enable/disable
	ssd1306_write_byte(0x14, SSD1306_CMD);//--set(0x10) disable
	ssd1306_write_byte(0xA4, SSD1306_CMD);// Disable Entire Display On (0xa4/0xa5)
	ssd1306_write_byte(0xA6, SSD1306_CMD);// Disable Inverse Display On (0xa6/a7) 
	ssd1306_write_byte(0xAF, SSD1306_CMD);//--turn on oled panel
	
	ssd1306_clear_screen(0x00);
}

/**
  * @brief  OLED turns on 
  * @param  None
  * @retval None
**/ 
void ssd1306_display_on(void){
		ssd1306_write_byte(0x8D, SSD1306_CMD);  
		ssd1306_write_byte(0x14, SSD1306_CMD);  
		ssd1306_write_byte(0xAF, SSD1306_CMD);  
	}
  
/**
  * @brief  OLED turns off
  * @param  None         
  * @retval None
**/
void ssd1306_display_off(void){
		ssd1306_write_byte(0x8D, SSD1306_CMD);  
		ssd1306_write_byte(0x10, SSD1306_CMD); 
		ssd1306_write_byte(0xAE, SSD1306_CMD);  
	}

/**
  * @brief  Refreshs the graphic ram
  * @param  None
  * @retval None
**/

void ssd1306_refresh_gram(void){
		uint8_t i, j;
		
		for (i = 0; i < 8; i ++) {  
			ssd1306_write_byte(0xB0 + i, SSD1306_CMD);    
			__SET_COL_START_ADDR();      
			for (j = 0; j < 128; j ++) {
				ssd1306_write_byte(s_chDispalyBuffer[j][i], SSD1306_DAT); 
			}
		}   
	}

/**
  * @brief   Clears the screen
  * @param  None
  * @retval  None
**/

void ssd1306_clear_screen(uint8_t chFill)  
{ 
	uint8_t i, j;
	
	for (i = 0; i < 8; i ++) {
		ssd1306_write_byte(0xB0 + i, SSD1306_CMD);
		__SET_COL_START_ADDR();
		for (j = 0; j < 128; j ++) {
			s_chDispalyBuffer[j][i] = chFill;
		}
	}
	
	ssd1306_refresh_gram();
}

/**
  * @brief  Draws a piont on the screen
  * @param  chXpos: Specifies the X position
  * @param  chYpos: Specifies the Y position
  * @param  chPoint: 0: the point turns off    1: the piont turns on 
  * @retval None
**/

void ssd1306_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint){
		uint8_t chPos, chBx, chTemp = 0;
		
		if (chXpos > 127 || chYpos > 63) {
			return;
		}
		chPos = 7 - chYpos / 8; // 
		chBx = chYpos % 8;
		chTemp = 1 << (7 - chBx);
		
		if (chPoint) {
			s_chDispalyBuffer[chXpos][chPos] |= chTemp;
			
		} else {
			s_chDispalyBuffer[chXpos][chPos] &= ~chTemp;
		}
		
		/*
		sint2uart(chXpos);
		sint2uart(chPos);
		sint2uart(s_chDispalyBuffer[chXpos][chPos]); */
	}

void ssd1306_draw_byte(uint8_t col, uint8_t row,  uint8_t dat){
		
		if (col > 127 || row > 7) {
			return;
		}
		s_chDispalyBuffer[col][row] = dat;
		
	}	
	
/**
  * @brief  Fills a rectangle
  * @param  chXpos1: Specifies the X position 1 (X top left position)
  * @param  chYpos1: Specifies the Y position 1 (Y top left position)
  * @param  chXpos2: Specifies the X position 2 (X bottom right position)
  * @param  chYpos3: Specifies the Y position 2 (Y bottom right position)
  * @retval 
**/

void ssd1306_fill_screen(uint8_t chXpos1, uint8_t chYpos1, uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot) 
{  
	uint8_t chXpos, chYpos; 
	
	for (chXpos = chXpos1; chXpos <= chXpos2; chXpos ++) {
		for (chYpos = chYpos1; chYpos <= chYpos2; chYpos ++) {
			ssd1306_draw_point(chXpos, chYpos, chDot);
		}
	}	
	
	ssd1306_refresh_gram();
}


/**
  * @brief Displays one character at the specified position    
  *         
  * @param  chXpos: Specifies the X position
  * @param  chYpos: Specifies the Y position
  * @param  chSize: 
  * @param  chMode
  * @retval 
**/
void ssd1306_display_char(uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode)
	{      	
		uint8_t i, j;
		uint8_t chTemp, chYpos0 = chYpos;
		
		chChr = chChr - ' ';				   
    for (i = 0; i < chSize; i ++) {
			if (chSize == 12) {
				if (chMode) {
					chTemp =  c_chFont1206[chChr][i];
				} else {
					chTemp = ~c_chFont1206[chChr][i];
				}
			} else {
				if (chMode) {
					chTemp =  c_chFont1608[chChr][i];
				} else {
					chTemp = ~c_chFont1608[chChr][i];
				}
			}
			
			for (j = 0; j < 8; j ++) {
				if (chTemp & 0x80) {
					ssd1306_draw_point(chXpos, chYpos, 1);
				} else {
					ssd1306_draw_point(chXpos, chYpos, 0);
				}
				chTemp <<= 1;
				chYpos ++;
				
				if ((chYpos - chYpos0) == chSize) {
					chYpos = chYpos0;
					chXpos ++;
					break;
				}
			}	  	 
    } 
	}

static uint32_t pow_(uint8_t m, uint8_t n)
{
	uint32_t result = 1;	 
	while(n --) result *= m;    
	return result;
}	


void ssd1306_display_num(uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen, uint8_t chSize)
{         	
	uint8_t i;
	uint8_t chTemp, chShow = 0;
	
	for(i = 0; i < chLen; i ++) {
		chTemp = (chNum / pow_(10, chLen - i - 1)) % 10;
		if(chShow == 0 && i < (chLen - 1)) {
			if(chTemp == 0) {
				ssd1306_display_char(chXpos + (chSize / 2) * i, chYpos, ' ', chSize, 1);
				continue;
			} else {
				chShow = 1;
			}	 
		}
	 	ssd1306_display_char(chXpos + (chSize / 2) * i, chYpos, chTemp + '0', chSize, 1); 
	}
} 


/**
  * @brief  Displays a string on the screen
  *         
  * @param  chXpos: Specifies the X position
  * @param  chYpos: Specifies the Y position
  * @param  pchString: Pointer to a string to display on the screen 
  *         
  * @retval  None
**/
void ssd1306_display_string(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode)
{
    while (*pchString != '\0') {
      if (chXpos > (SSD1306_WIDTH - chSize / 2)) {
			chXpos = 0;
			chYpos += chSize;
			if (chYpos > (SSD1306_HEIGHT - chSize)) {
				chYpos = chXpos = 0;
				ssd1306_clear_screen(0x00);
			}
		}
		
      ssd1306_display_char(chXpos, chYpos, *pchString, chSize, chMode);
      chXpos += chSize / 2;
      pchString ++;
    }
}

void ssd1306_draw_1616char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 32; i ++) {
		chTemp = c_chFont1612[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0; 
			ssd1306_draw_point(chXpos, chYpos, chMode);
			chTemp <<= 1;
			chYpos ++;
			if ((chYpos - chYpos0) == 16) {
				chYpos = chYpos0;
				chXpos ++;
				break;
			}
		}
	}
}

void ssd1306_draw_3216char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 64; i ++) {
		chTemp = c_chFont3216[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0; 
			ssd1306_draw_point(chXpos, chYpos, chMode);
			chTemp <<= 1;
			chYpos ++;
			if ((chYpos - chYpos0) == 32) {
				chYpos = chYpos0;
				chXpos ++;
				break;
			}
		}
	}
}

void ssd1306_draw_1612char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 32; i ++) {
		chTemp = c_chFont1612[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0; 
			ssd1306_draw_point(chXpos, chYpos, chMode);
			chTemp <<= 1;
			chYpos ++;
			if ((chYpos - chYpos0) == 16) {
				chYpos = chYpos0;
				chXpos ++;
				break;
			}
		}
	}
}
void ssd1306_display_string3216(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString)
	{
    while (*pchString != '\0') {
      ssd1306_draw_3216char(chXpos, chYpos, (uint8_t) *pchString);
      chXpos += 16;
      pchString ++;
    } 		
	}

void ssd1306_display_string1612(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString)
	{
    while (*pchString != '\0') {
      ssd1306_draw_1612char(chXpos, chYpos, (uint8_t) *pchString);
      chXpos += 16;
      pchString ++;
    } 		
	}

void ssd1306_draw_bitmap(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight)
{
	uint16_t i, j, byteWidth = (chWidth + 7) / 8;
	
    for(j = 0; j < chHeight; j ++){
        for(i = 0; i < chWidth; i ++ ) {
            if(*(pchBmp + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                ssd1306_draw_point(chXpos + i, chYpos + j, 1);
            }
        }
    }
}

uint8_t ssd1306_char16(uint8_t chXpos, uint8_t chYpos, char chChar){
		
		// MCU font generator source otions = C/Vertical bytes/Top side MSB
		TCDATA * font;
		font  = font10x16[(u08)chChar];
		for(u08 m=0;m<font[0];m++){
			ssd1306_draw_byte(m+chXpos,chYpos+0,font[2*m+1]);		
			ssd1306_draw_byte(m+chXpos,chYpos+1,font[2*m+2]);		
		}
		return font[0]+1;
	}

void    ssd1306_str16(uint8_t chX, uint8_t chY, char *str){
		
		// MCU font generator source otions = C/Vertical bytes/Top side MSB
		u08 pos=0;
		while (*str)		
		{
			pos += ssd1306_char16(pos+chX,chY,*str);
			str++;
			if(pos>127) return;
		}
	}

uint8_t ssd1306_char24(uint8_t chXpos, uint8_t chYpos, char chChar){
		TCDATA * font;
		font  = font24[(u08)chChar];
		for(u08 m=0;m<font[0];m++){
			ssd1306_draw_byte(m+chXpos,chYpos+0,font[3*m+1]);		
			ssd1306_draw_byte(m+chXpos,chYpos+1,font[3*m+2]);		
			ssd1306_draw_byte(m+chXpos,chYpos+2,font[3*m+3]);		
		}
		return font[0]+1;
	}

void    ssd1306_num24(uint8_t chX, uint8_t chY, char *str){
		u08 pos=0;
		while (*str)		
		{
			pos += ssd1306_char24(pos+chX,chY,*str);
			str++;
			if(pos>127) return;
		}
	}

/*-------------------------------END OF FILE-------------------------------*/

