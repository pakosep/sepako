/** 
  ******************************************************************************
  * @file    oled.c 
  * @author  sepako
  * @version 
  * @date    18-Janur-2020
  * @brief   This file includes the OLED driver for oled display moudle
  ******************************************************************************
  * @attention SSD1606 oled 0.96",SH1306 oled 1.3"
  ******************************************************************************
	* Uwaga niektóre komendy maj¹ argument w komendzie a inne po !!!.
  */

/* Includes ------------------------------------------------------------------*/

//#include <string.h>
//#include "inc/stm32f4xx.h"
//#include "hdr/hdr_bitband.h"
#include "inc/tool/delay.h"
//#include "inc/tool/UARTLib.h"

//#include "LIB_Config.h"

#include "Fonts.h"
#include "swiss16x.h"
#include "swiss24x.h"
#include "oled.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define OLED_CMD    0
#define OLED_DAT    1

#define OLED_WIDTH    128
#define OLED_HEIGHT   64

/* Private macro -------------------------------------------------------------*/

#define oled
#if !defined(SH1106) && !defined(oled)
	#warning Please select first the target OLED device(SH1106 or oled) in your application!
	#define oled  //define oled by default	
#endif

#define __SET_COL_START_ADDR() 	do { \
	oled_write_byte(0x00, OLED_CMD); \
	oled_write_byte(0x10, OLED_CMD); \
	} while(0)

/* Private variables ---------------------------------------------------------*/
//static uint8_t oled_buffer[128][8];
static uint8_t oled_buffer[128][8];
volatile OLED_hw_t oled_hw;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void oled_spi(u08 byte) 
	{
		uint8_t counter;		
		for(counter = 8; counter; counter--)
		{
			*(oled_hw.mosi) = (byte & 0x80) ? 1 : 0;
			byte <<= 1;
			//Delay_us(0);
			*(oled_hw.sck) = 1; 
			//*(oled_hw_p->sck) = 1; 
			//Delay_us(0);
			*(oled_hw.sck) = 0; 
		}
	}

static void oled_write_byte(uint8_t chData, uint8_t chCmd) 
  {
		*(oled_hw.cs) = 0;
		*(oled_hw.dc) = chCmd ? 1 : 0 ;
		oled_spi(chData);
		*(oled_hw.dc) = 1;
		*(oled_hw.cs) = 1;
	}   	  

void ssd1306_init(void) // 0.96" 3.3V
{
	
	*(oled_hw.cs) = 1;
	oled_hw.ofs  =  0; 
	*(oled_hw.dc) = 1;
	
	*(oled_hw.rst) = 1;
	Delay_ms(10);
	*(oled_hw.rst) = 0;
	Delay_ms(10);
	*(oled_hw.rst) = 1;
	
	const u08 init1[]={
		0xAE, //--turn off oled panel
		0x00, //---set low column address
		0x10, //---set high column address
		0x40, //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
		0x81,0xCF, //--set contrast control register
		0xA1, //--Set SEG/Column Mapping     
		0xC0, //Set COM/Row Scan Direction   
		0xA6, //--set normal display
		0xA8,0x3f, //--set multiplex ratio(1 to 64) 1/64 duty
		0xD3,0x00, //-set display offset	Shift Mapping RAM Counter (0x00~0x3F) --not offset
		0xd5,0x80, //--set display clock divide ratio/oscillator frequency --Set Clock as 100 Frames/Sec
		0xD9,0xF1, //--set pre-charge period --as 15 Clocks & Discharge as 1 Clock
		0xDA,0x12,  //--set com pins hardware configuration
		0xDB,0x40, //--set vcomh //Set VCOM Deselect Level
		0x20, //-Set Page Addressing Mode (0x20/0x21/0x22)
		0x02, //
		0x8D, //--set Charge Pump enable/disable
		0x14, //--set(0x10) disable
		0xA4, // Disable Entire Display On (0xa4/0xa5)
		0xA6, // Disable Inverse Display On (0xa6/a7) 
		0xAF 	//--turn on oled panel
	};
	
	for(u08 i=0;i<sizeof(init1);i++){
		oled_write_byte( init1[i] , OLED_CMD); 
	}
	
	oled_clear_screen(0x00);
}

void sh1106_init(void) // 1.3" 3.3V
{

	//gpio_pin_cfg(GPIOE, 2, GPIO_OUT_PP_25MHz);	// SCK
	//gpio_pin_cfg(GPIOE, 3, GPIO_OUT_PP_25MHz);	// SDA
	//gpio_pin_cfg(GPIOE, 4, GPIO_OUT_PP_25MHz);	// Reset
	//gpio_pin_cfg(GPIOE, 5, GPIO_OUT_PP_25MHz);	// DC
	//gpio_pin_cfg(GPIOE, 6, GPIO_OUT_PP_25MHz);	// CS
		
	
	oled_hw.ofs  =  2;
	
	*(oled_hw.cs) = 0; // Mozna na stale do masy
	*(oled_hw.dc) = 0;
	
	*(oled_hw.rst) = 1;
	//RST_oled = 1;	Delay_ms(20);
	//RST_oled = 0;	Delay_ms(20);
	//RST_oled = 1;	Delay_ms(20);
 	
	const u08 init1[]={
		//0xE0,			//--Read-Modify-Write Start
		0xAE,			//--turn off oled panel
		0xd5,0x80,//--set display clock divide ratio/oscillator frequency 100 Frames/Sec 
		0xA8,0x3F,//--set multiplex ratio(1 to 64)
		0xD3,0x00,//--set display offset Shift Mapping RAM Counter (0x00~0x3F)
		0x40,			//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
		0xAD,0x8b,//--Set Charge Pump DC-DC Control Mode ON/OFF POR=0x8B
		0xA0,			//--Set Segment Re-Map
		0xC8,			//--Set COM Output Scan Direction 0xC0 (up-down) or 0xC8 (down-up)
		0xDA,0x12,//--set com pins hardware configuration
		0x81,0x00,//--set contrast control register POR=0x80
		0xD9,0x1F,//--set pre-charge period
		0xDB,0x40,//--set VCOMH Deselect Level
		0xA6,			//--Disable Inverse Display On (0xA6/0xA7) 
		//0xA0,			//--Set SEG/Column Mapping     
		//0xA4,		//--Disable Entire Display On (0xa4/0xa5)
		0xAF, 			//--turn on oled panel
		//0xEE			//--Read-Modify-Write End
    
	};

	for(u08 i=0;i<sizeof(init1);i++){
		oled_write_byte( init1[i] , OLED_CMD); 
	}

	oled_clear_screen(0x00);
}


void oled_refresh_gram(void)
{
		uint8_t i, j;
		
		//oled_write_byte(0xB0,OLED_CMD); // Page addr   
		
		for (i = 0; i < 8; i ++) {  
			oled_write_byte(0xB0 + i, OLED_CMD); // Page addr   
			oled_write_byte(0x00 | oled_hw.ofs, OLED_CMD); 		// Col addr 4bit low
			oled_write_byte(0x10, OLED_CMD); 		// Col addr 4bit high
			for (j = 0; j < 128; j ++) {
				oled_write_byte(oled_buffer[j][i], OLED_DAT); 
			}
		}   
	}

void oled_show_byte(u08 row, u08 col, u08 dat)
	{
		uint8_t i, j;
		col += oled_hw.ofs; // sh1106 przesuniety o 2
		
		oled_write_byte(0xB0 | (row & 0x0f)			, OLED_CMD); // Page addr   
		oled_write_byte(0x00 | (col & 0x0f)			, OLED_CMD); // Col addr 4bit low
		oled_write_byte(0x10 | ((col>>4) & 0x0f), OLED_CMD); // Col addr 4bit high
		oled_write_byte(dat, OLED_DAT); 
		
	}

void oled_test(void)
	{
		oled_write_byte(0xB0 , OLED_CMD); // Page addr   
		oled_write_byte(0x00 , OLED_CMD); // Col addr 4bit low
		oled_write_byte(0x10 , OLED_CMD); // Col addr 4bit high
		for(u16 i=0;i<64;i++){
			oled_write_byte(0x0f, OLED_DAT); 
		}
		
	}

void oled_show_lin(u08 row, u08 scol, u08 ecol, u08 dat)
	{
		uint8_t i, j;
		scol += oled_hw.ofs;
		ecol += oled_hw.ofs;
		
		for(u08 c=scol; c<ecol; c++){
			oled_write_byte(0xB0 | (row & 0x0f)		, OLED_CMD); // Page addr   
			oled_write_byte(0x00 | (c & 0x0f)			, OLED_CMD); // Col addr 4bit low
			oled_write_byte(0x10 | ((c>>4) & 0x0f), OLED_CMD); // Col addr 4bit high
			oled_write_byte(dat, OLED_DAT); 
		}
		
	}

void oled_clear_screen(uint8_t chFill)  
{ 
	uint8_t i, j;
	
	for (i = 0; i < 8; i ++) {
		oled_write_byte(0xB0 + i, OLED_CMD); // ustaw strone
		oled_write_byte(0x00|oled_hw.ofs, OLED_CMD); 		
		oled_write_byte(0x10, OLED_CMD); 
		//__SET_COL_START_ADDR();
		for (j = 0; j < 128; j ++) {
			oled_buffer[j][i] = chFill;
		}
	}
	
	oled_refresh_gram();
}

void oled_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint)
{
		uint8_t chPos, chBx, chTemp = 0;
		
		if (chXpos > 127 || chYpos > 63) {
			return;
		}
		chPos = 7 - chYpos / 8; // 
		chBx = chYpos % 8;
		chTemp = 1 << (7 - chBx);
		
		if (chPoint) {
			oled_buffer[chXpos][chPos] |= chTemp;
			
		} else {
			oled_buffer[chXpos][chPos] &= ~chTemp;
		}
		
		/*
		sint2uart(chXpos);
		sint2uart(chPos);
		sint2uart(oled_buffer[chXpos][chPos]); */
	}

void oled_draw_byte(uint8_t col, uint8_t row,  uint8_t dat)
{
		
		if (col > 127 || row > 7) {
			return;
		}
		oled_buffer[col][row] = dat;
		
	}	
	

void oled_display_char(uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode)
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
					oled_draw_point(chXpos, chYpos, 1);
				} else {
					oled_draw_point(chXpos, chYpos, 0);
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


void oled_display_num(uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen, uint8_t chSize)
{         	
	uint8_t i;
	uint8_t chTemp, chShow = 0;
	
	for(i = 0; i < chLen; i ++) {
		chTemp = (chNum / pow_(10, chLen - i - 1)) % 10;
		if(chShow == 0 && i < (chLen - 1)) {
			if(chTemp == 0) {
				oled_display_char(chXpos + (chSize / 2) * i, chYpos, ' ', chSize, 1);
				continue;
			} else {
				chShow = 1;
			}	 
		}
	 	oled_display_char(chXpos + (chSize / 2) * i, chYpos, chTemp + '0', chSize, 1); 
	}
} 


void oled_display_string(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode)
{
    while (*pchString != '\0') {
      if (chXpos > (OLED_WIDTH - chSize / 2)) {
			chXpos = 0;
			chYpos += chSize;
			if (chYpos > (OLED_HEIGHT - chSize)) {
				chYpos = chXpos = 0;
				oled_clear_screen(0x00);
			}
		}
		
      oled_display_char(chXpos, chYpos, *pchString, chSize, chMode);
      chXpos += chSize / 2;
      pchString ++;
    }
}

void oled_draw_1616char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 32; i ++) {
		chTemp = c_chFont1612[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0; 
			oled_draw_point(chXpos, chYpos, chMode);
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

void oled_draw_3216char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 64; i ++) {
		chTemp = c_chFont3216[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0; 
			oled_draw_point(chXpos, chYpos, chMode);
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

void oled_draw_1612char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 32; i ++) {
		chTemp = c_chFont1612[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0; 
			oled_draw_point(chXpos, chYpos, chMode);
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
void oled_display_string3216(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString)
{
    while (*pchString != '\0') {
      oled_draw_3216char(chXpos, chYpos, (uint8_t) *pchString);
      chXpos += 16;
      pchString ++;
    } 		
	}

void oled_display_string1612(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString)
{
    while (*pchString != '\0') {
      oled_draw_1612char(chXpos, chYpos, (uint8_t) *pchString);
      chXpos += 16;
      pchString ++;
    } 		
	}

void oled_draw_bitmap(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight)
{
	uint16_t i, j, byteWidth = (chWidth + 7) / 8;
	
    for(j = 0; j < chHeight; j ++){
        for(i = 0; i < chWidth; i ++ ) {
            if(*(pchBmp + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                oled_draw_point(chXpos + i, chYpos + j, 1);
            }
        }
    }
}

uint8_t oled_char16(uint8_t chXpos, uint8_t chYpos, char chChar)
	{
		
		// MCU font generator source otions = C/Vertical bytes/Top side MSB
		TCDATA * font;
		font  = font10x16[(u08)chChar];
		for(u08 m=0;m<font[0];m++){
			oled_draw_byte(m+chXpos,chYpos+0,font[2*m+1]);		
			oled_draw_byte(m+chXpos,chYpos+1,font[2*m+2]);		
		}
		return font[0]+1;
	}

void    oled_str16(uint8_t chX, uint8_t chY, char *str)
	{
		
		// MCU font generator source otions = C/Vertical bytes/Top side MSB
		u08 pos=0;
		while (*str)		
		{
			pos += oled_char16(pos+chX,chY,*str);
			str++;
			if(pos>127) return;
		}
	}

uint8_t oled_char24(uint8_t chXpos, uint8_t chYpos, char chChar)
	{
		TCDATA * font;
		font  = font24[(u08)chChar];
		for(u08 m=0;m<font[0];m++){
			oled_draw_byte(m+chXpos,chYpos+0,font[3*m+1]);		
			oled_draw_byte(m+chXpos,chYpos+1,font[3*m+2]);		
			oled_draw_byte(m+chXpos,chYpos+2,font[3*m+3]);		
		}
		return font[0]+1;
	}

void    oled_num24(uint8_t chX, uint8_t chY, char *str)
	{
		u08 pos=0;
		while (*str)		
		{
			pos += oled_char24(pos+chX,chY,*str);
			str++;
			if(pos>127) return;
		}
	}

/*-------------------------------END OF FILE-------------------------------*/

