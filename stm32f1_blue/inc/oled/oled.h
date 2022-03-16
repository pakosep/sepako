/**
  ******************************************************************************
  * @file    oled.h
  * @author  Waveshare Team
  * @version 
  * @date    13-October-2014
  * @brief   This file contains all the functions prototypes for the oled OLED firmware driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _OLED_H_
#define _OLED_H_

/* Includes ------------------------------------------------------------------*/
//#include "inc/stm32f4xx.h"
//#include "config.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
	volatile u32 *mosi;
	volatile u32 *sck;
	volatile u32 *dc;
	volatile u32 *cs;
	volatile u32 *rst;
	u08 ofs;
} OLED_hw_t;

extern volatile OLED_hw_t oled_hw;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

void oled_show_byte(u08 row, u08 col, u08 dat);
void oled_show_lin(u08 row, u08 scol, u08 ecol, u08 dat);
void oled_refresh_gram(void);
void oled_test(void);

void oled_clear_screen(uint8_t chFill);
void oled_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint);
void oled_display_char(uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode);
void oled_display_num(uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen,uint8_t chSize);
void oled_display_string(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode);
void oled_draw_1616char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar);
void oled_draw_3216char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar);
void oled_draw_1612char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar);

void oled_draw_bitmap(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight);
void oled_display_string3216(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString);

void ssd1306_init(void);
void sh1106_init(void);


void oled_draw_byte(uint8_t col, uint8_t row,  uint8_t dat);
uint8_t oled_char16(uint8_t chXpos, uint8_t chYpos, char chChar);
uint8_t oled_char24(uint8_t chXpos, uint8_t chYpos, char chChar);
void oled_str16(uint8_t chX, uint8_t chY, char *str);
void oled_num24(uint8_t chX, uint8_t chY, char *str);

#endif

/*-------------------------------END OF FILE-------------------------------*/


