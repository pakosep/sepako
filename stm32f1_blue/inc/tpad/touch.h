#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"

#ifndef __TOUCH_H__
#define __TOUCH_H__

#define Key_Down 0x01
#define Key_Up   0x00 

typedef struct 
{
	u16 X0;
	u16 Y0;
	u16 X; 
	u16 Y;						   	    
	u08 Key_Sta;
	u08 Key_OK; 

	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;	   
extern Pen_Holder Pen_Point;

#define PEN  bitband_t m_BITBAND_PERIPH(&GPIOC->IDR, 13)  //PC13  INT

#define DOUT bitband_t m_BITBAND_PERIPH(&GPIOA->IDR, 6)   //PA6  MISO
#define TDIN bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 7)  //PA7  MOSI
#define TCLK bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 5)  //PA5  SCLK
#define TCS  bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 4)  //PA4  CS   
//ADS7843/7846/UH7843/7846/XPT2046/TSC2046
#define CMD_RDY 0x90  //0B10010000
#define CMD_RDX	0xD0  //0B11010000
#define TEMP_RD	0xF0  //0B11110000

//#define ADJ_SAVE_ENABLE	    

void Touch_Init(void);		 
int Read_ADS (u32 *x,u32 *y);	
int Read_ADS1(u32 *x,u32 *y);	
int Read_ADS2(u32 *x,u32 *y); 

void Draw_Touch_Point(u08 x,u16 y);
void Draw_Big_Point(u08 x,u16 y);  
void Touch_Adjust(void);          
void Save_Adjdata(void);		  
u08 Get_Adjdata(void); 			  
void Pen_Int_Set(u08 en); 		  
void Convert_Pos(void);       
u08 AI_Read_TP(void);           
u08 Is_In_Area(u08 x1,u16 y1,u08 x2,u16 y2);
#endif








