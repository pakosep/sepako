/****************************************Copyright (c)************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-----------------------------------------------------
** File name:			TFT.c
** Descriptions:		STM32 FSMC TFT
**						
**----------------------------------------------------------------------------
 SSD 1289 Driver 
**--Includes -----------------------------------------------------------------*/
#include <math.h>
#include "inc/tool/delay.h"
#include "tft.h"
#include "inc/tool/UARTLib.h"
#include "fonts.h"

#define VerPosition
/* Private variables ---------------------------------------------------------*/
static uint16_t DeviceCode;
uint32_t TFT_FONT = W_FONT_1408;

u16 ColCon[16]={
RGB565CONVERT(0x00, 0x00, 0x00),RGB565CONVERT(0x00, 0x00, 0x80),RGB565CONVERT(0x00, 0x80, 0x00),RGB565CONVERT(0x00, 0x80, 0x80),
RGB565CONVERT(0xcc, 0x00, 0x00),RGB565CONVERT(0x99, 0x00, 0xcc),RGB565CONVERT(0xcc, 0x99, 0x00),RGB565CONVERT(0xcc, 0xcc, 0xcc),
RGB565CONVERT(0x66, 0x66, 0x66),RGB565CONVERT(0x00, 0x00, 0xff),RGB565CONVERT(0x00, 0xff, 0x00),RGB565CONVERT(0x00, 0xff, 0xff),
RGB565CONVERT(0xff, 0x00, 0x00),RGB565CONVERT(0xff, 0x00, 0xff),RGB565CONVERT(0xff, 0xff, 0x00),RGB565CONVERT(0xff, 0xff, 0xff)};
#define TFT_dly	5
#define TFT_Xmax	319
#define TFT_Ymax	239

/* Private define ------------------------------------------------------------*/
static __inline void TFT_FSMCConfig(void){
		RCC->AHBENR  |= RCC_AHBENR_FSMCEN | RCC_AHBENR_SRAMEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;
		
		GPIOD->CRL = (GPIOD->CRL & 0x0f00ff00) | 0xb0bb00bb;
    GPIOD->CRH = (GPIOD->CRH & 0x00ff0000) | 0xbb00bbbb;
    
		GPIOE->CRL = (GPIOE->CRL & 0x0fffffff) | 0xb0000000;
    GPIOE->CRH = 0xbbbbbbbb;
    
		
	/*      FSMC_Bank1->BTCR[0] = (1<<12); ->   FSMC_BCR1->WREN = 1;
	- rejestr BCR1 odpowiada:      FSMC_Bank1->BTCR[0]
	- rejestr BTR1 odpowiada:      FSMC_Bank1->BTCR[1]
	- rejestr BWTR1 odpowiada:     FSMC_Bank1E->BWTR[0]
	*/
	// set FSMC_BCR1
	FSMC_Bank1->BTCR[0] = 0x1011;
  /*
  FSMC_Bank1->BTCR[0] |= (1<<12) ;  // Enable Write
  FSMC_Bank1->BTCR[0] |= (1<<14) |  // EXTMOD=0x1 mode B (This bit enables the FSMC to program inside the FSMC_BWTR register, so it allows different timings for read and write.)
               (1<<6)  |   					// FACCEN - NOR Flash memory access is enabled
               (1<<4)  |   					// MWID - 0x1 - 16 bit data width
               (1<<3)  |   					// NOR Flash
               (1<<0)  ;  */   			// MBKEN Enables the memory bank
  //SET FSMC_BTR1
	
	//FSMC_Bank1->BTCR[1] = 0x00000202;
	FSMC_Bank1->BTCR[1] = 	0x00000800;		//  FREQUENCY = 128MHz
	//FSMC_Bank1->BTCR[1] = 	0x00000400;	//  FREQUENCY = 72MHz
	//FSMC_Bank1->BTCR[1] = 	0x0fffffff;	// default
	//FSMC_Bank1E->BWTR[0] =  0x0fffffff;	//Default
	
  //  FSMC_Bank1->BTCR[1] = (0x5<<8) | 	(1<<0) ;   			// ADDSET = 0x1
  //SET FSMC_BWTR1
  //  FSMC_Bank1E->BWTR[0] = (0x5<<8) | (1<<0) ;   			// ADDSET = 0x1
}

static __inline __attribute__((always_inline)) void TFT_WriteReg(uint8_t TFT_Reg,uint16_t TFT_RegValue){
		TFT_REG = TFT_Reg; 				/* Write 16-bit Index, then Write Reg */
		TFT_RAM = TFT_RegValue;		/* Write 16-bit Reg */
	}

static __inline __attribute__((always_inline)) uint16_t TFT_ReadReg(uint8_t TFT_Reg){
		TFT_REG = TFT_Reg;		/* Write 16-bit Index (then Read Reg) */
		return (TFT_RAM);			/* Read 16-bit Reg */
	}
static __inline __attribute__((always_inline)) void TFT_WriteRAM_Prepare(void){	
		TFT_REG = R34;	
	}
static __inline __attribute__((always_inline)) uint16_t TFT_ReadRAM(void){
		// volatile uint16_t dummy; 
		/* Write 16-bit Index (then Read Reg) 	*/
		TFT_REG = R34; 		/* Select GRAM Reg 		*/
	 // dummy = TFT_RAM; 	/* Read 16-bit Reg 	*/
		return TFT_RAM;
	}
static __inline __attribute__((always_inline)) void TFT_WriteRAM(uint16_t RGB_Code)	{
		/* Write 16-bit GRAM Reg */
		TFT_RAM = RGB_Code;
	}										 
void TFT_SetCursor(uint16_t Xpos,uint16_t Ypos) {
		// Wypelnianie pixeli z lewej do prawej i z do³u z góry
		TFT_WriteReg(0x4e,Ypos); 	/* Row */
		TFT_WriteReg(0x4f,Xpos); 	/* Line */
		TFT_REG = R34;
		
		//TFT_WriteReg(0x4e,239-Ypos); 	/* Row */
		//TFT_WriteReg(0x4f,319-Xpos); 	/* Line */
		//TFT_REG = R34;
	}
void TFT_SetWindow(uint16_t xStart,uint16_t yStart,uint16_t xEnd,uint16_t yEnd) 	{
		
		// set window
		TFT_WriteReg(0x45,xStart); 							// R46 > R45
		TFT_WriteReg(0x46,xEnd);         				// R46 > R45
		TFT_WriteReg(0x44,(yEnd<<8) | yStart);  // MSB > LSB
		
		// set cursor
		TFT_WriteReg(0x4f,xStart); 						// Line 
		TFT_WriteReg(0x4e,yStart); 						// Row 
		
		TFT_REG = R34;
	}

void TFT_Init(void) {
	u08 txd=10;
  
	/* Configure the FSMC Parallel interface -------------------------------------*/
	TFT_FSMCConfig();
	/* Configure BackLight TFT ---------------------------------------------------*/
	TFT_BackLight_Init();		//TFT_BackLight(BKP->DR41);
	TFT_BackLight(30);
  Delay_ms(5);  /* delay 50 ms */
  //DeviceCode = TFT_ReadReg(0x0000);		//SSD1289
  //if(DeviceCode==0x8989) 							//0x8989
 if(1)
  {
		TFT_WriteReg(0x00,0x0001);    Delay_us(txd);  
    TFT_WriteReg(0x03,0xA8A4);    Delay_us(txd);   
    TFT_WriteReg(0x0C,0x0000);    Delay_us(txd);   
    TFT_WriteReg(0x0D,0x080C);    Delay_us(txd);   
    TFT_WriteReg(0x0E,0x2B00);    Delay_us(txd);   
    TFT_WriteReg(0x1E,0x00B0);    Delay_us(txd);   
    TFT_WriteReg(0x01,0x693F);    Delay_us(txd);   //2b3f 320*240 693F
    TFT_WriteReg(0x02,0x0600);    Delay_us(txd);
    TFT_WriteReg(0x10,0x0000);    Delay_us(txd);
		//                       TRANS   OEDef    WMode  DMode     ID1,ID0,AM
    TFT_WriteReg(0x11,0x6040|(0<<12)|(0<<11)|(0<<10)|(0b00<<8)|(0b110<<3) );    Delay_us(txd);   //0x6070
		//TFT_WriteReg(0x11,0x6070);
		
    TFT_WriteReg(0x05,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x06,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x16,0xEF1C);    Delay_us(txd);
    TFT_WriteReg(0x17,0x0003);    Delay_us(txd);
    TFT_WriteReg(0x07,0x0233);    Delay_us(txd);   //0x0233       
    TFT_WriteReg(0x0B,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x0F,0x0000);    Delay_us(txd);   
    TFT_WriteReg(0x41,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x42,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x48,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x49,0x013F);    Delay_us(txd);
    TFT_WriteReg(0x4A,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x4B,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x44,0xEF00);    Delay_us(txd);
    TFT_WriteReg(0x45,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x46,0x013F);    Delay_us(txd);
    TFT_WriteReg(0x30,0x0707);    Delay_us(txd);
    TFT_WriteReg(0x31,0x0204);    Delay_us(txd);
    TFT_WriteReg(0x32,0x0204);    Delay_us(txd);
    TFT_WriteReg(0x33,0x0502);    Delay_us(txd);
    TFT_WriteReg(0x34,0x0507);    Delay_us(txd);
    TFT_WriteReg(0x35,0x0204);    Delay_us(txd);
    TFT_WriteReg(0x36,0x0204);    Delay_us(txd);
    TFT_WriteReg(0x37,0x0502);    Delay_us(txd);
    TFT_WriteReg(0x3A,0x0302);    Delay_us(txd);
    TFT_WriteReg(0x3B,0x0302);    Delay_us(txd);
    TFT_WriteReg(0x23,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x24,0x0000);    Delay_us(txd);
    TFT_WriteReg(0x25,0x8000);    Delay_us(txd);
    TFT_WriteReg(0x4f,0);        
    TFT_WriteReg(0x4e,0);        
  }
	/*
 else {
  const uint8_t _SSD1298[35*3] =
  {
    0x28,0x00,0x06,
    0x00,0x00,0x01,    // Oscillation start
    0x03,0xae,0xa4,    //  Power Control 1   Line frequency and VHG,VGL voltage
    0x0c,0x00,0x04,    //  Power Control 2   VCIX2 output voltage
    0x0d,0x00,0x0c,    //  Power Control 3   Vlcd63 voltage
    0x0e,0x28,0x00,    //  Power Control 4   VCOMA voltage VCOML=VCOMH*0.9475-VCOMA
    0x1e,0x00,0xb5,    //  Power Control 5   VCOMH voltage
    0x01,0x23,0x3f,    //  Driver Output
    0x02,0x06,0x00,    //  TFT Driver AC control
    0x10,0x00,0x00,    //  Sleep mode
    0x11,0x68,0x18,    //  Entry mode 0x6830
    // 0x05,0x00,0x00,    //  Not used
    // 0x06,0x00,0x00,    //  Not used
    0x16,0xef,0x1c,
    0x07,0x03,0x33,    //  Display Control 1
    0x0b,0x00,0x00,    //  Frame cycle control
    0x0f,0x00,0x00,    //  Gate scan
    0x41,0x00,0x00,    //  Vertical scroll control 1
    0x42,0x00,0x00,    //  Vertical scroll control 2
    0x48,0x00,0x00,    //  First window start
    0x49,0x01,0x3f,    //  First window end
    0x4a,0x00,0x00,    //  Second window start
    0x4b,0x00,0x00,    //  Second window end
    // Don't need to init these here
    //0x44,0xEF,0x00,    //  Horizontal RAM start and end address
    //0x45,0x00,0x00,    //  Vertical RAM start address
    //0x46,0x01,0x3F,    //  Vertical RAM end address
    //0x4e,0x00,0x00,    //  GDDRAM X
    //0x4f,0x00,0x00,    //  GDDRAM Y
    0x30,0x07,0x07,    //  Gamma control 1-10
    0x31,0x02,0x02,
    0x32,0x02,0x04,
    0x33,0x05,0x02,
    0x34,0x05,0x07,
    0x35,0x02,0x04,
    0x36,0x02,0x04,
    0x37,0x05,0x02,
    0x3a,0x03,0x02,
    0x3b,0x03,0x02,
		//   0x23,0x00,0x00,    //  Not used
		//   0x24,0x00,0x00,
    0x25,0x80,0x00,    //  Frame frequency 65Hz
    0x26,0x70,0x00,    //
    0x20,0xb0,0xeb,
    0x27,0x00,0x7c,
  };
		uint8_t count = sizeof(_SSD1298)/3;
    const uint8_t* b = _SSD1298;
    while (count--)
    {
      uint8_t r = *b++;
      int hi = *b++;
      TFT_WriteReg(r,(hi << 8) | *b++);
    }
  } */
		Delay_ms(5);  /* delay 50 ms */
	}
void TFT_Clear(uint16_t Color) {
		uint32_t index=0;
		TFT_SetCursor(0,0); 
		
		for(index=0;index<76800;index++)
		{
			TFT_RAM=Color;
		}
	}

uint16_t TFT_GetPoint(uint16_t Xpos,uint16_t Ypos) {
  TFT_SetCursor(Xpos,Ypos);
  if( DeviceCode==0x7783 || DeviceCode==0x4531 || DeviceCode==0x8989 )
 return ( TFT_ReadRAM() );
  else
 return ( TFT_BGR2RGB(TFT_ReadRAM()) );
}

void TFT_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY,uint16_t *pic) {
		uint16_t  i;
		TFT_SetCursor(StartX,StartY);  
		
		for (i=0;i<(EndX*EndY);i++)
		{
			TFT_WriteRAM(*pic++);
		}
	}
void TFT_DrawPoint(uint16_t Xpos,uint16_t Ypos,uint16_t ColoR) {
		//if ( ( Xpos > 319 ) || ( Ypos > 239 ) ) return;
		Xpos = Xpos%320;
		Ypos = Ypos%240;
		TFT_SetCursor(Xpos,Ypos);  		
		TFT_WriteRAM(ColoR);
	}
//=====================================================================
void TFT_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)	{
		// Wypelnianie pixeli z do³u do góry i lewej do prawej 
    uint32_t n;
		
		//TFT_SetCursor(xsta,ysta);
		if ( xsta>319 || xend>319) return;
		if ( ysta>239 || yend>239) return;
		TFT_SetWindow(xsta,ysta,xend,yend);
		
		n=(u32)(yend-ysta+1)*(xend-xsta+1);    
		while(n--)
		{
			TFT_RAM = color;
		}
		
	}
// Bresenham's algorithm
void TFT_DrawLine(int x0, int y0, int x1, int y1, uint16_t bkColor)  {
    
    int dy = y1 - y0;
    int dx = x1 - x0;
    int stepx, stepy;
    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
    dy <<= 1;                                                  // dy is now 2*dy
    dx <<= 1;                                                  // dx is now 2*dx
		TFT_DrawPoint(x0,y0,bkColor);	/* first pixel */
    if (dx > dy) 
		{
      int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
      while (x0 != x1) 
			{
        if (fraction >= 0) {
          y0 += stepy;
          fraction -= dx;                                // same as fraction -= 2*dx
        }
        x0 += stepx;
        fraction += dy;                                    // same as fraction -= 2*dy
        TFT_DrawPoint(x0,y0,bkColor);
      }
    } else 
		{
      int fraction = dx - (dy >> 1);
      while (y0 != y1) 
			{
        if (fraction >= 0) 
				{
          x0 += stepx;
          fraction -= dy;
        }
        y0 += stepy;
        fraction += dx;
        TFT_DrawPoint(x0,y0,bkColor);
      }
    }
  } 
	
void TFT_DrawVerLine(int x1, int y1, int y2, uint16_t bkColor)	{
		if (y1 > 239) y1 = 239;
    if (y2 > 239) y2 = 239;
		
		int dy = (y2 - y1);
		int y; 		
		if(dy > 0)
		{
			y = dy/2; 
		 TFT_SetCursor(x1,y1+y);
		  
			while(y--) TFT_RAM=bkColor;
			
			y = dy/2; 	
		 TFT_SetCursor(x1+1,y2);
		  
			do{ TFT_RAM=bkColor; }
			while(y--);
		}
		else if(dy < 0)
		{
			y = -dy/2; 
		 TFT_SetCursor(x1,y1);
		  
			while(y--) TFT_RAM=bkColor;
			
			y = -dy/2; 	
		 TFT_SetCursor(x1+1,y1-y);
		  
			do{ TFT_RAM=bkColor; }
			while(y--);			
		}
		else 
		{
			TFT_SetCursor(x1+1,y1);
		  
			TFT_RAM=bkColor;
		}
	}
//#if ASCII_LIB > 0
//-------------------------------------------------------------------------------------------------
//#endif
uint16_t TFT_BGR2RGB(uint16_t color)			{
  uint16_t  r, g, b, rgb;

  b = ( color>>0 )  & 0x1f;
  g = ( color>>5 )  & 0x3f;
  r = ( color>>11 ) & 0x1f;
 
  rgb =  (b<<11) + (g<<5) + (r<<0);
//  (((r&248) << 8 )|((g&252)<<3)|(b>>3))  conwert rgb to 565

  return( rgb );
}

void TFT_BackLight_Init(void)							{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 			// w³¹czenie portu A oraz TIMERA2
    GPIOB->CRL    = (GPIOB->CRL & 0xff0fffff) | 0x00B00000;	// PB5 Alternete Function
    AFIO->MAPR   |= AFIO_MAPR_TIM3_REMAP_1;		// Remap TIM3 CH2->PB5
		
		//TIM3->CCMR1 = 0x6800;		//set PWM mode on CH1,CH2	0x06868
		//TIM3->CCER 	= 0x0010;		//enable CH2 output		0x010		
		//TIM3->CR1 	= 0x0000;		//Counter Disable			0x000
		
		TIM3->CCMR1 = TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
		TIM3->CCER 	= TIM_CCER_CC2E;
		TIM3->BDTR 	= TIM_BDTR_MOE;
    
		// Fpwm = Ftim/((PSC+1)*(ARR+1))--
		// Dpwm = 100 * CCRx/(ARR+1) 							[%]
    TIM3->PSC 	= 10000;	//set prescaler			4000
    TIM3->ARR 	= 99;		  //PWM reload count	99
		TIM3->CCR2 	= 50;		  //PWM Duty cycyle = (CCR2/ARR)*100%	50  
    
		TIM3->EGR 	= TIM_EGR_UG;
		TIM3->CR1 	= (TIM_CR1_ARPE | TIM_CR1_CEN); //w³¹czenie licznika i w³¹czenie generowania przerwañ tylko podczas prze³adowania licznika
	}
	
void TFT_BackLight( uint8_t logic_percent){
  if(logic_percent <= 100)
		{
			TIM3->CCR2 = (uint16_t)((TIM3->ARR + 1)*logic_percent)/100;
		}
	}
void TFT_SetScaledPixel(u16 x,u16 y, u16 px,u16 py, u16 xscale,  u16 yscale, uint16_t Color)					{
u16 i, j;
for(j = 0; j < yscale; j++)	{
	for(i = 0; i < xscale; i++)		{
//		TFT_SetPixel(x + (xscale * px) + i, y + (yscale * py) + j, color);
		TFT_DrawPoint(x + (xscale * px) + i,y + (yscale * py) + j,Color);
		}
	}
}

// X,Y wspolrzedne, px,py przesuniecie skali, xsacle,yscale skala
void TFT_SetSclPoint(u16 X,u16 Y, u16 px,u16 py, u16 xscale,  u16 yscale,u16 ColoR)										{
  if ( ( X > 319 ) || ( Y > 239 ) ) return;
  u16 i, j;
  for(j = 0; j < yscale; j++)	{
  	for(i = 0; i < xscale; i++)		{
			TFT_SetCursor(Y + (yscale * py) + j,X + (xscale * px) + i);
			TFT_WriteRAM(ColoR);
		}
  }
}
void TFT_ScaledFont(unsigned char code, u16 x, u16 y, u16 xscale, u16 yscale,u16 Color, u16 bkColor)	{
unsigned int xpos, ypos;
// char * ptr;
char ch=0;
code -= 32;

for(xpos = 0; xpos < 5; xpos++)  {
	 // ptr = (char *)(font5x8 + (5 * code) + xpos);
	 // ch = TFT_ReadByteFromROMMemory(ptr);
		/// ch = font5x8[5*code+xpos];
		for(ypos = 0; ypos < 7; ypos++)    {
		if(ch & (1 << (6-ypos)))	    {
			TFT_SetScaledPixel(x, y, xpos, ypos, xscale, yscale, Color);
			}
		else
		{
			TFT_SetScaledPixel(x, y, xpos, ypos, xscale, yscale, bkColor);
			}
		}
		}
	}
void TFT_WriteScaledText(char * str, u16 x, u16 y, u16 xscale, u16 yscale,u16 Color, u16 bkColor)			{
		while(*str)  {
		TFT_ScaledFont(*str++, x , y, xscale, yscale, Color, bkColor);
		x += (5*xscale) + 1;
		}
	}
void TFT_sdy(u16 x, u08 y1, u08 y2, u16 ColoR, u08 width)	{
		// Wypelnianie pixeli z do³u do góry i lewej do prawej 
		u08 i;
		
		//if ( x<wXmin || x>wXmax ) 	return;
		//if ( y1>TFT_Ymax || y2>TFT_Ymax)  return;
		
		if ( y2 > y1) {
			TFT_SetCursor(x,y1);
			for (i=0;i<=(y2-y1+width);i++)	{
				TFT_RAM = ColoR;
			}
		} else {
			TFT_SetCursor(x,y2);
			for (i=0;i<=(y1-y2+width);i++)		{
				TFT_RAM = ColoR;
			}
		}
	}

void TFT_dy (u16 x, u08 y1, u08 y2, u16 color, u08 width)	{
		// Wypelnianie pixeli z do³u do góry i lewej do prawej 
		u08 l;
		
		
		//if ( x<wXmin || x>wXmax ) 	return;
		//if ( y1>=wYmax && y2>=wYmax)  return;
		//if ( y1<=wYmin && y2<=wYmin)  return;
		
		u08 a=abs(y2-y1);
		u08 dy=a/2;
		// ograniczenie z gory
		
		//if (y1 >= TFT_YSTP-width-1) y1= TFT_YSTP-width-1;
		//if (y2 >= TFT_YSTP-width-1) y2= TFT_YSTP-width-1;
		
		if( y2>y1 ){											// rosnaca
			
			l = dy+1+width;
			TFT_SetCursor(x,y1);
			while(l--){  TFT_RAM=color; }		// pierwsza linia
			
			
			//l = (a+3)/2+width;
			l = dy + a%2+1+width;
			TFT_SetCursor(x+1,y1+dy);
			while(l--){ TFT_RAM=color; }		// druga linia
		}
		
		else if( y1>y2 ){									// malejaca
			l  = dy+1;
			TFT_SetCursor(x,y1-dy);
			while(l--){ TFT_RAM=color; }		// pierwsza linia
			
			
			//l = (a+3)/2;
			l = dy+a%2+1;
			TFT_SetCursor(x+1,y2);
			while(l--){ TFT_RAM=color; }		// druga linia
		}
		else{															// stala
			l = width;
			TFT_SetCursor(x+1,y1);
			do{ TFT_RAM=color; }	while(l--);
			
		}
	}
void TFT_sdy_grid(u16 x1, u16 color){	//V-Line Grid Wypelnianie pixeli z dolu do gory	i z lewej do prawej 
		TFT_SetCursor(x1,0);
		for (u16 i=0;i<(TFT_YSTP+2);i++)	{
		// Vertical grid
		// if( (i==50 || i==100 || i==150 || i==200) && (((x1-TFT_XSTR)%10)==0) ) TFT_RAM = LGRAY;
		// if( x1 > 263 && (i > 207)) continue;	// Pole na zegar
		if( ((i%50)==0) && (( x1-TFT_XSTR)%12)==0) {
				
				TFT_RAM=LGRAY;
			}
			else {
				if((x1-TFT_XSTR)%72==0) { 			// Horizontal grid - dimension box
					if(i%10==0) {
						TFT_RAM=LGRAY;  		// dimension dots
					}
					else TFT_RAM=color;
				}
				else TFT_RAM=color;
			}
		}
		
	}	
//=====================================================================
void TFT_font(TFONT chr) {
		//#define wid	8
		//#define hig	16
		//Wypelnianie pixeli z dolu do gory	i z lewej do prawej
		
		u16 znak;
		chr.c_char -= 32;
		
		TFT_SetWindow(chr.x, chr.y, chr.x+chr.font_W-1, chr.y+chr.font_H-1 );
		TFT_SetCursor(chr.x, chr.y);
		
		// Drukowanie czcionki
		for(u08 k=0;k<(chr.font_W);k++)	{
			znak = *( (u16 *)chr.font_addr + 8*chr.c_char + (k) );
			
			for(u08 n=0;(n<chr.font_H);n++){
				if(znak & 1)	TFT_WriteRAM( chr.FGcol );
				else					TFT_WriteRAM( chr.BGcol );
				znak >>= 1;
			}
		}  
		
	}

void TFT_txt(char* str, u16 x, u16 y,u16 fgCol) {
		TFONT chr;
		chr.x = x;
		chr.y = y;
		chr.FGcol = fgCol;
		chr.BGcol = BLACK;
		chr.font_addr = TFT_FONT;
		
		//u16 font = sizeof(ASCII_1408);
		
		switch (TFT_FONT)	{
			case W_FONT_1608:
			chr.font_W = 8;
			chr.font_H = 16;
			break;
			case W_FONT_1408:
			chr.font_W = 8;
			chr.font_H = 11;
			break;
		}
		
		while(*str)  {
			chr.c_char = *str++;
			TFT_font(chr);
			if(chr.c_char == '.')				chr.x += 3;
			else if(chr.c_char == '-')	chr.x += 4;
			else if(chr.c_char == (96+32) )	chr.x += 2;
			else if(chr.c_char == ':')	chr.x += 5;
			else chr.x += 8;	
		}
	}	
/*
void TFT_ShortLine(u16 x1, u16 y1, u16 y2, uint16_t color)	{
		//if (y1>y2) swap_bytes(&y1,&y2);
		u16 dy;
		if( y2>y1 ){
			dy = (y2 - y1)/2;
		//TFT_frame(x1,   y1,    x1,   y1+dy, color);
			TFT_VerL(x1,   y1,          dy,    color)
		//TFT_frame(x1+1, y1+dy, x1+1, y2,    color);
			TFT_VerL(x1+1, y1+dy,       dy,    color)
		}
		else if( y1>y2 ){
			dy = (y1 - y2)/2;
		//TFT_frame(x1,   y2+dy, x1,   y1,    color);
			TFT_VerL(x1,   y1,          dy,    color)
		//TFT_frame(x1+1, y2,    x1+1, y2+dy, color);
			TFT_VerL(x1,   y1,          dy,    color)
			
		}
		else{
			TFT_frame(x1,   y1,    x1+1, y1,    color);
		}
	} */
/*void TFT_char2(uint8_t c_char,uint8_t x, uint8_t y, u16 BGcol,u16 FGcol) {
		u16 znak;
		c_char -= 33;
		// Zamiana wierszy na kolumny (obrot czcionki o 90st)
		
		CS_CLR; 													// Wybor wyswietlacza
		s65_frame(x, y, x+4, y+7);
		// Drukowanie czcionki
		for(u08 k=0;k<5;k++)	{			
			znak = FONT5x8[c_char][k];;
			for(u08 n=0;n<8;n++){
				if(znak & 0x01)	lcd_dat16( FGcol);
				else						lcd_dat16( BGcol);
				znak >>= 1;
			}
		} 
		CS_SET; 								//dezaktywacja
	} */
void TFT_test(void)			{
		//TFT_ReadReg(0x11);
		//*hex2str(u32 hex,char *string ,u08 digit);
		//TFT_WriteScaledText(hex2str(TFT_ReadReg(0x11),u1buf ,6), 10, 0, 2, 2, Yellow, Black);
		
		//TFT_WriteReg(0x4e,15); 	/* Row */
		//TFT_WriteReg(0x4f,15); 	/* Line */
		//TFT_REG = R34;
		
		//TFT_SetWindow(1,1,1,1);
		//TFT_WriteRAM(White);
		//while(1);
		//TFT_SetCursor(3,3);
		
		TFT_SetWindow(0,0,2,2);
		for(int x=0; x<4; x++){
			TFT_WriteRAM(RED);
			Delay_ms(100);
		}
		
		while(1);
		//TFT_WriteScaledText(hex2str(TIM6->CNT,u1buf ,6), 2, 2, 1, 1, Yellow, Black);
		
	}
/*-------------------------------------------------------------------------------------------------
unsigned char TFT_ReadByteFromROMMemory(char * ptr)
{
  return pgm_read_byte(ptr);
 }
*/
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
