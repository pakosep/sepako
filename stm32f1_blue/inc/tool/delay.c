/****************************************Copyright (c)**************************************************                         
**
**                                
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			tool.c
** Descriptions:	
**						
**------------------------------------------------------------------------------------------------------
** Created by:			sepako
** Created date:		2012-1-12
** Version:					1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/
/* Includes ------------------------------------------------------------------*/

#include "delay.h" 

/* Private variables ---------------------------------------------------------*/
// static uint16_t TimerPeriod = 0;

/* Private define ------------------------------------------------------------*/

/*
extern volatile u32 Tim4,Time4;

void TIM4_start(void) 	{
		Tim4 = 0;	TIM4->CNT	= 0;
	}

void TIM4_stop(void) 	{
		Time4 = (TIM4->CNT | (Tim4 << 16));
	}


void Delay_us(u32 us)  {
		delay_us = SysTick->VAL; 
		while( STCLK_US*us > (delay_us - SysTick->VAL) );
	}

void Delay_ms(u32 ms)	{
		Delay_us(1000*ms);
 	}
*/

#define LOOP_DLY_1US  1

void Delay1us(uint16_t arg) 	{ 
		uint16_t Dly = (uint16_t)arg; 
		while(Dly--) 
		{ 
			for(volatile int i = LOOP_DLY_1US; i; i--); 
		} 
		/*
		0		-	0.4us
		1		-	1.02us
		2		-	1.65us		
		5		- 3.5us
		10	-	6.68us
		100	-	63.1us		
		*/
	} 
	
void Delay_us_(uint32_t arg) 	{ 
		//arg = (arg*86)/128;
		arg = (arg);
		while(arg--) Delay1us(1);
	}
	
void Delay_ms_(uint16_t arg) {
		while(arg--) Delay_us(1000);
	}
 
//inline void TIM4_stop(void)	{ Time4 = (TIM4->CNT | (Tim4 << 16));	}

// void Delayms(u32 ms)	{
//		Delayus(1000*ms);			
// 	}
 
 //void Delayus(u32 us)	{
 //delay_us = SysTick->VAL;
 //while( STCLK_US*us > (delay_us - SysTick->VAL) );			}

/*
void Delay (u08 type,u16 d){  //type=1 ms, =0 us
    TIM6->ARR = d;
    TIM6->CNT = 0;
    if(type == 1)TIM6->PSC = 36000;
    else TIM6->PSC = 36;
		
		//    TIM6->CR1 = TIM_CounterMode_Down;// | TIM_OPMode_Single;
		//    TIM6->CR1 &= ~TIM_CR1_OPM;		//clear OPM
		//    TIM6->CR1 |= TIM_CR1_OPM;		//set OPM
    TIM6->CR1 |= TIM_CR1_CEN;		//Enable counter
		//    while((TIM6->CR1 & 1) == 1);
    while(TIM6->SR == 0);
    TIM6->SR = 0;
	} 
	#define TIMER_FREQUENCY     3600000     // 1MHz

// Blocking usec delay function. Designed to be re-entrant.
void Delayus(uint16_t usec)	{
    uint16_t timeout = TIM6->CNT;
		// Conversion = (usec / 1MHz) * Freq
    if (usec <= MAX_DELAY)
    {
      timeout += (uint16_t)(((uint32_t)usec * (TIMER_FREQUENCY / 1000uL)) / 1000uL);
    }
    else
    {
      timeout += (uint16_t)(((uint32_t)MAX_DELAY * (TIMER_FREQUENCY / 1000uL)) / 1000uL);
    }
    // This method provides very simple wrap-around handling, BUT limits the
    // range to half of the available bits (due to the cast from unsigned to
    // signed). For this function, we don't care about getting the full 16-bit
    // range so simpler is better.
    while ((int16_t)(timeout - TIM6->CNT) >= 0)   ;
	}
	*/

 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
