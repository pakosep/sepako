/****************************************Copyright (c)**************************************************                         
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			tool.h
** Descriptions:		None
**------------------------------------------------------------------------------------------------------
** Created by:			sepako
** Created date:		2012-1-12
** Version:				  1.0
** Descriptions:		The original version
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/
//#include <stdint.h>
//#include <string.h>
//#include <stdlib.h>
#include "inc/stm32f10x.h"
#include "config.h"

// SysTick param
// Max delay 	SysTick[ms]=1000*2^24/(F_CPU/8)
// 200MHz -> 671ms
#define STCLK_MS (int32_t)( (F_CPU) / (8000ul) )
#define STCLK_US (int32_t)( (F_CPU) / (8000000ul) )
#define STCLK_DN (int32_t)( (F_CPU) / (40000000ul) )
#define TIM3_MS  1000
#define TIM3_US  1

//#define STCLK_LD (int32_t)( 0xfffffful )
#define STCLK_LD (uint32_t) ((1*F_CPU) /(8))
extern volatile uint32_t delay_us,trg0,trg1;
extern volatile int32_t  delay3_us;
extern volatile uint32_t Tim4,Tim6, Timt, Timt3, FLAG;
extern volatile uint8_t  sec;

/* Includes ------------------------------------------------------------------*/
//#define TIM4_start()  Tim4 = 0;	TIM4->CNT	= 0
//#define TIM4_stop()   Time4 = (TIM4->CNT | (Tim4 << 16))
//#define Delay_us(us)  delay_us = SysTick->VAL; while( STCLK_US*us > (delay_us - SysTick->VAL) );
//#define Delay_ms(ms)	Delay_us(1000*ms)

//void Delay1us(uint16_t arg);
//void Delay_us(uint32_t arg);
//void Delay_ms(uint16_t  ms);


//inline void foo (const char) __attribute__((always_inline));

/*
void Delay_us(u32 us);
void Delay_ms(u32 ms);
void TIM4_start(void);
void TIM4_stop(void);
*/

//void __attribute__((always_inline)) TIM6_start(void);
//void __attribute__((always_inline)) TIM6_stop(u32 *tim);		//  resolution 0.1 us
//void Delay_200n(u32 ns);
//void Delay_us(u32 us);
//void Delay_ms(u32 ms);

static inline u08 HAL_GetTick(void){	
		return sec;
	}
//=========================TIM6======================================
inline void __attribute__((always_inline)) TIM6_stop(volatile u32 *tim) 	{		//  resolution 0.1 us
		*tim = TIM6->CNT | (Tim6 << 16);
	}

inline void __attribute__((always_inline)) TIM6_start(void) 	{
		Tim6 = 0;	
		TIM6->CNT	= 0;
	}

//=========================TIM4======================================
inline void __attribute__((always_inline)) TIM4_stop(volatile u32 *tim) 	{		//  resolution 0.1 us
		*tim = TIM4->CNT | (Tim4 << 16);
	}

inline void __attribute__((always_inline)) TIM4_start(void) 	{
		Tim4 = 0;	
		TIM4->CNT	= 0;
	}

	
//=========================SysTick======================================	
inline void  __attribute__((always_inline)) TIM_start(void){
		Timt = SysTick->VAL;
	}
inline void  __attribute__((always_inline)) TIM_stop(volatile u32 *tim){	// resolution 1 [us]
		*tim = (Timt - SysTick->VAL);
	}


inline void Delay_200n(u32 ns)  {
		delay_us = SysTick->VAL; 
		while( STCLK_DN*ns > (delay_us - SysTick->VAL) );
	}

inline void __attribute__((always_inline)) Delay_us(u32 us)  {
		delay_us = SysTick->VAL; 
		while( STCLK_US*us > (delay_us - SysTick->VAL) );
	}

inline void __attribute__((always_inline)) Delay_ms(u32 ms)	{
		delay_us = SysTick->VAL; 
		while( STCLK_MS*ms > (delay_us - SysTick->VAL) );
 	}
//=========================TIM3======================================

inline void __attribute__((always_inline)) Delay3_us(s32 us)  {
		delay3_us = TIM3->CNT; 
		while( (us+1) > (delay3_us - TIM3->CNT) );
		//while( us + TIM3->CNT > delay3_us  );
	}

inline void __attribute__((always_inline)) Delay3_ms(s32 ms)  {
		delay3_us = TIM3->CNT; 
		
		while( (1+1000*ms) > (delay3_us - TIM3->CNT) );
	}

inline void  __attribute__((always_inline)) TIM3_start(void){
		Timt3 = TIM3->CNT;
	}
inline void  __attribute__((always_inline)) TIM3_stop(u32 *tim){	// resolution 1 [us]
		*tim = (Timt3 - TIM3->CNT)-1;
	}
// Maksymalnie 4000[us]
#define DelayUS_ASM(us) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" ((us*(F_CPU/1000000))/6) : "memory"\
		      );\
} while(0)

/*********************************************************************************************************
** END FILE
*********************************************************************************************************/



