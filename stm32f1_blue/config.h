/** \file config.h
 * \brief Basic configuration of the project
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2010-04-02
 */

/******************************************************************************
* project: stm32_blink_led
* chip: STM32F103RB
* compiler: arm-none-eabi-gcc (Sourcery G++ Lite 2009q3-68) 4.4.1
******************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"
#include "hdr/hdr_rcc.h"
#include "hdr/hdr_gpio.h"

/*
+=============================================================================+
| global definitions
+=============================================================================+
*/

#define MHz							1000000ul
#define CRYSTAL					8*MHz	///< quartz crystal resonator which is connected to the chip
//#define F_CPU					 40*MHz	///< desired target frequency of the core
#define F_CPU					 72*MHz	///< desired target frequency of the core
//#define F_CPU					 80*MHz	///< desired target frequency of the core
//#define F_CPU					 96*MHz	///< desired target frequency of the core
//#define F_CPU					120*MHz	///< desired target frequency of the core
//#define F_CPU					 48*MHz	///< desired target frequency of the core

//460800 230400 115200
#define UART1_SPEED 	115200 
#define UART2_SPEED 	115200 //460800
#define UART3_SPEED 	115200

#define TIM_US	(F_CPU/(8000000))
#define TIM_MS	(F_CPU/(8000))
#define TIM_SEC	(F_CPU/(8))

#define TIM3_ARR	0xffff
#define TC 							  6
//#define NVIC_VectTab_RAM 	0x20000000
#define 	NVIC_VectTab_RAM    ((uint32_t)0x20000000)
#define 	NVIC_VectTab_FLASH  ((uint32_t)0x08000000)
#define Offset 						0x0

/*
+=============================================================================+
| strange variables
+=============================================================================+
*/
#pragma pack(push)       /* zapamietaj biezaca wartosc wyrownania */
#pragma pack(1)          /* brak wyrownania */
//========================================================================
typedef struct {				
    u16    a0:12;
		u16    a1:12;
		
	}T_ADC;

/* Organizacja danych na karcie pamieci dane*/
typedef struct {
		u16    d0:10;   	// data 1		0-1023
		u16    d1:10;   	// data 2		0-1023
		u16    d2:10;   	// data 3		0-1023
    u16    d3:10;     // data 4 	0-1023
    u16    d4:10;     // data 5		0-1023
		u16    d5:10;     // data 6		0-1023
    u16    d6:10;     // data 7		0-1023
		u16    d7:10;			// data 8		0-1023
    u16    d8:10;     // data 9		0-1023
		u16    e0:11;   	// data	10	0-2047
    u16    e1:11;     // data	11	0-2047
		// 9*10 + 2*11 = 112 bitow
		
	}T_DBF0;	

typedef struct {				// 36*112 + 2*32 = 1*512*8 = Use 1 Block
		u32 		   		 dtim;
    T_DBF0    	dat[36];
		u32 		   			crc;
	}T_SD0;	

struct usb {
	uint8_t  bDat1;
	uint8_t  bDat2;
	uint16_t bDat3;

} __attribute__((packed));

//========================================================================
typedef struct {
		u16    d0:10;   	// data 1		0-1023
		u16    d1:10;   	// data 2		0-1023
		u16    d2:10;   	// data 3		0-1023
    u16    d3:10;     // data 4 	0-1023
    u16    d4:10;     // data 5		0-1023
		u16    d5:10;     // data 6		0-1023
    u16    d6:10;     // data 7		0-1023
		u16    d7:10;			// data 8		0-1023
		u16    d8:10;			// data 8		0-2047
		u16    e0:11;   	// data	0		0-2047
    u16    e1:11;     // data	1		0-2047
		
		// 9*10 + 2*11 = 112 bitow
	}T_DBF1;	

typedef struct {				//   288*112 + 16*32 = 8*512 = Use 8 Block
    T_DBF1  dat[288];		
		u32			d32[16];
	}T_SD1;	
//========================================================================

#pragma pack(pop)	//Przywroc wczesniejsza wartosc wyrownania

#define NChart	4 // Ilosc wykresow

struct tm {
		int     tm_sec;         /* sekundy od pe³nej minuty*/
		int     tm_min;         /* minuty  od pe³nej godziny*/
		int     tm_hour;        /* godzina na 24-godzinnym zegarze*/
		int     tm_mday;        /* dzieñ miesi¹ca */
		int     tm_mon;         /* miesi¹c licz¹c od zera */
		int     tm_year;        /* rok - 1900 */
		int     tm_wday;        /* dzieñ tygodnia niedziela ma numer 0*/
		int     tm_yday;        /* dzieñ roku licz¹c od zera*/
		int     tm_isdst;       /* znacznik czasu letniego */
	}ptm;

typedef struct ANODE {
		uint8_t				node_id;
		const char *	node_title;
		uint8_t				fun_prop;
		
		const struct ANODE * branch_next;
		const struct ANODE * branch_prev;
		
		const struct ANODE * node_next;
		const struct ANODE * node_prev;
		
	} TNODE;
#define DIMSDBLOCK	8 	// Rozmiar struktury w blokach po 512 byte
/*
+=============================================================================+
| global variables
+=============================================================================+
*/
#define __weak __attribute__ ((weak)) 

#define wMOSI							bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 7)
#define wMISO							bitband_t m_BITBAND_PERIPH(&GPIOA->IDR, 6)
#define wSCK							bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 5)
#define wCS 							bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 4)

#define LED1					bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 0)
#define LED2					bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 1)

#define flag_0				bitband_t m_BITBAND_SRAM(&FLAG,0)		//IRQ Usart1 key enter
#define f1_tpirq			bitband_t m_BITBAND_SRAM(&FLAG,1)		//IRQ Touch Pad
#define flag_2				bitband_t m_BITBAND_SRAM(&FLAG,2)		//IRQ Usart1 any key without enter
#define f3_rtc_1sec		bitband_t m_BITBAND_SRAM(&FLAG,3)
#define f4_enc28			bitband_t m_BITBAND_SRAM(&FLAG,4)
#define f5_eth				bitband_t m_BITBAND_SRAM(&FLAG,5)
#define flag_6				bitband_t m_BITBAND_SRAM(&FLAG,6)
#define f7_dma1				bitband_t m_BITBAND_SRAM(&FLAG,7)
#define f8_dma1				bitband_t m_BITBAND_SRAM(&FLAG,8)
#define flag_9				bitband_t m_BITBAND_SRAM(&FLAG,9)
#define flag_10				bitband_t m_BITBAND_SRAM(&FLAG,10)
#define flag_11				bitband_t m_BITBAND_SRAM(&FLAG,11)
#define flag_12				bitband_t m_BITBAND_SRAM(&FLAG,12)
#define flag_13				bitband_t m_BITBAND_SRAM(&FLAG,13)
#define flag_14				bitband_t m_BITBAND_SRAM(&FLAG,14)
#define flag_15				bitband_t m_BITBAND_SRAM(&FLAG,15)

#define SPI2CS  							bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 12)  //PB12  CS  
#define bPBI_6								bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 6)
#define bTIM1_SR_UIF					bitband_t m_BITBAND_PERIPH(&TIM1->SR, 0)
#define bTIM2_SR_UIF					bitband_t m_BITBAND_PERIPH(&TIM2->SR, 0)
#define bTIM6_SR_UIF					bitband_t m_BITBAND_PERIPH(&TIM6->SR, 0)
#define bTIM6_CR1_CEN					bitband_t m_BITBAND_PERIPH(&TIM6->CR1, 0)
#define bUSART1_CR1_RXNEIE		bitband_t m_BITBAND_PERIPH(&USART1->CR1, 5)
#define bUSART1_SR_RXNE				bitband_t m_BITBAND_PERIPH(&USART1->SR, 5)
#define bUSART2_SR_RXNE				bitband_t m_BITBAND_PERIPH(&USART2->SR, 5)
#define bUSART1_SR_TXE				bitband_t m_BITBAND_PERIPH(&USART1->SR, 7)
#define bUSART1_CR1_TXE				bitband_t m_BITBAND_PERIPH(&USART1->SR, 7)
#define bEXTI_PR_0						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 0)
#define bEXTI_PR_2						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 2)
#define bEXTI_PR_13						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 13)
#define bbEXTI_PR_6						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 6)
#define bbEXTI_FTSR_6					bitband_t m_BITBAND_PERIPH(&EXTI->FTSR, 6)

#define bRTC_CRL_RTOFF				bitband_t m_BITBAND_PERIPH(&RTC->CRL, 5)
#define bRTC_CRL_CNF					bitband_t m_BITBAND_PERIPH(&RTC->CRL, 4)
#define bRTC_CRL_SECF					bitband_t m_BITBAND_PERIPH(&RTC->CRL, 0)
#define bPWR_CR_DBP						bitband_t m_BITBAND_PERIPH(&PWR->CR,  8)
#define I2C2_CR1_SWRST_bb			bitband_t m_BITBAND_PERIPH(&I2C2->CR1,15)
#define I2C2_CR1_PE_bb				bitband_t m_BITBAND_PERIPH(&I2C2->CR1,0)

#define ADC1_CR2_ADON					bitband_t m_BITBAND_PERIPH(&ADC1->CR2,0)
#define ADC1_SR_EOC						bitband_t m_BITBAND_PERIPH(&ADC1->SR, 1)
#define ADC1_CR2_CAL_bb				bitband_t m_BITBAND_PERIPH(&ADC1->CR2,2)
#define ADC1_CR2_JSWSTART_bb	bitband_t m_BITBAND_PERIPH(&ADC1->CR2,21)
#define ADC1_SWSTART					bitband_t m_BITBAND_PERIPH(&ADC1->CR2,22)
#define ADC1_SR_JSTRT_bb			bitband_t m_BITBAND_PERIPH(&ADC1->SR,3)
#define ADC1_SR_JEOC_bb				bitband_t m_BITBAND_PERIPH(&ADC1->SR,2)

#define ADC1_CR1_SCAN_bb			bitband_t m_BITBAND_PERIPH(&ADC1->CR1,8)
#define ADC1_CR2_TSVREFE_bb		bitband_t m_BITBAND_PERIPH(&ADC1->CR2,23)
#define DMA1_Ch1_EN_bb				bitband_t m_BITBAND_PERIPH(&DMA1_Channel1->CCR,0)

#define PE2_i									bitband_t m_BITBAND_PERIPH(&GPIOE->IDR, 2)
#define PE2_o									bitband_t m_BITBAND_PERIPH(&GPIOE->ODR, 2)
#define PE4_i									bitband_t m_BITBAND_PERIPH(&GPIOE->IDR, 4)
#define PE4_o									bitband_t m_BITBAND_PERIPH(&GPIOE->ODR, 4)
#define PA0_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 0)
#define PA3_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 3)
#define PA4_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 4)
#define PA6_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 6)
#define PA7_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 7)
#define PA8_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 8)
#define PA9_o									bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 9)
#define PA10_o								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 10)
#define PA11_o								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 11)
#define PA15_o								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 15)

#define PC13_o								bitband_t m_BITBAND_PERIPH(&GPIOC->ODR, 13)
#define PC14_o								bitband_t m_BITBAND_PERIPH(&GPIOC->ODR, 14)

#define sMOSI_l								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 2)
#define sMISO_l								bitband_t m_BITBAND_PERIPH(&GPIOA->IDR, 1)
#define sSCK_l								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 0)

#define PB0_i									bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 0)
#define PB1_i									bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 1)
#define PB1_o									bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 1)
#define PB3_o									bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 3)
#define PB4_o									bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 4)
#define PB7_o									bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 7)
#define PB8_o									bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 8)
#define PB9_o									bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 9)
#define PB10_o								bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 10)
#define PB11_o								bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 11)
#define PB12_o								bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 12)
#define PB13_o								bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 13)
#define PB14_o								bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 14)
#define PB15_o								bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 15)
#define PB5_i									bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 5)

#define PB11_i								bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 11)

#define Sys_Tick_CTRL_EN			bitband_t m_BITBAND_PERIPH(&SysTick->CTRL,0)

#define BLUE_Board
#ifdef BLACK_Board
 #define LED PB12_o
#else
 #define LED PC13_o
#endif

#ifndef RX1276_RST1
	#define  RX1276_RST1	PA3_o
	#define  RX1276_CS1		PA4_o
	
	#define  RX1276_RST2	PB9_o
	#define  RX1276_CS2		PB8_o
#endif

#define b2b(b7,b6,b5,b4,b3,b2,b1,b0) ((unsigned char)((b7)*128 + (b6)*64 + (b5)*32 + (b4)*16 + (b3)*8 + (b2)*4 + (b1)*2 + (b0)))
typedef unsigned char TCDATA;
typedef TCDATA* TCLISTP;

//#define LCD_DC								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 4)
//#define LCD_CE								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 3)
//#define LCD_RST								bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 2)



/*
+=============================================================================+
| global functions' declarations
+=============================================================================+
*/
/*
 JTDO  PB3
 JTDI  PA15
 JTRST PB4
 JTCK  PA14 SWD
 JTMS  PA13 SWD
*/

/******************************************************************************
* END OF FILE
******************************************************************************/
#endif /* CONFIG_H_ */
