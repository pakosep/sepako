// Mini STM32F103VCT6 High Density 48kB RAM 256kB Flash
// Wylutowane L2 (Od³¹czone Vref od 3v3) 
#define showInclude
#define showDefine		
#define DeclareFunction
#define showGlobalVariable	
#define fromRAM				//Program in RAM 	0x20000000

#ifdef showInclude
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "inc/stm32f10x.h"
#include "config.h"
#include "gpio.h"

#include "hdr/hdr_bitband.h"
#include "hdr/hdr_rcc.h"
#include "hdr/hdr_gpio.h"

#include "inc/glcd/glcd.h"
#include "inc/tpad/touch.h"
#include "inc/nrf24/nrf.h"
#include "inc/tool/delay.h"
#include "inc/tool/UARTLib.h"
#include "inc/tool/sp_term.h"
//#include "inc/sdspi/matosd.h"
#include "inc/sdio/sdio.h"
#include "inc/spi/spi.h"
#include "inc/rtc/rtc.h"
#include "inc/flash/flash.h"
#include "inc/enc28j60/enc28j60.h"

#endif
//typedef uint8_t u08;
#ifdef showDefine
#define TC 										6

#define bPBI_6								bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 6)
#define bTIM2_SR_UIF					bitband_t m_BITBAND_PERIPH(&TIM2->SR, 0)
#define bTIM6_SR_UIF					bitband_t m_BITBAND_PERIPH(&TIM6->SR, 0)
#define bTIM6_CR1_CEN					bitband_t m_BITBAND_PERIPH(&TIM6->CR1, 0)
#define bUSART1_CR1_RXNEIE		bitband_t m_BITBAND_PERIPH(&USART1->CR1, 5)
#define bUSART1_SR_RXNE				bitband_t m_BITBAND_PERIPH(&USART1->SR, 5)
#define bUSART2_SR_RXNE				bitband_t m_BITBAND_PERIPH(&USART2->SR, 5)
#define bUSART1_SR_TXE				bitband_t m_BITBAND_PERIPH(&USART1->SR, 7)
#define bUSART1_CR1_TXE				bitband_t m_BITBAND_PERIPH(&USART1->SR, 7)
#define bEXTI_PR_2						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 2)
#define bEXTI_PR_13						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 13)
#define bEXTI_PR_6						bitband_t m_BITBAND_PERIPH(&EXTI->PR, 6)
#define bEXTI_FTSR_6					bitband_t m_BITBAND_PERIPH(&EXTI->FTSR, 6)

#define bRTC_CRL_RTOFF				bitband_t m_BITBAND_PERIPH(&RTC->CRL, 5)
#define bRTC_CRL_CNF					bitband_t m_BITBAND_PERIPH(&RTC->CRL, 4)
#define bRTC_CRL_SECF					bitband_t m_BITBAND_PERIPH(&RTC->CRL, 0)
#define bPWR_CR_DBP						bitband_t m_BITBAND_PERIPH(&PWR->CR,  8)
#define I2C2_CR1_SWRST_bb			bitband_t m_BITBAND_PERIPH(&I2C2->CR1,15)
#define I2C2_CR1_PE_bb				bitband_t m_BITBAND_PERIPH(&I2C2->CR1,0)

#define ADC1_CR2_ADON				bitband_t m_BITBAND_PERIPH(&ADC1->CR2,0)
#define ADC1_SR_EOC					bitband_t m_BITBAND_PERIPH(&ADC1->SR, 1)
#define ADC1_CR2_CAL_bb				bitband_t m_BITBAND_PERIPH(&ADC1->CR2,2)
#define ADC1_CR2_JSWSTART_bb	bitband_t m_BITBAND_PERIPH(&ADC1->CR2,21)
#define ADC1_SWSTART					bitband_t m_BITBAND_PERIPH(&ADC1->CR2,22)
#define ADC1_SR_JSTRT_bb			bitband_t m_BITBAND_PERIPH(&ADC1->SR,3)
#define ADC1_SR_JEOC_bb				bitband_t m_BITBAND_PERIPH(&ADC1->SR,2)

#define ADC1_CR1_SCAN_bb			bitband_t m_BITBAND_PERIPH(&ADC1->CR1,8)
#define ADC1_CR2_TSVREFE_bb		bitband_t m_BITBAND_PERIPH(&ADC1->CR2,23)
#define DMA1_Ch1_EN_bb				bitband_t m_BITBAND_PERIPH(&DMA1_Channel1->CCR,0)

#define I2C_CR2_FREQ_36MHz	0b100100

#define TIMEZONE            2
#define YEAR0               1900
#define EPOCH_YR            1970
#define SECS_DAY            (24L * 60L * 60L)
#define LEAPYEAR(year)      (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)      (LEAPYEAR(year) ? 366 : 365)

#define MAX_DELAY   INT16_MAX       // 32767 usec Max

#endif
#ifdef showGlobalVariable

volatile  int32_t  trg0,trg1,trg2,trg3,trg4,trg5,trg6,trg7,trg8,trg9;
volatile  int32_t  tim0,tim1,tim2,tim3,tim4,tim5,tim6,tim7,delay_us;
volatile uint32_t  Tim6,Time6,Timt;

typedef u32 time_T;
const int _ytab[2][12] ={
		{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
		{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};

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

u16 tlo=0;
char tbuf[16];
//typedef long int UnixTime;
//extern char u1buf[RBUF_SIZE];
extern volatile char *u1buf_ptr;
volatile u08 	flag=0;
volatile u16 	bColor,tX,tY,pX,pY,Color=GREEN;
volatile u32 	Tmp,xx,py=0,px=0;
char 					cbuf[20]={"1982-01-01 11:00:00"};
volatile u32 	FLAG=0;					//bit define in config.h
unsigned char config, dummy, var, payload, status_temp;
//extern unsigned char status;
//unsigned char but_flag = 0;
T_RTC rtc;
extern const uint16_t ASCII_1408[97][8];

#endif
// local functions' declarations
#ifdef DeclareFunction
static void	flash_latency(uint32_t frequency);
static void SysInit(void);
static void pll_start(uint32_t crystal, uint32_t frequency);
static void system_init(void);
void SysTickInit(void);

inline static void Usart1Init(void);
inline static void Usart2Init(void);

#endif
/*== INIT ===================================================================*/
void SysInit(void){
	system_init();
	pll_start(CRYSTAL, F_CPU);
#ifdef RAMCODE
	SCB->VTOR = NVIC_VectTab_RAM | (Offset & (uint32_t)0x1FFFFF80); //If in RAM
#endif
	GPIOB->CRL = 0x44484433;
	SysTickInit();
	//Tim6Init();					// Init Timer 6
	//Tim7Init();					// Init Timer 7
	//BkpIncrement();			// Programming counter
	Usart1Init();
	Usart2Init();
	//Tim2PWMInit();
	//ExtIInit();
	//RtcInit();
	//==== ENABLE IRQ ==============================================================

	//NVIC_EnableIRQ(TIM7_IRQn);
	//NVIC_EnableIRQ(TIM2_IRQn);
	//NVIC_EnableIRQ(EXTI0_IRQn);			// Line 0
	//NVIC_EnableIRQ(EXTI2_IRQn);			// Line 2
	//NVIC_EnableIRQ(EXTI9_5_IRQn);		// Line 5..9
	//NVIC_EnableIRQ(EXTI15_10_IRQn);	// Line 10..15
	//NVIC_EnableIRQ(RTC_IRQn);				// Real Time Clock Initail
	//I2C2_init();										// I2C number 2 Initial
	//Touch_Init();										// TFT TouchPAD Initial
	//nRF24L01_Init();
	//NVIC_EnableIRQ(DMA1_Channel1_IRQn);				// DMA1 Channel1
}

/*== START ===================================================================*/
void head(void){
		SysInit();
		tr_pen_color( TGREEN );
		
		UaPutS((const char *) "\f TEST ");
		
		#ifdef RAMCODE
		UaPutS((const char *) "\f Mini STM32F103VCT6 RAM ");
		#else
		UaPutS("\f Mini STM32F103VCT6 FLASH ");
		#endif
		//Delay_ms(100);
		TFT_Init();
		TFT_Clear(BLACK);
	}

int main(void){
		//head();
		//GUI_Text(120,110, "TFT",YELLOW,BLACK);
		//GUI_Text(120,110,hex2str(EXTI_FTSR_TR6, u1buf, 6),YELLOW,BLACK);
		//uint8_t __attribute__((unused)) outbuf[512];
		//uint8_t __attribute__((unused)) inbuf[512];
		//MemCard_Menu();
		//SDInit();
		//Radio();
		//Grfaphics();
		//OtherTest();
		//data_log();
		while(1);
	}

void Usart1Init(void)			{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		GPIOA->CRH = (GPIOA->CRH & 0xfffff00f)| 0x00000490; //PA9=TX, PA10=RX zerowanie
		USART1->CR1 |= (USART_CR1_UE |USART_CR1_TE | USART_CR1_RE	|USART_CR1_RXNEIE);
		USART1->BRR = (F_CPU+115200/2)/115200;
		NVIC_EnableIRQ(USART1_IRQn);
	}
void Usart2Init(void)			{
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		GPIOA->CRL = (GPIOA->CRL & 0xffff00ff)| 0x00004900; //PA2=TX, PA3=RX zerowanie
		USART2->CR1 |= (USART_CR1_UE |USART_CR1_TE | USART_CR1_RE	|USART_CR1_RXNEIE);
		USART2->BRR = ((F_CPU+115200/2)/115200)/2;
		NVIC_EnableIRQ(USART2_IRQn);
	}

void SysTickInit(void)		{
	SysTick->LOAD = STCLK_LD - 1;    // 72/8/SysTick_LOAD interrupt every x s
	SysTick->CTRL = SysTick_CTRL_TICKINT|SysTick_CTRL_ENABLE; // enable SysTick
}

static void flash_latency(uint32_t frequency)
{
	uint32_t wait_states;

	if (frequency < 24000000ul)			// 0 wait states for core speed below 24MHz
		wait_states = 0;
	else if (frequency < 48000000ul)// 1 wait state for core 24MHz < speed >48MHz
		wait_states = 1;
	else									// 2 wait states for core speed over 48MHz
		wait_states = 2;

	FLASH->ACR |= wait_states;				// set the latency
}

//============================================================================
static void pll_start(uint32_t crystal, uint32_t frequency){
	uint32_t mul;
	flash_latency(frequency);				// configure Flash latency for desired frequency
	mul = frequency / crystal;			// PLL multiplier calculation
	if (mul > 16)										// max PLL multiplier is 16
		mul = 16;

	RCC_CR_HSEON_bb = 1;						// enable HSE clock
	while (!RCC_CR_HSERDY_bb);			// wait for stable clock

	RCC->CFGR |= ((mul - 2)<<RCC_CFGR_PLLMUL_bit)|RCC_CFGR_PLLSRC|RCC_CFGR_PPRE1_DIV2;	
	// configuration of PLL: HSE x (mul), APB1 clk = /2
	RCC_CR_PLLON_bb = 1;						// enable PLL
	while (!RCC_CR_PLLRDY_bb);			// wait for PLL lock

	RCC->CFGR |= RCC_CFGR_SW_PLL;		// change SYSCLK to PLL
	while (((RCC->CFGR) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	// wait for switch
}
static void system_init(void){
	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
			RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;
	//  gpio_init();
	// enable GPIO A,B,C,D,E + USART1
}
/******************************************************************************
 * END OF FILE
 ******************************************************************************/
/* GPIO Configuration
GPIOx->CRL		PIN 0-7		
GPIOx->CRH		PIN 8-15

C1 C0 M1 M0		HEX
0	 0	0	 0		0x0		Input Analog 
0	 0	0	 1		0x1		Output Push-pull	10MHz
0	 0	1	 0		0x2		Output Push-pull	2MHz
0	 0	1	 1		0x3		Output Push-pull	50MHz

0	 1	0	 0		0x4		Input floating 
0	 1	0	 1		0x5		Output Open Drain	10MHz
0	 1	1	 0		0x6		Output Open Drain	2MHz
0	 1	1	 1		0x7		Output Open Drain	50MHz

1	 0	0	 0		0x8		Input  Pull Up/Down 
1	 0	0	 1		0x9		Output Alternate Push-pull	10MHz
1	 0	1	 0		0xA		Output Alternate Push-pull	2MHz
1	 0	1	 1		0xB		Output Alternate Push-pull	50MHz

1	 1	0	 0		0xC		Reserved 
1	 1	0	 1		0xD		Output Alternate Open-drain	10MHz
1	 1	1	 0		0xE		Output Alternate Open-drain	2MHz
1	 1	1	 1		0xF		Output Alternate Open-drain	50MHz
 */
