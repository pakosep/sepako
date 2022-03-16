// Blue Mini STM32F103C8T6 High Density 20kB RAM 64kB Flash
// Timery TIM1,2,3,4 2xI2C 2xSPI 3xUSART
#define showInclude
#define showDefine		
#define DeclareFunction
#define showGlobalVariable	
#define fromRAM				//Program in RAM 	0x20000000

#ifdef showInclude
#include "inc/tool/delay.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "inc/stm32f10x.h"
#include "config.h"
#include "gpio.h"

#include <stdio.h>

#include "hdr/hdr_bitband.h"
#include "hdr/hdr_rcc.h"
#include "hdr/hdr_gpio.h"

#include "inc/tft/tft.h"
//#include "inc/tpad/touch.h"
//#include "inc/nrf24/nrf.h"

#include "inc/tool/UARTLib.h"
#include "inc/tool/sp_term.h"
//#include "inc/sdspi/matosd.h"
#include "inc/sdio/sdio.h"
#include "inc/spi/spi.h"
#include "inc/rtc/rtc.h"
#include "inc/flash/flash.h"
#include "inc/dht/dht.h"
#include "inc/opto/tsl2561.h"
#include "inc/spi/softspi.h"
#include "inc/lcd/uc1608.h"
#include "inc/sx1278/SX1278.h"
#include "inc/oled/oled.h"
#include "inc/i2c/i2c.h"

#include "inc/mcp2515/can.h"

//#include "inc/enc28j60/enc28j60.h"
//#include "inc/enc28j60/test_web_client.h" 
//#include "inc/enc28j60/ip_arp_udp_tcp.h"

#include "inc/i2c/i2c.h"

#endif
//typedef uint8_t u08; 
#ifdef showDefine
#define TC 										6

#define TIMEZONE            	2
#define YEAR0               	1900
#define EPOCH_YR            	1970
#define SECS_DAY            	(24L * 60L * 60L)
#define LEAPYEAR(year)      	(!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)      	(LEAPYEAR(year) ? 366 : 365)

#define MAX_DELAY   INT16_MAX       // 32767 usec Max

#endif
#ifdef showGlobalVariable

volatile uint32_t  trg0,trg1,trg2,trg3,trg4,trg5;
volatile int32_t   t3rg0,t3rg1,t3rg2,delay3_us;
volatile uint32_t  Tim0,Tim1,Tim2,delay_us;
volatile uint32_t  Tim4,Tim6,Time6,Timt,Timt3;

const int _ytab[2][12] ={
	{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
	{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};

extern volatile char *U1buf_ptr;
volatile u32 	FLAG=0;					//bit define in config.h


T_RTC Rtc;
static T_SD1 	 TSpm;				// struktura pomiarowa
u08 *p_TSpm = (u08 *)&TSpm;	// wskaźnik na strukture do odczytu z karty pamiêci

extern const uint16_t ASCII_1408[97][8];

//const u16 CRC_progmem __attribute__((section (".myBufSection"))) = 0xabab;

//volatile uint32_t Sec=0;

	
#endif
// local functions' declarations
#ifdef DeclareFunction
void	flash_latency(uint32_t frequency);
static void SysInit(void);
static void pll_start(uint32_t crystal, uint32_t frequency);
static void rcc_init(void);
void 				SetDate(void);

u16 	torgb(u08 R,u08 G,u08 B);
void 				I2C2_init(void);
void 				I2C2_READ_REG( u08 adres, u08 reg_adres, u08 * dane, u08 len );
void				ReadNRF(void);
void 				WriteNRF(void);
void				Menu(void);
void 				nRFserver(void);
void 				MultiNRF(void);
void 				Receiver(void);
void 				OtherTest(void);

void io_pin_cfg(GPIO_TypeDef *port_ptr, int pin, int mode_cnf_value);

inline static void Uart1Init(void);
inline static void Uart2Init(void);
inline static void Uart3Init(void);
inline static void Tim2PWMInit(void);
inline static void Tim4Init(void);
inline static void Tim3Init(void);
inline static void ExtIInit(void);
inline static void ADC1Init(void);
inline static void ADC1DmaInit(void);
inline static void ADC1_DMA_Tim4_Init(void);
inline static void SysTickInit(void);
static void gmtime_r(const u32 *, struct tm *);
void nrf24con(void);
void TPDraw(void);
void Grfaphics(void);
void Console(void);
void ClockGraph(void);

uint32_t adc_get_internals(uint32_t avg_cycles);
void adc1_init_injected(uint32_t avg_cycles, uint32_t channels);
uint32_t adc_get_injected(uint32_t avg_cycles);
void Mem2MemDMAInit(void *src_buf, void *dst_buf);
void Mem2MemDMA(void *src_buf, u32 offset );
void Intro1(void);
void Intro2(void);	
void DACDmaInit(void);
void MemCard_Menu(void);
void Test_avg(void);
void Radio(void);
void CopyClock(char *dczas, T_RTC *Rtc);
void CopyTime(char *dczas, T_RTC *Rtc);				
void CopyDate(char *dczas, T_RTC *Rtc);				
void TPDraw(void);
void EN25QH64_Tool(void);
void ENC28J60_Tool(void);
void http_client(void);
void qtouch(void);
u32 qtouch_mesaure(void);
void data_logger(void);

void BMP280_spi(void);
void BMP280_i2c(void);
void TSL2561_i2c(void);

void MAX44009_i2c(void);
void SleepMode_(u08);
void lexmark_lcd(void);
void LoRaRX(void);
void LoRaTX(void);
void uv_meter(void);
void wiznet5500(void);
void Zegarek(void);
void veml6075(void);
void ili9486(void);
void rtc_calibrate(void);
void SPI_Tool_Menu(void);
void SPI_Tool(void);
void lion_updown(void);
void oblicz(void);
void minios(void);

#endif
/*== INIT ===================================================================*/
void SysInit(void){
#ifndef RAMCODE
	flash_latency(F_CPU);				// configure Flash latency for desired frequency
#endif
	pll_start(CRYSTAL, F_CPU);

#ifdef RAMCODE
  // __disable_irq();
	//SCB->VTOR = NVIC_VectTab_RAM | (Offset & (uint32_t)0x1FFFFF80); //If in RAM
	SCB->VTOR = NVIC_VectTab_RAM;
	//__DSB();
	//__enable_irq();
	
#endif
 //	SCB->CCR |= SCB_CCR_STKALIGN ;
	
	rcc_init();
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_1; // enable pin PA15=JTDI,PB3=JTDO,PB4=JNTRST,
	SysTickInit();
	//Tim4Init();						// Init Timer 4 - Pomiar czasu
	
	//==== ENABLE IRQ ==============================================================
	//Warninng !!! not use "NVIC_Enable..." before Init section 
	//NVIC_EnableIRQ(TIM7_IRQn);
	//NVIC_EnableIRQ(TIM2_IRQn);
	//NVIC_EnableIRQ(EXTI0_IRQn);			// Line 0
	//NVIC_EnableIRQ(EXTI1_IRQn);			// Line 1
	//NVIC_EnableIRQ(EXTI2_IRQn);			// Line 2
	//NVIC_EnableIRQ(EXTI3_IRQn);			// Line 3
	//NVIC_EnableIRQ(EXTI4_IRQn);			// Line 4
	//NVIC_EnableIRQ(EXTI9_5_IRQn);			// Line 5..9
	//NVIC_EnableIRQ(EXTI15_10_IRQn);	// Line 10..15
	//NVIC_EnableIRQ(RTC_IRQn);				// Real Time Clock Initail
	//NVIC_EnableIRQ(DMA1_Channel1_IRQn);				// DMA1 Channel1
}

/*== MAIN ===================================================================*/
void head(void){
		//Delay_ms(200);
		tr_pen_color( TGREEN );
	#ifdef RAMCODE
		UaPutK("\f\r\n Mini STM32F103C8T6 RAM   "); // Boot from RAM 	 = B0+ B1+
	#else
		UaPutK("\r\n Mini STM32F103C8T6 FLASH "); // Boot from FLASH = B0- B1+
	#endif
		uint2uart(F_CPU/1e6);	UaPutK(" MHz");
		tr_pen_color( TYELLOW );
	}
static u08 dma_buf[24];		

int main(void){
		
		SysInit();
		 #define LED  PC13_o
		 GPIOC->CRH = (GPIOC->CRH & 0xff0fffff) | 0x00100000;	// LED Blue  PC13		
		/*
		uint16_t wart[16][16]; //tablica
		wart[5][5] = 15;
		uint16_t *BKPSRAMADDR0 = (uint16_t *)0x40024000UL;//wskaźnik do pocz¹tku przestrzeni adresowej BACKUP SRAM
		*(BKPSRAMADDR0 + 5) = wart[5][5];
		BKPSRAMADDR0 				= &wart[5][5];
		//wart[licz1][licz2]= *(BKPSRAMADDR0 + licz3);
		*/
		//lion_updown();
		//oblicz();
		minios();
	}
/*== MAIN ===================================================================*/
void minios(void){
	
		char swh,buf[12];
		u16 vr1,vr2,cnt,cnt2=0,vbus;
		s16 cur;
		_Bool tf;
	
		GPIOB->CRL = (GPIOB->CRL & 0x0fffffff) | 0x10000000;	// PB7
		Uart1Init(); // TTY4 Green->PA10_RX, White->PA9_TX 
			Uart2Init(); // PA2_TX, PA3_RX
			
			PutChar = UART1_putc;	tr_pen_color ( TYELLOW );
			UaPutS("\f UART1 "); UaPutS (uint2str( (F_CPU/1)/(1*USART1->BRR-1),buf));	
			UaPutS (" b/s \r\n");
			
			PutChar = UART2_putc;	tr_pen_color ( TYELLOW );
			UaPutS("\f UART2 "); UaPutS (uint2str( (F_CPU/1)/(1*USART1->BRR-1),buf));	
			UaPutS(" b/s \r\n");
			cnt=0;
			
			u32 tim,tim4;
			u08 i2reg[2]; //i2reg[0]=MSB i2reg[1]=LSB
			u08 i2weg[3];
			
			i2c1r_init();
			i2weg[0]=0;     // Register
			i2weg[1]=0x1f;  // MSB Data
			i2weg[2]=0x77;  // LSB Data
			i2c1_write(INA219_ADDR,i2weg,3);
		
			i2weg[0]=5;    	// Register
			i2weg[1]=0x1; 	// MSB Data
			i2weg[2]=0x95; 	// LSB Data
			i2c1_write(INA219_ADDR,i2weg,3);
			
			i2c1_read (INA219_ADDR,0,i2reg,2);
			tr_locate( 1,3);
			UaPutS("Config="); unt2uart(i2reg[0] + (i2reg[0]<<8) ,5); 
			tr_locate( 1,2);
			
		while(1){
			
			if(STCLK_MS*500 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
				//LED ^= 1;	
				
				PutChar = UART2_putc;
				unt2uart(cnt++,5); UaPutS(" "); unt2uart(cnt2,6); UaPutS("\r");
				PutChar = UART1_putc;
				unt2uart(cnt  ,5); UaPutS("\r");
				
				i2c1_read( INA219_ADDR,2,i2reg,2);
				vbus = ((i2reg[1] | (i2reg[0]<<8))>>3)*4;
				tr_locate(1,3);
				UaPutS("vbus=");
				//unt2uart(vbus,5);
				int2uart (vbus ,4,3);
				
				i2c1_read(INA219_ADDR,4,i2reg,2);
				cur = ((i2reg[1]) | (i2reg[0]<<8));
				UaPutS(" cur=");
				int2uart (cur,4,3);
				
				tr_locate(1,2);
			}
			
			if(PB11_i==1) {
				LED ^= 1;	
				cnt2++;
			}
			
			if(p_Fifo1->rct){
				UART_getChar((char*)&swh);
				//PutChar = UART1_putc;
				//UaPutS("zeruj");
				switch (swh){
					case 'a':
					UaPutS("\r\n");
					UaPutS("\t\ta Podaj liczbe ");
					vr1 = UART_getNum();
					UaPutS(" num=");	
					unt2uart(vr1,4);
					tr_locate( 1, 2 );
					//var1 = UART_getNum();
					//var2 = UART_getHex();
					break;
					case 'b':
					//UaPutS("");
					PB7_o ^= 1;
					break;
					default:
					break;
				}
			}
		}
	}
	
void liontrend(s16 v[],s16 c[]){
	
		v[6] = (100*(v[0]-v[1]))/(v[2]-v[3]); //a=(y2-y1)/(x2-x1)
		v[5] = v[0]-v[6]*v[2]/100;  //b=y-ax
		v[4] = v[1]-v[6]*v[3]/100;  //b=y-ax;
		v[7] = 100*(339-v[5])/v[6]; //x=(y-b)/a
		
		c[6] = (100*(c[0]-c[1]))/(c[2]-c[3]);
		c[5] = c[0]-c[6]*c[2]/100;//y-ax
		c[4] = c[1]-c[6]*c[3]/100;//y-ax
		c[7] = c[5]+c[6]*v[7]/100;// szukana pojemnosc
		
		UaPutS("v="); sint2uart(v[6]); sint2uart(v[5]); sint2uart(v[4]); sint2uart(v[7]); UaPutS("\r\n"); 
		UaPutS("c="); sint2uart(c[6]); sint2uart(c[5]); sint2uart(c[4]); sint2uart(c[7]); UaPutS("\r\n"); 
	}
void oblicz(void){
	
		BkpRegInit();
		Uart1Init(); // TTY4 Green->PA10_RX, White->PA9_TX 
		
		char buf[12];
		GPIOB->CRH = (GPIOB->CRH & 0xf000ffff) | 0x05550000;
		//Delay_ms(100);
		PB12_o = 0;		// rozladowanie
		PB13_o = 0;   // ladowanie
		PB14_o = 1;   // dzwonek
		
		PutChar = UART1_putc;	tr_pen_color ( TYELLOW );
		UaPutS("\f UART1 "); UaPutS (uint2str( (F_CPU/1)/(1*USART1->BRR-1),buf));	
		UaPutS (" b/s \r\n");
		
		//         v1 v2  x1 x2 c b a
		s16 v[8]={365,356,0, 30,0,0,0,0};
		s16 c[8]={2000 ,6400 ,0, 30,0,0,0,0};
		
		s16 a[8]={342,336,0,2,0,0,0,0};
		s16 b[8]={114,263,0,2,0,0,0,0};
		
		liontrend(v,c);
		liontrend(a,b);
		//v[6] = (100*(v[0]-v[1]))/(v[2]-v[3]);
		//v[5] = v[0]-v[6]*v[2]/100;//y-ax
		//v[4] = v[1]-v[6]*v[3]/100;//y-ax
		//unt2uart (cnt++,3);
		
		//UaPutS("v="); sint2uart(v[6]); sint2uart(v[5]); sint2uart(v[4]); sint2uart(v[7]); UaPutS("\r\n"); 
		//UaPutS("c="); sint2uart(c[6]); sint2uart(c[5]); sint2uart(c[4]); sint2uart(c[7]); UaPutS("\r\n"); 
		//UaPutS("v[0]="); sint2uart(v[0]);
		
		while(1);
}
void lion_updown(void){
		BkpRegInit();
		Uart1Init(); // TTY4 Green->PA10_RX, White->PA9_TX 
		Uart2Init(); // PA2_TX , PA3_RX
		//Uart3Init(); // 
		//head();
		
		//char * txt1="ala ma kota";
		char buf[12];
		
		PutChar = UART1_putc;	tr_pen_color ( TYELLOW );
		UaPutS("\f UART1 "); UaPutS (uint2str( (F_CPU/1)/(1*USART1->BRR-1),buf));	
		UaPutS (" b/s \r\n");
		//PutChar = UART2_putc;	tr_pen_color ( TYELLOW );
		//UaPutS("\f UART2 "); UaPutS (uint2str( (F_CPU/1)/(2*USART2->BRR-1),buf));
		//UaPutS (" b/s \r\n");
		// PutChar = USART3_putc;	tr_pen_color ( TYELLOW );
		// UaPutS("\f UART3 "); UaPutS (uint2str( (F_CPU/1)/(2*USART3->BRR-1),buf));	UaPutS (" b/s \r\n");
		//SPI_Tool();
		//SPI_Tool_Menu();
		
		u32 tim,tim4;
		u08 i2reg[2]; //i2reg[0]=MSB i2reg[1]=LSB
		u08 i2weg[3];
		
		static s16 cur,vbus,vacu;
		static u32 Vavg,cap;
		
		static s16 pow,cal,ene,shunt;
		static u08 cnt,idt;
		bool stop=0,beep=1;
		
		i2c1r_init();
		// Rshunt = 0.10156 ohm
		i2weg[0]=0;    // Register
		i2weg[1]=0x1f; // MSB Data
		i2weg[2]=0x77; // LSB Data
		i2c1_write(INA219_ADDR,i2weg,3);
		
		i2weg[0]=5;    // Register
		i2weg[1]=0x1; // MSB Data
		i2weg[2]=0x95; // LSB Data
		i2c1_write(INA219_ADDR,i2weg,3);
		
		i2c1_read (INA219_ADDR,0,i2reg,2);
		//UaPutS(" 0x"); hex2uart (INA219_ADDR,2);
		//UaPutS(" 0x"); hex2uart((i2reg[0]<<8)+i2reg[1],4);
		//UaPutS("\r\n ");
		
		GPIOB->CRH = (GPIOB->CRH & 0xf000ffff) | 0x05550000;
		//Delay_ms(100);
		PB12_o = 0;		// rozladowanie
		PB13_o = 0;   // ladowanie
		PB14_o = 1;   // dzwonek
		
		//BKP->DR1 = 0;
		//BKP->DR2 = 0;
		cap = (BKP->DR2<<16) + BKP->DR1;
		u32 *cap2 = (u32*)&BKP->DR1; 
		volatile uint32_t *pBKP = (volatile uint32_t *)(&BKP->DR1);
		 
		/*
		#define BKPSRAM_BASE 0x40024000
		 // Write to Backup SRAM with 32-Bit Data 
   for (i = 0x0; i < 0x100; i += 4) {
       *(__IO uint32_t *) (BKPSRAM_BASE + i) = i;
   }

   // Check the written Data 
   for (i = 0x0; i < 0x100; i += 4) {
          if ((*(__IO uint32_t *) (BKPSRAM_BASE + i)) != i){
              errorindex++;
          }
   }
		*/
		
		PB13_o = 0;		PB12_o = 1;		beep=0; PB14_o = 1; // AP
		trg1  = SysTick->VAL;	
		while(1){
			if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){
				trg0  = SysTick->VAL;
				
				//i2c1_read(INA219_ADDR,1,i2reg,2);
				//shunt =i2reg[1] | (i2reg[0]<<8); //  & 0b0111111111111111;
				i2c1_read( INA219_ADDR,2,i2reg,2);
				vbus = ((i2reg[1] | (i2reg[0]<<8))>>3)*515/128 ;
				//i2c1_read( INA219_ADDR,3,i2reg,2);
				//pow =  i2reg[1] | (i2reg[0]<<8);
				
				i2c1_read( INA219_ADDR,4,i2reg,2);
				cur = ((i2reg[1]) | (i2reg[0]<<8)) ;
				vacu = vbus + cur*108/1000;
				pow = vacu*cur/1000;
				cap += abs(cur);
				BKP->DR1 = cap & 0x00ff;
				BKP->DR2 = cap>>16;
				
				//i2c1_read( INA219_ADDR,5,i2reg,2);
				//cal = ((i2reg[1]) | (i2reg[0]<<8));
				PutChar = UART1_putc;
				unt2uart (cnt++,3);
				int2uart (vacu ,4,3);				UaPutS("V");
				int2uart (cur  ,5,0);				UaPutS("mA");
				int2uart (pow  ,4,3);				UaPutS("W");
				int2uart (cap/36  ,6,2);	UaPutS("mAh");
				UaPutS("\r ");
				
				PutChar = UART2_putc; //przypisanie callback 
				int2uarz (vacu ,4,3); 			UaPutC(',');
				int2uarz (cur  ,4,3); 			UaPutC(',');
				int2uarz (pow  ,4,3); 			UaPutC(',');
				int2uarz (cap/36 ,6,5);
				//UaPutS("\r ");
				cnt++;
				//LED ^= 1;	
			}
			
			if( (STCLK_MS*5000 < (trg1 - SysTick->VAL)) ){	
				trg1  = SysTick->VAL;	
				if(vacu > 2000 && vacu < 2750){
					PB12_o = 0;
					PB13_o = 0;
					beep=1;
				}
				
				if(vacu > 4190 && cur < 100){
					PB13_o = 0;
					PB12_o = 1;
					cap = 0;
					BKP->DR1 = 0;
					BKP->DR2 = 0;
				} 
			}
			
			if( (STCLK_MS*500   < (trg2 - SysTick->VAL))) {
				trg2  = SysTick->VAL;	
				LED ^= 1;	
				if(beep){	PB14_o ^= 1; }
			}
			
			char swh;
			u16 var1,var2;
			
			if(p_Fifo1->rct){
				UART_getChar( (char*)&swh);
				//PutChar = UART1_putc;
				//UaPutS("zeruj");
				switch (swh)	{
					case 'z':
					//UaPutS("Zeruj Licznik pojemnosci");
					cap = 0;
					BKP->DR1 = 0;
					BKP->DR2 = 0;
					//var1 = UART_getNum();
					//UaPutS("\r\n Read Reg1=0x");	
					//var1 = UART_getNum();
					//var2 = UART_getHex();
					break;
					case 'a':
					PB12_o = 0;
					PB13_o = 1;
					beep=0; PB14_o = 1;
					break;
					case 'd':
					PB13_o = 0;
					PB12_o = 1;
					beep=0; PB14_o = 1;
					break;
					case 'q':
					PB13_o = 0;
					PB12_o = 0;
					PB14_o = 1;
					break;
					case 'b':
					//PB14_o ^= 1;
					beep  ^= 1;
					PB14_o = 1;
					break;
					case 'v':
					//PB14_o ^= 1;
					//pomv=1;
					break;
					default:
					break;
				}
			}
		}
	}
void uht(void){
	static u32 cnt1,cnt2,cnt3;
		AM23_hw_t am23;
		GPIOB->CRL = (GPIOB->CRL & 0xffffff0f) | 0x00000060;
		am23.pin = (u32*) m_BITBAND_PERIPH(&GPIOB->IDR, 1); 
		am23.prt = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR, 1); 
		am23.typ = AM2302;
		
 		#define COSTXT "UART_SPEED"
		
		
		while(0){

			if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){
				trg0  = SysTick->VAL;
				LED ^= 1;
				//UaPutS (" \r");
				//TIM_start();
				//unt2uart  (cnt1=cnt1+1,10);
				//TIM_stop(&tim4);
				//UaPutS ("\t");
				//unt2uart(tim4/TIM_US,8);
				
				//UaPutS2(" \r");
				//unt2uart2 (tim4/TIM_US,5);
				//UaPutS2(" [us]");
				if(!AM23_read(&am23)){
						
					//UaPutS("\r H=");
					unt2uart (cnt1++,6);
					int2uart (am23.AMh,3,1);	
					//UaPutS("[%] T=");
					UaPutC(',');
					int2uart (am23.AMt,3,1);	
					//UaPutS("[C] ");
					
					UaPutS("\r\n");
				}
			}
			
			if(STCLK_MS*800 < (trg1 - SysTick->VAL)){
				trg1  = SysTick->VAL;
				//UaPutS2(" \r");
				//unt2uart2 (cnt2=cnt2+1,10);
				//UaPutS2("\r H=");
				//unt2uart2 (am23.AMh,3);	
				//UaPutS2("\t T=");
				//unt2uart2 (am23.AMt,3);	
			}
			
			if(STCLK_MS*200 < (trg2 - SysTick->VAL)){
				trg2  = SysTick->VAL;
				UaPutS(" \r");
				unt2uart (cnt3=cnt3+1,10);	
			}
		}
		
		//head();
		//SPI_Tool();
		//veml6075();
		//rtc_calibrate();
		//ili9486();
		
		//Zegarek();
		//SleepMode(1);
		//uv_meter();
		//wiznet5500();		
		//LoRaRX();
		//LoRaTX();
		
		// no run Info : SWD DPIDR 0x2ba01477 =  CS32F103C8t6 CPU-TAP-ID
		//    run Info : SWD DPIDR 0x1ba01477
		// #7F7F7F #8B6914 #800080
	} 
void rtc_calibrate(void){
		u08 reg_adr,tm[4],ContReg,buf[2];
		char tbuf[10];
		u32 tim,tim4;
		
		RtcInit();
		i2c1_init();
		GPIOA->CRH = (GPIOA->CRH & 0x0ff00fff) | 0x10011000;
		GPIOB->CRL = (GPIOB->CRL & 0xfff00fff) | 0x00011000;
		
		oled_hw.sck  = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,4);
		oled_hw.mosi = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,3);
		oled_hw.dc   = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,12);
		oled_hw.cs   = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,11);
		oled_hw.rst  = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,15);
		
		//sh1106_init();
		ssd1306_init();
		
		buf[0]=0x0e;
		buf[1]=0x40; //0x40=1Hz
		
		//UaPutS("\r\nDS3231 read \r\n");
		//i2c1_write(DS3231_ADDR, buf, 2);
		/*
		while(!iPB5)	
		while(iPB5) 	TIM_start();
		while(!iPB5)	TIM_stop(&tim);
		
		while(iPB5) 	TIM4_start();
		while(!iPB5)	TIM4_stop(&tim4);
		*/
		//UaPutS("\r\n");
		//hum2uart(tim/TIM_US);
		num2uart (tim/TIM_US,7,0);	
		
		//hum2uart(tim2);
		num2uart (tim4,8,1);	
		//UaPutS("\r\n");
		
		while(1){
			
			if( TIM_MS*500 < (trg0 - SysTick->VAL) ){
				trg0 = SysTick->VAL; 
				
				i2c1_read( DS3231_ADDR,0,tm,3);
				//UaPutS("\r< ");
				//hex2uart(tm[2],2);
				//hex2uart(tm[1],2);
				//hex2uart(tm[0],2);
				i2c1_read( DS3231_ADDR,0x0e,&ContReg,1);
				//UaPutS("CR=");
				//hex2uart(ContReg,2);
				//int2str(ContReg,tbuf,5,0);		oled_num24(0,0,tbuf);
				
				while(!PB5_i)
				while(PB5_i) 	TIM_start();
				while(!PB5_i)
				while(PB5_i)	TIM_stop(&tim4);
				//num2uart (tim4,8,1);	
				TIM_start();
				Delay_us(123);
				
				int2str(tim4,tbuf,6,0); 
				int2str(123,tbuf,6,0); 
				
				oled_str16(0,0,tbuf);
				
				//num2uart (Tim4/TIM_US,8,0);	
				//UaPutS(" >");
				oled_refresh_gram();
				
			}
		}
	}
void testwsk(void){
		
		uint16_t tab1[]={8,513,734,612,87};
		uint8_t  tab2[]={7,99 ,1  ,2  ,113};
		
		uint8_t  * wsk1 = (u08*)tab1;
		uint16_t * wsk2 = (u16*)tab2;
		
		wsk1 += 3;
		wsk2 += 1;
		
		int a = *wsk1;
		int b = *wsk2;
	}
void veml6075(void){
		
		char swh,tbuf[12];
		p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='l';
		u08 sx[8],reg,dat,buf_adr[3],c_adr=0;
		u16 *veml = (u16*)&buf_adr[0];
		u16 cnt=0x0;
		
		struct {
			u16 UVA;
			u16 UVB;
			u16 UVD;
			u16 UV1;
			u16 UV2;
			u32 UVI;
			u32 UVAc;
			u32 UVBc;
			
		}uv;
		
		LED=1;
		
		GPIOA->CRH = (GPIOA->CRH & 0x0ff00fff) | 0x10011000;
		GPIOB->CRL = (GPIOB->CRL & 0xfff00fff) | 0x00011000;
		
		oled_hw.sck  = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,4);
		oled_hw.mosi = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,3);
		oled_hw.dc   = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,12);
		oled_hw.cs   = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,11);
		oled_hw.rst  = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,15);
		
		//sh1106_init();
		ssd1306_init();
		
		//hex2str(GPIOA->CRH ,tbuf,8);		oled_num24(1,3,tbuf);
		//hex2str(GPIOB->CRL ,tbuf,8);		oled_num24(1,0,tbuf);
		
		//ssd1306_init();
		//ssd1306_clear_screen(0x00);
		//ssd1306_refresh_gram();
		
		//i2c2_init();
		buf_adr[0]=0x0;
		buf_adr[1]=0x10;	// 100ms
		buf_adr[2]=0x0;
		//i2c2_write(0x20, buf_adr, 3);
		
		oled_refresh_gram();
		
		while(1){
			if(STCLK_MS*20 < (trg0 - SysTick->VAL)){	
				trg0  = SysTick->VAL;	
				int2str(cnt++ ,tbuf,5,0);		oled_num24(0,0,tbuf);
				oled_refresh_gram();	
				
				LED^=1;
			}
		}
		
		while(1){
			
			if(p_Fifo1->rct){
				UART_getChar( (char*)&swh);
				
				switch (swh)	{
					case 'r':
					//strBlock = UART_getNum();
					UaPutK("\r\n reg = 0x");	reg=UART_getHex();
					UaPutK("\r\n dat = 0x");
					i2c2_read(0x20,reg,buf_adr,2);	hex2uart(*veml,4);
					break;
					case 'w':
					UaPutK("\r\n reg = 0x"); buf_adr[0]=UART_getHex();
					UaPutK("\r\n dt0 = 0x"); buf_adr[1]=UART_getHex();
					UaPutK("\r\n dt1 = 0x"); buf_adr[2]=UART_getHex();
					
					i2c2_write(0x20, buf_adr, 3);
					reg = buf_adr[0];
					UaPutK("\r\n dat = 0x");
					i2c2_read(0x20,reg,buf_adr,2);	hex2uart(*veml,4);
					break;
					case 'c':
					UaPutK("\f");
					UaPutK("\n\r\n======= i2c CS8416 Audio Interface ===============\r\n");
					UaPutK(" r - Read   c - Clear page  \r\n");
					UaPutK(" w - Write  l - Loop read   \r\n");
					UaPutK("==================================================\r\n > ");
					break;
					case 'l':
					//strBlock = UART_getNum();
					UaPutK("oop \r\n");
					do{
						
						if(STCLK_MS*500 < (trg0 - SysTick->VAL)){	
							trg0  = SysTick->VAL;	
							//i2c2_read(0x20,0x07,buf_adr,2);	hex2uart(*veml,4);
							//i2c2_read(0x20,0x09,buf_adr,2);	hex2uart(*veml,4);
							
							i2c2_read(0x20,0x07,buf_adr,2); uv.UVA=*veml; unt2uart (uv.UVA ,4); UaPutK(","); //UVA
							i2c2_read(0x20,0x09,buf_adr,2);	uv.UVB=*veml; unt2uart (uv.UVB ,4); UaPutK(","); //UVB
							i2c2_read(0x20,0x08,buf_adr,2);	uv.UVD=*veml; unt2uart (uv.UVD ,4); UaPutK(","); //UVD
							i2c2_read(0x20,0x0a,buf_adr,2);	uv.UV1=*veml; unt2uart (uv.UV1 ,4); UaPutK(","); //UVCMP1
							i2c2_read(0x20,0x0b,buf_adr,2);	uv.UV2=*veml; unt2uart (uv.UV2 ,4); UaPutK(","); //UVCMP2
							
							uv.UVAc = (100*(uv.UVA-uv.UVD)-(333*(uv.UV1-uv.UVD)-250*(uv.UV2-uv.UVD)));
							uv.UVBc = (100*(uv.UVB-uv.UVD)-(366*(uv.UV1-uv.UVD)-275*(uv.UV2-uv.UVD)));
							uv.UVAc /= 100;
							uv.UVBc /= 100;
							//sint2uart (uv.UVAc); UaPutK(",");	
							//sint2uart (uv.UVBc); UaPutK(",");	
							uv.UVI  = ((uv.UVBc*125) + (uv.UVAc*110))/2;
							
							
							//unt2uart (uv.UVI  ,5); UaPutK("\r\n");	
							//num2uart (uv.UVI ,6, 0);
							//sint2uart (uv.UVI); 
							//UaPutK("\r\n");	
							
							int2str(uv.UVI ,tbuf,5,4);		oled_num24(0, 4,tbuf);
							//int2str(uv.UVA ,tbuf,4,0);		oled_num24(0, 0,tbuf);
							//int2str(uv.UVB ,tbuf,4,0);		oled_num24(64,0,tbuf);
							
							int2str(cnt++ ,tbuf,4,0);		oled_num24(0,0,tbuf);
							
							oled_refresh_gram();	
							LED^=1;
							
						}
						
					} 
					//while (!p_Fifo1->rct);
					while (1);
					break;
					
					default:
					UaPutK("\r");
					break;
				}
			}
		}
	}
void Zegarek(void){
		u32 cnt=0,uxt;
		char str_num[10];
		char tbuf[9] ={"00-00-00"};
		char dbuf[11]={"0000-00-00"}; // data
		
		RtcInit();
		
		//ssd1306_init();
		//ssd1306_clear_screen(0x00);
		//ssd1306_str16(0,4,"Zegarek");
		//ssd1306_refresh_gram();
		while(1){
			
			if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){
				trg0  = SysTick->VAL;
				rtc_GetTime(1,&Rtc);
				CopyTime(tbuf,&Rtc);
				CopyDate(dbuf,&Rtc);
				//ssd1306_str16(0,0,num2str(Lp++ ,str_num,5,0));
				//ssd1306_num24(0,0,num2str(Lp++ ,str_num,5,0));
				//uxt = (RTC->CNTH<<16) | RTC->CNTL;
				//ssd1306_str16(0,6,hex2str(uxt,str_num,8));
				
				//ssd1306_num24(0,4,dbuf);
				//ssd1306_num24(0,0,tbuf);
				//ssd1306_refresh_gram();	
			}
		}
	}
uint8_t softSPI_w(uint8_t byte)	{
		uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			if (byte & 0x80)			wMOSI = 1;
			else			wMOSI = 0;
			
			byte <<= 1;
			wSCK = 1; /* a slave latches input data bit */
			Delay_us(10);
			if (wMISO)			byte |= 0x01;
			
			wSCK = 0; /* a slave shifts out next output data bit */
			Delay_us(10);
		}
		return(byte);
	}

uint8_t softSPI_r(uint8_t byte)	{
		uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			if (byte & 0x80)			wMOSI = 1;
			else				wMOSI = 0;
			
			wSCK = 0; /* a slave shifts out output data bit */
			byte <<= 1;
			
			if (wMISO)		byte |= 0x01;
			wSCK = 1; /* a slave latches input data bit */
		}
		return(byte);
	}
void ili9486(void){
		char swh,reg,dat;
		
		p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='c';
		
		GPIOA->CRL = (GPIOA->CRL & 0x0000ffff) | 0x34330000;	//   PA7=MOSI, PA6=MISO, PA5=SCK, PA4=CE 
		
		while(1){
			
			if(p_Fifo1->rct){
				UART_getChar( (char*)&swh);
				
				switch (swh)	{
					case 'r':
					//strBlock = UART_getNum();
					UaPutK("\r\n reg = 0x");	reg=UART_getHex();
					UaPutK("\r\n dat = 0x");
					wCS = 0;		dat = softSPI_r(reg); 	wCS = 1;
					hex2uart(dat,2);
					
					break;
					case 'w':
					wCS = 0;
					for(u08 i=0;i<64;i++){
						hex2uart(softSPI_r(i),2);
					}
					wCS = 1;
					UaPutK("\r\n");
					
					break;
					case 'c':
					UaPutK("\f");
					UaPutK("\n\r\n======= ILI9486 ==================================\r\n");
					UaPutK(" r - Read   c - Clear page  \r\n");
					UaPutK(" w - Write  l - Loop read   \r\n");
					UaPutK("==================================================\r\n > ");
					break;
					default:
					UaPutK("\r");
					break;
				}
			}
		}
		
	}

void wiznet5500(void)		{
	u16 AMh;
	s16 AMt;
	char strg[10];
	SPI1_init_();
	//GPIOA->CRL = (GPIOA->CRL & 0x000fffff) | 0x54500000;	// PA5~7
	GPIOB->CRL = (GPIOB->CRL & 0xffffff0f) | 0x00000010;	
	GPIOC->CRH = (GPIOC->CRH & 0xf0ffffff) | 0x01000000;	// LED Blue  PC14
	
	DHT_Init();
	//ssd1306_init();
	//ssd1306_clear_screen(0xff);
	//ssd1306_display_string3216(0,32, (const u08*) unt2str(12345678, strg, 8));
	//ssd1306_refresh_gram();
	
	/*
	PB1_o = 1;
	PC14_o = 1;
	PC14_o = 0;	Delay_ms(2);	PC14_o = 1;
	Delay_ms(200);
	PB1_o = 0;
	
	softSPI_w(0x0);
	softSPI_w(0x0);
	softSPI_w(0x0);
	reg=softSPI_w(0x0); 
	*/ 
	
	//spi1_rw(0x0);
	//spi1_rw(0x1);
	//spi1_rw(0x0);
	//reg=spi1_rw(0x0);
	
	
	//UaPutK("\r\n W5500(1)=");
	//uint2uart(reg);
	//ssd1306_display_string    (0,0 , (const u08*)"Temp[C]   Hum[%]", 16, 1);
	//ssd1306_draw_point(0, 8, 1);
	u16 var=1;
	u08 tx[24]={0,1,2,3,4,5,6,7,8,9,8,4,6,8,10,12,14,15,13,12,11,10,9,8};
	//u08  tx[8]={0,1,2,3,4,5,6,7};
	u08 bar[8]={0x1,0x3,0x7,0xf,0x1f,0x3f,0x7f,0xff};
	
	for(u08 n=0;n<24;n++){
		//var = 1<<tx[n];
		//ssd1306_draw_byte(n,7,var>>8);
		//ssd1306_draw_byte(n,6,var&0xff);
		var = 1<<tx[n];
		if(var>255){ 
			//ssd1306_draw_byte(n,7,bar[tx[n]]);
			//ssd1306_draw_byte(n,6,bar[7]);
		}else{
			//ssd1306_draw_byte(n,7,0);
			//ssd1306_draw_byte(n,6,bar[tx[n]]);
		}
		
	} 
	
	//ssd1306_refresh_gram();
	

	
	while(1){
		
		if(STCLK_MS*1000  < (trg0 - SysTick->VAL)){
			trg0 = SysTick->VAL;
			
			if (!(DHT_read( &AMh, &AMt,1,&AM2302_pin)) ){		// AM2302 read T & H
			 	
			} 
			LED^=1;
			
			//ssd1306_display_string3216(0,32, (const u08*) unt2str(AMh, strg, 3));
			//ssd1306_display_string    (0,0 , (const u08*) int2str(AMt, strg, 3,1), 16, 1);
			//ssd1306_display_string    (45,0 , "C", 16, 1);
			
			//ssd1306_display_string    (0 ,16, (const u08*) int2str(AMh, strg, 3,1), 16, 1);
			//ssd1306_display_string    (45,16 , "%", 16, 1);
			//ssd1306_display_string3216(0 ,32,(const u08*) unt2str(AMt, strg, 3));
			//ssd1306_display_string3216(64,32,(const u08*) unt2str(AMh, strg, 3));
			if(flag_9);
			//ssd1306_display_string3216(48,32,(const u08*) ";");
			else
			//ssd1306_display_string3216(48,32,(const u08*) ":");
			flag_9 ^=1;
			
			//ssd1306_refresh_gram();//
		}
	}
	
}
void uv_meter(void)			{
		
		u16 adc5;
		char strg[10];
		
		GPIOA->CRL = (GPIOA->CRL & 0x000fffff) | 0xaa000000; // ADC12_IN5		
		
		PB10_o = 1;
		LED = 1;
		I2C2_init();
		u08 a[256],b;
		//i2c2_scan(a,&b);
		
		
		ADC1Init();
		ADC1->SQR1 = ( 0<<20 	); 		//length and sequence 12..15 conversion 13-16 on channel	(0..17)
		ADC1->SQR3 = ( 5<<0*5 );		//(channel<<sequence*5) sequence=0..5   
		ADC1->SMPR2 = (4<<5*3 );
		ADC1->CR1 = ADC_CR1_SCAN;			// multi channel mode		
		ADC1->CR2 |= ADC_CR2_ADON ;		// On ADC
		
		//ssd1306_init();
		//ssd1306_clear_screen(0x00);
		//ssd1306_draw_point(10, 10, 0);
		u08 cnt=0;
		
		while(1){
			while(PB0_i==0);
			//ssd1306_display_string3216(0,32, (const u08*) unt2str((cnt++)%20, strg, 2));
			//ssd1306_refresh_gram();
			//i2c2_read( 0x21, 0x0, i2, 2 );
			i2c2_scan(a,&b);
			//ssd1306_display_string3216(48,32, (const u08*) unt2str(b, strg, 2));
			//ssd1306_refresh_gram();
			//if((cnt%10)==0)ssd1306_clear_screen(0xff);
			while(PB0_i==1);
			b=0;
		}
		
		while(1){ 
			for(u08 n=0;n<8;n++){
				ADC1_CR2_ADON = 1;							// start conversion ADC
				//Delay_200n(9);
				Delay_us(10);
				if(ADC1->DR>1180)	adc5 += ADC1->DR-1180 ;
			}
			adc5 /= 8;
			//ssd1306_display_string3216(0,16, (const u08*) uint2str(k++,strg));
			//ssd1306_display_string3216(0,0,(const u08*) unt2str(adc5, strg, 4));
			
			LED = 1;
			//i2c2_read( 0x10, 0x0, i2, 2 );
			LED = 0;
			
			//ssd1306_display_string3216(0,0, (const u08*) unt2str(adc5, strg, 4));
			//ssd1306_display_string    (0,0,(const u08*) unt2str(adc5  , strg, 4),16,1);
			//ssd1306_display_string3216(0,32, (const u08*) unt2str(adc5, strg, 4));
			//ssd1306_display_string3216(0,32, (const u08*) unt2str(b, strg, 4));
			
			//ssd1306_refresh_gram();
			Delay_ms(200);
			//ssd1306_init();
			//test();
			//ssd1306_refresh_gram();
			
		}
	}
void IRQLoraInit(void)	{
		/*
			Library ST				 Manual RM0008
			AFIO->EXTICR[0] => AFIO_EXTICR1 GPIO 0..3			
			AFIO->EXTICR[1] => AFIO_EXTICR2 GPIO 4..7
			AFIO->EXTICR[2] => AFIO_EXTICR3 GPIO 8..11
			AFIO->EXTICR[3] => AFIO_EXTICR4 GPIO 12..15
			Jednocześnie pin moze byc skonfigurowany tylko dla jednego portu
		 */
		AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;		// Enable interrupt port PA0
		EXTI->PR   = EXTI_PR_PR0;			// Clear interupt request EXTI0
		EXTI->RTSR 	 = EXTI_RTSR_TR0;
		//EXTI->FTSR 			= EXTI_FTSR_TR6 | EXTI_FTSR_TR0;	// Failing trigger enabled
		EXTI->IMR  =  EXTI_IMR_MR0;		// Interrupt mask register EXTI_IMR
		NVIC_EnableIRQ(EXTI0_IRQn);			
	}
void LoRaRX(void)				{


		static SX1278_hw_t SX1278_hw1;
		static SX1278_t 	 SX1278_1;
		u16 	message_length,ret,err=0,hu;		
		char 	buf_rx[64],str_num[8];
		uint8_t cnt=1,min=0;
		s16 prssi;
		s16 t1,t2;
		u16 Cnt;
		u32 cn;
		
		struct usb dev[]= {{
			.bDat2 = 1,
			.bDat1 = 2,

		}};
		
		GPIOA->CRL = (GPIOA->CRL & 0xfff00000) | 0x00044343;	//   PA2=MOSI, PA1=MISO, PA0=SCK, PB8=CE PB9=Reset
		GPIOB->CRH = (GPIOB->CRH & 0xffffff00) | 0x00000033;	//   
		SX1278_hw1.mosi = (u32 *)m_BITBAND_PERIPH(&GPIOA->ODR, 2); 
		SX1278_hw1.miso = (u32 *)m_BITBAND_PERIPH(&GPIOA->IDR, 1); 
		SX1278_hw1.sck  = (u32 *)m_BITBAND_PERIPH(&GPIOA->ODR, 0); 
		SX1278_hw1.nss  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 8); 
		SX1278_hw1.rst  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 9); 
		SX1278_hw1.dio0 = (u32 *)m_BITBAND_PERIPH(&GPIOB->IDR, 1); 
		
		SX1278_1.hw = &SX1278_hw1;			// wybor modulu		
		SX1278_begin(&SX1278_1, SX1278_433MHZ, SX1278_POWER_11DBM, SX1278_LORA_SF_7,SX1278_LORA_BW_500KHZ);
		//strcpy (buf_tx, "RX_data ");
		//strcat (buf_tx, uint2str(1000,str_num));
		//message_length = strlen(buf_tx); 
		//*SX1278_hw1.rst = 1;
		
		//while(1);
		ret = SX1278_LoRaEntryRx(&SX1278_1, 7, 2000);

		/*
		if(STCLK_MS*500 < (trg0 - SysTick->VAL)){
			trg0 = SysTick->VAL;
			LED^=1;
		} */
		
		//UC1608_chr16(0,0,'1');
		//memcpy (lcd  ,"aaaaaaaaa",9);
		//memcpy (lcd  ,"T",1);
		//memcpy (lcd+3,"\0",1);
		//UC1608_str16(0, 6, "lora_rx");
		//UC1608_str16(0, 6, lcd);
		//UC1608_str16(100,  0,UC1608_unt2str( 234,str_num,3,1));
		//while(1);
		
		trg1  = SysTick->VAL;
		//memcpy (lcd+0,"h=",2);
		//memcpy (lcd+5,"T",1);
		
		
		//UC1608_str16(0 ,0,UC1608_unt2str( 0,str_num,3,1));		UC1608_str16(35,0,"^");
		//UC1608_str16(50,0,UC1608_int2str( 0,str_num,3,1));		UC1608_str16(90,0,"{");
		flag_10=0;
		u16 x=0;
		
		//UaPutK("\r\n ");
		while(0){
			
			if(STCLK_MS*500 < (trg1 - SysTick->VAL)){	
				trg1  = SysTick->VAL;
				//UaPutC('.');
				LED ^= 1;
				//uint2uart(x); UaPutK("\t ");
				uint2uart(dev[0].bDat1); UaPutK("\t ");

				if(x%10 == 0) UaPutK("\r\n ");
				x++;
				dev[0].bDat1++;
				
			}
		}
		
		while(1){
			
			
			//if( SX1278_LoRaRxPacket(&SX1278_1) > 5 ){
			if( (flag_10==0) && *SX1278_hw1.dio0==1 )  {
				
				//UaPutC('-');
				//if(STCLK_MS*1000 < (trg1 - SysTick->VAL)){	
				//	trg1  = SysTick->VAL;
				
				//SX1278_LoRaEntryRx(&SX1278_1, 7, 2000);
				//SX1278_LoRaRxPacket(&SX1278_1);  // Odczyt danych do buforów
				SX1278_LoRaRx(&SX1278_1);  // Odczyt danych do buforów
				//UaPutC('+');
				
				//SX1278_read(&SX1278_1, (uint8_t *) buf_rx, ret);
				//UaPutK("\r\n RX->");
				//UaPutS(buf_rx);
				//sint2uart(ret);
				//strcpy (buf_rx, "        ");
				//Delay_ms(100);				
				
				prssi = SX1278_SPIRead(&SX1278_1, LR_RegPktRssiValue)-164;
				//sint2str(rssi,str_num);
				//int2str(rssi ,str_num,3,0); UC1608_str (3, 0,str_num);
				//memcpy (UC1608_Buf,' ',150*8);
			
				//SX1278_read(&SX1278_1, (uint8_t *) buf_rx);
				buf_rx[7] = 0;
				memcpy(buf_rx, SX1278_1.rxBuffer, 7);
				

				//cn = buf_rx[4]+(buf_rx[5]<<8)+(buf_rx[6]<<16);
				//hu = buf_rx[0]+(buf_rx[1]<<8);
				t1 = (s16)(buf_rx[0]+(buf_rx[1]<<8));
				t2 = (s16)(buf_rx[2]+(buf_rx[3]<<8));
				Cnt= (s16)(buf_rx[4]+(buf_rx[5]<<8));
				//UaPutK("\r\n");
				//UaPutS((char *)SX1278_1.rxBuffer);
				//uint2uart(SX1278_1.readBytes);	
					//uint2uart(message_length);	
					
					//uint2uart(cn);	UaPutK("  ");
					sint2uart(t1);	//UaPutK("  ");
					UaPutC(',');					
					sint2uart(t2);	//UaPutK("  ");
					UaPutC(',');					
					sint2uart(Cnt);	//UaPutK("  ");
					UaPutS("\n");					
					//int2uart(tu,3,1); //UaPutK(" RSSI=");
					//int2uart(prssi,3,0); UaPutK("dBm");
					
					//tu = -(buf_rx[2]+((buf_rx[3]&127)<<8)); 
					//UC1608_str16(0, 0,UC1608_unt2str( min++,str_num,3,0));
					
					//UC1608_str16(0 ,0,UC1608_unt2str( hu,str_num,3,1));	UC1608_str16(35,0,"^");	// Wilgotnosc
					//UC1608_str16(0 ,0,UC1608_unt2str( hu,str_num,3,1));	UC1608_str16(35,0,"{");	// temperatura
					
					//memcpy (lcd+0,UC1608_unt2str( hu,str_num,3,1),4);
					//memcpy (lcd+4 ,"^}",2);
					//memcpy (lcd+6,UC1608_int2str( tu,str_num,3,1),4);
					//memcpy (lcd+12,"{\0",2);
					//UC1608_str16(0,0,lcd);
					//UC1608_str16(0,  0,UC1608_int2str( buf_rx[0]+(buf_rx[1]<<8),str_num,3,1));
					//UC1608_str16(60, 0,UC1608_int2str( buf_rx[2]+(buf_rx[3]<<8),str_num,3,1));
					//UC1608_str16(0,6,UC1608_int2str( buf_rx[3],str_num,3,0));
					
					cnt=1;
					
					LED = 0;
					trg1  = SysTick->VAL;
				
				flag_10 = 1;
			}
			
			if(flag_10 && (*SX1278_hw1.dio0)==0 ) { flag_10=0; }
			
			if((STCLK_MS*300 < (trg1 - SysTick->VAL)) && LED==0){	
					trg1  = SysTick->VAL;
				//UaPutC('.');
				LED = 1;
			}
		}
	}
//=========================================================================================================	
void LoRaRX1(void)	{
		// SX1278, 16cm antena 433 MHz https://blog.domski.pl/stm32-hal-driver-for-lora-sx1278-wireless-module/
		char swh;
		u08 reg,dat,sx[5];
		static SX1278_hw_t SX1278_hw1;
		static SX1278_t 	 SX1278_1;
		u16 Lp=0;
		
		//ssd1306_init();
		//ssd1306_clear_screen(0x00);
		p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='0';
		LED=1;
		
		// RX
		GPIOB->CRH = (GPIOB->CRH & 0x0000ff00) | 0x43430033;	//   
		SX1278_hw1.mosi = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 12); 
		SX1278_hw1.miso = (u32 *)m_BITBAND_PERIPH(&GPIOB->IDR, 13); 
		SX1278_hw1.sck  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 14); 
		SX1278_hw1.nss  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 8); 
		SX1278_hw1.rst  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 9); 
		SX1278_hw1.dio0 = (u32 *)m_BITBAND_PERIPH(&GPIOB->IDR, 15); 
		
		sx[0]=SX1278_433MHZ;
		sx[1]=SX1278_POWER_11DBM;
		sx[2]=SX1278_LORA_SF_7;
		sx[3]=SX1278_LORA_BW_500KHZ;
		sx[4]=10;
		
		SX1278_1.hw = &SX1278_hw1;			// wybor modulu		
		SX1278_begin(&SX1278_1,sx[0],sx[1],sx[2],sx[3]);
		SX1278_LoRaEntryRx(&SX1278_1, 64, 2000);
		
		int 	message_length=65;		
		static char 	buf_rx[256],str_num[12];
		s16 prssi;
		u32 tim=tim;
		
		flag_9=0;
		
		TIM_start();
		TIM_stop(&tim);				
		UaPutK(" Time=");		uint2uart(tim/TIM_US);			
		//ssd1306_str16(0,0,"gpqxy56");
		
		
		//ssd1306_refresh_gram();
		
		while(1){
			
			if(p_Fifo1->rct){
				UART_getChar( (char*)&swh);
				
				switch (swh)	{
					
					case 'q':
					//strBlock = UART_getNum();
					//*SX1278_hw1.nss = 0;
					UaPutK("\r\n");
					for (u08 n=0;n<0x14;n++){
						dat = SX1278_SPIRead(&SX1278_1, n);
						hex2uart(dat,2);
					}
					break;
					case 'r':
					//strBlock = UART_getNum();
					UaPutK("\r\n Read Reg1=0x");	reg=UART_getHex();
					UaPutK(" Reg1 0x");						hex2uart(reg,2);
					UaPutK("= 0x");						
					dat = SX1278_SPIRead(&SX1278_1, reg);
					hex2uart(dat,2);
					break;
					case 'p':
					UaPutK("\r\n 0-3("); uint2uart(sx[1]);UaPutK(") POWER=");					
					sx[1] = UART_getNum();					
					break;
					case 's':					
					UaPutK("\r\n 0-6("); uint2uart(sx[2]);UaPutK(") SF=");					
					sx[2] = UART_getNum();					
					break;
					case 'b':					
					UaPutK("\r\n 0-9("); uint2uart(sx[3]);UaPutK(") BW=");					
					sx[3] = UART_getNum();					
					break;
					case 'd':					
					UaPutK("\r\n sek("); uint2uart(sx[4]);UaPutK(") Delay=");					
					sx[4] = UART_getNum();					
					break;
					
					default:
					break;
				}
				
				SX1278_begin(&SX1278_1,sx[0],sx[1],sx[2],sx[3]);
				SX1278_LoRaEntryRx(&SX1278_1, 64, 2000);
				
				UaPutK("\r\n Menu > ");
				/*
				if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
				#A52A2ALX_RES = 0; Delay_us(10); LX_RES = 1;	// reset
				*/
			}
			
			// ================ Odbior RX ====================
			
			if( *SX1278_hw1.dio0 ){
				 message_length = SX1278_LoRaRxPacket(&SX1278_1);
				if(message_length){
					
					SX1278_read(&SX1278_1, (uint8_t *) buf_rx);
					UaPutK("\r\n prssi->");	
					prssi = SX1278_SPIRead(&SX1278_1, LR_RegPktRssiValue)-164;
					sint2uart(prssi);
					
					//memcpy (UC1608_Buf,' ',150*8);
					
					//ssd1306_str16(0 ,4,int2str(buf_rx[0]+(buf_rx[1]<<8),str_num,3,1));
					//ssd1306_str16(45,4,"%");
					//ssd1306_str16(50,6,num2str(buf_rx[4]+(buf_rx[5]<<8)+(buf_rx[6]<<16),str_num,7,0));
					
					//ssd1306_str16(0 ,6,int2str(buf_rx[2]+(buf_rx[3]<<8),str_num,3,1));
					//ssd1306_str16(45,6,"{");
					//ssd1306_str16(70,4,num2str(-1*prssi ,str_num,3,0));
					
					//ssd1306_refresh_gram();
					
					//UaPutK("' rssi=");		sint2uart(prssi);
					//UaPutK(" pkt_len=");	sint2uart(message_length);
					//UaPutS(".");
					LED ^= 1;
					Lp=0;
			 	} 
			}
			// ================ Odbior RX ====================
			if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){	
				trg0  = SysTick->VAL;	
				//ssd1306_str16(0,0,num2str(Lp++ ,str_num,5,0));
				//ssd1306_num24(0,0,num2str(Lp++ ,str_num,5,0));
				//ssd1306_refresh_gram();
			}
		}
		
	}
void LoRaTX(void)		{
		// SX1278, 16cm antena 433 MHz https://blog.domski.pl/stm32-hal-driver-for-lora-sx1278-wireless-module/
		
		u08 sx[5];
		static SX1278_hw_t SX1278_hw1;
		static SX1278_t 	 SX1278_1;
		
		int 	message_length=65;		
		static char 	buf_tx[10],str_num[5];
		u16 n;
		
		u32 tim=tim,alr,cnt=0;
		#define DivSec	16
		
		AM23_hw_t am23;
		//u16 AMh;
		//s16 AMt;
		
		// Low power		
		RtcInit();
		Tim3Init();		
		p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='0';
		
		//TX
		GPIOA->CRL = (GPIOA->CRL & 0xfff00000) | 0x00044343;	//   PA2=MOSI, PA1=MISO, PA0=SCK, PB8=CE PB9=Reset
		GPIOB->CRH = (GPIOB->CRH & 0xffffff00) | 0x00000033;	//   
		SX1278_hw1.mosi = (u32 *)m_BITBAND_PERIPH(&GPIOA->ODR, 2); 
		SX1278_hw1.miso = (u32 *)m_BITBAND_PERIPH(&GPIOA->IDR, 1); 
		SX1278_hw1.sck  = (u32 *)m_BITBAND_PERIPH(&GPIOA->ODR, 0); 
		SX1278_hw1.nss  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 8); 
		SX1278_hw1.rst  = (u32 *)m_BITBAND_PERIPH(&GPIOB->ODR, 9); 
		SX1278_hw1.dio0 = (u32 *)m_BITBAND_PERIPH(&GPIOB->IDR, 1); 
		
		sx[0] = SX1278_433MHZ;
		sx[1] = SX1278_POWER_11DBM;
		sx[2] = SX1278_LORA_SF_7;
		sx[3] = SX1278_LORA_BW_500KHZ;
		sx[4] = 10; // Przerwa miedzy pomiarami
		
		SX1278_1.hw = &SX1278_hw1;			// wybor modulu		
		SX1278_begin(&SX1278_1,sx[0],sx[1],sx[2],sx[3]);
		
		// Pomiar wilgotnosci i temperatury czujnikiem AM2302
		
		GPIOA->CRH = (GPIOA->CRH & 0x0fffffff) | 0x60000000;
		//am23.pin = (u32*) m_BITBAND_PERIPH(&GPIOA->IDR, 15); 
		//am23.prt = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR, 15); 
		//am23.typ = AM2302;
		
		flag_9 = 0;		
		LED	   = 1;		
		
		GPIOA->CRH = (GPIOA->CRH & 0xfff0f0ff)|0x00020200 ; //low power
		//GPIOA->CRH = (GPIOA->CRH & 0xfff0ffff)|0x00020000 ; 
		GPIOA->ODR = 0b0001010000000000;
		GPIOA->ODR |= (1<<10)|(1<<12); 
		
		#define OMIT0_
		#ifdef  OMIT0
		//SleepMode(1);
		PWR->CR  |= (PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);  // 
		PWR->CR  &= ~PWR_CR_PDDS;   		
		SCB->SCR |= SCB_SCR_SLEEPDEEP;	
		
		alr = ((RTC->CNTH<<16) | RTC->CNTL) + DivSec*2 ;		
		while(!bRTC_CRL_RTOFF); 			//wait to synchro
		bRTC_CRL_CNF = 1;							//Start edit mode
		RTC->ALRH =  alr >>16;
		RTC->ALRL =  alr & 0xffff;
		//RTC->CRH  |= RTC_CRH_ALRIE; //Interrupt
		//RTC->CRL &= ~RTC_CRL_ALRF;
		RTC->PRLL = 0x8000/DivSec;
		bRTC_CRL_CNF = 0;					  	//End edit mode
		while(!bRTC_CRL_RTOFF); 	  	//wait to synchro
		
		//AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
		AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
		//EXTI->PR   			= EXTI_PR_PR17;	
		EXTI->RTSR 	 		= EXTI_RTSR_TR17 | EXTI_RTSR_TR10 ;
		//EXTI->RTSR 	 		= EXTI_RTSR_TR17  ;
		//EXTI->IMR  			|= EXTI_IMR_MR17;		// Interrupt mask register EXTI_IMR
		EXTI->EMR  			= EXTI_EMR_MR17 | EXTI_EMR_MR10;		  // Event mask register EXTI_EMR
		//EXTI->EMR  			= EXTI_EMR_MR17 ;		  // Event mask register EXTI_EMR
		//NVIC_EnableIRQ(RTCAlarm_IRQn);
		#endif
		flag_9=1;	
		
		while(1){
			if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL; 
				LED ^= 1;
			
				strcpy (buf_tx," ");		
				//strcat (buf_tx,unt2str(tim/TIM_MS,str_num,7));
				strcat (buf_tx,unt2str(n++,str_num,5));
				strcat (buf_tx," ");
				message_length = strlen(buf_tx); 
				
				SX1278_LoRaEntryTx (&SX1278_1, message_length, 2000);
				
				//TIM_start();
				// ==== Start transmisji ====
				SX1278_LoRaTxPacketStart(&SX1278_1, (uint8_t *) buf_tx, message_length);  // Start transmisji
				flag_9=1;	
				SX1278_sleep(&SX1278_1);
			}
			
		}
		
		while(1){
			
			// ================ Nadawanie TX =================
			
			
			//#define OMIT1
			#ifdef  OMIT1
			if(0){
			//if(STCLK_MS*500 < (trg0 - SysTick->VAL) && flag_9==0){
			//	trg0 = SysTick->VAL; 
				//if(TIM3_MS*500 < (t3rg0 - TIM3->CNT) && flag_9==0){
				// t3rg0 = TIM3->CNT;
			 
			 LED ^= 1;
				
				strcpy (buf_tx," ");		
				//strcat (buf_tx,unt2str(tim/TIM_MS,str_num,7));
				strcat (buf_tx,unt2str(n++,str_num,5));
				strcat (buf_tx," ");
				message_length = strlen(buf_tx); 
				
				SX1278_LoRaEntryTx (&SX1278_1, message_length, 2000);
				
				//TIM_start();
				// ==== Start transmisji ====
				SX1278_LoRaTxPacketStart(&SX1278_1, (uint8_t *) buf_tx, message_length);  // Start transmisji
				flag_9=1;
				
				
			}
			#endif
			// ==== Koniec transmisji ====
			#define OMIT2
			#ifdef OMIT2
			// Obsluga z usypianiem
			
			/*if(TIM3_MS*4000 < (t3rg0 - TIM3->CNT)){	t3rg0 = TIM3->CNT; */
			if(flag_9){
				//LED=0;
				
				TIM_start();
				if(!AM23_read(&am23)){
					
				} 
				
				SX1278_standby(&SX1278_1);
				//strcpy (buf_tx,"H=");
				//strcat (buf_tx,int2str(am23.AMh,str_num,3,1));
				//strcat (buf_tx," T=");
				//strcat (buf_tx,int2str(am23.AMt,str_num,3,1));
				//message_length = strlen(buf_tx); 
				message_length = 7; 
				buf_tx[0]=am23.AMh&0xff;	buf_tx[1]=am23.AMh>>8;
				buf_tx[2]=am23.AMt&0xff;	buf_tx[3]=am23.AMt>>8;
				buf_tx[4]=cnt;		    		buf_tx[5]=cnt>>8;				buf_tx[6]=cnt>>16;
				cnt++;
				
				//UaPutK(" H="); num2uart(am23.AMh,3,1);	
				//UaPutK(" T="); num2uart(am23.AMt,3,1);	
				
				SX1278_LoRaEntryTx (&SX1278_1, message_length, 2000);
				SX1278_LoRaTxPacketStart(&SX1278_1, (uint8_t *) buf_tx, message_length);  // Start transmisji
				
				flag_9 = 0;
				
			}
			
			
			if(*SX1278_hw1.dio0 && flag_9==0 ){
				
				
				SX1278_LoRaTxPacketEnd(&SX1278_1);
				TIM_stop(&tim);				
				//UaPutK("\r\n > TX ");		uint2uart(tim/TIM_MS);			UaPutK("[ms] ");								
				//UaPutK(" Sleep...");
				SX1278_sleep(&SX1278_1);
				tim = RTC->CNTL;
				SysTick->CTRL &= ~SysTick_CTRL_ENABLE; // STOP SysTick
				Delay3_us(50);
				//LED=1;
				
				//Delay3_ms(1000);
				__WFE();
				//__SEV();
				//__WFE();				
				__NOP();				
				
				SysTick->CTRL |= SysTick_CTRL_ENABLE;  // START SysTick
				pll_start(CRYSTAL, F_CPU);				
				
				tim = RTC->CNTL-tim;
				//uxt = (RTC->CNTH<<16) | RTC->CNTL;
				
				alr = ((RTC->CNTH<<16) | RTC->CNTL) + DivSec*sx[4];				
				while(!bRTC_CRL_RTOFF); 	//wait to synchro
				bRTC_CRL_CNF = 1;					//Start edit mode
				RTC->ALRH =  alr >>16;
				RTC->ALRL =  alr & 0xffff;
				//RTC->CRL &= ~RTC_CRL_ALRF;
				bRTC_CRL_CNF = 0;					//End edit mode
				while(!bRTC_CRL_RTOFF); 	//wait to synchro
				//UaPutK(" Wake-Up "); uint2uart((tim*1000)/DivSec);	UaPutK("[ms]");
				
				flag_9 = 1;
			}
			// ================ Nadawanie TX =================
			//if(STCLK_MS*200 < (trg1 - SysTick->VAL) && flag_9==0){
			//trg1 = SysTick->VAL;
			#endif
			
		}
	}
void col1(void)			{
			dma_buf[0]=0xe0;	dma_buf[8]  =0xe0;	dma_buf[16]=0xe0;
			dma_buf[1]=0xe0;	dma_buf[9]  =0xe0;	dma_buf[17]=0xe0;
			dma_buf[2]=0xe0;	dma_buf[10] =0xe0;	dma_buf[18]=0xe0;
			dma_buf[3]=0xe0;	dma_buf[11] =0xe0;	dma_buf[19]=0xe0;
			dma_buf[4]=0xe0;	dma_buf[12] =0xe0;	dma_buf[20]=0xe0;
			dma_buf[5]=0xf8;	dma_buf[13] =0xe0;	dma_buf[21]=0xe0;
			dma_buf[6]=0xf8;	dma_buf[14] =0xe0;	dma_buf[22]=0xe0;
			dma_buf[7]=0xf8;	dma_buf[15] =0xe0;	dma_buf[23]=0xe0;
		}
void col2(void)			{
			dma_buf[0]=0xe0;	dma_buf[8]  =0xe0;	dma_buf[16]=0xe0;
			dma_buf[1]=0xe0;	dma_buf[9]  =0xe0;	dma_buf[17]=0xe0;
			dma_buf[2]=0xe0;	dma_buf[10] =0xe0;	dma_buf[18]=0xe0;
			dma_buf[3]=0xe0;	dma_buf[11] =0xe0;	dma_buf[19]=0xe0;
			dma_buf[4]=0xe0;	dma_buf[12] =0xe0;	dma_buf[20]=0xe0;
			dma_buf[5]=0xe0;	dma_buf[13] =0xe0;	dma_buf[21]=0xe0;
			dma_buf[6]=0xe0;	dma_buf[14] =0xe0;	dma_buf[22]=0xe0;
			dma_buf[7]=0xe0;	dma_buf[15] =0xe0;	dma_buf[23]=0xe0;
	}	
void col0(void)			{
			dma_buf[0]=0;	dma_buf[8]  =0x0;	dma_buf[16]=0x0;
			dma_buf[1]=0;	dma_buf[9]  =0x0;	dma_buf[17]=0x0;
			dma_buf[2]=0;	dma_buf[10] =0x0;	dma_buf[18]=0x0;
			dma_buf[3]=0;	dma_buf[11] =0x0;	dma_buf[19]=0x0;
			dma_buf[4]=0;	dma_buf[12] =0x0;	dma_buf[20]=0x0;
			dma_buf[5]=0;	dma_buf[13] =0x0;	dma_buf[21]=0x0;
			dma_buf[6]=0;	dma_buf[14] =0x0;	dma_buf[22]=0x0;
			dma_buf[7]=0;	dma_buf[15] =0x0;	dma_buf[23]=0x0;
	}
void ws2812led(void){
		
		//UaPutK("\f");
		//tr_cls(1);
		
		GPIOA->CRL = (GPIOA->CRL & 0x0000ffff) | 0xBBB30000;	// PA5~7
		
		//GPIOC->CRH = (GPIOC->CRH & 0xfff0ffff) | 0x00010000;	// enc28_CS
		
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_BR_2|	SPI_CR1_MSTR;	// 0x037c
		SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_MSTR|SPI_CR1_BR_0|SPI_CR1_BR_1;	// 0x037c
		SPI1->CR2 |= SPI_CR2_TXDMAEN;
		SPI1->CR1 |= SPI_CR1_SPE;		
		
		//---- DMA ----
		uint32_t CCR_reg = 0; 
		
		//memset ( dma_buf, 0xf8, 64 );	// 0xE0=0, 0xf8=1
		//for(i=0;i<4;i++) {
		
		col0();
		
		RCC->AHBENR   			|= RCC_AHBENR_DMA1EN;		// enable clock for DMA1
		DMA1_Channel3->CCR 	= 0;										//Disable channel
		DMA1_Channel3->CMAR = (uint32_t)dma_buf;	 	//Destination address:
		DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;	//Source PERIPHERAL address:
		//DMA2_Channel1->CNDTR = sizeof(dst_buf);		//Buffor size :
		DMA1_Channel3->CNDTR = 24;		//Buffor size :
		
		//CCR_reg |=  DMA_CCR1_PL;		 		// Priorytet high
		CCR_reg &=  ~DMA_CCR1_MSIZE;	 	// 8bit
		CCR_reg &=  ~DMA_CCR1_PSIZE;	 	// 8bit
		//CCR_reg |=  DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0;	// 16bit/16bit
		CCR_reg |=  DMA_CCR1_MINC;	 		// Increment memory address enabled
		//CCR_reg |=  DMA_CCR1_PINC;	 	// Increment peripheral address enabled
		CCR_reg |= DMA_CCR1_CIRC;	 			// Mode Circular mode disabled
		CCR_reg |= DMA_CCR1_DIR;		 		// Dir Read from memory
		//CCR_reg |=  DMA_CCR1_TCIE; 		// Transfer complete interrupt enable
		DMA1_Channel3->CCR =  CCR_reg;	
		
		DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3;
		DMA1_Channel3->CCR |= DMA_CCR1_EN;
		
		
		UaPutK("\r\n DMA start...");
		u08 fl=0;
		
		while(1){
			
			if(STCLK_MS*100 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
			}
			
			if(STCLK_MS*200 < (trg1 - SysTick->VAL)){
				trg1 = SysTick->VAL;
				
				//tr_locate(9,9);
				//UaPutK("Licznik=0x");
				//hex2uart(inc++,4);
				//DMA1_Channel3->CCR 	&= ~DMA_CCR1_EN;
				
				
				//DMA1_Channel3->CCR 	&= ~DMA_CCR1_EN;
				//col0();
				//DMA1_Channel3->CCR |=  DMA_CCR1_EN;
				Delay_us(50);
				
				//SPI1->CR1 &= ~SPI_CR1_SPE;		
				//DMA1_Channel3->CCR 	&= ~DMA_CCR1_EN;
				
				if(fl){
					col1();
					fl=0;
				} else {
					col1();
					fl=1;
					LED ^= 1;
				}
				
				Delay_us(260);
				col0();
				
				//SPI1->CR1 |= SPI_CR1_SPE;		
				//DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3;
				//DMA1_Channel3->CNDTR = 24;		//Buffor size :
				//DMA1_Channel3->CCR |= DMA_CCR1_EN;
				
				//DMA1_Channel3->CCR |=  DMA_CCR1_EN;
				
				//DMA1_Channel3->CCR |=  DMA_CCR1_EN;
				//PA4_o = 1;
			}
		}
		
		#ifdef NOTUSE
			
			//if(STCLK_MS*5000 < (trg0 - SysTick->VAL)){
			//	trg0 = SysTick->VAL;
			if(TIM3_MS*5000 < (t3rg0 - TIM3->CNT)){
				t3rg0 = TIM3->CNT;
				LED=1;
				UaPutK("\r\n > Sleep");
				Delay3_ms(1);
				__WFI();	
				
				pll_start(CRYSTAL, F_CPU);
				//Delay3_ms(1);
				UaPutK("\r\n > Wake-Up uxt=");
				uxt = RTC->CNTL+(RTC->CNTH<<16);
				uint2uart(uxt);
				//trg0 = SysTick->VAL;
				t3rg0 = TIM3->CNT;
				
			}
			
			if(TIM3_MS*1000 < (t3rg1 - TIM3->CNT)){
				t3rg1 = TIM3->CNT;
				UaPutK(".");
			}
			LED=0;
			//LED=1;
			
			
			//Delay3_ms(1000);
			
			//TIM_stop(&tim);
			//UaPutK("\r\n");
			//uint2uart(tim);
			//hex2uart(tim/TIM_US,5);
			
		#endif
		
	}
void lexmark_lcd(void){
		
		char switcH;
		u32 tim;
				
		
		
		// 0.2703693125
		// 0.2710821250
		//0x57+1,0x58+0,0x05+0
		
		//0x58+0,0x08+0,0x01+0
		//0x58+1,0x08+0,0x01+0
		//0x59+0,0x08+0,0x01+0
		//0x59+1,0x08+0,0x01+0
		//0x5A+0,0x08+0,0x01+0
		//0x5A+1,0x08+0,0x01+0
		//0x5B+0,0x08+0,0x01+0
		//0x5B+1,0x08+0,0x01+0
		
		/*
		for(u32 x=0;x<7700;x++){
			GPIOA->ODR = 0 ;
			Delay_us(1);
			LX_CLK  = 1; Delay_us(1);
			LX_CLK  = 0; Delay_us(2);
		}*/
		
		UC1608_Init();
		
		//GPIOA->ODR =  0xae;		// disable display
		//Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);
		//GPIOA->ODR =  (3<<6)|(32);		// 
		//Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);
		UC1608_Clr();
		
		UC1608_str (0, 0,"Witaj szkolo         ");
		UC1608_str (1, 0,"Zapraszam do pracy        ");
		UC1608_Refresh();
		Delay_ms(1000);
		
		UC1608_str (1, 0,"                          ");
		UC1608_Refresh();
		
		TIM_start();
		//UC1608_Refresh();
		TIM_stop(&tim);
		UaPutS(" T[us]="); sint2uart(tim/TIM_US);
		
		
		while(1){
		
			//UC1608_Cmd(0xae);
			
			//UC1608_Clr();
			//UC1608_Cmd(UC1608_ALL_PIX_ON);
			
			//while(1);
			
			//UC1608_Cmd(UC1608_DISP_OFF);
			//UaPutK("\r\n start ");
			
			
			/*
			for(u08 w=2;w<6;w++){
				UC1608_xy(40, w);
				LX_CD  = 1;
				for(n=0;n<80;n++) UC1608_Data(0xff);	
			} */
			
			//UC1608_Cmd(UC1608_ALL_PIX_OFF);
			//UC1608_Cmd(UC1608_DISP_ON);
			
			//UC1608_Clr();
			
		}
		
		
		//LX_CD  = 0; 
		//GPIOA->ODR =  0xaf;		// enable display
		//Delay_us(2);			LX_CLK  = 1; Delay_us(1);			LX_CLK  = 0; Delay_us(3);
		//LX_CD  = 1;
		
		
		p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='n';
		
		while(1){	
		
			if(p_Fifo1->rct){
				UART_getChar( (char*)&switcH);
				
				switch (switcH)	{
					case 'n':
					//strBlock = UART_getNum();
					
					UaPutK("\r\n set ");
					break;
				}
			}
			
			/*
			if(STCLK_MS*1000 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
				LX_RES = 0; Delay_us(10); LX_RES = 1;	// reset
				
			}
			*/
			
			//LX_CLK  = 1; Delay_us(1);
			//LX_CLK  = 0; Delay_us(3);
			//GPIOA->ODR = (GPIOA->ODR & 0xff00 ) | ( 0x00ff & a[n])	;
			//GPIOA->ODR = (a[n]<<1)|b[n] ;
		}
		
	}
void softUTX_init(void){
		#define p_SofTX		bitband_t m_BITBAND_PERIPH(&GPIOC->ODR, 14)	// out Soft UART
		GPIOC->CRH = (GPIOC->CRH & 0xf0ffffff) | 0x01000000;  				// PC14 = softUART
	}
void sofUTX(volatile u08 bTX)	{  /// Sotware Transmit USART Frame [8N1] with TIMER PORTB(2)
		u08  cTX = 1 ;
		//4us Tclk = 250 kbps
		p_SofTX=0;				  //START Bit
		Delay_200n(18);
		while(cTX != 0){		//Wait to send Data
		 if(bTX & cTX)  p_SofTX=1;
		 else 					p_SofTX=0;
		 cTX <<= 1;
		 Delay_200n(18);
		}
		p_SofTX=1;				//STOP Bit		
		Delay_us(10);
	}
void ToLED(u16 B,u16 A,u08 dot)	{
		sofUTX(B&0x00ff);	// segment LO
		sofUTX(B>>8);			// segment HI
		sofUTX(A&0x00ff);	// segment LO
		sofUTX(A>>8);			// segment HI
		sofUTX(dot);
	}
void cmd_LCD(uint8_t byte)	{
		uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			if (byte & 0x80)	sMOSI = 1;
			else							sMOSI = 0;
			byte <<= 1;
			sSCK = 1; /* a slave latches input data bit */
			Delay_us(2);
			sSCK = 0; /* a slave shifts out next output data bit */
			Delay_us(2);
		}
		
	}
u16  CalculateLux(u16 iGain, u16 tInt, u16 ch0, u16 ch1, int iType)	{
		//----------------------------------------------------------------------------
		// first, scale the channel values depending on the gain and integration time 16X, 402mS is nominal.
		// scale if integration time is NOT 402 msec
		
    u32 chScale;
    u32 channel1;
    u32 channel0;
     
    s32 ratio;
    u32 ratio1;
     
    u16 b, m;
     
    s32 temp;
     
    u32 lux;
     
    switch (tInt)
    {
      case 0: // 13.7 msec
       chScale = CHSCALE_TINT0;
      break;
      
      case 1: // 101 msec
       chScale = CHSCALE_TINT1;
      break;
      
      default: // assume no scaling
       chScale = (1 << CH_SCALE);
      break;
    }
		
    // scale if gain is NOT 16X
    if (!iGain) chScale = chScale << 4; // scale 1X to 16X
		
    // scale the channel values
    channel0 = (ch0 * chScale) >> CH_SCALE;
    channel1 = (ch1 * chScale) >> CH_SCALE;
//----------------------------------------------------------------------------

// find the ratio of the channel values (Channel1/Channel0)
// protect against divide by zero
    ratio1 = 0;
    if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;

// round the ratio value

    ratio = (ratio1 + 1) >> 1;

// is ratio <= eachBreak ?
    //u16 b, m;
    switch (iType)
    {
      case 0: // T, FN and CL package
        if ((ratio >= 0) && (ratio <= K1T))
           {b=B1T; m=M1T;}
            else if (ratio <= K2T)
                {b=B2T; m=M2T;}
            else if (ratio <= K3T)
                {b=B3T; m=M3T;}
            else if (ratio <= K4T)
                {b=B4T; m=M4T;}
            else if (ratio <= K5T)
                {b=B5T; m=M5T;}
            else if (ratio <= K6T)
                {b=B6T; m=M6T;}
            else if (ratio <= K7T)
            {   b=B7T; m=M7T;}
            else if (ratio > K8T)
                {b=B8T; m=M8T;}
            break;
			
       case 1:// CS package
            if ((ratio >= 0) && (ratio <= K1C))
            {b=B1C; m=M1C;}
            else if (ratio <= K2C)
            {b=B2C; m=M2C;}
            else if (ratio <= K3C)
            {b=B3C; m=M3C;}
            else if (ratio <= K4C)
            {b=B4C; m=M4C;}
            else if (ratio <= K5C)
            {b=B5C; m=M5C;}
            else if (ratio <= K6C)
            {b=B6C; m=M6C;}
            else if (ratio <= K7C)
            {b=B7C; m=M7C;}
            else if (ratio > K8C)
           {b=B8C; m=M8C;}
       break;
			
    }
		
    //u32 temp;
    temp = (u32)((channel0 * b) - (channel1 * m));
		
    // do not allow negative lux value
    if (temp < 0) temp = 0;
		
    // round lsb (2^(LUX_SCALE?1))
    temp += (1 << (LUX_SCALE-1));
		
    // strip off fractional portion
    lux = (u16)(temp >> LUX_SCALE);
		
    return(lux);
	}
void TSL2561_i2c(void){
		u08 buf[3];
		u32 tim;
		u16 ch0,ch1,lux=lux;
		
		#define TSL2561_ADR  0x72  // 0x72
		#define TIMING  0x81  
		#define DATA0LOW  0x8C 
		#define DATA0HIGH 0x8D 
		#define DATA1LOW  0x8E 
		#define DATA1HIGH 0x8F
		
		GPIOB->CRH = (GPIOB->CRH & 0xffff00ff) | 0x0000dd00;	// ...009900 PB11=SDA (Open-drain) PB10=SCL(Push-pull),
		
		RCC_APB1ENR_I2C2EN_bb = 1;         	// enable clock for I2C2 module
		I2C2_CR1_SWRST_bb = 1;            	// force software reset of I2C peripheral
		I2C2_CR1_SWRST_bb = 0;
		I2C2->TRISE = 37;               		// limit slope
		//I2C2->CCR = (1<<15) | (F_CPU/(4*400000));   // setup speed (400kHz)
		I2C2->CCR = (F_CPU/(4*400000));             // setup speed (100kHz)
		//I2C2->CCR = (1<<15) | (F_CPU/(4*100000));               			// setup speed (400kHz)
		//I2C2->CCR =  100;               			// setup speed (800kHz)
		I2C2->CR2 = I2C_CR2_FREQ_36MHz;     // config I2C2 module
		I2C2_CR1_PE_bb = 1;               	// enable peripheral
		/*
		i2c2_read(0xd0, 0x05, buf,1 );
		UaPutK("\r\n i2c_DS1207 = 0x");	
		hex2uart(buf[0],2);
		while(1);
		*/
		
		//UaPutK("\r\n  I2C ");	
		//hex2uart(GPIOB->CRH,8);
		
		buf[0]=0x80;
		buf[1]=0x03; 
		i2c2_write(TSL2561_ADR,buf,2); // init control Power UP
		buf[0]=0x81;
		buf[1]=0x02; 
		i2c2_write(TSL2561_ADR,buf,2); // init Timing 
		
		while(1){
			
			
			if(STCLK_MS*300 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
				//LED ^= 1;
				
				//i2c2_read(TSL2561_ADR, 0x8a, buf,1 ); // read id
				//hex2uart(buf[0],2);
				
				i2c2_read(TSL2561_ADR, DATA0LOW, buf,2 );
				ch0 = (buf[1]<<8)+buf[0];
				i2c2_read(TSL2561_ADR, DATA1LOW, buf,2 );
				ch1	= (buf[1]<<8)+buf[0];
				//hex2uart(ch0,4);
				//hex2uart(ch1,4);
				//hex2uart(buf[0],2);
				
				lux=CalculateLux(0,2,ch0,ch1,0);
				//UaPutK(" lux=");
				//hex2uart(lux,4);
				//sint2uart(lux);
				//UaPutK(" ");	
				sint2uart(ch0);
				UaPutK(",");	
				sint2uart(ch1);
				UaPutK("\r\n");	
				//printf("%d,%d\r\n",chan0,chan1) SerialPlot
				
				
				//UaPutK("\r\n  I2C ");	
				
				TIM_start();
				//Delay_us(10);
				TIM_stop(&tim);
				//UaPutS("tt="); sint2uart(tim);
				TIM_stop(&tim);
				//UaPutS(" tp="); sint2uart(tim/TIM_US);
				//BLED ^= 1;
			}	
		}
	}

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define APDS9960_ADR 0x72  // 0x72
typedef enum {
    /* Acceptable parameters for setMode */
    POWER = 0,
    AMBIENT_LIGHT,
    PROXIMITY,
    WAIT,
    AMBIENT_LIGHT_INT,
    PROXIMITY_INT,
    GESTURE,
    ALL
	} ir_mode_t;
void set_mode( u08 mode, bool enable )	{
		u08 buf[3];
    /* Read current ENABLE register */
		i2c1_read(APDS9960_ADR, APDS9960_ENABLE, buf,1 );
		uint8_t reg_val = buf[0];
		
    if( reg_val == ERROR )
    return;
		
    mode = MIN( mode, ALL );
		
    if( mode == ALL )
    {
      reg_val = 0x00;
      if( enable )
      reg_val = 0x7F;
    } else {
      reg_val &= ~( 1 << mode );
      if( enable )
      reg_val |= ( 1 << mode );
    }
		
    /* Write value back to ENABLE register */
    
		buf[0]=APDS9960_ENABLE;			buf[1]=reg_val; 			i2c1_write(APDS9960_ADR,buf,2); 
	}
void MAX44009_i2c(void){
		u08 buf[10];
		u32 tim,exponent,mantissa,lux;
		u16 ch0,ch1,aR,aG,aB,aC;
		tim=tim;
		
		#define MAX44009_ADR 0x94  // 0x94
		#define TSL2561_ADR  0x72  // 0x72
		
		
		// =========================== I2C2 Init ==============================
		GPIOB->CRH    = (GPIOB->CRH & 0xffff00ff) | 0x0000dd00;	// PB11=SDA (Open-drain) PB10=SCL(Push-pull),
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		//I2C2->CR1  |= I2C_CR1_SWRST;
		I2C2->TRISE   = 37;               								// limit slope
		//I2C2->CCR = (1<<15) | (F_CPU/(4*400000));   		// setup speed (400kHz)
		I2C2->CR2     = 0;
		I2C2->CCR     = (F_CPU/(4*400000));             	// setup speed (100kHz)
		//I2C2->CR2   = I2C_CR2_FREQ_36MHz;     					// config I2C2 module
		I2C2->CR1    |= I2C_CR1_PE;											  // I2C2 Pheripherial enable
		// =========================== I2C1 Init ==============================
		AFIO->MAPR   |= AFIO_MAPR_I2C1_REMAP;		// Remap TIM3 CH2->PB5
		GPIOB->CRH    = (GPIOB->CRH & 0xffffff00) | 0x000000dd;	// PB9=SDA (Alternate Open-drain) PB8=SCL(Alternate Open-drain),
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		I2C1->TRISE   = 37;               		// limit slope
		I2C1->CR2     = 0;
		I2C1->CCR     = (F_CPU/(4*400000));   // setup speed (400kHz)
		I2C1->CR1    |= I2C_CR1_PE;						// I2C2 Pheripherial enable
		
		/*
		i2c2_read(0xd0, 0x05, buf,1 );
		UaPutK("\r\n i2c_DS1207 = 0x");	
		hex2uart(buf[0],2);
		while(1);
		*/
		
		//UaPutK("\r\n  I2C ");	
		//hex2uart(GPIOB->CRH,8);
		//# Select configuration register, 0x02(02)
		//#       0x40(64)    Continuous mode, Integration time = 800 ms
		
		// TSL2561_Init
		buf[0]=0x80;
		buf[1]=0x03; 
		//i2c2_write(TSL2561_ADR,buf,2); // init control Power UP
		buf[0]=0x81;
		buf[1]=0x12; 
		//i2c2_write(TSL2561_ADR,buf,2); // init Timing 
		
		// MAX44009_Init
		buf[0]=0x02;
		buf[1]=0x40; 
		i2c2_write(MAX44009_ADR,buf,2); // init control Power UP
		
		/*
		buf[0]=APDS9960_ATIME;			buf[1]=DEFAULT_ATIME; 			i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=APDS9960_ATIME;			buf[1]=APDS9960_WTIME; 			i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=DEFAULT_PROX_PPULSE;	buf[1]=APDS9960_PPULSE; 		i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=DEFAULT_POFFSET_UR;	buf[1]=APDS9960_POFFSET_UR; i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=DEFAULT_POFFSET_DL;	buf[1]=APDS9960_POFFSET_DL; i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=DEFAULT_CONFIG1;			buf[1]=APDS9960_CONFIG1; 		i2c1_write(APDS9960_ADR,buf,2); 
    
		//ir_gesture_set_proximity_gain( DEFAULT_PGAIN );
    //ir_gesture_set_ambient_light_gain( DEFAULT_AGAIN );
		
		buf[0]=DEFAULT_PERS;				buf[1]=APDS9960_PERS; 		i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=DEFAULT_CONFIG2;			buf[1]=APDS9960_CONFIG2; 	i2c1_write(APDS9960_ADR,buf,2); 
		buf[0]=DEFAULT_CONFIG3;			buf[1]=APDS9960_CONFIG3; 	i2c1_write(APDS9960_ADR,buf,2); 
		*/
		
		set_mode( AMBIENT_LIGHT, true );
		//set_mode( POWER, true );
		
		i2c1_read(APDS9960_ADR, APDS9960_ID, buf,1 );
		UaPutK("\r\n APDS9960_ID=0x");	
		hex2uart(buf[0],2);
		
		Delay_ms(200);
		while(1){
			
			
			if(STCLK_MS*100 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
				//LED ^= 1;
				UaPutK("\r\n");	
				//i2c2_read(TSL2561_ADR, 0x8a, buf,1 ); // read id
				//hex2uart(buf[0],2);
				
				i2c2_read(MAX44009_ADR, 0x03, buf,2 );
				
				exponent = (buf[0] & 0xF0) >> 4;
				mantissa = ((buf[0] & 0x0F) << 4) | (buf[1] & 0x0F);
				//lux = (1<<exponent) * mantissa * 0.045; // printf("Ambient Light luminance : %.2f lux \n", luminance);
				lux = ((1<<exponent) * mantissa * 45 ) /1000 ;
				//lux = pow(2,exponent) * mantissa * 45;
				
				sint2uart(lux);
				UaPutK(",");	
				//num2uart(lux,7,0);
				i2c2_read(TSL2561_ADR, DATA0LOW, buf,2 );
				ch0 = (buf[1]<<8)+buf[0];
				i2c2_read(TSL2561_ADR, DATA1LOW, buf,2 );
				ch1	= (buf[1]<<8)+buf[0];
				
				lux=CalculateLux(1,2,ch0,ch1,0);
				sint2uart(lux);
				
				UaPutC(',');
				
				
				memset ( buf, 1, 10);
				i2c1_read(APDS9960_ADR, 0x94, buf,8 );
				aC = (buf[1]<<8)+buf[0];
				aR = (buf[3]<<8)+buf[2];
				aG = (buf[5]<<8)+buf[4];
				aB = (buf[7]<<8)+buf[6];
				
				sint2uart(aC); UaPutC(','); //hex2uart((buf[1]<<8)+buf[0],4);
				sint2uart(aR); UaPutC(','); //hex2uart((buf[3]<<8)+buf[2],4);
				sint2uart(aG); UaPutC(','); //hex2uart((buf[5]<<8)+buf[4],4);
				sint2uart(aB); UaPutC(',');//hex2uart((buf[7]<<8)+buf[6],4);
				
				
				//i2c1_read(APDS9960_ADR, 0x81, buf,1 );
				//hex2uart(buf[0],2);
				//hex2uart(ch0,4);
				//hex2uart(ch1,4);
				//hex2uart(buf[0],2);
				//printf("%d,%d\r\n",chan0,chan1) SerialPlot
				
			}	
		}
	}
void BMP280_i2c(void){
		u08 buf[24];
		u16 dig_T1,dig_P1;
		s16 dig_T2,dig_T3,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
		u32 adc_T,var1,var2,t_fine,T,p,adc_P,tim;
		
		#define BMP280_ADR 0xEC
		GPIOB->CRH = (GPIOB->CRH & 0xffff00ff) | 0x0000bf00;	// ...009d00 PB10=SCL(Push-pull),PB11=SDA (Open-drain)
		
		RCC_APB1ENR_I2C2EN_bb = 1;         	// enable clock for I2C2 module
		I2C2_CR1_SWRST_bb = 1;            	// force software reset of I2C peripheral
		I2C2_CR1_SWRST_bb = 0;
		I2C2->TRISE = 37;               		// limit slope
		//I2C2->CCR = (1<<15) | (F_CPU/(4*400000));   // setup speed (400kHz)
		//I2C2->CCR = (F_CPU/(4*100000));             // setup speed (100kHz)
		I2C2->CCR = (1<<15) | 25;               			// setup speed (800kHz)
		I2C2->CR2 = I2C_CR2_FREQ_36MHz;     // config I2C2 module
		I2C2_CR1_PE_bb = 1;               	// enable peripheral
		
		i2c2_read(BMP280_ADR,  0xD0, buf,1 );
		UaPutK("\r\n i2c_BMP280 = 0x");	
		hex2uart(buf[0],2);
		
		// set oversampling and mode
		buf[0]=0xf4;
		buf[1]=0xb7; // 0xb3 0x27
		i2c2_write(BMP280_ADR,buf,2); // set mode
		// filter
		buf[0]=0xf5;
		buf[1]=0x10; // 0x27
		i2c2_write(BMP280_ADR,buf,2); // set mode
		
		i2c2_read(BMP280_ADR,  0x88, buf,6+16 );
		dig_T1 = (u16)(((u16)(buf[1]<<8)) | buf[0]);
		dig_T2 = (s16)((buf[3]<<8) | buf[2]);
		dig_T3 = (s16)((buf[5]<<8) | buf[4]);
		
		dig_P1 = (u16)(((u16)(buf[7]<<8)) | buf[6]);
		dig_P2 = (s16)((buf[9]<<8 ) | buf[8]);
		dig_P3 = (s16)((buf[11]<<8) | buf[10]);
		dig_P4 = (s16)((buf[13]<<8) | buf[12]);
		dig_P5 = (s16)((buf[15]<<8) | buf[14]);
		dig_P6 = (s16)((buf[17]<<8) | buf[16]);
		dig_P7 = (s16)((buf[19]<<8) | buf[18]);
		dig_P8 = (s16)((buf[21]<<8) | buf[20]);
		dig_P9 = (s16)((buf[23]<<8) | buf[22]);
		
		i2c2_read(BMP280_ADR,  0xf4, buf,1 ); hex2uart(buf[0],2);
		i2c2_read(BMP280_ADR,  0xf5, buf,1 ); hex2uart(buf[0],2);
		
		
		while(1){
			
			/*
			if(bit==0){
				pll_start(CRYSTAL, F_CPU);
				bit=1;
			} */
			
			if(STCLK_MS*500 < (trg0 - SysTick->VAL)){
				trg0 = SysTick->VAL;
				
				UaPutK("\r\n BMP280 ");	
				
				//buf[0]=0xf4;
				//buf[1]=0xb3; // 0xb3 0x27
				//i2c2_write(BMP280_ADR,buf,2); // set mode
				
				// Obliczenie temperatury ========================================================//
				i2c2_read(BMP280_ADR,  0xFA, buf,3 );
				
				TIM_start();
				
				adc_T = ( (u32)((u16)((buf[0]) << 8) | buf[1] ) << 4) | ( (buf[2]) >> 4);
				var1 = ((((adc_T>>3)  - ((s32)dig_T1<<1))) * ((s32)dig_T2)) >> 11;
        var2 = (((((adc_T>>4) - ((s32)dig_T1)) * ((adc_T>>4) - ((s32)dig_T1))) >> 12)
				* ((s32)dig_T3))>>14;
        
        t_fine = var1 + var2;
				//T     = (t_fine * 5 + 128) >> 8; // resolution 0.01 C
				//T     = (t_fine*25)/128 +5; 		 // resolution 0.001 C
				T     = (t_fine * 50 + 1280) >> 8; // resolution 0.001 C
				
				//Delay_us(10);
				TIM_stop(&tim);
				UaPutS("tt="); sint2uart(tim);
				
				//sint2uart(adc);
				UaPutK(" T=");	
				num2uart(T,5,3);
				
				// Obliczenie cinienia ==========================================================//
				
				i2c2_read(BMP280_ADR,  0xF7, buf,3 );				
				adc_P   = ((u32)((u16)(buf[0] << 8) | buf[1])<<4) | (buf[2]>>4);
				TIM_start();
				
				var1 = (((s32)t_fine)>>1) - 64000UL; 
				var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((s32)dig_P6); 
				var2 = var2 + ((var1*((s32)dig_P5))<<1); 
				var2 = (var2>>2)+(((s32)dig_P4)<<16); 
				var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((s32)dig_P2) * var1)>>1))>>18; 
				var1 =((((32768+var1))*((s32)dig_P1))>>15); 
				if (var1 == 0) 
				{ 
					//return 0; // avoid exception caused by division by zero 
					break;
				} else { 
					p = (((u32)(((s32)1048576)-adc_P)-(var2>>12)))*3125; 
					if (p < 0x80000000) { p = (p << 1) / ((u32)var1); 	}  
					else  							{ p = (p 			 /  (u32)var1)*2; } 
					var1 = (((s32)dig_P9) * ((s32)(((p>>3) * (p>>3))>>13)))>>12; 
					var2 = (((s32)(p>>2)) * ((s32)dig_P8))>>13; 
					p = (u32)((s32)p + ((var1 + var2 + dig_P7) >> 4)); 
					p = (p*5)/8;
					p = (p*5)/8;
					UaPutK(" p= ");	
					num2uart(p,9,3);
				}
				
				TIM_stop(&tim);
				UaPutS(" tp="); sint2uart(tim/TIM_US);
				
				//LED ^= 1;
			}
			
			if( STCLK_MS*1000 < (trg1 - SysTick->VAL)){
				trg1 = SysTick->VAL;
			}
			
		}
	}
void BMP280_spi(void){
	// MASTER
		/*
		BMP280	I2C		SPI
		SCL  		SCL  	SCK			10k->Vcc
		SDA  		SDA  	MOSI		10k->Vcc
		CSB  		NC		CS			10k->Vcc
		SDO  		ADD  	MISO   	10k->GND
		*/
		GPIOA->CRL = (GPIOA->CRL&0x0000ffff)|0xB4B30000;	// PA4~7
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|1<<3|SPI_CR1_MSTR;	
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|7<<3|SPI_CR1_MSTR|SPI_CR1_DFF;		
		SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|7<<3|SPI_CR1_MSTR|SPI_CR1_CPOL|SPI_CR1_CPHA;		
		//SPI1->CR1 = 														 SPI_CR1_MSTR;		
		//SPI1->CR2 |= SPI_CR2_SSOE;		
		//SPI1->CR2  = 0;			
		SPI1->CR1 |= SPI_CR1_SPE;		
		
		trg0 = SysTick->VAL;		trg1 = SysTick->VAL;		trg2 = SysTick->VAL;
		
		u08 v0,v1;
		u08 value[3],ctrl_meas;
		u32 adc,var1,var2;
		
		u16 dig_T1;
		s16 dig_T2,dig_T3;
		u32 tFine,T;
		
		RX1276_CS1 = 0;
		
		spi1_rw(0xE0 & 0x7f);   // reset write
		spi1_rw(0xB6); 			    // reset value
		
		Delay_ms(10);
		
		spi1_rw(0xf4 & 0x7f); 	// Select Control register 
		spi1_rw(0x27); 					// Write  Control register 
		
		
		//spi1_rw(0); // Write  Control register 
		RX1276_CS1 = 1;
		
		
		while(1){
			
			
			if( STCLK_MS*2000 < (trg2 - SysTick->VAL)){
				trg2 = SysTick->VAL;
				//LED ^= 1;
				
				RX1276_CS1 = 0;
				
				spi1_rw(0xF4 & 0x7f); 	// Select Control register 
				spi1_rw(0x27); 					// Write  Control register 
				
				spi1_rw(0xF4);
				ctrl_meas = spi1_rw(0xff);
				
				spi1_rw(0x88);
				v0 = spi1_rw(0xff);
				v1 = spi1_rw(0xff);
				dig_T1 = (u16)(((u16)(v1<<8)) | v0);
				
				spi1_rw(0x8A);
				v0 = spi1_rw(0xff);
				v1 = spi1_rw(0xff);
				dig_T2 = (s16)((v1<<8) | v0);
				
				spi1_rw(0x8C);
				v0 = spi1_rw(0xff);
				v1 = spi1_rw(0xff);
				dig_T3 = (s16)((v1<<8) | v0);
				
				spi1_rw(0xFA);
				value[0] = spi1_rw(0xff);
				value[1] = spi1_rw(0xff);
				value[2] = spi1_rw(0xff);
				
				//spi_dat1 = spi1_rw(0xff);
				//spi_dat0 = spi1_rw(0x0);
				//==Init==//
				
				RX1276_CS1 = 1;
				
				UaPutK("\r\n BMP280T1 = 0x");	
				hex2uart(ctrl_meas,2);
				
				hex2uart(dig_T1,4);
				hex2uart(dig_T2,4);
				hex2uart(dig_T3,4);
				
				//hex2uart(spi_dat0,2);				
				//hex2uart(spi_dat1,2);
				//hex2uart(spi_dat2,2);
				
				
				adc = ( (u32)((u16)((value[0]) << 8) | value[1] ) << 4) | ( (value[2]) >> 4);
				//adc = (( (value[0] << 8) | value[1]) << 4) | ((value[2]) >> 4);
				//adc  =  (u32)(value[0]<<12) | (u16)(value[1]<<4) | (0x0f&(value[2]>>4)) ;
				var1 = ((((adc>>3) - ((s32)dig_T1<<1))) * ((s32)dig_T2)) >> 11;
        var2 = (((((adc>>4) - ((s32)dig_T1)) * ((adc>>4) - ((s32)dig_T1))) >> 12)
				* ((s32)dig_T3))>>14;
        //var2 = ((adc>>4) - ((int32_t)dig_T1));
        //var2 = (((var2 * var2) >> 12) * ((int32_t)dig_T3))>>14;
        tFine = var1 + var2;
				T     = (tFine * 5 + 128) >> 8;
				
				UaPutK(" T=");	
				sint2uart(adc);
				UaPutK(" ");	
				num2uart(T,5,3);
				//UaPutK(" Sleep ");	
				//__WFI();
				//__WFE();
			}
		}
		
	}
void fun_tool(u08 znak,T_SD1 * sda){
	u32 strBlock,tim;
	u08 buffer[512];
	u16 crc16;
	u08 *p_buf;
	static T_RTC datetime;
	*sda = *sda;


	switch(znak){
	//================================================================================
	case 'r':
		UaPutK("\r\n FLASH memory Read = ");
		UaPutK("\r\n FLASH memory Read = ");
		//UaPutK("\r\n sectorNumber =0x");
		strBlock = UART_getNum();
		
		//strBlock=UART_getHex();
		strBlock = strBlock > 254 ? 254 : strBlock;
		UaPutK("\r\n Flash memory Page ="); sint2uart(strBlock);
		strBlock = 0x800*strBlock + 0x08000000;
		//buffer = FLASH_PAGGE_122
		//p_buf = (u08 *) FLASH_PAGGE_120;
		//p_buf = (u08 *) W_FONT_1408;
		//p_buf = (u08 *) buffer;
		p_buf = (u08 *) strBlock;
		if(1){
			//if(disk_read (0,buffer,strBlock,1) == RES_OK ){
			UaPutS("\r\n ");
			
			hex2uart(strBlock, 8);
			for(u08 k=0;k<16;k++){
				hex2uart(k, 2);
			}
			
			UaPutS("\r\n\n ");
			for(u32 i=0;i<2048;i+=32){
				//hex2uart(strBlock*512 + i, 8);	// sd card
				hex2uart(strBlock + i, 8);	// memory
				for(u08 k=0;k<32;k++){
					hex2uart(p_buf[k+i], 2);
				}
				UaPutS("  ");
				for(u08 k=0;k<32;k++){
					if ( p_buf[k+i]<0x20 || p_buf[k+i]>127 ) UaPutC('.');
					else UaPutC(p_buf[k+i]);
				}
				UaPutS("\r\n ");
			}
			
			//*((u32*)p_buf+127) = 0;		// CRC obliczamy bez CRC
			//crc16_spi2((u16*)p_buf,sizeof(ASCII_1408_)/2, &crc16 );
			crc16_spi2((u16*)p_buf,2048, &crc16 );
			UaPutK(" crc = ");	hex2uart(crc16,4);
			
		} else UaPutK("\r\n Read Error !!!");
		break;
		//================================================================================
	case 'w':

		UaPutK("\r\n Flash memory Page = ");
		strBlock = UART_getNum();
		strBlock = strBlock > 254 ? 254 : strBlock;
		strBlock = (u32)(0x800*strBlock + 0x08000000);

		TIM_start();
		EraseFlashPage(strBlock);
		TIM_stop(&tim);
		UaPutS("\r\n Kasowanie pamieci FLASH = "); sint2uart(tim/TIM_US);
		//ProgramFlashFromAddr( (u32)strBlock,(u16*)ASCII_1408,776);
		//TIM_stop(&tim);
		//UaPutK("\r\n Czas zapisu do FLASH = "); sint2uart(tim/TIM_US);

		//sint2uart(sizeof(ASCII_1408_));


		break;
		//================================================================================
	case 'u':
		UaPutK("\r\n Godzina = ");
		datetime.hor = UART_getNum();
		UaPutK("\r\n Minut   = ");
		datetime.min = UART_getNum();
		UaPutK("\r\n Sekunda = ");
		datetime.sec = UART_getNum();

		datetime.yer=2016;	// 1970..2106
		datetime.mon=11;		// 1..12	miesi¹c
		datetime.mdy=18;		// 1..31	dzien miesiaca
		datetime.hor=10;		// 0..23	godzina 24h
		datetime.min=50;		// 0..59	minuta
		datetime.sec=30;		// 0..59	sekunda
		rtc_SetTime(&datetime );
		//UaPutS("\r\n uxt ="); sint2uart(utc);
		//utc = ((RTC->CNTL + (RTC->CNTH<<16)) - Off_TIME)%180;
		//UaPutS("\r\n uxt ="); sint2uart(utc);
		//rtc_GetTime(1, &rtc);

		break;
		//================================================================================
	case 'm' :
		UaPutS("\r\n Read Start !");
		for(u32 n=0;n<125;n++){
			TIM_start();
			//disk_read (0, p_sdbuf, (u32)n, 1);
			TIM_stop(&tim);
			tim = tim/TIM_US;
			//UaPutK("\r\n sd[ms] ");
			if(n%1000==0) UaPutK(".");
			if(tim>1000 || tim<700){ UaPutK("\r\n"); int2uart(n,7,0); int2uart(tim,6,3); }
		}
		UaPutS("\r\n Read Done !");
		break;
		//================================================================================
	case 'c' :
		TFT_Clear(BLACK);
		//ramka();
		UaPutS("\r\n Clear !");
		break;
		//================================================================================
	case 'o':
		UaPutK("Read sector");
		UaPutK("\r\n sectorNumber = ");
		//strBlock = UART_getNum((char *)uaBuf);
		u08 sd_status;
		strBlock=UART_getNum();
		TIM_start();
		//i=SD_readSingleBlock(buffer,strBlock);
		sd_status=mci_read_sect (strBlock,buffer,1);

		TIM_stop(&tim);
		if(sd_status){
			UaPutS("\r\n ");

			hex2uart(strBlock, 8);
			for(u08 k=0;k<16;k++){
				hex2uart(k, 2);
			}
			
			UaPutS("\r\n\n ");
			for(u32 i=0;i<512;i+=16){
				hex2uart(strBlock*512 + i, 8);
				for(u08 k=0;k<16;k++){
					hex2uart(buffer[k+i], 2);
				}
				UaPutS("  ");
				for(u08 k=0;k<16;k++){
					if ( buffer[k+i]<0x20 || buffer[k+i]>127 ) UaPutC('.');
					else UaPutC(buffer[k+i]);
				}
				UaPutS("\r\n ");
			}
			UaPutK(" t_rsdio = ");	uint2uart(tim/TIM_US);
			//TIM_start();
			crc16_spi2( (u16*) buffer,256, &crc16 );
			//TIM_stop(&tim);
			UaPutK(" crc = ");	hex2uart(crc16,4);
			//UaPutK(" t_crc = ");	uint2uart(tim/TIM_US);
		
		} else UaPutK("\r\n Read Error !!!");
		break;
		//================================================================================

		//================================================================================
	}
	UaPutS("\r\n ");
}
//=========================================================================================================
static inline __attribute__((always_inline)) void Start_ADC1_J(u16 ndtr){
		ndtr=ndtr;
		//DMA1_Channel1->CNDTR = ndtr;					// Buffor size 
		//TIM3->CR1 |= TIM_CR1_CEN;							// Timer3 start
		TIM4->CR1 |= TIM_CR1_CEN;							// Timer4 start
		///DMA1_Ch1_EN_bb = 1;										// DMA Start
		
		//while(dma_buf[0+10*nADC] == ADC1->DR); 
		///while(!(ADC1->SR & ADC_SR_EOC) );
		
		while(!(DMA1->ISR & DMA_ISR_TCIF1));	// Wait for ending transfer DMA
		DMA1->IFCR = DMA_IFCR_CTCIF1;					// Clear flag
		///DMA1_Ch1_EN_bb = 0;										//DMA Stop
		
		//ADC1_CR2_ADON  = 0;		
		//TIM3->CR1 &= ~TIM_CR1_CEN;						// Timer3 stop
		TIM4->CR1 &= ~TIM_CR1_CEN;						// Timer4 stop
	}
static inline __attribute__((always_inline)) void Start_ADC1_DMA1(u16 cndtr){
		DMA1_Channel1->CNDTR = cndtr;	// Buffor size 
		
		ADC1_CR2_ADON  = 1;						// start conversion ADC1
		ADC1_CR2_ADON  = 1;						// start conversion ADC1
		DMA1_Ch1_EN_bb = 1;						// DMA1_CH1 START
		while ( !(DMA1->ISR & DMA_ISR_TCIF1) ){		}	
		DMA1_Ch1_EN_bb = 0;					// DMA1_CH1 STOP
		ADC1_CR2_ADON  = 0;					// stop conversion
		DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1;
	}
static inline __attribute__((always_inline)) void Start_ADC1_DMA1_irq(u16 cndtr){
		
		DMA1_Channel1->CNDTR = cndtr;	// Buffor size 
		DMA1_Ch1_EN_bb = 1;						// DMA1_CH1 START
		ADC1_CR2_ADON  = 1;						// start conversion ADC1
		ADC1_CR2_ADON  = 1;						// start conversion ADC1		
		//ADC1_SWSTART = 1;
		
		//ADC1->CR2 |= ADC_CR2_ADON;					// start conversion
		//ADC1->CR2 |= ADC_CR2_ADON;					// start conversion
		
		LED1 = 1;
	}
static inline __attribute__((always_inline)) void rkp(void){
		while(p_Fifo1->rct==0);	
		p_Fifo1->rct--; 
	}
void Intro2(void){	
		#define nMES	1 // Ilosc pomiarow
		
		UaPutS("\r\n Start programu... \r\n"); 
		
		static u08 yb[nMES],yn[nMES],mn[4]={1,1},dl[4]={1,1};
		static u16 val[10],rz[4]={0,0};
		static char tbuf[20]={"0000-00-00 00:00:00"};
		uint32_t tim=0;
		u08 i,p=0;
		u16 x,c_dma1,r_dma_buf,idx,c_num,n1=10,cnt_dma=0;
		char chr,str[3];
		n1=n1;
		#define WID 0	// szerokosc lini wykresu
		
		//=======Punkty graniczne obszaru roboczego wykresu=========
		#undef wXmin 
		#define wXmin		31 				//31 			dolna granica wykresu X
		#undef wXmax
		#define wXmax		(319-1)	  //319-1 	górna granica wykresu X
		#undef wYmin
		#define wYmin		1 				//14 			dolna granica wykresu Y
		#undef wYmax
		#define wYmax		(239-12) 	//240-13 	górna granica wykresu Y
		#undef wXlen
		#define wXlen		((wXmax)-(wXmin)+(1)) //szerokoæ wykresu 
		
		
		static volatile u16	dma_buf[nMES*(wXlen+1)];	
		
		UaPutS("\r\n Debug0..."); 
		RtcInit();
		rtc_GetTime(1,&Rtc);
		//UaPutS("\r\n Debug1..."); 
		ADC1_DMA_Tim4_Init();
		f7_dma1 = 0;
		//UaPutS("\r\n Debug2..."); 
		
		//DACDmaInit();
		//Mem2MemDMAInit(ad1_buf, dma_buf);
		
		
		/*
		TIM_start();
		for(u16 n=0;n<100;n++){
			//TFT_dy( n, dma_buf[1+n*4], dma_buf[1+(n+1)*4],WHITE,0);
			TFT_dy( 3*n, 120, 120+n	,WHITE,0);
			TFT_dy( 3*n, 119, 119-n	,WHITE,0);
		}
		TIM_stop(&tim);
		UaPutS("\r\n TIM= ");
		num2uart(tim/TIM_US,8, 0); 
		
		while(1);
		*/
		
		TFT_Fill(wXmin-1,wYmin-1,wXmax+1,wYmin-1,DGRAY);	// pozioma dolna
		TFT_Fill(wXmin-1,wYmax+1,wXmax+1,wYmax+1,DGRAY);	// pozioma górna
		TFT_Fill(wXmin-1,wYmin-1,wXmin-1,wYmax+1,DGRAY);	// pionowa lewa
		TFT_Fill(wXmax+1,wYmin-1,wXmax+1,wYmax+1,DGRAY);	// pionowa prawa
		
		u08 f=nMES;
		while(f--){
			yb[f] =wYmin;	
			yn[f] =wYmin;
		}
		
		/*
		u08 dl=240;
		for(n=0;n<wXlen;n++){
			
			if(dl==10) dl = n+50;
			else 			 dl = 10;
			if(n>5 && n<10) dl=100;
			
			dma_buf[1+n*nMES] = dl; 			
		} */
		
		//UaPutS("\r\n wXlen="); //sint2uart(wXlen);
		//for(u08 n=0;n<20;n++){ sint2uart(dma_buf[n]);		}
		UaPutS("\r\n wXlen ");	sint2uart(wXlen); 
		
		DMA1_Channel1->CMAR = (uint32_t)dma_buf;	 	//Destination address:
		DMA1_Channel1->CNDTR = wXlen*nMES;					//Buffor size 
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);				  //DMA1 Channel1 IRQ
		//DMA1_Ch1_EN_bb = 1;												//DMA1_CH1 START
		///DMA1_Channel1->CCR |= DMA_CCR1_EN;				  //DMA1_CH1 START
		///TIM4->CR1 |= TIM_CR1_CEN;										// Timer4 start
		
		//Delay_ms(60);					// czas na wypelnienie bufora
		TIM4->PSC   = BKP->DR41; //1000-1
		TIM4->ARR   = BKP->DR42; //1200-1
		
		//c=DMA1_Channel1->CNDTR/nMES;
		
		/*
		UaPutS("\r\n c1=");	sint2uart(DMA1_Channel1->CNDTR/nMES);
		UaPutS("\r\n wXlen="); //sint2uart(wXlen);
		for(u08 n=0;n<200;n++){	sint2uart(dma_buf[n]);		}
		UaPutS("\r\n c2=");	sint2uart(DMA1_Channel1->CNDTR/nMES);
		*/
		//rkp();
		
		for(u16 i=0;i<wXlen;i++){
			//dma_buf[wXlen-i]=i*10;
			
			dma_buf[i]=0x10;
		}
		
		while(1){
			
			if( STCLK_MS*BKP->DR41 < (trg0 - SysTick->VAL) ){
				trg0 = SysTick->VAL;
				cnt_dma=(cnt_dma+1)%wXlen;
				dma_buf[cnt_dma]=0x1;
				
			}
			
			//if (c_dma1 != DMA1_Channel1->CNDTR/nMES){
			if (c_dma1 != wXlen-cnt_dma){
				//c_dma1 = DMA1_Channel1->CNDTR/nMES;
				c_dma1 = wXlen-cnt_dma;
				
				cnt_dma=(cnt_dma+1)%wXlen;
				dma_buf[cnt_dma]=0x1;
				TIM_stop(&tim);
				n1=tim/(TIM_MS*1000);
				TIM_start();
				
				/*
				UaPutS("\r\n dma="); 
				if((wXlen-c_dma1) == 0){
					sint2uart(wXlen-1); 
					sint2uart(dma_buf[wXlen-1]);
				}
				else{
					
					num2uart(c_dma1,4,0); 
					num2uart(wXlen-c_dma1-1,4,0); 
					for(u08 i=0;i<8;i++)
					num2uart(dma_buf[i],5,0);
					
				} */
				
				/*
				//if( STCLK_MS*1000 < (trg0 - SysTick->VAL) ){
				//	trg0 = SysTick->VAL;
				*/
				
				//Start_ADC1_J(nMES*nMES);
				///dma_buf[0+x*nMES]= ADC1->DR;
				
				//ADC1Init_J();
				//==============================================
				//Start_ADC1_DMA1_irq((wXmax-wXmin)*nMES);
				
				TFT_Fill(wXmin+(wXmax-wXmin)/2,wYmin-1,wXmin+(wXmax-wXmin)/2,wYmax+1,DGRAY);	// pionowa srodkowa
				TFT_Fill(wXmin-1,(wYmax-wYmin)/2,wXmax+1,(wYmax-wYmin)/2,DGRAY);						// pozioma srodkowa
				
				TFT_SetWindow(wXmin,wYmin,wXmax,wYmax);
				//UaPutS("\r\n");
				for(x=0;x<wXlen;x++){
					
					
					if( x>0 && x<15 ) {
						
						//hex2uart(yn[0],2); 
						//num2uart(yb[0],1,0); UaPutC(',');	num2uart(yn[0],1,0); UaPutC(' ');
						//hex2uart(dma_buf[x],2); 
						//hex2uart(idx,3); 
						
					}
					
					if(x>wXlen-15 && x<wXlen ) {
						
						//hex2uart(yn[0],2);  
						//hex2uart(dma_buf[x],2); 
						//hex2uart(idx,3); 
					}
					
					
					// Czyszczenie
					
					if( x>0 && x< wXlen-0 ){
						//UaPutS("\r\n ="); sint2uart(n); sint2uart(i); sint2uart(yp[i]); sint2uart(yb[i]); sint2uart(yn[i]); 
						for(i=0; i<nMES;i++){
							TFT_dy(wXmin+x-1, yb[i], yn[i],BLACK,WID);
						}
						
						
					}	
					// Rysowanie
					if(x>1){
						for(i=0; i<nMES;i++){
							TFT_dy(wXmin+x-2, yb[i], yn[i],ColCon[6+i],WID);
						}
					}
					
					
					//if(x == wXlen-1) {	idx=(wXlen+0-c_dma1)*nMES; }	// ostatni punkt
					//if(x<(wXlen-c_dma1))	idx=(wXlen-c_dma1-1-x)*nMES; 
					//else 									idx=(wXlen-					x)*nMES; 
					
					if(x<cnt_dma)	idx=(0    +cnt_dma-x-1)*nMES; 
					else 					idx=(wXlen+cnt_dma-x-1)*nMES; 
					
					
					for(i=0;i<nMES;i++){
						r_dma_buf =(dma_buf[i+idx]-rz[i])*mn[i]/dl[i]; 
						yb[i]=yn[i];
						
						if(r_dma_buf < (wYmax-wYmin) ){
						yn[i]=wYmin + (r_dma_buf);	}
						else {	yn[i]=wYmax-wYmin;		}
						
					}
					
					if(x==wXlen-2){
						for(i=0; i<nMES;i++){
							val[i]=dma_buf[i+idx];
						}
					}
				}
			}
			
			
			if( STCLK_MS*500 < (trg1 - SysTick->VAL) ){
				trg1 = SysTick->VAL;
				//u16 o=4*60;
				//UaPutS("\r\n ADC1= ");
				//num2uart(tim/TIM_US,8, 3); 
				//TFT_txt(int2str(x,str,1,0),0,0, YELLOW); // rodek skali 	x
				//TFT_txt(int2str(x,str,2,0),0,10, YELLOW); // rodek skali 	x
				//TFT_txt(int2str(vAD[0],str,3,0),0,229-12*0,ColCon[6]);
				//TFT_txt(int2str(vAD[1],str,3,0),0,229-12*1,ColCon[6]);
				
				
				TFT_txt(int2str(val[0],str,4,0),30,229-12*0,WHITE);
				//TFT_txt(int2str(val[1],str,4,0),70,229-12*0,YELLOW);
				TFT_txt(int2str(tim/TIM_MS,str,5,0),110,229-12*0,ColCon[6]);
			}
			
			
			
			if(p_Fifo1->rct){	// Wywolanie interfejsu szeregowego gdy odebrano znak
				UART_getChar(&chr);
				if(chr=='q'){
					UaPutS("\r\n TIM4->PSC/ARR="); 	sint2uart(TIM4->PSC);	UaPutC('/'); c_num=UART_getNum();
					if(c_num){ 
						BKP->DR41=c_num; TIM4->PSC=BKP->DR41;
						BKP->DR42=c_num; TIM4->ARR=BKP->DR42;
					}
					
					
					//UaPutS("\r\n DMA1->ISR="); 	sint2uart(DMA1->ISR); 
					//UaPutS("\r\n DMA1->CCR="); 	sint2uart(DMA1_Channel1->CCR); 
					UaPutS("\r\n");
					chr='b';
				}
				
				if(chr=='b'){	
					
					TFT_Clear(BLACK);
					TFT_Fill(wXmin-1,wYmin-1,wXmin-1,wYmax+1,DGRAY);	// pionowa lewa
				}
			}
			/*
			if(f7_dma1){
				
				//sint2uart(DMA1_Channel1->CNDTR); 
				LED2   ^= 1;
				f7_dma1 = 0;
				f8_dma1  = 1;
				trg2   = SysTick->VAL;
			}
			
			if( f8_dma1 && STCLK_MS*10 < (trg2 - SysTick->VAL) ){
				LED2  ^= 1;
				f8_dma1 = 0;
			}
			*/
			if(f3_rtc_1sec){	// Aktualizacja zegara
				f3_rtc_1sec = 0;
				
				rtc_GetTime(0, &Rtc);
				CopyClock(tbuf,&Rtc); TFT_txt(tbuf,180, 229,YELLOW);
				//if((Rtc.sec%2)==0){	f11_rtc_2sec = 1;	}
				//if((Rtc.sec%5)==0){	f6_rtc_5sec  = 1;	}
				p=(p+1)%(wYmax-wYmin);
				
			}
		}
	}
//=========================================================================================================
u32  qtouch_mesaure(void)	{
		u32 i=0;
		PE2_o = 0;
		while(PE4_i == 1);
		Delay_us(2);
		PE2_o = 1;
		do{i++;} while (PE4_i == 0);
		PE2_o = 0;
		return i;
	}
void qtouch(void)					{
		u32 qt=0;
		GPIOE->CRL = (GPIOE->CRL & 0xfff0f0ff) | 0x00040300;	
		while(1){
			if(STCLK_MS*500 < (trg0 - SysTick->VAL) ){
				trg0 = SysTick->VAL;
				qt = qtouch_mesaure();
				UaPutS("\r\n QT =");		sint2uart(qt);
			}
		}
	}

#ifdef NOTUSE
void http_client(void){
		
		char tbuf[20]={"0000-00-00 00:00:00"};
		f5_eth = 0;
		u08 switcH,reg_adr=0;
		reg_adr=reg_adr;
		
		RegisterEthEventCallback(zapytanie_http);	
		
		rtc_GetTime(1,&Rtc);
		CopyClock(tbuf,&Rtc);
		TFT_txt(tbuf,180, 229,YELLOW);
		
		init_simple_client();
		flag_6 = 1;
		
		//p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='s';
		//TIM_start(); 
		//spi1_rw(0xff);
		//Delay_us(100);
		//TIM_stop(&tim); 
		//UaPutK("\r\n Start loop...");
		//sint2uart(tim/TIM_US);
		
		while(1){
			if(flag_6) 	eth_event();
			
			
			if(f4_enc28){
				f4_enc28 = 0;
				//plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
				//plen=packetloop_arp_icmp_tcp(buf,enc28j60PacketReceive(BUFFER_SIZE, buf));
				//simple_server();
				UaPutC('.');
				//UaPutK("\r\n plen=");
				//sint2uart(plen);
			}
			
			if(STCLK_MS*1000 < (trg0 - SysTick->VAL) ){
				trg0 = SysTick->VAL;
				f5_eth = 1;
				//LED1 ^= 1;
				
			}
			
			if(f3_rtc_1sec==1){
				rtc_GetTime(1, &Rtc);
				CopyClock(tbuf,&Rtc);
				TFT_txt(tbuf,180, 229,YELLOW);
				f3_rtc_1sec=0;
			}
			
			if(p_Fifo1->rct){
				UART_getChar((char*)&switcH);
				
				switch (switcH)	{
					case 'r':
						UaPutK("\r\n PhyRegister addr = 0x");
						reg_adr=UART_getHex();
						UaPutK("\r\n Value = 0x");
						//hex2uart( enc28j60PhyReadH(reg_adr),4);
					break ;
					case 'i':
						UaPutK("\r\n Init client");
						init_simple_client();
					break ;
					case 's':
						UaPutK("\r\n Start client http");
						flag_6 = 1;
						UaPutK("\r\n enc28j60Revision = ");
					break ;
				}
				
			}
			
		}
		
	}

void ENC28J60_Tool(void){
		// 8,388,608 bytes, 32768 pages(256 bytes)
		
		u08 switcH,dinit,reg_adr,i2c_dat;
		
		char tbuf[20]={"0000-00-00 00:00:00"};
		//u16 crc16,crc_dat;
		MMCFG mmcfg;
		//cdata = (u16*) buffer;
		p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='i';
		
		f4_enc28 = 0;
		LED2=0;
		I2C2_init();
		
		//rtc_SetTime(&Rtc);
		rtc_GetTime(1,&Rtc);
		CopyClock(tbuf,&Rtc);
		TFT_txt(tbuf,180, 229,YELLOW);
		
		for(;;)
		{
			//Menu1();
			UaPutK("\n\r\n======= ENC28J60 Tool ============================\r\n");
			UaPutK(" s - Stat Reg  r - Read  memory  e - Erase page     \r\n");
			UaPutK(" c - CRC Sum   w - Write memory  E - Erase All     \r\n");
			UaPutK("==================================================\r\n > ");
			
			while(!p_Fifo1->rct){
				if(f4_enc28){
					f4_enc28 = 0;
					//plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
					//plen=packetloop_arp_icmp_tcp(buf,enc28j60PacketReceive(BUFFER_SIZE, buf));
					//simple_server();
					//UaPutC('.');
					//UaPutK("\r\n");
					
				}
			}
			//if(p_Fifo1->rct){	// Wywolanie interfejsu szeregowego gdy odebrano znak
			
			UART_getChar((char*)&switcH);
			//UaPutC('\f');
			UaPutK("\r\n ");
				switch (switcH)	{
				//========== Init Card SD/MMC ==================================
				case 's':
					
					dinit=mci_init();
					if(dinit){
						if(	mci_read_config (&mmcfg)){
							UaPutK("\r\nSD Card ser_num    = "); uint2uart(mmcfg.sernum);
							UaPutK("\r\nSD Card block_num  = "); uint2uart(mmcfg.blocknr);
							UaPutK("\r\nSD Card read_blen  = "); uint2uart(mmcfg.read_blen);
							UaPutK("\r\nSD Card write_blen = "); uint2uart(mmcfg.write_blen);
						}
					} else UaPutK("\r\nSD Card Init ERROR ");
					
				break;
				case 'i':
					//init_simple_client();
					//simple_server_client();
					
				break ;
				case 'r':
					UaPutK("\r\n PhyRegister addr = 0x");
					reg_adr=UART_getHex();
					UaPutK("\r\n Value = 0x");
					//hex2uart( enc28j60PhyRead(reg_adr),4);
				break ;
				case 'w':
					//UaPutK("\r\n Start simple server ... ");
					//UaPutK("\r\n sizeof  ( un_int)= ");		uint2uart(sizeof(u16));
					UaPutK("\r\n simple_server_client() ");
					//simple_server_client();
					//rev = enc28j60getrev();
					//enc28j60read();
				break ;
				case 't':
					UaPutK("\r\n i2c addr = 0x");
					reg_adr=UART_getHex();
					UaPutK("\r\n Value = 0x");
					
					while(1){
						if(STCLK_MS*1000 < (trg0 - SysTick->VAL) ){
							trg0 = SysTick->VAL;
							
							i2c2_read( DS3231_ADDR,reg_adr, &i2c_dat, 1 );
							hex2uart( i2c_dat,2);
							
							if(p_Fifo1->rct) break;
						}
						if(f4_enc28){
							f4_enc28 = 0;
							//plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
							//simple_server();
						}
					}
					
				break ;
			}
			
		}
	}
#endif
//=========================================================================================================
void DHT_Tool(void)				{
	
		//== Init section =======
		GPIOA->CRH = (GPIOA->CRH & 0x0ff00fff) | 0x10011000;
		GPIOB->CRL = (GPIOB->CRL & 0xfff00fff) | 0x00011000;
		
		//dht_hw.pin  = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,4);
		oled_hw.mosi = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,3);
		//== Init section =======
}
void SPI_Tool_Menu(void) 	{
	UaPutK("\n\r\n======= STM32F1 SPI TOOL =========================\r\n");
	UaPutK(" i - Init      r - Read  sector  e -                \r\n");
	UaPutK(" d - Delete    f - Write sector  s -                \r\n");
	UaPutK(" m - Modf      c - Copy  sector  n -               \r\n");
	UaPutK("==================================================\r\n > ");
}
void SPI_Tool(void)				{
	u32 d,tim;
	s32 numClus=numClus,lenBlock;
	u08 switcH,znak,reg;
	u16 crc=1,x=0,y1=0,y2=0;
	char tbuf[12];
	
	LED=1;
		
		GPIOA->CRH = (GPIOA->CRH & 0x0ff00fff) | 0x10011000;
		GPIOB->CRL = (GPIOB->CRL & 0xfff00fff) | 0x00011000;
		
		oled_hw.sck  = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,4);
		oled_hw.mosi = (u32*) m_BITBAND_PERIPH(&GPIOB->ODR,3);
		oled_hw.dc   = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,12);
		oled_hw.cs   = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,11);
		oled_hw.rst  = (u32*) m_BITBAND_PERIPH(&GPIOA->ODR,15);
		
	//sh1106_init();
	//ssd1306_init();
	int2str(12345678 ,tbuf,8,0);		
	oled_num24(0, 4,tbuf);
	oled_num24(0, 0,tbuf);
	oled_refresh_gram();	
	
	#define SPI2_CS PB12_o
	SPI2_init();
	//mcp2515_init(0);
	//mcp2515_read_status(1);
	
	//p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='i';
	for(;;)
	{
		SPI_Tool_Menu();
		while(!p_Fifo1->rct);
		//if(p_Fifo1->rct){	// Wywolanie interfejsu szeregowego gdy odebrano znak
		
		UART_getChar((char*)&switcH);
		//UaPutC('\f');
		UaPutK("\r\n ");
		switch (switcH)	{
		//========== Init Card SD/MMC ==================================
		case 'i':
			
			SPI2_init();
			UaPutK("\r\nInit spi    = "); uint2uart(1);
			
			break;
			//========== Card Info =========================================
		case 'r':
			SPI2_CS = 0; 
			UaPutK("\r\n Reg = ");	reg=UART_getNum();
			spi2_rw(reg);						// Read status register
			UaPutK("\r\n Reg = ");	hex2uart(spi2_rw(0x00),2);
			SPI2_CS = 1;			
			
			break;
			//========== Write block ========================================
			case 'c':
			UaPutK("Copy memory");
			UaPutK("\r\n Src Mem adr = 0x");	reg=UART_getHex();
			UaPutK("\r\n How many byte = ");	reg=UART_getNum();
			UaPutK("\r\n Dst mem adr = 0x");	reg=UART_getHex();
			
			
			break;
			//========== Default ===========================================
			default :
			UaPutK("Nieznana Komenda");
			break;
		}
	}
}

#ifdef NOTUSE
void Radio(void){
	u08 txBUF[10]={1,2,3,4,5,4,3,2,1,0};
	u08 znak;
	u16 n=0;

	nRF24L01_Init();
	UaPutS("\r\n Radio Init ");
	TX_Mode();
	for(;;){
		nRF24_tx(txBUF);
		UaPutS("\r\n TX->nRF24L01 ");
		sint2uart(n++);
		Delay_ms(2000);

	}

}
void Test_avg(void){
	static u16 y[320],a[320],ys;
	u32 tim=0;
	//static u16 b[320];
	//static volatile uint32_t * const jdrs[] = {&ADC1->JDR1, &ADC1->JDR2, &ADC1->JDR3, &ADC1->JDR4};
#define DT 7
	adc1_init_injected(1, 1);
	for(;;){

		if( STCLK_MS*300 < (trg0 - SysTick->VAL) ){
			trg0 = SysTick->VAL;
			TIM_start();
			for(u16 n=0;n<320;n++){

				/*
					if(n==0){
						y[0] = y[1];
					}
					else{
						TFT_dy(n-1, y[n-2], y[n-1], BLACK,0);	
						TFT_dy(n-1, a1		,		 a2 , BLACK,1);	
						a1=a2;
					}
				 */
				// Clear
				if(n>0){
					TFT_dy( n-1, ys, y[n], BLACK, 1);
					//TFT_dy( n-1, a[n-1], a[n],   BLUE, 1);
				}
				ys=y[n];

				//y[n] = (adc_get_injected(1))/16;
				ADC1_CR2_JSWSTART_bb = 1;
				while (ADC1_SR_JEOC_bb == 0);
				//ADC1_SR_JEOC_bb = 0;
				y[n]= ((ADC1->JDR1-36)/4-400)  ;

				a[n] = a[n] * DT;
				a[n] = a[n] + y[n];
				a[n] = a[n] / (DT+1);
				//a[n] = a[n] * DT;
				//a[n] = a[n] + y[n];
				//a[n] = a[n] / (DT+1);

				if(n>0){
					TFT_dy( n-1, y[n-1], y[n], YELLOW, 1);
					//TFT_dy( n-1, a[n-1], a[n],   BLUE, 1);
				}

				//Delay_us(500);
			}
			TIM_stop(&tim);
		}

		if( STCLK_MS*300 < (trg1 - SysTick->VAL) ){
			trg1 = SysTick->VAL;

			//UaPutS("\r\n" ); 			uint2uart(y[n]);
			UaPutS("\r\n Time [us] = " ); 	num2uart(tim/TIM_US, 6,0);
		}
		//TFT_Clear(BLACK);

	}
}
//===========================================================================
void EN25QH64_Tool(void){
		// 8,388,608 bytes, 32768 pages(256 bytes)
		
		u08 switcH,dinit,data,sr=0;
		u16 ID,crc16,crc_dat;
		//MMCFG mmcfg;
		u32 addr,to_addr,tim,tout;
		static u08 buffer[512];
		u16 * cdata;
		//cdata = (u16*) buffer;
		//p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='i';
		SPI2_init();
		crc16_spi1_init();
		disk_init(0);
		//crc16_spi3_init();
		for(;;)
		{
			//Menu1();
			UaPutK("\n\r\n======= EN25QH64 Tool ============================\r\n");
			UaPutK(" s - Stat Reg  r - Read  memory  e - Erase page     \r\n");
			UaPutK(" c - CRC Sum   w - Write memory  E - Erase All     \r\n");
			UaPutK("==================================================\r\n > ");
			
			while(!p_Fifo1->rct);
			//if(p_Fifo1->rct){	// Wywolanie interfejsu szeregowego gdy odebrano znak
			
			UART_getChar((char*)&switcH);
			//UaPutC('\f');
			UaPutK("\r\n ");
				switch (switcH)	{
				//========== Init Card SD/MMC ==================================
				case 'i':
					/*
					dinit=mci_init();
					if(dinit){
						if(	mci_read_config (&mmcfg)){
							UaPutK("\r\nSD Card ser_num    = "); uint2uart(mmcfg.sernum);
							UaPutK("\r\nSD Card block_num  = "); uint2uart(mmcfg.blocknr);
							UaPutK("\r\nSD Card read_blen  = "); uint2uart(mmcfg.read_blen);
							UaPutK("\r\nSD Card write_blen = "); uint2uart(mmcfg.write_blen);
						}
					} else UaPutK("\r\nSD Card Init ERROR ");
					*/
					dinit=disk_init(0);
					if(dinit){
					UaPutK("\r\nSD Card Init = "); sint2uart(CardType);
					} else UaPutK("\r\nSD Card Init ERROR ");
				break;
				//========== Card Info =========================================
				case 's':
						SPI2CS = 0; 
						spi2_rw(0x05);						// Read status register
						UaPutK("  Status Register = 0x");	hex2uart(spi2_rw(0x00),2);
						SPI2CS = 1;			
						
				break;
				case 'd':
					ID=0;
					UaPutK("read ID");
					SPI2CS = 0;
					spi2_rw( 0x5a);
					spi2_rw( 0x00);
					spi2_rw( 0x00);
					spi2_rw( 0x00);
					spi2_rw( 0x00);
					//spi2_rw( 0xff);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					ID=spi2_rw( 0x00); uint2uart(ID);
					
					SPI2CS = 1;
					UaPutK("\r\nID = "); uint2uart(ID);
					
				break;
				case 'c':	// ==================== Check CRC memory ======================
					UaPutK("Read memory from addr = 0x");
					addr=UART_getHex();
					UaPutK("\r\n Read memory to   addr = 0x");
					to_addr=UART_getHex();
					UaPutK("\r\n ");
					
					SPI1->CR1 &= ~SPI_CR1_CRCEN; // Reset CRC
					SPI1->CR1 |=  SPI_CR1_CRCEN;
					
					TIM_start();
						SPI2CS = 0;
						spi2_rw(0x03);				// Read data
						spi2_rw(addr>>16);
						spi2_rw(addr>>8);
						spi2_rw(addr);
						ID=0;
						tout=0;
						
						while (addr<to_addr){
							//crc_dat = (spi2_rw(0x00)<<8) + spi2_rw(0x00);
							spi1_rw(spi2_rw(0x00) + (spi2_rw(0x00)<<8));
							
							addr    += 2;
							if(!(addr%0x40000)) UaPutC('.');
						}
						
						SPI2CS = 1;
						TIM_stop(&tim);
						crc16 = SPI1->TXCRCR;
						//crc16_spi1_init();
						
						UaPutK("\r\n sum = ");	hex2uart(tout,8);
						UaPutK(" crc16 = ");	hex2uart(crc16,4);
						//crc16_spi2_init();
						//crc16_spi2( (u16*) buffer,256, &crc16 );
						//crc16_spi1( (u16*) buffer,256, &crc16 );
						//SPI2_init();
						//UaPutK(" crc16 = ");	hex2uart(crc16,4);
						
						UaPutK("Tim[us] = ");	uint2uart(tim/TIM_US);
						// CRC16 = 0xdc9e, CRCEN25QH64 = 0cdc9e
						
				break;
				case 'C':	// ==================== Check CRC memory ======================
					UaPutK("Read memory from addr = 0x");
					addr=UART_getHex();
					UaPutK("\r\n Read memory to   addr = 0x");
					to_addr=UART_getHex();
					UaPutK("\r\n ");
					
					SPI1->CR1 &= ~SPI_CR1_CRCEN; // Reset CRC
					SPI1->CR1 |=  SPI_CR1_CRCEN;
					
					TIM_start();
						
						ID=0;
						tout=0;
						
						while (addr<to_addr){
							if(!SD_readSingleBlock(buffer,addr)){
								cdata = (u16*) buffer;
								for (u16 n=0;n<256;n++){
									//spi1_rw((u16)buffer[n*2]);
									spi1_rw(*cdata++);
								}
							} else {
								UaPutK("\r\n Blad odczytu karty SD ");
								break;
							}
							addr++;
							if((addr%512)==0) UaPutC('.');
							
						}	
						
						TIM_stop(&tim);
						crc16 = SPI1->TXCRCR;
						
						
						UaPutK("\r\n sum = ");	hex2uart(tout,8);
						UaPutK(" crc16 = ");	hex2uart(crc16,4);
						UaPutK("Tim[us] = ");	uint2uart(tim/TIM_US);
						// CRC16 = 0xdc9e, CRCEN25QH64 = 0cdc9e
						
				break;
				case 'r':	//===================== Read memory ===========================
					ID=0;
					UaPutK("Read memory addr = 0x");
					addr=UART_getHex();
					
					TIM_start();
						SPI2CS = 0;
						spi2_rw(0x03);				// Read data
						spi2_rw(addr>>16);
						spi2_rw(addr>>8);
						spi2_rw(addr);
						for (u16 n=0;n<512;n++){
							buffer[n]=spi2_rw(0x00); 
						}
						SPI2CS = 1;
						TIM_stop(&tim);
						
						if(1){
							UaPutS("\r\n ");
							hex2uart(addr, 8);
							for(u08 k=0;k<16;k++){
								hex2uart(k, 2);
							}
							UaPutS("\r\n\n ");
							for(u32 i=0;i<512;i+=16){
								hex2uart(addr + i, 8);
								for(u08 k=0;k<16;k++){
									hex2uart(buffer[k+i], 2);
								}
								UaPutS("  ");
								for(u08 k=0;k<16;k++){
									if ( buffer[k+i]<0x20 || buffer[k+i]>126 ) UaPutC('.');
									else UaPutC(buffer[k+i]);
								}
								UaPutS("\r\n ");
							}
							crc16_spi2_init();
							crc16_spi2( (u16*) buffer,256, &crc16 );
							UaPutK(" crc16 = ");	hex2uart(crc16,4);
							SPI2_init();
							
							UaPutK("Tim[us] = ");	uint2uart(tim/TIM_US);
							//TIM_start();
							//TIM_stop(&tim);
							
						} 
				break;
				case 't':
					UaPutK("Test \r\n Pasuja = ");
					addr = 0xff;
					while(addr &  0b011010){
						//if(addr & 0b00111100) hex2uart(addr,2);
						hex2uart(addr,2);
						addr = addr-1; 
						
					}
				break;
				case 'w': //=================== Write memory ============================
						
						UaPutK("Write memory addr = 0x");
						addr=UART_getHex();
						UaPutK("\r\n Write memory data = 0x");
						data=UART_getHex();
						
						TIM_start();
						for(u16 x=0;x<0x2;x++){
							SPI2CS = 0; spi2_rw(0x06);	SPI2CS = 1;			// WREN
							
							SPI2CS = 0;
							spi2_rw(0x02);				// write enab data
							spi2_rw(addr>>16);
							spi2_rw(addr>>8);
							spi2_rw(0);
							
							for (u16 n=0;n<256;n++){
								spi2_rw(data); 
							}
							SPI2CS = 1;
							
							SPI2CS = 0; 
							spi2_rw(0x05);						// Read status register
							tout = 0xffff;
							while(spi2_rw(0x00) & 0xff){	// Write in progress
								sr = spi2_rw(0x00);
								if(!tout--) {
									UaPutK(" Error writing ");
									UaPutK(" SR = ");	hex2uart(spi2_rw(0x00),2);
									SPI2CS = 1;			
									break;
								}
								Delay_ms(1);
							}
							//Delay_us(2000);
							
							SPI2CS = 1;			
							addr += 256;
						}
						
						TIM_stop(&tim);
						
						UaPutK("\r\n Tim[us] = ");	uint2uart(tim/TIM_US);
						UaPutK("\r\n SR = ");hex2uart(sr,2);
						
				break;
				case 'e':	//====================== erase block =========================
						
						UaPutK("Erase memory block(0-0x7f) = 0x");
						addr=UART_getHex();
						
						SPI2CS = 0; spi2_rw(0x06);	SPI2CS = 1;			// WREN
						TIM_start();
						SPI2CS = 0;
						spi2_rw(0xd8);				// 0xd8 Erase Block (0xffff bytes)
						spi2_rw(addr<<16);
						spi2_rw(00);
						spi2_rw(00);	
						SPI2CS = 1;
						
						SPI2CS = 0; 
						spi2_rw(0x05);						// Read status register
						tout = 0xffff;
						while(spi2_rw(0x00) & 0xff){	// erase in progress
							if(!tout--) break;
							Delay_ms(1);
						}
						UaPutK(" SR = ");	hex2uart(spi2_rw(0x00),2);
						SPI2CS = 1;			
						TIM_stop(&tim);
						
						UaPutK("\r\n Tim[us] = ");	uint2uart(tim/TIM_US);
				break;
				case 'E':	//======================= erase all ==========================
						UaPutK("Erase All... ");
						TIM_start();
						SPI2CS = 0; spi2_rw(0x06);	SPI2CS = 1;			// WREN
						SPI2CS = 0; spi2_rw(0xc7);	SPI2CS = 1;			// Erase All
						
						
						SPI2CS = 0; 
						spi2_rw(0x05);						// Read status register
						tout = 0xfffff;
						while(spi2_rw(0x00) & 0xff){	// erase in progress
							if(!tout--) {
								UaPutK(" Error erasing");
								break;
							}
							Delay_ms(10);
							if((tout%1024==0)) hex2uart(tout,5);
						}
						UaPutK(" SR = ");	hex2uart(spi2_rw(0x00),2);
						SPI2CS = 1;			
						TIM_stop(&tim);
						
						UaPutK("\r\n Tim[us] = ");	uint2uart(tim/TIM_US);
				break;
				case 'p':	//======================= Program from SD Card =================
					addr = 0;
					//for(u32 i=0;i<16;i++){
					for(u32 i=0;i<16384;i++){
						if(!SD_readSingleBlock(buffer,i)){
						for(u08 q=0;q<2;q++){
							SPI2CS = 0; spi2_rw(0x06);	SPI2CS = 1;			// WREN
							SPI2CS = 0;
							spi2_rw(0x02);				// write enab data
							spi2_rw(addr>>16);
							spi2_rw(addr>>8);
							spi2_rw(addr);
							
							for (u16 n=0;n<256;n++){
								spi2_rw(buffer[256*q+n]); 
							}
							SPI2CS = 1;
							
							SPI2CS = 0; 
							spi2_rw(0x05);						// Read status register
							tout = 0xffff;
							while(spi2_rw(0x00) & 0xff){	// Write in progress
								Delay_ms(1);
								if(!tout--){
									UaPutK("\r\n Error Programming ");
									break;
								}
							}
							SPI2CS = 1;
							
							addr += 256;
						}						
						
						if((i%256)==0) UaPutC('.');
						
					} else UaPutK("\r\n Error Reading SD Card ");
				}
				break;
			}
		}
	}
#endif
void Menu1(void) 					{
	UaPutK("\n\r\n======= STM32F4 SD/MMC Card ======================\r\n");
	UaPutK(" i - Init      r - Read  sector  e - Erase sector   \r\n");
	UaPutK(" d - Delete    f - Write sector  s - Statistic Card \r\n");
	UaPutK(" m - Modf mem  c - Copy  sector  n - Next Cluster  \r\n");
	UaPutK("==================================================\r\n > ");
}
void MemCard_Menu(void)		{
	u32 strBlock,dstBlock,StrBlock,d,tim;
	s32 numClus=numClus,lenBlock;
	u08 switcH,znak;
	u16 crc=1,x=0,y1=0,y2=0;
	MMCFG mmcfg;
	TFT_Init();
	TFT_Clear(BLACK);

	//u32 * ptrlB = &lenBlock;
	//unsigned char __attribute__((unused)) option, data;
	static u08 buffer[512*16],dinit;
	//crc16_spi2_init();
	//dinit=disk_init(0);
	//dinit=mci_init();
	
	crc16_spi2_init();

	if(dinit){
		UaPutK("\r\nSD Card Init = "); sint2uart(CardType);
	} else UaPutK("\r\nSD Card Init ERROR ");

	p_Fifo1->rct=1;	p_Fifo1->rwi=1;	p_Fifo1->rbuf[0]='i';
	for(;;)
	{
		Menu1();
		while(!p_Fifo1->rct);
		//if(p_Fifo1->rct){	// Wywolanie interfejsu szeregowego gdy odebrano znak

		UART_getChar((char*)&switcH);
		//UaPutC('\f');
		UaPutK("\r\n ");
		switch (switcH)	{
		//========== Init Card SD/MMC ==================================
		case 'i':
			
			dinit=mci_init();
			if(dinit){
				if(	mci_read_config (&mmcfg)){
					UaPutK("\r\nSD Card ser_num    = "); uint2uart(mmcfg.sernum);
					UaPutK("\r\nSD Card block_num  = "); uint2uart(mmcfg.blocknr);
					UaPutK("\r\nSD Card read_blen  = "); uint2uart(mmcfg.read_blen);
					UaPutK("\r\nSD Card write_blen = "); uint2uart(mmcfg.write_blen);
				}
			} else UaPutK("\r\nSD Card Init ERROR ");
			/*
			dinit=disk_init(0);
			if(dinit){
				UaPutK("\r\nSD Card Init = "); sint2uart(CardType);
			} else UaPutK("\r\nSD Card Init ERROR ");
			*/
			break;
			//========== Card Info =========================================
		case 's':
			UaPutK("Statistic Card  ");
			if(dinit){
				//disk_ioctl(0,GET_SECTOR_SIZE ,&lenBlock); UaPutK("\r\n SECTOR_SIZE      = "); uint2uart((u16)lenBlock);
				//if(disk_ioctl(0,GET_SECTOR_COUNT,&numBlock) == RES_OK) { UaPutK("\r\n SECTOR_COUNT     = 0x"); hex2uart(numBlock,7);	   }
				//if(disk_ioctl(0,GET_BLOCK_SIZE  ,&strBlock) == RES_OK) { UaPutS("\r\n ERASE_BLOCK_SIZE = ");   uint2uart((u16)strBlock); }
				//UaPutS("\r\n CARD_SIZE [KB]   = "); uint2uart( (u16)lenBlock*((numBlock/1024)) );
				if(	mci_read_config (&mmcfg)){
					UaPutK("\r\nSD Card ser_num    = "); uint2uart(mmcfg.sernum);
					UaPutK("\r\nSD Card block_num  = "); uint2uart(mmcfg.blocknr);
					UaPutK("\r\nSD Card read_blen  = "); uint2uart(mmcfg.read_blen);
					UaPutK("\r\nSD Card write_blen = "); uint2uart(mmcfg.write_blen);
				}
			}else UaPutK("\r\nSD Card Not Init");
			break;
			//========== Write block ========================================
		case 'f':
			UaPutK("Fill sector");
			UaPutK("\r\n Start Block  = 0x");		strBlock=UART_getHex();
			//strBlock = UART_getNum((char *)uaBuf);
			UaPutK("\r\n End   Blocks = 0x");		dstBlock=UART_getHex();
			dstBlock = dstBlock > strBlock ? dstBlock - strBlock + 1 : 1;
			//dstBlock = dstBlock - strBlock + 1;
			UaPutK("\r\n Num Blocks   = ");				uint2uart(dstBlock);
			UaPutK("\r\n Data to fill = 0x");
			znak=UART_getHex();
			for(u32 i=0;i<512;i++){
				buffer[i] = znak;
			}
			while(dstBlock--){
				TIM_start();
				//i = SD_writeSingleBlock(buffer,strBlock);
				d = mci_write_sect(strBlock, buffer, 1);
				if(d==0) {
					UaPutS("\r\n Fill error = 0x");
					hex2uart(strBlock,8); hex2uart(d,2);
				}
				else {
					if(!(strBlock%1024)){
						TIM_stop(&tim);
						UaPutS("\r\n Fill OK ");
						hex2uart(strBlock,  8);
						UaPutS(" Time[us] = " );
						num2uart(tim, 8,0);
					}
				}
				strBlock++;
			}
			break;
			//========== Modify Memory =====================================
		case 'm':

			UaPutS("Modify memory");
			UaPutS("\r\n Memory Addr = 0x");

			strBlock=UART_getHex();
			//if (SD_readSingleBlock(buffer, strBlock/512) == 0){
			if (mci_read_sect	(strBlock/512, buffer, 1)){
				UaPutS("\r\n Memory Data = 0x");
				hex2uart(buffer[strBlock%512], 2);
				UaPutS("\r\n New Data    = 0x");
				StrBlock=UART_getHex();
				buffer[strBlock%512] = StrBlock;
				//if (SD_writeSingleBlock(buffer,strBlock/512) == 0 ) UaPutS("\r\n Modify OK");
				if (mci_write_sect		(strBlock/512, buffer,1))  UaPutS("\r\n Modify OK");
			}
			break;
			//========== Copy Memory =======================================
		case 'c':
			UaPutK("Copy memory");
			UaPutK("\r\n Src Mem adr = 0x");	strBlock=UART_getHex();
			UaPutK("\r\n How many byte = ");	lenBlock=UART_getNum();
			UaPutK("\r\n Dst mem adr = 0x");	dstBlock=UART_getHex();
			
			u32 dbl = dstBlock%512, sBlk=strBlock/512, sMem=strBlock%512;
			for(u32 bl=(dstBlock/512);bl <= ((dstBlock+lenBlock-1)/512); bl++){
				if (mci_read_sect(bl,buffer,1)){
					UaPutK("\r\n ReadB 0x");							hex2uart(sBlk++,3);
					UaPutC(',');													hex2uart(sBlk, 	3);
					UaPutK("  From 0x");							    hex2uart(sMem, 	3);
					sMem = sMem + (0x200-dbl);
					UaPutK("->0x");											  hex2uart(sMem + (0x200-dbl), 3);
					UaPutK("  WriteB 0x");								hex2uart(bl, 		3);
					UaPutK("  0x");					  						hex2uart(dbl, 	3);
					UaPutK("->0x");
					if (bl == ((dstBlock+lenBlock)/512))	hex2uart((dstBlock+lenBlock)%512, 3);		//Last block
					else																	hex2uart(0x200, 3);
					dbl = 0;
				}
			}
			/*
					if (SD_readSingleBlock( strBlock) == 0){				
						UaPutS("\r\n Dst Block = ");
						strBlock = UART_getNum((char *)uaBuf);
						if (SD_writeSingleBlock(strBlock) == 0 ) UaPutS("\r\n Copy OK");
					}
			 */
			break;
			//========== Read sector =======================================
		case 'r':
			UaPutK("Read sector");
			UaPutK("\r\n sectorNumber = 0x");
			//strBlock = UART_getNum((char *)uaBuf);
			strBlock=UART_getHex();
			TIM_start();
			//d=SD_readSingleBlock(buffer,strBlock);
			d=mci_read_sect (strBlock,buffer,1);
			TIM_stop(&tim);
			if(d){
				UaPutS("\r\n ");
				hex2uart(strBlock, 8);
				for(u08 k=0;k<16;k++){
					hex2uart(k, 2);
				}
				UaPutS("\r\n\n ");
				for(u32 i=0;i<512;i+=16){
					hex2uart(strBlock*512 + i, 8);
					for(u08 k=0;k<16;k++){
						hex2uart(buffer[k+i], 2);
					}
					UaPutS("  ");
					for(u08 k=0;k<16;k++){
						if ( buffer[k+i]<0x20 || buffer[k+i]>126 ) UaPutC('.');
						else UaPutC(buffer[k+i]);
					}
					UaPutS("\r\n ");
				}
				UaPutK(" t_rsdio = ");	uint2uart(tim/TIM_US);
				TIM_start();
				crc16_spi2( (u16*) buffer,256, &crc );
				//crc8_spi2((u08 *) buffer,512,&crc );
				TIM_stop(&tim);
				UaPutK(" crc = ");	hex2uart(crc,4);
				UaPutK(" t_crc = ");	uint2uart(tim/TIM_US);
			} else { 
				UaPutK("\r\n Read Error = ");
				uint2uart(d);
			}
			break;
			//========== Read sector =======================================
		case 'R':
			if(CardType){
				UaPutK("Read multi sector");
				//UaPutK("\r\n Start Sector = 0x");		strBlock=UART_getHex();
				//UaPutK("\r\n End   Sector = 0x");		dstBlock=UART_getHex();
				strBlock = 0;
				dstBlock = 1000000;
				dstBlock = dstBlock > strBlock ? dstBlock - strBlock + 1 : 1;
				UaPutK("\r\n Num Sectors  = ");			uint2uart(dstBlock);
				x=0;
				TFT_Clear(BLACK);
				while(dstBlock--){
					//i = SD_readSingleBlock(buffer,strBlock);
					d = mci_read_sect (strBlock,buffer,1);
					if(!d) {
						UaPutS("\r\n Read error = 0x");
						hex2uart(strBlock,8); hex2uart(d,2);
						break;
					}
					else {
						if(!(strBlock%256)){
							TIM_stop(&tim);
							if(!(strBlock%4096)){
								UaPutS("\r\n Read OK ");
								hex2uart(strBlock,  8);
								UaPutS(" y2 = " );
								uint2uart(y1); UaPutS("\t" ); uint2uart(y2);
								//y2 = (tim-421235)/1024;
							}
							TIM_start();
							y2 = tim/(3*TIM_MS);
							if(x){
								TFT_dy( x, y1, y2, YELLOW, 0);
								//TFT_dy( x, 0, 10, YELLOW, 1);
							}else {
								TFT_Clear(BLACK);
							}
							y1=y2;
							x = (x+1)%(wXmax-wXmin);
						}
					}
					strBlock++;
					if(p_Fifo1->rct) break;
				}
			} else UaPutK("\r\nSD Card Init ERROR ");
			break;
			//========== Last use sector ===================================
		case 'u':
			UaPutK("View last sector");
			UaPutK("\r\n First Sector = 0x");
			strBlock=UART_getHex();
			UaPutK("\r\n Searching ");
			numClus =  1;
			lenBlock = 1;
			//#define _USE_ADD 1
#define _USE_LINEAR 1

#if _USE_LINEAR
			do{
				//if( SD_readSingleBlock(buffer,strBlock++) ) {
				if( mci_read_sect (strBlock++,buffer,1) ) {
					UaPutK("\r\n Exit Read Error !!!");
					break;
				}
				else {
					//for(u32 i=0;i<512;i++)	numClus += buffer[i];
					//crc16_spi2((u16*)buffer,256,&crc);
					if(!(strBlock%1024)) UaPutC('.');
				}
			}
			while(crc != 0x822d );
#endif
#if _USE_ADD
			for(;;){
				if( SD_readSingleBlock(buffer,strBlock)  ) {
					UaPutK("\r\n Exit Read Error !!!");
					break;
				}
				else{
					crc16_spi2((u16*)buffer,256,&crc);
					if(crc == 0x822d){
						lenBlock += numClus;
						if(!(strBlock%0x200)) UaPutC('+');
						strBlock += lenBlock;
						//UaPutK("\r\n Find Sector =+ 0x");	hex2uart(strBlock, 8);
						//UaPutK("\r\n     lenBlock=+ ");		sint2uart(lenBlock);
						if( lenBlock < 0 ) break;						// srodek przedzialu
					}
					else{
						numClus = -1;
						lenBlock += numClus;
						if(!(strBlock%0x200)) UaPutC('-');
						strBlock -= lenBlock;
						//UaPutK("\r\n Find Sector =- 0x");	hex2uart(strBlock, 8);
						//UaPutK("\r\n     lenBlock=- ");		sint2uart(lenBlock);
					}
				}
			}
#endif

			UaPutK("\r\n Find Sector = 0x");	hex2uart(strBlock, 8);
			UaPutK("\r\n     lenBlock= ");		sint2uart(lenBlock);
			break;
			//========== Testing =======================================
		case 't':
			UaPutK("Read sector");
			UaPutK("\r\n sectorNumber = 0x");
			//strBlock = UART_getNum((char *)uaBuf);
			strBlock=UART_getHex();
			UaPutK("\r\n sectorNumber = 0x");		hex2uart(strBlock, 8);
			while(strBlock <  31116288 ){
				TIM_start();
				//i=SD_readSingleBlock(buffer,strBlock);
				if( mci_read_sect (strBlock,buffer,1*16)) {
					crc16_spi2((u16*)buffer,256*16, &crc );
					if( crc ) {
						UaPutK("\r\n Block = ");	hex2uart(strBlock,8); hex2uart(crc,4);
					}
					strBlock+=1*16;
					TIM_stop(&tim);
				} else UaPutK("\r\n Read Error ");
				if(p_Fifo1->rct) break;
			}
			//UaPutK(" time = ");	uint2uart(tim/TIM_US);
			UaPutK("\r\n Block = ");	hex2uart(strBlock,8); hex2uart(crc,4);
			break;
			//========== Testing =======================================
		case 'q':
			if(1) {
				//if(disk_ioctl(0,GET_SD_CMD9 ,buffer) == RES_OK) {
				//if(disk_ioctl(0,GET_SD_STATUS ,buffer) == RES_OK) {
				UaPutK("\r\n SD_Status \r\n ");
				hex2uart(0,4);
				for(u08 k=0;k<8;k++){ hex2uart(k, 2);	}
				UaPutK("\r\n\n ");
				for(u32 i=0;i<64;i+=8){
					unt2uart(i*8,4);	UaPutC(' ');
					for(u08 k=0;k<8;k++){
						hex2uart(buffer[k+i], 2);
					}
					UaPutS("  ");
					for(u08 k=0;k<8;k++){
						if ( buffer[k+i]<0x20 || buffer[k+i]>127 ) UaPutC('.');
						else UaPutC(buffer[k+i]);
					}
					UaPutS("\r\n ");
				}
			} else UaPutK("\r\n Read Error \r\n ");
			break;
		case 'x':
			UaPutK("Testing ...");
			//LED1 ^= 1;
			break;
			//========== Default ===========================================
		default :
			UaPutK("Unkr_dma_buf Command !");
			break;
		}
	}
}
#ifdef NOTUSE
		void quicksort(int tablica[], int x, int y){
			int i,j,v,temp;
			i=x;
			j=y;
			v=tablica[div(x+y,2).quot];
			do
			{
				while (tablica[i]<v) i++;
				while (v<tablica[j]) j--;
				if (i<=j)
				{
					temp=tablica[i];
					tablica[i]=tablica[j];
					tablica[j]=temp;
					i++;
					j--;
				}
			}
			while (i<=j);
			if (x<j) quicksort(tablica,x,j);
			if (i<y) quicksort(tablica,i,y);
		}

		void bublesort(u16* arr, uint8_t len)	{
			uint8_t i,j;
			int tmp;
			for(i=0;i<(len-1);i++)
				for(j=0;j<(len-(i+1));j++)
					if(arr[j] > arr[j + 1]) {
						tmp = arr[j];
						arr[j] = arr[j + 1];
						arr[j + 1] = tmp;
					}
		}

		void bubbleSort(int* array, int size) {
			int swapped;
			int i;
			for (i = 1; i < size; i++)
			{
				swapped = 0;    //this flag is to check if the array is already sorted
				int j;
				for(j = 0; j < size - i; j++)
				{
					if(array[j] > array[j+1])
					{
						int temp = array[j];
						array[j] = array[j+1];
						array[j+1] = temp;
						swapped = 1;
					}
				}
				if(!swapped){
					break; //if it is sorted then stop
				}
			}
		}
		
		/*
		void shellsort(long *a, long n){ 
		// Sedgewick-Incerpi sequence  
				long step[16] = {1, 3, 7, 21, 48, 112, 336, 861, 1968, 4592, 13776, 33936, 86961, 
				198768, 463792, 1391376}; 
				long i, j, k, s, t; 
				k = (n < 256) ? 6 : 15; 
				do { for (s=step[k], i=s-1; i < n; i++) 
				{ t = a[i]; for (j=i; j >= s && t > a[j-s]; a[j] = a[j-s], j -= s); a[j] = t; } } 
				while (k);
			} */

		void lsort(long *intArray, long SIZE){ 
			int i,j;
			for (i=0;i<(SIZE-1);i++)
			{ for(j=i;j<(SIZE-1);j++) 
			{ int tmp = intArray[j]; if( intArray[j+1] > intArray[j]) { intArray[j] = intArray[j+1]; intArray[j+1] = tmp; } }}
		}

		void TFT_VLine(u16 x1){
			TFT_REG = 0x004e;
			TFT_RAM = 239-239;
			TFT_REG = 0x004f;
			TFT_RAM = 319-x1;
			TFT_REG = R34;
			for (u08 i=0;i<240;i++)	TFT_RAM = BLACK;
		}
		void Intro1(void){
			char znak;
			u16 xCN=0;
			U1buf_ptr = u1buf;
			while(1)
			{
				//UaPutC('\f');
				UaPutS("\r\n Liczba  =");
				uint2uart(xCN);
				UaPutS("\r\n Adres = 0x");

				Menu();
				//LED1 = 1;
				UART_getChar(&znak);
				//while(flg_1 == 0);
				//FLG &= ~1;
				//while((FLG & 1)==0);
				//FLG &= ~1;
				//flg_1 = 0;
				UaPutC('\f');
				//switch (u1buf[0])	{
				switch (znak)			{
				//==========  ================================
				case 'r':
					//UaPutS("\r\n getSetNextCluster Get, Set or Write ? ");
					UaPutS("\r\n Pressed key r ");
					//strBlock = UART_getNum((char *)u2buf);
					//znak = UART_getChar();
					if(znak == 'g'){	// Read next clusters
						UaPutS("\r\n Start Cluster = ");
						//test = str2hex((char *)u1buf);
					}
					break;
					//==========  ================================
				case 'w':
					UaPutS("\r\n Pressed key w ");
					break;
					//==========  ================================
				default:
					UaPutS("\r\n Pressed other key ");
					break;
				}
				xCN++;
			}
		}
		
		void Mem2MemDMAInit(void *dst_buf, void *src_buf ){
			//DMA Configuration
			uint32_t CCR_reg = 0;
			RCC->AHBENR   			|= RCC_AHBENR_DMA2EN;	// enable clock for DMA2
			DMA2_Channel1->CCR 	= 0;										//Disable channel
			DMA2_Channel1->CMAR = (uint32_t)dst_buf;	 	//Destination address:
			DMA2_Channel1->CPAR = (uint32_t)src_buf;		//Source address:
			//DMA2_Channel1->CNDTR = sizeof(dst_buf);			//Buffor size :
			DMA2_Channel1->CNDTR = 320/2;		//Buffor size :
			CCR_reg |=  DMA_CCR1_MEM2MEM;	//Memory to memory mode
			CCR_reg |=  DMA_CCR1_PL;			//Priorytet high
			//CCR_reg |=  DMA_CCR1_MSIZE_1 | DMA_CCR1_PSIZE_1;	// 32bit/32bit
			CCR_reg |=  DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0;	// 16bit/16bit
			CCR_reg |=  DMA_CCR1_MINC;	//Increment memory address enabled
			CCR_reg |=  DMA_CCR1_PINC;	//Increment peripheral address enabled
			CCR_reg &= ~DMA_CCR1_CIRC;	//Mode Circular mode disabled
			CCR_reg &= ~DMA_CCR1_DIR;		//Dir Read from peripheral
			//CCR_reg |=  DMA_CCR1_TCIE;	//Transfer complete interrupt enable
			DMA2_Channel1->CCR =  CCR_reg;
			DMA2_Channel1->CCR |= DMA_CCR1_EN;
		}
		void Mem2MemDMA(void *src_buf, u32 offset ){
			//DMA Configuration
			DMA2_Channel1->CCR &= ~DMA_CCR1_EN;
			DMA2_Channel1->CNDTR = 320;			//Buffor size :
			DMA2_Channel1->CPAR = (uint32_t)src_buf + offset*2;		//Source address:
			DMA2_Channel1->CCR |= DMA_CCR1_EN;
		}
		void ReadNRF(void)		{
			u08 reg = 7,data;
			//char tbuf[16];
			UaPutS("\r\n Input   = 0x");
			reg = str2hex((char *)u1buf);
			if(reg == 0xFA) data = GPIOA->IDR & 0xff;
			else						nRF24_readBuf(reg, &data, 1);

			UaPutS("\r\n Reg(");
			hex2uart(reg,2);
			UaPutS(") = 0x");
			hex2uart(data,2);
			for (int i=0;i<8;i++){
				UaPutC(' ');
				hex2uart((data&0x80)>>7,1);
				data <<= 1;
			}
			UaPutS("\r\n                7 6 5 4 3 2 1 0");
			//GUI_Text(250,220, int2str(reg,tbuf,4,0),WHITE,BLACK);
		}
		void WriteNRF(void)		{
			u08 reg,data,dat;
			//char tbuf[16];
			UaPutS("\r\n Reg = 0x");
			reg = str2hex((char *)u1buf);
			UaPutS("\r\n Dat = 0x");
			dat = str2hex((char *)u1buf);
			//if (NCE == 1) nce=1;	CE_LOW();
			data = nRF24_writeReg(reg, dat);
			// if (nce)	CE_HIGH();
			UaPutS("\r\n Sts = 0x");
			hex2uart(data,2);
			//GUI_Text(250,220, int2str(reg,tbuf,4,0),WHITE,BLACK);
		}
		void MultiNRF(void)		{
			u08 reg,cnt,ci;
			u08 data[5];
			//char tbuf[16];
			UaPutS("\r\n RegAdr  = 0x");
			reg = str2hex((char *)u1buf);
			UaPutS("\r\n RegCnt  = 0x");
			cnt = str2hex((char *)u1buf);
			nRF24_readBuf(reg, data, cnt);
			UaPutS("\r\n Reg(");
			hex2uart(reg,2);
			UaPutS(") = 0x");
			for(ci=0;ci<cnt;ci++){
				hex2uart(data[ci],2);
				UaPutC(' ');
			}
		}
		/*===========================================================================*/
		void Menu(void) 			{
			UaPutS("\n\r\n==== ARM-STM32 ======================\r\n");
			UaPutS(" r - Read   o - RX_mode	 m - Mult_rd  \r\n");
			UaPutS(" w - Write  n - TX_mode	 s - Reset  ");
			UaPutS("\r\n=====================================\r\n# ");
		}
		void OtherTest(void)	{
		#define DIVADC 4
			u16 dx = 0, navg = DIVADC, Tadc[321];
			u32 tlo=0;
			flag = 0; tX = 4; tY = 120;
			
			//u16 col = TFT_BGR2RGB(YELLOW);
			//u16 col = RGB565CONVERT( 0,255 ,0 );	
			ptm.tm_sec = 59;
			while(0)	/// Oscyloskop
			{						
				tlo = BKP->DR1 | (BKP->DR2 << 16);
				bTIM6_CR1_CEN = 1;
				if(dx == 320) {
					dx = 0;
					TFT_Clear(BLACK);
				}
				
				//==========================================================================
				Tim6 = 0;	TIM6->CNT = 0;
				//==========================================================================
				//LED1 = 1;				
				//Delay1us(tlo);
				//Delay_us(tlo);	
				//TFT_Clear(BLACK);							//16007 us
				//while(dx++<240)		TFT_DrawPoint(dx/120,dx,col);
				//TFT_DrawLine(10,0,11,dy,col);				
				//TFT_DrawVerLine(dx++,dy,adc,BLACK);
				navg = 0;
				while(navg++ < 320){
					//TFT_ShortLine(navg,Tadc[navg-1],Tadc[navg],BLACK);
				}
				//==========================================================================	
				Time6 = ((TIM6->CNT | (Tim6 << 16))*5)/18;		
				//==========================================================================
				navg = 0;
				while(navg++ < 320){
					ADC1_CR2_ADON = 1;					// start conversion ADC PA3
					while(!ADC1_SR_EOC);				// wait for the end of convertion
					Tadc[navg-1]= ADC1->DR/16 ;
				}
				Tadc[0] = Tadc[0];
				navg = 0;
				while(navg++ < 320){
					//TFT_ShortLine(navg,Tadc[navg-1],Tadc[navg],YELLOW);
				}
				//adc = (adc/DIVADC - BKP->DR4);
				//navg = DIVADC;
				//TFT_DrawPoint(dx++,adc,YELLOW);		
				//TFT_DrawVerLine(dx,dy,    adc,YELLOW);
				//TFT_DrawVerLine(dx,dy+1,adc+1,YELLOW);
				//TFT_DrawVerLine(dx,dy+2,adc+2,YELLOW);
				//dx++;
				LED1 = 0;				
				bTIM6_CR1_CEN = 0;
				
				GUI_Text(250,220, int2str(ADC1->DR,tbuf,7,0),0xffff,BLACK);			
				GUI_Text(250,200,int2str(Time6,tbuf,6,1),0xffff,BLACK); 
				Delay_ms(BKP->DR3);
				//TFT_DrawLine(10,0,11,dy,BLACK);				
				//dy = BKP->DR3;
				//GUI_Text(10,10,tbuf,0xffff,BLACK);	
			}
			
			while(0) /// TFT demo
			{
				TFT_Clear(tlo);
				TFT_WriteScaledText( unt2str(tlo,tbuf,5), 100, 100, 4, 4, WHITE, tlo);
				tlo += 1;
				Delay_ms(10);
			}
			
			//u32 licz=8;
			u32 baf[321];
			u16 x;
			while(0)  //Oscyloskop
			{
				for(x=0;x<320;x++)
				{
					//		y=10*sqrt(x);
					/*		do{ licz--;
						bADC1_CR2_ADON = 1;					// start conversion ADC
						while(!bADC1_SR_EOC);				// wait for the end of convertion
						Pom += ADC1->DR;
						}
						while(licz); */
					ADC1_CR2_ADON = 1;					// start conversion ADC
					while(!ADC1_SR_EOC);				// wait for the end of convertion
					baf[x] = ADC1->DR /18 ;
					//	baf[x] = 120+120*sin((float)x/20) ;
					//	LED1 = ~LED1;
				}
				//TFT_Clear(BLACK);
				TFT_Clear(BKP->DR39);
				for(x=1;x<320;x++)
				{
					// TFT_DrawLine(x-1,baf[x-1],x,baf[x],BKP->DR39); //Drow activ line
					// TFT_DrawPoint(x,baf[x],YELLOW);
				}
				Delay_ms(100);
				for(x=1;x<320;x++)
				{
					TFT_DrawLine(x-1,baf[x-1],x,baf[x],0x07ff); //Drow mask line
				}	//		Delay(0,400);
			}
			x=0;
			//TFT_WriteScaledText( unt2str(x,tbuf,5), 0, 0, 2, 3, YELLOW, BLACK);	
			
			while(0)  /// Przesuwajacy sie liczby
			{
				//if(flag & 0b10) { ClockGraph(); flag &= ~(0b10); }
				//I2C2_READ_REG(0, nc++,&data, 1 ); //adr, reg_adr, data, len
				Delay_ms (50);  //type=1 ms, =0 us
				TFT_WriteScaledText( unt2str(x,tbuf,5), x, x, 2, 3, YELLOW, BLACK);
				x += 1;
				if(x>240) x=0;
			}
		}
		void nrf24con(void)		{
			
			void reset(void){
				GPIOA->BSRR = (1<<0);			// Power OFF
				Delay_ms(500);
				GPIOA->BRR  = (1<<0);			// Power ON
			}
			
			void CX(void){
				RX_Mode();
				u08 out_rf;
				u32 cnt=0;
				for (int ch=0; ch<128; ch++){
					nRF24_writeReg(RF_CH, ch);
					CE_HIGH();
					Delay_us(350);
					while( cnt<0xfff ){
						nRF24_readBuf(CD, &out_rf, 1);
						if (out_rf > 0){
							UaPutS("\r\n Carrier ON channel = 0x");
							hex2uart(ch,2);
							cnt=0xffe;
						}
						cnt++;
					}
					cnt=0;
					CE_LOW();
					Delay_us(150);
				}
			}
			
			u08 rx_buf[TX_PLOAD_WIDTH+1];
			void RX(void){
				RX_Mode();
				u08 sts=0,st=0;
				UaPutS("\r\n RX Mode ");
				//nRF24_writeReg(CONFIG, 0x0f);
				while(1){
					CE_HIGH();
					Delay_us(130);
					do
					{
						nRF24_readBuf(STATUS, &sts, 1);
					}while ((sts & RX_DR) == 0);
					CE_LOW();
					nRF24_readBuf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);
					//nRF24_writeReg(FLUSH_RX, 0x0);
					UaPutS("\r\n Reg(");
					hex2uart(st++,2);
					UaPutS(") = ");
					UaPutS((char *)rx_buf);
					UaPutC(' ');
					hex2uart(sts,2);
					//for(st=0;st<TX_PLOAD_WIDTH;st++){	hex2uart(rx_buf[st],2);		UaPutC(' ');}
					//Delay_us(150);
					nRF24_writeReg(STATUS, sts);
					Delay_us(10);
					
					//do{	nRF24_readBuf(STATUS, &sts, 1); }while(sts & RX_DR);
					//TIM6->CNT = 0;
					//UaPutS("\r\n TimDelay = ");
					//UaPutS(int2str(Tim6_,tbuf,6,1));
					//Tim6_ = ((TIM6->CNT)*5)/18;
					//hex2uart(Tim6_,4);
				}
			}
			
			while(1)
			{
				Menu();
				UaPutC('\f');
				switch (u1buf[0])			{
				case 'r': ReadNRF();			break;
				case 'w': WriteNRF();			break;
				case 'm': MultiNRF();			break;
				case 'o': RX();						break;
				case 'c': CX();						break;
				case 's': reset();				break;
				case 'n': TX_Mode();		break;
				case 'x': RX_Mode();			break;
				case 'z': OtherTest();		break;
				default: 									break;
				}
			}
			
		}
		void Console(void)		{
			u32 licz;
			volatile int Pom,Pom_=0,dY,cR,cG,cB,cC;
			//time_T UnixTime;
			u16 x=0;
			//  char tbuf[16];
			//  char cbuf[20];
			UaPutS("\f ");
			UaPutS("\r\n Start terminala  \r\n");
			// GPIOB->BSRR = (1<<1)|(1<<(0+16));
			UaPutS(" Compilation = ");
			uint2uart(BKP->DR42);
			//	int2str(BKP->DR42,tbuf);
			//	UaPutS(tbuf);
			int test;
			//	char *dczas = cbuf;
			cR=cG=0;
			cB=0;
			while(1) 
			{
				//	count = count_max;		do{count --;}while(count);
				//	UaPutC(0x0C);
				//	ADC1->CR2 |= ADC_CR2_ADON;			// start convertion
				//	while(!(ADC1->SR & ADC_SR_EOC));	// wait for the end of convertion
				Pom = 0;
				UaPutS("\f Time 32 ADC conversion = ");
				//	TIM6->PSC = 36;
				//	TIM6->CNT = 0;
				//	TIM6->CR1 |= TIM_CR1_CEN;
				//	uint2uart(TIM6->CNT);
				do{ licz--;
				ADC1_CR2_ADON = 1;					// start conversion ADC
				while(!ADC1_SR_EOC);				// wait for the end of convertion
				Pom += ADC1->DR;
				}
				while(licz);
				UaPutC(' ');
				//	uint2uart(TIM6->CNT);
				licz = 2;
				Pom = 100;
				Delay_ms(100);
				//		UaPutS("\f ADC = ");
				//		UaPutS(int2str(Pom,tbuf));
				UaPutC(' ');

				UaPutS("\r\n R+ = ");
				for(int i=0;i<32;i++){	// R+
					cR = i;
					cC = torgb(cR,cG,cB);
					TFT_Clear(cC);
					uint2uart(cC);
					Delay_ms(Pom);
				}
				UaPutS("\r\n RB- = ");
				for(int i=31;i>=0;i--){	//B-
					cB = i;
					cC = torgb(cR,cG,cB);
					TFT_Clear(cC);
					uint2uart(cC);
					Delay_ms(Pom);
				}
				UaPutS("\r\n RG+ = ");
				for(int i=0;i<64;i+=2){	//RG+
					cG = i;
					cC = torgb(cR,cG,cB);
					TFT_Clear(cC);
					uint2uart(cC);
					Delay_ms(Pom);
				}
				UaPutS("\r\n -RG = ");
				for(int i=31;i>=0;i--){	//-RG
					cR = i;
					cG = 63;
					cC = torgb(cR,cG,cB);
					TFT_Clear(cC);
					uint2uart(cC);
					Delay_ms(Pom);
				}
				UaPutS("\r\n GB+ = ");
				for(int i=0;i<32;i++){	//GB+
					cB = i;
					cC = torgb(cR,cG,cB);
					TFT_Clear(cC);
					uint2uart(cC);
					Delay_ms(Pom);
				}
				UaPutS("\r\n G-B = ");
				for(int i=63;i>=0;i-=2){	//G-B
					cG = i;
					cC = torgb(cR,cG,cB);
					TFT_Clear(cC);
					uint2uart(cC);
					Delay_ms(Pom);
				}
				
				//Rysuje wykres
				if(Pom-Pom_>0) 			dY=1;
				else if(Pom-Pom_==0)	dY=0;
				else 					dY=-1;
				do{
					TFT_DrawPoint(x,Pom_-16,torgb(Pom/8,Pom/4,Pom/8));
					//	TFT_Clear(torgb(Pom/8,Pom/4,Pom/8));
					Pom_ += dY;
				}
				while(Pom != Pom_);
				x++;
				if(x>320){
					TFT_Clear(BLACK);
					x=0;
				}
				
				/*
				UnixTime = RTC->CNTL + (RTC->CNTH<<16);
					gmtime_r (&UnixTime, &ptm);
				memcpy(dczas,"1982-01-01 11:4::10",20);
				memcpy(dczas,int2str(ptm.tm_year,tbuf),4); dczas += 5;
				memcpy(dczas,int2str(ptm.tm_mon,tbuf),2);  dczas += 3;
				memcpy(dczas,int2str(ptm.tm_mday,tbuf),2); dczas += 3;
				memcpy(dczas,int2str(ptm.tm_hour,tbuf),2); dczas += 3;
				memcpy(dczas,int2str(ptm.tm_min,tbuf),2);  
				if(ptm.tm_min > 9) dczas += 3; else dczas += 2;
				memcpy(dczas,int2str(ptm.tm_sec,tbuf),2);
				dczas = (char *)cbuf;
				UaPutS(cbuf);
				 */
				/*
				UaPutS(int2str(ptm.tm_year,tbuf));			UaPutC('-');
				UaPutS(int2str(ptm.tm_mon,tbuf));			UaPutC('-');
				UaPutS(int2str(ptm.tm_mday,tbuf));			UaPutC(' ');
				UaPutS(int2str(ptm.tm_hour,tbuf));			UaPutC(':');
				UaPutS(int2str(ptm.tm_min,tbuf));			UaPutC(':');
				UaPutS(int2str(ptm.tm_sec,tbuf));
				 */
				
				if(0) 
				{
					if(flag == 1)
					{
						UaPutS("\r\n Out = ");
						test = str2hex((char *)u1buf);
						//UaPutS(int2str(test,tbuf));
						UaPutC(test);
						//	UaPutS(int2str(str2int((char *)u1buf,(int *)test),tbuf));
						UaPutS("\r\n In  = ");
						flag = 0;
						memset((char *)u1buf,0,RBUF_SIZE);
					}
				}
			}
		}
		void Grfaphics(void)	{  // Rozne efekty graficzne
			u16 color=320;
			float a,x,y,x1=0,y1=120;
			
			TFT_Clear(BLACK);
			
			// 	TFT_SetWindows(50,50,100,100);
			// 	GUI_Text(0,0,"DEF",WHITE,RED);
			
			for (x=0;x<320;x++) {		/// demo 2 - rosnace sinusy
				y = x*sin((float)x/20)/4 + 120;
				TFT_DrawLine(x1,y1,x,y,RED);
				TFT_DrawLine(x1+50,y1,x+50,y,GREEN);
				TFT_DrawLine(x1+20,y1,x+20,y,BLUE);
				TFT_DrawPoint(x+10,y+10,YELLOW);
				TFT_DrawPoint(x+10,y+11,YELLOW);
				x1=x; y1=y;
			}
			//TFT_DrawLine(0, 0, 200, 200,GREEN);
			while(1);
		}
		void ClockGraph(void)	{
			time_T UnixTime;
			char tbuf[16];
			char *dczas = cbuf;
			if(ptm.tm_sec == 59){
				UnixTime = RTC->CNTL + (RTC->CNTH<<16);
				gmtime_r (&UnixTime, &ptm);
				//	GUI_Text(tX,tY,cbuf,BLACK,BLACK);
				memcpy(dczas,unt2str(ptm.tm_year,tbuf,4),4); dczas += 5;
				memcpy(dczas,unt2str(ptm.tm_mon,tbuf,2),2);  dczas += 3;
				memcpy(dczas,unt2str(ptm.tm_mday,tbuf,2),2); dczas += 3;
				memcpy(dczas,unt2str(ptm.tm_hour,tbuf,2),2); dczas += 3;
				memcpy(dczas,unt2str(ptm.tm_min,tbuf,2),2);  dczas += 3;
				memcpy(dczas,unt2str(ptm.tm_sec,tbuf,2),2);
			}
			else {
				ptm.tm_sec += 1;
				dczas += 17;
				memcpy(dczas,unt2str(ptm.tm_sec,tbuf,2),2);
			}
			dczas = (char *)cbuf;
			//	GUI_Text(tX,tY,dczas,YELLOW,BLACK);
			GUI_SclText(dczas, tX, tY + 40, 1, 2,YELLOW, BLACK);
			TFT_WriteScaledText(dczas, tX, tY, 1, 2, GREEN, BLACK);
		}
		u16  torgb(u08 R,u08 G,u08 B){
			bColor = B + (G<<5) + (R<<11);
			return bColor;
		}
	

void buble(u16* nodeX, int nodes) {
			/* Sort the nodes, via a simple "Bubble" sort. */
			u16 i = 0,swap;
			while (i < nodes-1)
			{
				if (nodeX[i]>nodeX[i+1])
				{
					swap = nodeX[i];
					nodeX[i] = nodeX[i+1];
					nodeX[i+1] = swap;
					if(i) { i--; }
				}
				else { i++; }
			}
		}
		

void TPD_window(u08 px, u08 py, u16 color){
		char str[10];
		
		//TFT_Fill(64*px+dm,64*py-8+dm,px*64+64-dm,py*64+64-8-dm,color);
		//TFT_txt(int2str(px,str,1,0),64*px+32,64*py+32, color);
		
		//TFT_Fill(40*px+2,40*py+2,px*40+38,py*40+38,color);
		TFT_txt (int2str(px,str,1,0),40*px+15,40*py+15, color);
		
	}

void TPDraw(void)			{  // rysuj na panelu dotykowym
			u32 Tx,Ty,px,py;
			char str[10];
			u08 lf1_tpirq=0;
			
			
			px = 0;
			py = 0;
			#define sp1	8
			
			Touch_Init();											
			//bublesort(tpY, 4);
			
			TFT_Clear(BLACK);
			for (u08 n=0;n<9;n++){
				
				TFT_Fill(n*40==320?319:n*40,0,n*40==320?319:n*40 ,239,YELLOW);
				
			}
			
			for(u08 n=0;n<7;n++){
			
				TFT_Fill(0,n*40==240?239:n*40,319,n*40==240?239:n*40,YELLOW);
			}
			
			for(u08 x=0;x<8;x++){
				for(u08 y=0;y<6;y++){
					TFT_txt(int2str(x,str,1,0),40*x+15,40*y+15, YELLOW);
				}
			}
			
			while(1){
				
				if(f1_tpirq){
					trg3 		   = SysTick->VAL;			// set timeout flag
					
					
					px = 0;
					py = 0;
					
					for(u16 x=0; x<8; x++){
						Read_ADS1(&Ty,&Tx); 
						px += Tx;
						py += Ty;
						//TFT_dy (x ,Py ,py ,GREEN,0 ); // Rysuj punkt wykresy
					}
					px /=8;
					py /=8;
					
					px = 350 - ((17*px)/(182));	
					py = 260 - ((17*py)/(252));
					
					px /= 40;
					py /= 40;
					
					f1_tpirq 	= 0;
					lf1_tpirq = 1;
					
					if((px < 8) && (py < 6)) {
						UaPutS("\r\n");
						sint2uart(px); sint2uart(py); 
						
						TPD_window( px, py, BLACK);
						//TFT_Fill(64*px+dms,64*py-8+dms,px*64+64-dms,py*64+64-8-dms,GREEN);
					}
				}
				
				if(lf1_tpirq && (STCLK_MS*300 < (trg3 - SysTick->VAL)))		{ // timeout repeat touchpad
					LED2 = 0;
					bbEXTI_FTSR_6 	= 1;						// Enabled failing triger 
					lf1_tpirq 		 	= 0;
					
					//TFT_Fill(64*px+dms,64*py-8+dms,px*64+64-dms,py*64+64-8-dms,BLACK);
					TPD_window( px, py, GREEN);
					
				}
			}
			
			
		}
#endif
//============================================================================
// IRQ Handler
//============================================================================
void CopyClock(char *dczas, T_RTC *lrtc)	{
		char tbuf[5];
		//u32 utc = RTC->CNTL + (RTC->CNTH<<16);
		//rtc_unix2time (utc, rtc);
		//rtc_GetTime(0, lrtc);
		//char tbuf[20]={"00:00:00 0000-00-00"};
		dczas += 0;	memcpy(dczas,unt2str(lrtc->yer,tbuf,4) ,4);
		dczas += 5; memcpy(dczas,unt2str(lrtc->mon,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->mdy,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->hor,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->min,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->sec,tbuf,2) ,2);
		
	}
void CopyTime(char *dczas,  T_RTC *lrtc)	{
		char tbuf[2];
		//rtc_GetTime(0, lrtc);
		//rtc_GetTim2(0, lrtc);
		
		dczas += 0; memcpy(dczas,unt2str(lrtc->hor,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->min,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->sec,tbuf,2) ,2);
	}
void CopyDate(char *dczas,  T_RTC *lrtc)	{
		char tbuf[5];
		//rtc_GetTime(0, lrtc);
		dczas += 0;	memcpy(dczas,unt2str(lrtc->yer,tbuf,4) ,4);
		dczas += 5; memcpy(dczas,unt2str(lrtc->mon,tbuf,2) ,2);
		dczas += 3; memcpy(dczas,unt2str(lrtc->mdy,tbuf,2) ,2);
	}
void ExtIInit				(void)			{
		/*
			Library ST				 Manual RM0008
			AFIO->EXTICR[0] => AFIO_EXTICR1 GPIO 0..3			
			AFIO->EXTICR[1] => AFIO_EXTICR2 GPIO 4..7
			AFIO->EXTICR[2] => AFIO_EXTICR3 GPIO 8..11
			AFIO->EXTICR[3] => AFIO_EXTICR4 GPIO 12..15
			Jednoczenie pin moze byc skonfigurowany tylko dla jednego portu
		 */
		// These bits is software to select the source input for EXTIx external interrupt.
		
		//AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PE;		// Enable interrupt port PE
		//AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;		// Enable interrupt port PB
		AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;			// Enable interrupt port PA
		//AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;		// Enable interrupt port PA
		//EXTI->PR   = EXTI_PR_PR4;			// Clear interupt request EXTI
		// Interrupt Mask on line x
		// EXTI->PR = EXTI_PR_PR6;		// Clear interupt request EXTI
		//EXTI->PR   = EXTI_PR_PR4;			// Clear interupt request EXTI
		
		// Rising trigger event configuration bit of line x
		// EXTI->RTSR 			|= EXTI_RTSR_TR2|EXTI_RTSR_TR6|EXTI_RTSR_TR13;
		// EXTI->RTSR 			|= EXTI_RTSR_TR6;
		EXTI->RTSR 	 = EXTI_RTSR_TR3;
		
		// Falling trigger event configuration bit of line x
		//EXTI->FTSR 		= EXTI_FTSR_TR3 ;	// Failing trigger enabled
		
		// Interrupt mask register EXTI_IMR
		//EXTI->IMR 			=  EXTI_IMR_MR6 | EXTI_IMR_MR0;
		//EXTI->IMR  =  EXTI_IMR_MR3;		// Interrupt mask register EXTI_IMR
			EXTI->EMR  =  EXTI_EMR_MR3;		// Event mask register EXTI_IMR
		
	}
void SysTick_Handler(void)			{
		
		register uint32_t N;
		register int32_t n;
		
		//	disk_timerproc();
		//Timt += STCLK_LD;
		N = Timt + STCLK_LD; Timt = N;
		
		n = delay_us + STCLK_LD; delay_us = n;
		//n = Timer1; if (n) Timer1 = --n;
		//n = Timer2; if (n) Timer2 = --n;
		
		n = (Tim0 + STCLK_LD); Tim0 = n;
		n = (Tim1 + STCLK_LD); Tim1 = n;
		
		n = (trg0 + STCLK_LD); trg0 = n;
		n = (trg1 + STCLK_LD); trg1 = n;
		n = (trg2 + STCLK_LD); trg2 = n;
		n = (trg3 + STCLK_LD); trg3 = n;
		n = (trg4 + STCLK_LD); trg4 = n;
		n = (trg5 + STCLK_LD); trg5 = n;
		//LED1 ^= 1;
		
	}
__attribute__((interrupt)) void TIM2_IRQHandler(void)				{
		bTIM2_SR_UIF = 0;
		TIM2->SR = 0;
		
	}
__attribute__((interrupt)) void TIM3_IRQHandler(void)				{
		TIM3->SR = 0;
		register uint32_t N;
		register int32_t  n;
		
		N = Timt3     + TIM3_ARR; Timt3 = N;
		n = delay3_us + TIM3_ARR; delay3_us = n;
		//n = Timer1; if (n) Timer1 = --n;
		//n = Timer2; if (n) Timer2 = --n;
		
		n = (t3rg0 + TIM3_ARR); t3rg0 = n;
		n = (t3rg1 + TIM3_ARR); t3rg1 = n;
		n = (t3rg2 + TIM3_ARR); t3rg2 = n;
		
		//LED^=1;
		
	}
__attribute__((interrupt)) void EXTI0_IRQHandler(void)			{
		if(PB0_i){
			flag_9=1;
		}
		EXTI->PR   = EXTI_PR_PR0;				// Clear interupt request
	}
__attribute__((interrupt)) void EXTI1_IRQHandler(void)			{
		//bEXTI_PR_0 = 1;
		EXTI->PR   = EXTI_PR_PR1;				// Clear interupt request
		LED ^=1;
	}
__attribute__((interrupt)) void EXTI2_IRQHandler(void)			{
		bEXTI_PR_2 = 1;
		LED ^= 1;
	}
__attribute__((interrupt)) void EXTI3_IRQHandler(void)			{
		EXTI->PR   = EXTI_PR_PR3;				// Clear interupt request
		LED ^=1;
	}
__attribute__((interrupt)) void EXTI4_IRQHandler(void)			{
		EXTI->PR   = EXTI_PR_PR4;			
		LED ^= 1;
	}
__attribute__((interrupt)) void EXTI15_10_IRQHandler(void)	{
		if(bEXTI_PR_13) {
			bEXTI_PR_13 = 1;
			//LED2 = ~LED2;
		}
		
	}
__attribute__((interrupt)) void EXTI9_5_IRQHandler	 (void)	{
		if((bPBI_6 == 0)&&(EXTI->PR & EXTI_PR_PR6)) {			// check request PB6 touchpad
			//__disable_irq();
			//NVIC_DisableIRQ(EXTI9_5_IRQn);
			f1_tpirq   = 1;									// set IRQ_TP FLAG
			EXTI->PR   = EXTI_PR_PR6;				// Clear interupt request
			//bbEXTI_PR_6   = 1;
			//LED1 = ~LED1;								 	// debug
			EXTI->FTSR &= ~EXTI_FTSR_TR6;  	// Failing trigger disabled
			//bbEXTI_FTSR_6 	= 0;			
			//EXTI->FTSR 	 |= EXTI_FTSR_TR6;
		}
	}
__attribute__((interrupt)) void RTC_IRQHandler 	(void)			{
		// LED1 = ~LED1;
		// flag |= (1<<1);
		bRTC_CRL_SECF = 0;		//Clear Interrupt Flag bit
		f3_rtc_1sec = 1;
		while( bRTC_CRL_RTOFF == 0 );
	}
__attribute__((interrupt)) void RTCAlarm_IRQHandler(void){
		RTC->CRL &= ~(RTC_CRL_ALRF); // potrzebne do wakeup
		EXTI->PR |= EXTI_PR_PR17;
		//LED ^= 1;
		
	}
__attribute__((interrupt)) void DMA1_Channel1_IRQHandler(void){
		if(DMA1->ISR & DMA_ISR_GIF1){
			//DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1;
			DMA1->IFCR =                  DMA_IFCR_CTCIF1;
			//ADC1_CR2_ADON  = 0;					// ADC1 STOP
			//DMA1_Ch1_EN_bb = 0;					// DMA1_CH1 STOP
			f7_dma1 = 1;
			LED1   ^= 1;
			//UaPutC('.');
		}
		
	}
__attribute__((interrupt)) void FSMC_IRQHandler	(void)			{
	;
}
//============================================================================
// Init Function
//============================================================================
void ADC1_DMA_Tim4_Init(void)		  {		
		//================ DMA Configuration =============================
		RCC->AHBENR   |= RCC_AHBENR_DMA1EN;					// enable clock for DMA1
		DMA1_Channel1->CCR  = 0;										// Disable channel
		//DMA1_Channel1->CMAR = (uint32_t)dma_buf;	// Destination address:
		DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;	// Source address:
		
		DMA1_Channel1->CCR =  DMA_CCR1_PL_0 | DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0 | DMA_CCR1_TCIE 
		| DMA_CCR1_MINC | DMA_CCR1_CIRC;							
		//DMA1_Channel1->CCR |= DMA_CCR1_EN;				//DMA1_CH1 START
		
		
		//================= Timer_3 Configuration ==================================
		/*RCC->APB1ENR = RCC_APB1ENR_TIM3EN;
		TIM3->PSC = 100-1;
    TIM3->ARR = 120-1;
    //TIM3->EGR = TIM_EGR_UG;
    TIM3->CR2 = TIM_CR2_MMS_1; */
		//================= Timer_4 Configuration ==================================
		//GPIOB->CRH   |= (GPIOB->CRH & 0xfffffff0) | 0x00000000;	// PB9 out
		GPIOB->CRH   |= 0x00000090;	// PB9 out
		Delay_us(10);
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		TIM4->PSC   = 100-1;
    TIM4->ARR   = 120-1;
    TIM4->CCR4  = 10;		//width pulse on PB9 out channel
		TIM4->CCMR1 = 0;
		TIM4->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
		TIM4->CCER  = TIM_CCER_CC4E;
		//Delay_us(10);
		
		//================= ADC Configuration ====================================
		RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;			// enable clock for ADC1
		RCC->CFGR 		|= RCC_CFGR_ADCPRE_DIV4;  	// 2/4/6/8 str98
		GPIOA->CRL = (GPIOA->CRL & 0xffff0000);
		//ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_JEXTTRIG | ADC_CR2_JEXTSEL_1; // TIM2
		
		ADC1->CR1 |= ADC_CR1_SCAN;	// multi channel mode
		ADC1->CR2 =  ADC_CR2_DMA | ADC_CR2_ADON | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0;
		Delay_us(1);
		//NVIC->ISER[0] |= NVIC_ISER_SETENA_18;
		
		//Kana³y czytane w odwrotnej kolejnoci do parametru "sequence"
		ADC1->SQR1  = 0<<20;
		//ADC1->SQRx length and sequence (channel << sequence*5)
		//ADC1->SQR3  = 0<<3*5 | 3<<2*5 | 2<<1*5 | 1<<0*5;
		ADC1->SQR3  = 0<<1*5 | 1<<0*5 ;
		//ADC1->SMPRx sample time (cycles_time(0..7) << channel*3)
		ADC1->SMPR2 = 7<<0*3 | 7<<1*3 ;
		
		ADC1->CR2 |= ADC_CR2_RSTCAL;					// reset calibration
		while(ADC1->CR2 & ADC_CR2_RSTCAL);		// wait for reset
		ADC1->CR2 |= ADC_CR2_CAL;							// start calibration
		while(ADC1->CR2 & ADC_CR2_CAL);				// wait till calibration is done
		ADC1->CR2 |= ADC_CR2_ADON;						// start conversion
		while( !( ADC1->SR & ADC_SR_STRT ));
		
		//TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
		TIM3->CR1 = TIM_CR1_CEN;
		while( ! (ADC1->SR & ADC_SR_EOC) );
		
		
	}
void ADC1Init(void)				{
		
		RCC->APB2ENR   |= RCC_APB2ENR_ADC1EN;	// enable clock for ADC1
		RCC->CFGR 		 |= RCC_CFGR_ADCPRE_DIV4;  // 2/4/6/8 str98
		RCC_APB2RSTR_ADC1RST_bb = 1;		// Reset ADC1
		RCC_APB2RSTR_ADC1RST_bb = 0;
		
		///GPIOA->CRL = (GPIOA->CRL & 0xffffff00);
		//GPIOA->BSRR  =   0x1<<1+16;	// reset
		//GPIOA->BSRR  =   0x1<<1+0;		// set
		
		//ADC1->SQRx length and sequence (channel << sequence*5)
		///ADC1->SQR1 = ( 1<<20 	); 		//length and sequence 12..15 conversion 13-16 on channel	(0..17)
		//ADC1->SQR2 = ( 0<<0*5 ); 	//sequence 6..11 conversion (7..12) on channel (0..17)
		///ADC1->SQR3 = ( 0<<0*5 | 1<<1*5 );		//(channel<<sequence*5) sequence=0..5 conversions (1..6)  
		
		//ADC1->SMPRx sample time (cycles_time(0..7) << channel*3)
		///ADC1->SMPR2 = (4<<0*3 | 4<<1*3);
		///ADC1->CR1 = ADC_CR1_SCAN;			// multi channel mode
		//ADC1->CR2 |= ADC_CR2_CONT ;	// Continuous mode
		///ADC1->CR2 |= ADC_CR2_ADON ;		// On ADC
		
		/*
		ADC1->CR2 |= ADC_CR2_RSTCAL;					// reset calibration
		while(ADC1->CR2 & ADC_CR2_RSTCAL);		// wait for reset
		
		ADC1->CR2 |= ADC_CR2_CAL;							// start calibration
		while(ADC1->CR2 & ADC_CR2_CAL);				// wait till calibration is done
		
		ADC1->CR2 |= ADC_CR2_ADON;						// start conversion
		while( !( ADC1->SR & ADC_SR_STRT ));
		*/
		//while(!(ADC1->SR & ADC_SR_EOC));		// wait for the end of convertion
		//ADC1->DR;														// do the dummy read
		
	}
void ADC1DmaInit(void)		{
		
		#define ADC1_Cycle_Time 7			// Czas konwersji  0-7
		#define ADC1_Num_Seq 		4			// Liczba pomiarow 1-16
		
		//================ RCC Configuration =============================
		RCC->AHBENR  |= RCC_AHBENR_DMA1EN;				// enable clock for DMA1
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;			// enable clock for ADC1
		RCC->CFGR 	 |= RCC_CFGR_ADCPRE_DIV2;  	// 2/4/6/8 str98
		
		//================ TIMER3 Configuration ==========================
		
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		TIM2->EGR = TIM_EGR_CC2G | TIM_EGR_TG;
		TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
		TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
		
		TIM2->CCER &= ~TIM_CCER_CC2P;
		TIM2->CCER |= TIM_CCER_CC2E;
		TIM2->CR2  |= TIM_CR2_MMS_1 | TIM_CR2_MMS_0;
		
		TIM2->CR1 = 0;
		TIM2->PSC = 60000-1;
		TIM2->CCR2= 0x0F;
		TIM2->ARR = 10000;
		TIM2->CR1 |= TIM_CR1_CEN;
		
		//================ DMA Configuration =============================
		uint32_t CCR_reg = 0;
		DMA1_Channel1->CCR = 0;											//Disable channel
		//DMA1_Channel1->CMAR = (uint32_t)dma_buf;	 	//Destination address:
		DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;		//Source address:
		
		//DMA1_Channel1->CNDTR = 4;									//Buffor size:
		CCR_reg |=  DMA_CCR1_PL_0;									//Priorytet medium
		CCR_reg |=  DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0;	// 16bit/16bit
		//CCR_reg |=  DMA_CCR1_MSIZE_0;							//16bit/8bit
		CCR_reg |=  DMA_CCR1_MINC;									//Increment memory address enabled
		//CCR_reg |=  DMA_CCR1_PINC;								//Increment peripheral address disabled
		//CCR_reg |=  DMA_CCR1_CIRC;								//Mode Circular mode enabled
		CCR_reg |=  DMA_CCR1_TCIE;									//Transfer complete interrupt enable
		
		DMA1_Channel1->CCR =  CCR_reg;							
		//DMA1_Channel1->CCR |= DMA_CCR1_EN;				//DMA1_CH1 START
		NVIC_EnableIRQ(DMA1_Channel1_IRQn);				  // DMA1 Channel1
		
		//================ ADC Configuration =============================
		
		RCC_APB2RSTR_ADC1RST_bb = 1;							// Reset ADC1
		RCC_APB2RSTR_ADC1RST_bb = 0;
		
		GPIOA->CRL = (GPIOA->CRL & 0xffff0000);
		GPIOA->CRL = (GPIOA->CRL | 0x00000000);
		//ADC1->SQRx length and sequence (channel << sequence*5)
		ADC1->SQR1 = ( (ADC1_Num_Seq-1) << 20 ); 		//length and sequence chann 		 13-16
		//ADC1->SQR2 = ( 0<<0*5   ); 								//sequence conversion on channel 7-12
		ADC1->SQR3 =  (0<<0*5) | (1<<1*5) |(2<<2*5) | (3<<3*5) ;	//sequence conversion on channel 1-6
		//ADC1->SMPRx sample time (cycles_time << channel*3)
		ADC1->SMPR2 = (ADC1_Cycle_Time<<0*3) | (ADC1_Cycle_Time<<1*3) |	
		(ADC1_Cycle_Time<<2*3) | (ADC1_Cycle_Time<<3*3);
		ADC1->CR1  = ADC_CR1_SCAN;		// multiple channel				
		//ADC1->CR2 |= ADC_CR2_EXTTRIG|ADC_CR2_EXTSEL_0|ADC_CR2_EXTSEL_1|ADC_CR2_EXTSEL_2; //TIM2 triger
		//ADC1->CR2 |= ADC_CR2_CONT ;		//Continuous mode 
		//ADC1->CR2 |= ADC_CR2_DMA;			//DMA request on
		ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0
		| ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2; //TIM2 triger
		
		if(!(ADC1->CR2 & ADC_CR2_ADON)) ADC1->CR2 |= ADC_CR2_ADON;
		
		ADC1->CR2 |= ADC_CR2_RSTCAL;				// reset calibration
		while(ADC1->CR2 & ADC_CR2_RSTCAL);	// wait for reset
		ADC1->CR2 |= ADC_CR2_CAL;						// start calibration
		while(ADC1->CR2 & ADC_CR2_CAL);			// wait till calibration is done
		
		ADC1->CR2 |= ADC_CR2_ADON;					// start conversion
		while( !( ADC1->SR & ADC_SR_STRT ));
		
	}
void DACDmaInit(void)			{
	RCC->APB1ENR  |= RCC_APB1ENR_DACEN;
	GPIOA->CRL   	&= 0xff00ffff;
	GPIOA->CRL   	|= 0x00dd0000;
	//DAC->CR				= DAC_CR_TSEL1|DAC_CR_TEN1|BOFF1;
	//DAC->CR				= DAC_CR_EN1|DAC_CR_EN2;
	DAC->CR				= DAC_CR_TEN1|DAC_CR_TEN2|DAC_CR_WAVE1|DAC_CR_WAVE2|DAC_CR_EN1|DAC_CR_EN2|
			DAC_CR_TSEL1_1|DAC_CR_TSEL2_1|(10<<24)|(10<<8);
	//DAC->DHR12R1	= 0;
	//DAC->DHR12R2	= 0;
	DAC->DHR12RD	= (0<<16) | 900;
	//DAC->SWTRIGR	= DAC_SWTRIGR_SWTRIG1|DAC_SWTRIGR_SWTRIG2;
}
void Uart1Init(void)			{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	GPIOA->CRH = (GPIOA->CRH & 0xfffff00f) | 0x00000490; // PA9=TX, PA10=RX
	USART1->CR1 |= (USART_CR1_UE |USART_CR1_TE | USART_CR1_RE	|USART_CR1_RXNEIE);
	USART1->BRR = (F_CPU+UART1_SPEED/2)/UART1_SPEED;
	NVIC_EnableIRQ(USART1_IRQn);
}
void Uart2Init(void)			{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOA->CRL = (GPIOA->CRL & 0xffff00ff) | 0x00004900; // PA2=TX, PA3=RX 
	USART2->CR1 |= (USART_CR1_UE |USART_CR1_TE | USART_CR1_RE	|USART_CR1_RXNEIE);
	USART2->BRR = ((F_CPU+UART2_SPEED/2)/UART2_SPEED)/2;
	NVIC_EnableIRQ(USART2_IRQn);
}
void Uart3Init(void)			{
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	GPIOB->CRH  = (GPIOB->CRH & 0xffff00ff)| 0x00004900; //PB10=TX, PB11=RX zerowanie
	USART3->CR1 |= (USART_CR1_UE |USART_CR1_TE | USART_CR1_RE	|USART_CR1_RXNEIE);
	USART3->BRR = ((F_CPU+UART3_SPEED/2)/UART3_SPEED)/2;
	//NVIC_EnableIRQ(USART3_IRQn);
}
void Tim2PWMInit(void)		{
	RCC->APB1ENR |=(RCC_APB1ENR_TIM2EN); 	//w³¹czenie portu A oraz TIMERA2
	//GPIOA->CRL   = 0x444444AA; 						//ustawienie pir_dma_buf 0,1  portu A na wyjscie
	TIM2->CR1 	= 0x0000;	//Counter disabled			0x000
	TIM2->PSC 	= 0xffff;	//set prescaler				0x0ff
	TIM2->ARR 	= 0x00ff;	//set PWM reload count		0xfff
	TIM2->CCR1 	= 0x004f;	//set PWM start value 50%	0x07f
	TIM2->CCR2 	= 0x004f;	//set PWM start value 50%	0x07f
	TIM2->CCMR1 = 0x6868;	//set PWM mode on CH1,CH2	0x06868
	TIM2->CCER 	= 0x0011;	//enable CH1,CH2 output		0x101
	TIM2->DIER 	= 0x0000;	//enable update interrupt	0x000
	TIM2->DIER = TIM_DIER_UIE; 	// set interrupt
	TIM2->EGR 	= 0x0001;				// enable update		0x001
	//TIM2->CR1 	= (TIM_CR1_CEN | TIM_CR1_URS); 
	//w³¹cz. licznika i w³¹cz. generowania przerwañ tylko podczas prze³adowania licznika
	TIM2->CR1 	= 0x0001;	//Counter enabled			0x001
}
void Tim4Init(void)				{
	// Enable Peripheral Clock

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	// De-init TIMER peripheral registers to their default reset values.
	RCC_APB1RSTR_TIM6RST_bb = 1;
	RCC_APB1RSTR_TIM6RST_bb = 0;

	// Clock Division = 0
	// Auto-Reload Preload = NOT buffered
	// Edge-Aligned mode
	// Direction = UP
	// One Pulse Mode = Off
	// Update Request can be anything
	// Update Event = Enabled
	// Counter Disabled (for r_dma_buf)
	TIM6->CR1 = 0;

	// Free-Running Period
	TIM6->ARR = 0xFFFFu;
	// Time-Base configuration - Scale to fixed Frequency of 1MHz
	// TIM6->PSC = (36000000 / TIMER_F_CPU) - 1;
	TIM6->PSC = (F_CPU/(10000000ul)) - 1;	// Resolution 0.1 us //36-1=1us
	// No Repetition Counter
	TIM6->RCR = 0;

	TIM6->DIER = TIM_DIER_UIE;
	// Force an update to load our ARR and Pre-Scalar shadow registers
	TIM6->EGR = TIM_EGR_UG;
	// Enable the Counter
	TIM6->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM6_IRQn);
}
void TIM4_IRQHandler(void){
		TIM4->SR = 0;
		
		Tim4++;
	}
void Tim3Init(void)				{
		// Timeout for EN28j60 Ethernet module
		
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		// De-init TIMER peripheral registers to their default reset values.
		//RCC_APB1RSTR_TIM7RST_bb = 1;
		//RCC_APB1RSTR_TIM7RST_bb = 0;
		
		TIM3->CR1 = 0;
		TIM3->PSC = (F_CPU/(1000000)) - 1;
		TIM3->ARR = TIM3_ARR - 1;
		
		TIM3->DIER = TIM_DIER_UIE;
		TIM3->CR1  |= TIM_CR1_DIR;
		TIM3->CR1  |= TIM_CR1_CEN;
		NVIC_EnableIRQ(TIM3_IRQn);
		
	}
void SysTickInit(void)		{
	SysTick->LOAD = STCLK_LD - 1;    // 72/8/SysTick_LOAD interrupt every x s
	SysTick->CTRL = SysTick_CTRL_TICKINT|SysTick_CTRL_ENABLE; // enable SysTick
}
void adc1_init_injected(uint32_t conversions,uint32_t channel)	{

#define 	ADC_CR2_JEXTSEL_JSWSTART  ADC_CR2_JEXTSEL
#define 	ADC_JSQR_JL_bit		20
#define 	ADC_JSQR_JSQ4_bit	15
#define 	ADC_JSQR_JSQ3_bit	10
#define 	ADC_JSQR_JSQ2_bit	5
#define 	ADC_JSQR_JSQ1_bit	0

	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV2;		// divide the ADC clock by 6 -> f_SAMP = 0.85MHz
	RCC->APB2ENR   |= RCC_APB2ENR_ADC1EN;	// enable clock for ADC1
	//ADC1->SMPRx sample time (cycles_time_bit << channel*3)
	ADC1->SMPR2 = (0<<channel*3);

	ADC1->JSQR = ((conversions - 1) << ADC_JSQR_JL_bit) |	// configure desired number of conversions
			(channel << ADC_JSQR_JSQ4_bit) |
			(channel << ADC_JSQR_JSQ3_bit) |
			(channel << ADC_JSQR_JSQ2_bit) |
			(channel << ADC_JSQR_JSQ1_bit);
	// enable ADC, start calibration, enable external software trigger for Injected Group
	ADC1->CR2 = ADC_CR2_JEXTSEL_JSWSTART | ADC_CR2_JEXTTRIG | ADC_CR2_CAL | ADC_CR2_ADON;
	//ADC1->CR2 = ADC_CR2_CAL | ADC_CR2_ADON | ADC_CR2_CONT;
	ADC1_CR1_SCAN_bb = 1;									// enable scan mode (multi channel mode)

	ADC1_CR2_JSWSTART_bb = 1;						// start Injected Group
	while (ADC1_SR_JSTRT_bb == 0);			// wait for conversion to start
	//ADC1_SR_JSTRT_bb = 0;								// reset "Injected Group start of conversion" flag

	//if (channels & (ADC_INIT_CH16 | ADC_INIT_CH17))	// if temperature sensor or internal voltage regulator channels are selected...
	//ADC1_CR2_TSVREFE_bb = 1;						// ... enable them

	while (ADC1_CR2_CAL_bb == 1);					// wait for calibration end
}
uint32_t adc_get_internals(uint32_t avg_cycles)		{

	static volatile uint32_t * const jdrs[] = {&ADC1->JDR1, &ADC1->JDR2, &ADC1->JDR3, &ADC1->JDR4};
	uint32_t i, value = 0;

	ADC1_CR2_JSWSTART_bb = 1;						// start Injected Group
	while (ADC1_SR_JSTRT_bb == 0);			// wait for conversion to start
	ADC1_SR_JSTRT_bb = 0;								// reset "Injected Group start of conversion" flag
	while (ADC1_SR_JEOC_bb == 0);				// wait for conversions to end
	ADC1_SR_JEOC_bb = 0;								// reset "end of conversion" flag

	for (i = 0; i < avg_cycles; i++)		// accumulate desired number of samples
		value += *jdrs[i];

	if (avg_cycles > 1)									// should the samples be averaged?
	{
		//value += (avg_cycles / 2);				// yes - calculate the rounded average
		value /= avg_cycles;
	}

	return value;
}
uint32_t adc_get_injected(uint32_t avg_cycles)		{

	static volatile uint32_t * const jdrs[] = {&ADC1->JDR1, &ADC1->JDR2, &ADC1->JDR3, &ADC1->JDR4};
	uint32_t i, value = 0;

	ADC1_CR2_JSWSTART_bb = 1;
	while (ADC1_SR_JEOC_bb == 0);				// wait for conversions to end
	ADC1_SR_JEOC_bb = 0;								// reset "end of conversion" flag

	for (i = 0; i < avg_cycles; i++)		// accumulate desired number of samples
		value += *jdrs[i];

	if (avg_cycles > 1)									// should the samples be averaged?
	{
		//value += (avg_cycles / 2);				// yes - calculate the rounded average
		value /= avg_cycles;
	}

	return value;
}
void NVIC_conf(void)	{
#define AIRCR_VECTKEY_MASK   ((uint32_t)0x05FA0000)
	//---------------------------------------------------------
	SCB->AIRCR |= AIRCR_VECTKEY_MASK | (3<<8);      //[Optional] Define interrupt fielding group, PRIGROUP[1:0]=11. see: PM0056 Programming Manual, page 135
	//---------------------------------------------------------

	NVIC->IP[ADC1_2_IRQn]  = (2<<4);
	NVIC->IP[DMA1_Channel1_IRQn] = (1<<4);
	NVIC->IP[TIM1_CC_IRQn] = (0<<4);           //Set the highest priority for TIMER1, see: PM0056 Programming Manual, page 126
	//for IP[IQRn] see vector table in reference manual (position), Chapter: 9.Interrupts and events

	NVIC->ISER[0]= (1<<TIM1_CC_IRQn )|(1<<ADC1_2_IRQn)|(1<<DMA1_Channel1_IRQn);   //Enable interrupt for TIMER1 CC mode, see: PM0056 Programming Manual, page 121
}
//============================================================================
// Tools Function
//============================================================================
void SetDate(void)	{
	char tbuf[16],u1buf[16];
	//Wprowadzanie pelnej daty i czasu
	UaPutS("\r\n rok = ");
	u16 yer = str2int((char *)u1buf);
	memset((char *)u1buf,0,16);
	UaPutS("\r\n mies = ");
	u08 mon = str2int((char *)u1buf);
	memset((char *)u1buf,0,16);
	UaPutS("\r\n dzien = ");
	u08 day = str2int((char *)u1buf);
	memset((char *)u1buf,0,16);
	UaPutS("\r\n gdz = ");
	u08 hor = str2int((char *)u1buf);
	memset((char *)u1buf,0,16);
	UaPutS("\r\n min = ");
	u08 min = str2int((char *)u1buf);
	memset((char *)u1buf,0,16);
	UaPutS("\r\n sek = ");
	u08 sec = str2int((char *)u1buf);
	memset((char *)u1buf,0,16);
	u32 Uxt = 1924984799;
	do{
		gmtime_r (&Uxt, &ptm);
		if(ptm.tm_year == yer) 	{
			if(ptm.tm_mon == mon) 	{
				if(ptm.tm_mday == day)  {
					if(ptm.tm_hour == hor)  {
						if(ptm.tm_min == min) 	{
							if(ptm.tm_sec == sec)		{
								gmtime_r (&Uxt, &ptm);
								UaPutS("\r\n ");
								UaPutS(uint2str(ptm.tm_year,tbuf));		UaPutC('-');
								UaPutS(uint2str(ptm.tm_mon,tbuf));		UaPutC('-');
								UaPutS(uint2str(ptm.tm_mday,tbuf));		UaPutC(' ');
								UaPutS(uint2str(ptm.tm_hour,tbuf));		UaPutC(':');
								UaPutS(uint2str(ptm.tm_min,tbuf));		UaPutC(':');
								UaPutS(uint2str(ptm.tm_sec,tbuf));		UaPutS("\r\n TimStmp = ");
								UaPutS(uint2str(Uxt,tbuf));
								break;
							} else Uxt --;
						}else Uxt -= 60;
					}else Uxt -= 3600;
				} else Uxt -= 86400;
			} else Uxt -= 86400;
		} else Uxt -= 86400;
	}
	while(Uxt);
}
void gmtime_r(const u32 *timer, struct tm *tmbuf)	{
	u32 time = *timer;
	u32 dayclock, dayno;
	int year = EPOCH_YR;

	dayclock = (u32) time % SECS_DAY;
	dayno = (u32) time / SECS_DAY;

	tmbuf->tm_sec = dayclock % 60;
	tmbuf->tm_min = (dayclock % 3600) / 60;
	tmbuf->tm_hour = dayclock / 3600;
	tmbuf->tm_wday = (dayno + 4) % 7; // Day 0 was a thursday
	while (dayno >= (u32) YEARSIZE(year))
	{
		dayno -= YEARSIZE(year);
		year++;
	}
	tmbuf->tm_year = year - YEAR0;
	tmbuf->tm_yday = dayno;
	tmbuf->tm_mon = 0;
	while (dayno >= (u32) _ytab[LEAPYEAR(year)][tmbuf->tm_mon])
	{
		dayno -= _ytab[LEAPYEAR(year)][tmbuf->tm_mon];
		tmbuf->tm_mon++;
	}
	tmbuf->tm_mday = dayno + 1;
	tmbuf->tm_isdst = 1;
	tmbuf->tm_hour += TIMEZONE - tmbuf->tm_isdst;
	tmbuf->tm_year += YEAR0;
	tmbuf->tm_mon  += 1;
	//  return tmbuf;
}
void flash_latency(uint32_t frequency) {
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
static void rcc_init(void){
	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
	//RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	//RCC->APB2ENR   |= RCC_APB2ENR_AFIOEN;     	// enable clock for alternate function
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
