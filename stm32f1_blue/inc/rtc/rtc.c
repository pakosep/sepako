#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"
#include "rtc.h"
#include "inc/tool/UARTLib.h"

//const u08 _ytab[2][12] ={
//  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
//  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};

const u08 samurai[] = {31,28,31,30,31,30,31,31,30,31,30,31};
extern T_RTC Rtc;

void BkpRegInit(void)		{
		RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN ;  //Enable Power and clocks
		bPWR_CR_DBP = 1;			   //Enable access to the Backup register and RTC;
		//BKP->DR42 ++;
	}
	
void RtcInit(void)						{
		RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN ;  //Enable Power and clocks
		//bPWR_CR_DBP = 1;			   							//Enable access to the Backup and RTC registers		
		PWR->CR = PWR_CR_DBP;
    //BKP->CR = BKP_CR_TPAL | BKP_CR_TPE;		// Tamper enable ( PINC_13 only input !!!)
		
		RCC->BDCR |= RCC_BDCR_LSEON  ; 					//start RTC must be after PWR->CR
		while( ~RCC->BDCR & RCC_BDCR_LSERDY );
		RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_0 ;
		
		/* lub
		RCC->BDCR = RCC_BDCR_RTCEN | RCC_BDCR_LSEON | RCC_BDCR_RTCSEL_0 ; 
		while( ~RTC->CRL & RTC_CRL_RTOFF );
		while(!bRTC_CRL_RTOFF);
		*/
		// Ustawienie daty i czasu - poczatek
		
		u32 uxt;
		Rtc.yer=2019;	// 1970..2106 
		Rtc.mon=9;	// 1..12 
		Rtc.mdy=12;	// 1..31 
		Rtc.hor=10;	// 0..23 
		Rtc.min=46;	// 0..59 
		Rtc.sec=45;	// 0..59 
		Rtc.wdy=4;	// 0..6 (Niedziela..Sobota) 
		rtc_time2unix ( &uxt, &Rtc);
		
		while(!bRTC_CRL_RTOFF); 	//wait to synchro
		bRTC_CRL_CNF = 1;					//Start edit mode
		RTC->PRLL = 0x7ffe;				//Set prescaler		ones
		RTC->CNTL = uxt&0xffff;		//Set time low		set counter/point time
		RTC->CNTH = (uxt>>16);		//Set time high		set counter/point time		
		bRTC_CRL_CNF = 0;					//End edit mode
		while(!bRTC_CRL_RTOFF); 	//wait to synchro
		
		// Ustawienie daty i czasu - koniec 
		
		RTC->CRH = RTC_CRH_SECIE;		// Set time interrupt
		NVIC_EnableIRQ(RTC_IRQn);		// Real Time Clock IRQ
		
		//rtx_SetTime(&rtx);
	}
	
u08  rtc_time2unix (uint32_t *uxt, const T_RTC *rtx)	{ // Oblicz unixtime z daty i czasu
		uint32_t utc, i, y;
		// yer,mon,mdy,hor,min,sec
		y = rtx->yer - 1970;
		if (y > 2106 || !rtx->mon || !rtx->mdy) return 1;
		
		utc = y / 4 * 1461; y %= 4;
		utc += y * 365 + (y > 2 ? 1 : 0);
		for (i = 0; i < 12 && i + 1 < rtx->mon; i++) {
			utc += samurai[i];
			if (i == 1 && y == 2) utc++;
		}
		utc += rtx->mdy - 1;
		utc *= 86400;
		utc += rtx->hor * 3600 + rtx->min * 60 + rtx->sec;
		
		utc -= (long)(_RTC_TDIF * 3600);
		*uxt = utc;
		return 0;
	}

T_RTC * rtc_unix2time (uint32_t utc, T_RTC *rtx)	{ //Oblicz date i czas z unixtime
		uint32_t n, i, d;
		//if (!rtx_getutc(&utc)) return 0;
		utc += (long)(_RTC_TDIF * 3600);
		
		rtx->sec = (uint8_t) (utc % 60); utc /= 60;
		rtx->min = (uint8_t) (utc % 60); utc /= 60;
		rtx->hor = (uint8_t) (utc % 24); utc /= 24;
		rtx->wdy = (uint8_t) ((utc + 4) % 7);
		
		rtx->yer = (uint16_t)(1970 + utc / 1461 * 4); utc %= 1461;
		n = ((utc >= 1096) ? utc - 1 : utc) / 365;
		rtx->yer += n;
		
		utc -= n * 365 + (n > 2 ? 1 : 0);
		for (i = 0; i < 12; i++) {
			d = samurai[i];
			if (i == 1 && n == 2) d++;
			if (utc < d) break;
			utc -= d;
		}
		rtx->mon = (uint8_t)(1 + i);
		rtx->mdy = (uint8_t)(1 + utc);
		
		return rtx;
	}
void rtc_GetTime(u08 force , T_RTC *rtx )		{
		u32 utc = RTC->CNTL + (RTC->CNTH<<16);
		u32 n, i, d;
		
		utc += (long)(_RTC_TDIF * 3600);
		
		rtx->sec = (uint8_t) (utc % 60); 
		if(!rtx->sec || force){
			
			utc /= 60;			rtx->min = (uint8_t) (utc % 60); 
			if(!rtx->min || force){
				
				utc /= 60; rtx->hor = (uint8_t) (utc % 24); 
				if(!rtx->hor || force){
					utc /= 24;
					rtx->wdy = (uint8_t) ((utc + 4) % 7);
					rtx->yer = (uint16_t)(1970 + utc / 1461 * 4); utc %= 1461;
					
					
					n = ((utc >= 1096) ? utc - 1 : utc) / 365;
					rtx->yer += n;
					
					utc -= n * 365 + (n > 2 ? 1 : 0);
					for (i = 0; i < 12; i++) {
						d = samurai[i];
						if (i == 1 && n == 2) d++;
						if (utc < d) break;
						utc -= d;
					}
					rtx->mon = (uint8_t)(1 + i);
					rtx->mdy = (uint8_t)(1 + utc);
					
				}
			}
		}
	}

void rtc_SetTime(T_RTC *rtx )		{
		u32 uxt;
		//Ustawienie daty i czasu
/*
		rtx->yer=2017;	// 1970..2106
		rtx->mon=5;			// 1..12
		rtx->mdy=18;		// 1..31
		rtx->hor=14;		// 0..23
		rtx->min=30;		// 0..59
		rtx->sec=0;			// 0..59
		rtx->wdy=4;			// 0..6 (Niedziela=0..Sobota=6) 
*/

		if (!rtc_time2unix (&uxt, rtx)	){
			
			while(!bRTC_CRL_RTOFF); //wait to synchro
			bRTC_CRL_CNF = 1;				//Start edit mode
			//RTC->PRLL = 0x7fff;		//Set prescaler		ones
			RTC->CNTL = uxt&0xffff;		  //Set time low		set counter/point time
			RTC->CNTH = (uxt>>16);		  //Set time high		set counter/point time		
			bRTC_CRL_CNF = 0;				//End edit mode
			while(!bRTC_CRL_RTOFF); //wait to synchro
		}
	}

void SleepMode(u08 sleep){
		// Po obsluzeniu przerwania ponownie bedzie wprowadzony tryb uspienia
		// !!! Zmniejszony pobór dziala gdy program startuje z pamieci flash,
		// Od³¹czony JTAG, porty tryb niep³ywaj¹cy
		// 0 - SLEEP    mode 6  mA (RTC, GPIO, RAM)
		// 1 - STOP     mode 30 uA (RTC, GPIO, RAM)
		// 2 - STANDBAY mode 3  uA (all reset)
		
		RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN; 
		
    //PWR->CSR |= PWR_CSR_EWUP;        		// Enable WKUP pin PA0
    //PWR->CR  |= PWR_CR_CWUF; 						// Clear Wake-up flag 		
    /* Set SLEEPDEEP bit of Cortex System Control Register 			*/   
	  //SCB->SCR   |=  SCB_SCR_SEVONPEND;		// 
		//SCB->SCR   &= ~SCB_SCR_SLEEPONEXIT; // don't sleep if in interrupt routine				
		//SCB->SCR   |= SCB_SCR_SLEEPONEXIT;  // don't sleep if in interrupt routine				
		
    switch (sleep)
    {
			case 0: // Sleep Mode 6 or 30 mA @80MHz 17mA 
				SCB->SCR &= ~SCB_SCR_SLEEPDEEP;
				SCB->SCR |=  SCB_SCR_SLEEPONEXIT; // eneter sleep on return from interrupt routine
				//SCB->SCR   &= ~SCB_SCR_SLEEPONEXIT; // don't sleep if in interrupt routine				
      break;
      case 1: // Stop Mode 35 uA (180uA), po wybudzeniu FCPU=HSI(8MHz)
				//SCB->SCR |=  SCB_SCR_SEVONPEND;
				PWR->CR  |= (PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
				PWR->CR  &= ~PWR_CR_PDDS;   		
				SCB->SCR |=  SCB_SCR_SLEEPDEEP;							
				//PWR->CR  |=  PWR_CR_LPDS;   		// Voltage regulator in low-power mode during stop mode
      break;
			case 2: // Standby Mode 4 uA(1.5mA), utrata SRAM,rejestrow poza BKP i RCC_SCR
				SCB->SCR |=  SCB_SCR_SLEEPDEEP;			// Select Deep Sleep mode (stop or standby mode)
				PWR->CR  |=  PWR_CR_PDDS; 					// Select STANDBY mode   3uA all reset
			break;
			
		}
		
	}

/*
 // Loops until two consecutive read returning the same value.
  do {
    timespec->tv_sec = ((uint32_t)(RTC->CNTH) << 16) + RTC->CNTL;
    time_frac = (((uint32_t)RTC->DIVH) << 16) + (uint32_t)RTC->DIVL;
  } while ((timespec->tv_sec) != (((uint32_t)(RTC->CNTH) << 16) + RTC->CNTL));

  timespec->tv_msec = (uint16_t)(((STM32_RTCCLK - 1 - time_frac) * 1000) /  STM32_RTCCLK);
*/
