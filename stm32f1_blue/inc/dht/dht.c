#include "dht.h" 
#include "inc/tool/delay.h"
#include "inc/tool/UARTLib.h"
//#include "inc/tool/flash.h"

//NRF24L01

extern volatile u32 trg0,trg1;

void DHT_Init(void){
		GPIOA->CRH = (GPIOA->CRH & 0x0fffffff) | 0x60000000;
		//GPIOC->CRH = (GPIOC->CRH & 0xfff0ffff) | 0x00060000;	// 
		//gpio_pin_cfg(GPIOC, 1, GPIO_OUT_OD_25MHz_PULL_UP);	//DAT
		//gpio_pin_cfg(GPIOC, 2, GPIO_OUT_OD_25MHz_PULL_UP);	//DAT
		AM2302_prt = 1;
	}
	
void DHT_Start(void){
		
		*(&AM2302_pin+0x20) = 0;	//Init H & T Mesaure
		Delay_ms(1);
		*(&AM2302_pin+0x20) = 1;	//Init H & T Mesaure
		//AM2302_prt = 1;
		
	}


u08  DHT_read(u16* AMh, s16* AMt, DHT czuj, volatile long unsigned int * P_PIN){
		u32 AMht=0;		// Humidity & Temperature
		u08 AMs =0;		// Parity bit
		//u32 tim1,tim2,tim3;
		
		//UaPutS("\r\n AM2302_pin  =0x"); hex2uart(&AM2302_prt,8);
		//UaPutS("\r\n P_pin       =0x"); hex2uart(P_PIN+0x20,8);
		
		//TIM_start();
		//*(P_PIN+0x20) = 0;	//Init H & T Mesaure
		//Delay_ms(1);
		//*(P_PIN+0x20) = 1;	//Init H & T Mesaure
		
		*(&AM2302_pin+0x20) = 0;	//Init H & T Mesaure
		Delay_ms(1);
		*(&AM2302_pin+0x20) = 1;	//Init H & T Mesaure
		
		Delay_us(1);
		trg1 = SysTick->VAL;		
		while( *(P_PIN) == 1)
		{ if(TIM_US*40 < (trg1 - SysTick->VAL)) return 1;} 	// < 40us
		
		Delay_us(1);
		trg1 = SysTick->VAL;		
		while( *(P_PIN) == 0)
		{ if(TIM_US*100 < (trg1 - SysTick->VAL)) return 2;} 	// < 100us
		
		//Delay_us(120);
		Delay_us(1);
		trg1 = SysTick->VAL;		
		while( *(P_PIN) == 1)
		{ if(TIM_US*100 < (trg1 - SysTick->VAL)) return 3;} 	// < 100us
		
		for(u08 n=0;n<40;n++){
			trg1 = SysTick->VAL;
			while( *(P_PIN)==0)
			{ if(TIM_MS*2 < (trg1 - SysTick->VAL)) return 4;} // Low <54us
			
			trg1 = SysTick->VAL;
			while( *(P_PIN)==1)
			{ if (TIM_MS*2 < (trg1 - SysTick->VAL)) return 5;} // High <81us
			
			trg1 = trg1 - SysTick->VAL;
			
			
			if(n<32){		// Humidity & Temperature
				AMht <<= 1;
				if( trg1 > 40*TIM_US){ AMht |= 1; }
			}else{			// Parity bit
				AMs <<= 1;
				if( trg1 > (s32)40*TIM_US){ AMs  |= 1; }
			}
		}
		
		if(czuj==AM2302){
			*AMh = AMht >> 16;
			*AMt = AMht & 0xffff;
		}else{
			*AMh = (AMht>>24);
			*AMt = (AMht>>8)&0xff; 
		}
		//TIM_stop(&tim3);
		//UaPutS("\r\n Time = ");
		//Int2uart(tim3*341/512,5,1);
		//sint2uart(tim3/TIM_US);
		return 0;
	}
	
u08  AM23_read(AM23_hw_t * hw){
		u32 AMht=0;		// Humidity & Temperature
		u08 AMs =0;		// Parity bit
		//u32 tim1,tim2,tim3;
		
		*(hw->prt) = 0;	//Init H & T Mesaure
		Delay_ms(1);
		*(hw->prt) = 1;	//Init H & T Mesaure
		
		Delay_us(1);
		trg1 = SysTick->VAL;		
		while( *(hw->pin) == 1)
		{ if(TIM_US*40 < (trg1 - SysTick->VAL)) return 1;} 	// < 40us
		
		Delay_us(1);
		trg1 = SysTick->VAL;		
		while( *(hw->pin) == 0)
		{ if(TIM_US*100 < (trg1 - SysTick->VAL)) return 2;} 	// < 100us
		
		//Delay_us(120);
		Delay_us(1);
		trg1 = SysTick->VAL;		
		while( *(hw->pin) == 1)
		{ if(TIM_US*100 < (trg1 - SysTick->VAL)) return 3;} 	// < 100us
		
		for(u08 n=0;n<40;n++){
			trg1 = SysTick->VAL;
			while( *(hw->pin)==0)
			{ if(TIM_MS*2 < (trg1 - SysTick->VAL)) return 4;} // Low <54us
			
			trg1 = SysTick->VAL;
			while( *(hw->pin)==1)
			{ if (TIM_MS*2 < (trg1 - SysTick->VAL)) return 5;} // High <81us
			
			trg1 = trg1 - SysTick->VAL;
			
			if(n<32){		// Humidity & Temperature
				AMht <<= 1;
				if( trg1 > 40*TIM_US){ AMht |= 1; }
			}else{			// Parity bit
				AMs <<= 1;
				if( trg1 > (s32)40*TIM_US){ AMs  |= 1; }
			}
		}
		
		if(hw->typ==AM2302){
			hw->AMh = AMht >> 16;
			hw->AMt = AMht & 0xffff;
		}else{
			hw->AMh = (AMht>>24);
			hw->AMt = (AMht>>8)&0xff; 
		}
		
		return 0;
	}
	
	/*
	if(!(DHT_read( &AMh, &AMt,1,&AM2302_pin)) ){		// AM2302 read T & H
	gph[7].var = AMh;
	gph[8].var = AMt;
	*/
