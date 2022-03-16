#include <string.h>
#include "inc/stm32f10x.h"
//#include "hdr/hdr_bitband.h"
#include "inc/tool/delay.h"
#include "spi.h"
#include "gpio.h"

void SPI1_init_(void){
		
		//GPIOA->CRL &= 0x000fffff;//PA4~7
		//GPIOA->CRL |= 0x94900000;//PA4~7
		GPIOA->CRL = (GPIOA->CRL & 0x000fffff) | 0x94900000;	// PA5~7
		//GPIOC->CRH = (GPIOC->CRH & 0xff0fffff) | 0x00100000;	// SD_CS PC13
		//GPIOC->CRH = (GPIOC->CRH & 0xfff0ffff) | 0x00010000;	// enc28_CS
		
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
		SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_BR_2|	SPI_CR1_MSTR;	// 0x037c
		SPI1->CR1 |= SPI_CR1_SPE;		
	}
	
void SPI1_init(void)	{	
		
		/* SLAVE
		GPIOA->CRL &= 0x0000ffff;
		GPIOA->CRL |= 0x44440000;
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
		SPI1->CR1  = SPI_CR1_SSM|SPI_CR1_RXONLY ;		
		SPI1->CR1 |= SPI_CR1_SPE; */
		
		// MASTER
		GPIOA->CRL = (GPIOA->CRL&0x000fffff)|0xB4B00000;	// PA4~7
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|1<<3|SPI_CR1_MSTR;	
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|7<<3|SPI_CR1_MSTR|SPI_CR1_DFF;		
		SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|7<<3|SPI_CR1_MSTR;		
		//SPI1->CR1 = 														 SPI_CR1_MSTR;		
		//SPI1->CR2 |= SPI_CR2_SSOE;		
		//SPI1->CR2  = 0;			
		SPI1->CR1 |= SPI_CR1_SPE;		
    
		Delay_ms(10);								// Power on reset 10.3 ms
	}
	
u16 SPI1_master(u16 cmd){ 
		u16 spi_out=0;  
		
		//sTim0 = 500;
		//while (((SPI1->SR & SPI_SR_TXE) == 0) && sTim0);		/// while TX buffer not empty
		//SPI1->DR = 0xff;	
		
		while ((SPI1->SR & SPI_SR_TXE) == 0);		/// while TX buffer not empty
		//while (!TXE1);
		SPI1->DR = cmd;	
		while ((SPI1->SR & SPI_SR_RXNE) == 0);	/// while RX buffer empty
		//while (!RXNE1);
		spi_out = SPI1->DR;		
		return spi_out;
	}
	
///=========================================================================
void SPI2_init(void)	{
		
		// MASTER
		GPIOB->CRH = (GPIOB->CRH&0x0000ffff)|0x94930000;
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		SPI2->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2|SPI_CR1_MSTR;		// 0x037c
		SPI2->CR1 |= SPI_CR1_SPE;		
		
		//Delay_ms(10);								// Power on reset 10.3 ms
	}
void SPI2_slave_init(void)	{
		
		// SLAVE
   	GPIOB->CRH = (GPIOB->CRH & 0x0000ffff)|0x44440000;
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		SPI2->CR1  = SPI_CR1_SSM |SPI_CR1_RXONLY;		
		SPI2->CR2 |= SPI_CR2_RXDMAEN;		
		//SPI2->CR1  = SPI_CR1_RXONLY;		
		SPI2->CR1 |= SPI_CR1_SPE; 
		Delay_ms(10);								// Power on reset 10.3 ms
	}

u08  SPI2_slave(u08 cmd){
		u08 spi_out=0; 
		cmd = cmd;
		//while ((SPI2->SR & SPI_SR_TXE) == 0);		/// while TX buffer not empty
		//SPI2->DR = cmd;
		while ((SPI2->SR & SPI_SR_RXNE) == 0);	/// while RX buffer empty
		spi_out = SPI2->DR;		
		return spi_out;
	} 		 
///=========================================================================
void SPI2MemDMAInit(void *dst_buf){
		//DMA Configuration
		uint32_t CCR_reg = 0;
		RCC->AHBENR   			|= RCC_AHBENR_DMA1EN;	// enable clock for DMA2
		DMA1_Channel4->CCR 	= 0;										//Disable channel
		DMA1_Channel4->CMAR = (uint32_t)dst_buf;	 	//Destination address:
		DMA1_Channel4->CPAR = (uint32_t)&SPI2->DR;		//Source PERIPHERAL address:
		//DMA2_Channel1->CNDTR = sizeof(dst_buf);			//Buffor size :
		DMA1_Channel4->CNDTR = 8192;		//Buffor size :
		
		CCR_reg |=  DMA_CCR1_PL;		 // Priorytet high
		CCR_reg &=  ~DMA_CCR1_MSIZE;	 // 8bit
		CCR_reg &=  ~DMA_CCR1_PSIZE;	 // 8bit
		//CCR_reg |=  DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0;	// 16bit/16bit
		CCR_reg |=  DMA_CCR1_MINC;	 // Increment memory address enabled
		//CCR_reg |=  DMA_CCR1_PINC;	 // Increment peripheral address enabled
		CCR_reg &= ~DMA_CCR1_CIRC;	 // Mode Circular mode disabled
		CCR_reg &= ~DMA_CCR1_DIR;		 // Dir Read from peripheral
		//CCR_reg |=  DMA_CCR1_TCIE; // Transfer complete interrupt enable
		DMA1_Channel4->CCR =  CCR_reg;	
		DMA1_Channel4->CCR |= DMA_CCR1_EN;
	}
///=========================================================================
uint16_t spi1_rw(uint16_t data){
    while( !(SPI1->SR & SPI_SR_TXE) );
    SPI1->DR = data;
    while( !(SPI1->SR & SPI_SR_RXNE) );
    data = SPI1->DR;
    return data;
	}

uint16_t spi2_rw(uint16_t data){
    while( !(SPI2->SR & SPI_SR_TXE) );
    SPI2->DR = data;
    while( !(SPI2->SR & SPI_SR_RXNE) );
    data = SPI2->DR;
    return data;
	}

u08  SPI1_ReadWrite(u08 data){ // enc28j60
		return spi1_rw(data);
	}

//====CRC8_SPI2=======================================================================================
void crc8_spi2_init(void){
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_CRCEN;
		//RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
	}
void crc8_spi2(u08 * data,u08 len,u08 *crc  ){
		
		SPI2->CR1 &= ~SPI_CR1_CRCEN; // Reset CRC
    SPI2->CR1 |=  SPI_CR1_CRCEN;	
		for(u16 i=0;i<len;i++)  spi2_rw(data[i]); // max 256 
		*crc = SPI2->TXCRCR;
	}
//====CRC16_SPI2======================================================================================
void crc16_spi2_init(void){
		// Uwaga wazna predkosc wyjscia alternatywnego
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		GPIOB->CRH = (GPIOB->CRH & 0xff0fffff) | 0x00B00000;	// PB13 SPI2_SCK
		SPI2->CRCPR = 0x8005;
    SPI2->CR1   = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_CRCEN | SPI_CR1_DFF ;
		
	}
void crc16_spi2(u16 * data,u16 len, u16 *crc ){
		
		SPI2->CR1 &= ~SPI_CR1_CRCEN; // Reset CRC
    SPI2->CR1 |=  SPI_CR1_CRCEN;
		for(u16 i=0;i<len;i++){
			spi2_rw((u16)*data++);
		}
		*crc = SPI2->TXCRCR;
	}
//====CRC16_SPI1======================================================================================
void crc16_spi1_init(void){
		// Uwaga wazna predkosc wyjscia alternatywnego
		GPIOA->CRL    = (GPIOA->CRL & 0xff0fffff) | 0x00B00000;	// PA5  SPI1_SCK		
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		SPI1->CRCPR   = 0x8005;
    SPI1->CR1     = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_CRCEN | SPI_CR1_DFF| SPI_CR1_BR_0 ;
	}
void crc16_spi1 (u16 * data,u16 len, u16 *crc ){
		
		SPI1->CR1 &= ~SPI_CR1_CRCEN; // Reset CRC
    SPI1->CR1 |=  SPI_CR1_CRCEN;
		for(u16 i=0;i<len;i++)  spi1_rw((u16)*data++);
		*crc = SPI1->TXCRCR;
	}
//======================================================================================================
// DMA test //
/* 
SPI2MemDMAInit(rx2);
		rx2[0]=0;rx2[1]=0;rx2[2]=0;rx2[3]=0;
		while(1){
			
			//while(nx++)SPI1_master(0x33);
			//nCE1  = 0;
			//SSOE2 = 0;
			if(DMA1_Channel4->CNDTR == 0){
				DMA1_Channel4->CCR &= ~DMA_CCR1_EN;
				DMA1_Channel4->CNDTR = 8192;
				DMA1_Channel4->CCR |= DMA_CCR1_EN;
			}
			
			//while(MOSI2==1);
			SPI2->CR1 |= SPI_CR1_SPE; 
			while(DMA1_Channel4->CNDTR);
			SPI2->CR1 &= ~SPI_CR1_SPE; 
			
			UaPutS("\r\n ");
			nx=0;
			for(u16 bx=0;bx<255;bx++){
				UaPutS("\r\n ");	
				hex2uart(bx, 2);
				UaPutS("  ");
				for(u08 ax=0;ax<32;ax++){
					hex2uart(rx2[nx++],2);
				}
			}
		}
	*/
