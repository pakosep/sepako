#ifndef __SPFTSPI_H
#define __SPFTSPI_H
	
#ifndef sMISO
//#define sMISO bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 14)  // PA6  MISO		
//#define sMOSI bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 15)  // PA7  MOSI	
//#define sSCK  bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 13)  // PA5  SCLK			
#endif 

#define sMISO bitband_t m_BITBAND_PERIPH(&GPIOA->IDR, 6)  // PA6  MISO		
#define sMOSI bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 7)  // PA7  MOSI	
#define sSCK  bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 5)  // PA5  SCLK			
#define sCSS  bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 4)  // PA4  SCLK			

uint8_t softSPI_sr1(uint8_t byte); // send and receive
uint8_t softSPI_sr0(uint8_t byte); // send and receive
uint8_t inverse_byte(uint8_t byte);

#endif 