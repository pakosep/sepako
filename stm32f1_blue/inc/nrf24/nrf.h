#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"

#define NCE  bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 12)  //	CE  Chip En b.czerw
#define CSN  bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 8)  // CSN Spi  En czerwony
//#define PIRQ bitband_t m_BITBAND_PERIPH(&GPIOA->IDR, 3)  //	IRQ					rozowy
#define MISO bitband_t m_BITBAND_PERIPH(&GPIOB->IDR, 14)  // PA6  MISO		b.zielony
#define MOSI bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 15)  // PA7  MOSI		zielony
#define NCLK bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 13)  // PA5  SCLK		b.rozowy
// +3.3 niebiseski
// GND  b.niebieski
//CONSTANTS

#define   LED          		bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 0) 
#define   LED_ON          bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 0) = 1
#define   LED_OFF         bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 0) = 0

#define  CSN_HIGH()		CSN=1
#define  CSN_LOW()		CSN=0
#define  CE_HIGH()		NCE=1
#define  CE_LOW()			NCE=0

//==============================================================================

#define TX_ADR_WIDTH    5   // 5 bytes TX(RX) address width
#define TX_PLOAD_WIDTH  10  // 10 bytes TX payload

// SPI(nRF24L01) commands
#define READ_RG	        0x00  // Define read command to register
#define WRITE_RG	      0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
//#define NOP             0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address

#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address

#define   RX_DR           0x40
#define   TX_DS           0x20
#define   MAX_RT          0x10

#define   Pow_00dBm    	0x06
#define   Pow_06dBm    	0x04
#define   Pow_12dBm    	0x02
#define   Pow_18dBm    	0x00

#define   nrf_00dBm    	0x06
#define   nrf_06dBm    	0x04
#define   nrf_12dBm    	0x02
#define   nrf_18dBm    	0x00
#define   nrf_2Mbit    	0x08
#define   nrf_1Mbit    	0x00
#define   nrf_250kb    	0x20

#define   nrf_ard250 		0x00	//2M,5b addr,15b pay or 1M,5b addr, 5b pay
#define   nrf_ard500 		0x10	
#define   nrf_ard750 		0x20
#define   nrf_ard1000		0x30
#define   nrf_ard1250		0x40
#define   nrf_ard1500		0x50
#define   nrf_arc				0x0f	//Auto Retransmit Count

#define		RF_setup			((nrf_1Mbit) | (nrf_00dBm))
#define		LNA_HCURR			0x00
#define		nRF_POW				((Pow_00dBm) + (LNA_HCURR))
#define		RF_channel		62		//1-128 Channels

// SPI(nRF24L01) commands
//#define RF_READ_REG   0x00  // Define read command to register
//#define RF_WRITE_REG  0x20  // Define write command to register


void	nRF24L01_Init(void);
void	RX_Mode(void);     
void 	TX_Mode(void);
void  nRF24_tx(u08 *tx);

u08 	nRF24_readReg(unsigned char reg);
u08 	nRF24_writeReg(uint8_t reg, uint8_t byte); 

u08 	nRF24_readBuf(u08 reg,u08 *pBuf, u08 bytes);
u08 	nRF24_writeBuf(u08 reg, u08 *pBuf, u08 bytes);








