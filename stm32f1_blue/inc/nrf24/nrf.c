#include "nrf.h" 
//#include "inc/glcd/GLCD.h"
//#include "hdr/hdr_gpio.h"
#include "inc/tool/delay.h"

//NRF24L01
#define CSN_TIME      2
#define CE_HIGH_TIME  10000
#define nDLY      2

inline static u08	SPI_rw(u08 cmd);    

unsigned int sta;
unsigned char TX_ADDRES[TX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x10}; // Define a static TX address
// unsigned char rx_com_buffer[10];
// unsigned char tx_com_buffer[10];
// unsigned char accept_flag;

#define  RX_DR_RF  ((sta>>6)&0x01)
#define  TX_DS_RF  ((sta>>5)&0x01)
#define  MAX_RT_RF ((sta>>4)&0x01)

#define nrfSPI2

#if defined nrfSPI1
#define  SPI_CR1 	SPI1->CR1
#define  SPI_SR		SPI1->SR
#define  SPI_DR		SPI1->DR
#elif defined nrfSPI2
#define  SPI_CR1 	SPI2->CR1
#define  SPI_SR		SPI2->SR
#define  SPI_DR		SPI2->DR
#endif


void nRF24L01_Init(void)
	{			    		   
		GPIOB->CRH &= 0x0000fff0;//
		GPIOB->CRH |= 0x94914441;//
    //TCS = 1;
		
		RCC->APB1ENR   |= RCC_APB1ENR_SPI2EN;
		SPI_CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_1|SPI_CR1_BR_0|SPI_CR1_MSTR;		// 0x037c
		//SPI_CR1   = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2|SPI_CR1_BR_1|SPI_CR1_BR_0|
		SPI_CR1 |= SPI_CR1_SPE;		//		
		//GPIOA->BRR  = (1<<0);		// GND ON
		Delay_ms(10);							// Power on reset 10.3 ms
		CSN_HIGH();								// SPI select OFF
		CE_LOW();									// Chip Enable ON
		
	}

inline static u08 SPI_rw(u08 cmd)    
	{ 
		u08 spi_out=0;    		
		// GPIOA->BSRRH = GPIO_Pin_4;
		while ((SPI_SR & SPI_SR_TXE) == 0);		/// while TX buffer not empty
		SPI_DR = cmd;	
		while ((SPI_SR & SPI_SR_RXNE) == 0);	/// while RX buffer empty
		spi_out = SPI_DR;		
		//GPIOA->BSRRL = GPIO_Pin_4;
		Delay_us(2);
		return spi_out;
	} 		 

u08 nRF24_readReg(unsigned char reg)
	{
    unsigned char reg_val;
   CSN_LOW();      
		Delay_us(nDLY);
    SPI_rw(reg);             // select register 
    reg_val = SPI_rw(0); 		// read value
    CSN_HIGH();    
		Delay_us(nDLY);
    return(reg_val);            
	}

u08 nRF24_readBuf_(u08 reg,u08 *pBuf, u08 bytes)
	{
    unsigned char status,i;
   CSN_LOW();		
	 Delay_us(nDLY);
    status = SPI_rw(reg);	// Select register to write to and read status byte
    for(i=0;i<bytes;i++)    pBuf[i] = SPI_rw(0);
    CSN_HIGH();
		Delay_us(nDLY);
    return(status);
	}

u08 nRF24_readBuf(uint8_t reg, uint8_t *data_out, uint8_t lenght){
   uint8_t i,stat;
   CSN_LOW();
   stat = SPI_rw(reg);
   for(i=0; i<lenght; i++)    { *data_out++ = SPI_rw(0xff); }
   CSN_HIGH();
	 return (stat);
	}/* nrf24l01_read_register */

u08 nRF24_writeReg(u08 reg, u08 value){
		u08 stat;
   CSN_LOW();
   stat = SPI_rw(reg | 0x20);	 
   SPI_rw(value);
   CSN_HIGH();
	 return stat;
	}

u08 nRF24_writeReg_(u08 reg, u08 value)
	{
    unsigned char status;
    CSN_LOW();
		Delay_us(nDLY);
    status = SPI_rw(reg);	// select register 
    SPI_rw(value);				// set value
    CSN_HIGH();
		Delay_us(nDLY);
    return(status);
	}

u08 nRF24_writeBuf(u08 reg, u08 *pBuf, u08 bytes)
	{
    unsigned char status,i;
   CSN_LOW();		
	 Delay_us(nDLY);
    status = SPI_rw(reg);	// Select register to write to and read status byte
    for(i=0; i<bytes; i++) 	// then write all byte in buffer(*pBuf)
    SPI_rw(*pBuf++);
    CSN_HIGH();
		Delay_us(nDLY);
    return(status);
	}

u08 nRF_read_status(void){
   uint8_t stat;
   CSN_LOW(); 
   stat = SPI_rw(0xff);	//NOP
   CSN_HIGH();
	 return (stat);
	}


void RX_Mode(void)
	{
    CE_LOW();
		Delay_us(10);
    nRF24_writeBuf(WRITE_RG + RX_ADDR_P0, TX_ADDRES, TX_ADR_WIDTH);	//0x0a(5) RX  Addr Pipe0
    nRF24_writeReg(WRITE_RG + EN_AA, 			0x01);						// 0x01 Enable Auto.Ack:Pipe0
    nRF24_writeReg(WRITE_RG + EN_RXADDR, 	0x01); 						// 0x02 Enable Pipe0
    nRF24_writeReg(WRITE_RG + RF_CH, 			RF_channel);					// 0x05 Select RF channel 40
    nRF24_writeReg(WRITE_RG + RX_PW_P0, 	TX_PLOAD_WIDTH); 	// 0x11 RX Payload data Pipe0
    nRF24_writeReg(WRITE_RG + RF_SETUP, 	RF_setup);					// 0x06 Power, Data Rate
    nRF24_writeReg(WRITE_RG + CONFIG, 		0x0f);						// 0x00 Set PWR_UP bit, enable CRC(2 bytes)
				//& Prim:RX. RX_DR_RF enabled..		
    CE_HIGH(); 
		// Set CE pin high to enable RX device
		// This device is now ready to receive one packet of 16 bytes payload 
		//from a TX device sending to address
		// '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and
		// datarate = 2Mbps.		
	}

void TX_Mode(void)
	{	
    CE_LOW();//CE=0
		Delay_us(10);
    nRF24_writeBuf(WRITE_RG + TX_ADDR, TX_ADDRES, TX_ADR_WIDTH);			//0x10(5)
    nRF24_writeBuf(WRITE_RG + RX_ADDR_P0, TX_ADDRES, TX_ADR_WIDTH);	//0x0A(5)
    //nRF24_writeBuf(WR_TX_PLOAD, BUF, TX_PLOAD_WIDTH); 	//0xA0 Writes data to TX payload
    nRF24_writeReg(WRITE_RG + EN_AA, 			0x01); 	//0x01 Enable Auto.Ack:Pipe0
    nRF24_writeReg(WRITE_RG + EN_RXADDR, 	0x01); 	//0x02 Enable Pipe0
    nRF24_writeReg(WRITE_RG + SETUP_RETR, 0x1a); 	//0x04 500us + 86us, 10 retrans...
    nRF24_writeReg(WRITE_RG + RF_CH, 			RF_channel);//0x05 Select RF channel 40    
    nRF24_writeReg(WRITE_RG + RF_SETUP, 	RF_setup);	//0x06 TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
    nRF24_writeReg(WRITE_RG + CONFIG, 		0x0e);	//0x00
    // Set PWR_UP bit, enable CRC(2 bytes)
    // & Prim:TX. MAX_RT_RF & TX_DS_RF enabled..		
    CE_HIGH(); 			// Set CE pin high 
		Delay_us(20);		// TX Payload data
		CE_LOW();				// CE=0		
	}

void nRF24_tx(u08 *tx){
		
		u08 stat=0,cnt=0;
		
		TX_Mode();
		//while(1){
		if(1){
			nRF24_writeBuf(WR_TX_PLOAD, tx, TX_PLOAD_WIDTH);     // write playload to TX_FIFO
			CE_HIGH(); 			// Set CE pin high 
			Delay_us(10);		// TX Payload data
			CE_LOW();				// CE=0		
			
	//=== Transmiter =======================
			
			do	{
				stat = 	nRF_read_status();		
				cnt++;
				if( cnt > 160 ){
					//UART_puts				("\r\n Transmitted Error !");
					//stat = nRF_read_status();
					//UART_puts("\r\n sts=0x");
					//UART_puts(hex2str(ubuf,stat,2));
					//;
					//delayms(400);
					//return;
					break;
				}
			}
			while ((stat & TX_DS) == 0);
			
			{ 
				//stat = nRF_read_status();		
				//stat = nRF_readReg(STATUS);
				//stat = nRF_read_status();		
				
				if(stat&TX_DS){	 
					//UART_puts("\r\n Sent Normal -> ");	
					//UART_puts("sts = 0x");
					//UART_puts(hex2str(ubuf,stat,2));
					//UART_puts(hex2str(ubuf,cnt ,2));
				}
				else if(stat&MAX_RT){
					//UART_puts("\r\n Sent Retry  -> ");
					Delay_ms(100);
					nRF24_writeReg(FLUSH_TX, 0);
				}
				
				// clear RX_DR or TX_DS or MAX_RT interrupt flag
				nRF24_writeReg( WRITE_RG | STATUS,stat&0xf0); 
				//UART_puts(" us ");
				//UART_puts(" ,OBS=");			hex2uart(obs1,2);
				//UART_puts(" ,STS=");			hex2uart(stat,2);
				
			}
			
		}
	}