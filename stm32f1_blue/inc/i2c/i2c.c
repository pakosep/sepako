#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"
#include "i2c.h"

void i2c1_init(void)	{
		
		GPIOB->CRL    = (GPIOB->CRL & 0x00ffffff) | 0xdd000000;	// PB7=SDA (Alternate Open-drain) PB6=SCL(Alternate Open-drain),
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		
		I2C1->CR1  &= ~I2C_CR1_PE;					// I2C1 Pheripherial enable
		I2C1->TRISE = 37;               		// limit slope
		I2C1->CCR   = (1<<15) | (F_CPU/(8*100000));               			// setup speed (100kHz)
		I2C1->CR2   = I2C_CR2_FREQ_36MHz;   // config I2C1 module
		I2C1->CR1  |= I2C_CR1_PE;						// I2C1 Pheripherial enable
	}

void i2c1r_init(void)	{
		
		AFIO->MAPR   |= AFIO_MAPR_I2C1_REMAP;		// Remap TIM3 CH2->PB5
		GPIOB->CRH    = (GPIOB->CRH & 0xffffff00) | 0x000000dd;	// PB9=SDA (Alternate Open-drain) PB8=SCL(Alternate Open-drain),
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		
		I2C1->CR1  &= ~I2C_CR1_PE;						// I2C1 Pheripherial enable
		I2C1->TRISE = 37;               		// limit slope
		I2C1->CCR   = (1<<15) | (F_CPU/(8*100000));               			// setup speed (100kHz)
		I2C1->CR2   = I2C_CR2_FREQ_36MHz;     // config I2C1 module
		I2C1->CR1  |= I2C_CR1_PE;						// I2C1 Pheripherial enable
	}

	
void i2c1_write(uint8_t dev_adr, uint8_t* data, uint32_t length){
		static uint32_t dummy;
		dummy = dummy;
		// Slave potwierdza odbior ACKiem
		I2C1_CR1_START_bb = 1;				// request a start
		while (I2C1_SR1_SB_bb == 0);	// wait for start to finish
		dummy = I2C1->SR1;						// read of SR1 clears the flag
		I2C1->DR = dev_adr;						// transfer address
		
		while (I2C1_SR1_ADDR_bb == 0);// wait for address transfer
		dummy = I2C1->SR1;						// clear the flag
		dummy = I2C1->SR2;						// clear the flag
	 
		while (length--)						// transfer whole block
		{
			while (I2C1_SR1_TxE_bb == 0);		// wait for DR empty
			I2C1->DR = *data++;							// trsnsfer one byte, increment pointer
		}
	 
		while (I2C1_SR1_TxE_bb == 0 || I2C1_SR1_BTF_bb == 1);	// wait for bus not-busy
		I2C1_CR1_STOP_bb = 1;					// request a stop
	}

void i2c1_read( u08 dev_adr, u08 reg_adres, u08 * dane, u08 len ) {
		static uint32_t dummy;
		dummy = dummy;
		// Master potwierdza odbior ACKiem
		
		//I2C2_CR1_START_bb = 1;							// request a start
		//while (I2C2_SR1_SB_bb == 0);				// wait for start to finish
		I2C1->CR1 |= I2C_CR1_START;						// request a start
		while( !( I2C1->SR1 & I2C_SR1_SB ));	// wait for start to finish (read of SR1 clears the flag)
		I2C1->DR = dev_adr; 										// transfer address
		
		while( !( I2C1->SR1 & I2C_SR1_ADDR ));
		//while (I2C2_SR1_ADDR_bb == 0);	// wait for address transfer
		dummy = I2C1->SR2;							// clear the flag
		
		while( !( I2C1->SR1 & I2C_SR1_TXE ));
		//while (I2C2_SR1_TxE_bb == 0);
		I2C1->DR = reg_adres;
		
		while( !( I2C1->SR1 & I2C_SR1_BTF ));
		//while (I2C2_SR1_BTF_bb == 0);
		I2C1->CR1 |= I2C_CR1_START;
		
		while( !( I2C1->SR1 & I2C_SR1_SB ));
		//while (I2C2_SR1_SB_bb == 0);
		I2C1->DR = dev_adr | 0x01;
		
		while( !( I2C1->SR1 & I2C_SR1_ADDR ));
		//while (I2C2_SR1_ADDR_bb == 0);
		dummy = I2C1->SR2;
		
		I2C1->CR1 |= I2C_CR1_ACK;
		
		while( len )
		{
			if( len == 1 ) I2C1->CR1 &= ~I2C_CR1_ACK;
			while( !( I2C1->SR1 & I2C_SR1_RXNE ));
			*( dane++ ) = I2C1->DR;
		 len--;
		}
		I2C1->CR1 |= I2C_CR1_STOP;
	}



void i2c2_init(void)			{
		
		GPIOB->CRH = (GPIOB->CRH & 0xffff00ff) | 0x0000ea00;	// PB10=SCL(Push-pull),PB11=SDA (Open-drain)
		
		RCC_APB1ENR_I2C2EN_bb = 1;         	// enable clock for I2C2 module
		I2C2_CR1_SWRST_bb = 1;            	// force software reset of I2C peripheral
		I2C2_CR1_SWRST_bb = 0;
		I2C2->TRISE = 37;               		// limit slope
		I2C2->CCR = (1<<15)|(F_CPU/(8*100000));               			// setup speed (100kHz)
		I2C2->CCR = 178;               			// setup speed (100kHz)
		I2C2->CR2 = I2C_CR2_FREQ_36MHz;     // config I2C2 module
		I2C2_CR1_PE_bb = 1;               	// enable peripheral
	}

void i2c2_write(uint8_t dev_adr, uint8_t* data, uint32_t length){
		static uint32_t dummy;
		dummy = dummy;
		// Slave potwierdza odbior ACKiem
		I2C2_CR1_START_bb = 1;				// request a start
		while (I2C2_SR1_SB_bb == 0);	// wait for start to finish
		dummy = I2C2->SR1;						// read of SR1 clears the flag
		I2C2->DR = dev_adr;						// transfer address
		
		while (I2C2_SR1_ADDR_bb == 0);// wait for address transfer
		dummy = I2C2->SR1;						// clear the flag
		dummy = I2C2->SR2;						// clear the flag
	 
		while (length--)						// transfer whole block
		{
			while (I2C2_SR1_TxE_bb == 0);		// wait for DR empty
			I2C2->DR = *data++;							// trsnsfer one byte, increment pointer
		}
	 
		while (I2C2_SR1_TxE_bb == 0 || I2C2_SR1_BTF_bb == 1);	// wait for bus not-busy
		I2C2_CR1_STOP_bb = 1;					// request a stop
	}

void i2c2_read( u08 dev_adr, u08 reg_adres, u08 * dane, u08 len ) {
		static uint32_t dummy;
		dummy = dummy;
		// Master potwierdza odbior ACKiem
		
		//I2C2_CR1_START_bb = 1;							// request a start
		//while (I2C2_SR1_SB_bb == 0);				// wait for start to finish
		I2C2->CR1 |= I2C_CR1_START;						// request a start
		while( !( I2C2->SR1 & I2C_SR1_SB ));	// wait for start to finish (read of SR1 clears the flag)
		I2C2->DR = dev_adr; 										// transfer address
		
		while( !( I2C2->SR1 & I2C_SR1_ADDR ));
		//while (I2C2_SR1_ADDR_bb == 0);	// wait for address transfer
		dummy = I2C2->SR2;							// clear the flag
		
		while( !( I2C2->SR1 & I2C_SR1_TXE ));
		//while (I2C2_SR1_TxE_bb == 0);
		I2C2->DR = reg_adres;
		
		while( !( I2C2->SR1 & I2C_SR1_BTF ));
		//while (I2C2_SR1_BTF_bb == 0);
		I2C2->CR1 |= I2C_CR1_START;
		
		while( !( I2C2->SR1 & I2C_SR1_SB ));
		//while (I2C2_SR1_SB_bb == 0);
		I2C2->DR = dev_adr | 0x01;
		
		while( !( I2C2->SR1 & I2C_SR1_ADDR ));
		//while (I2C2_SR1_ADDR_bb == 0);
		dummy = I2C2->SR2;
		
		I2C2->CR1 |= I2C_CR1_ACK;
		
		while( len )
		{
			if( len == 1 ) I2C2->CR1 &= ~I2C_CR1_ACK;
			while( !( I2C2->SR1 & I2C_SR1_RXNE ));
			*( dane++ ) = I2C2->DR;
		 len--;
		}
		I2C2->CR1 |= I2C_CR1_STOP;
	}
void i2c2_scan(u08 * buf_adr, u08 * c_adr){
		char str[14];
		
		// ============= Hardware Scan ================================= //
		u08 dummy;
		dummy=dummy;
		// =========================== I2C2 Init ==============================
		
		GPIOB->CRH    = (GPIOB->CRH & 0xffff00ff) | 0x0000d900;	// SDA (Alternate Open-drain) SCL(Alternate Open-drain),
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		//I2C1_CR1_SWRST_bb = 1;            	// force software reset of I2C peripheral
		//I2C1_CR1_SWRST_bb = 0;
		
		I2C2->CR1    &= ~I2C_CR1_PE;					// I2C2 Pheripherial enable
		I2C2->TRISE   = 37;               		// limit slope
		I2C2->CR2     = 0;
		I2C2->CCR     = (F_CPU/(4*100000));   // setup speed (100kHz)
		I2C2->CR1    |= I2C_CR1_PE;						// I2C2 Pheripherial enable
		
		//LcdGotoXYFont ( 0, 0 );
		//LcdFStr ( FONT_1X,"Start... ");	LcdUpdate();
		*c_adr = 0;
		//trg0 = SysTick->VAL;
		for(u08 adr=0x0;adr<0xff;adr++){
			
			I2C2_CR1_START_bb = 1;				// request a start
			while (I2C2_SR1_SB_bb == 0);	// wait for start to finish
			dummy = I2C2->SR1;						// read of SR1 clears the flag
			I2C2->DR = adr;								// transfer address
			
			while (I2C2_SR1_ADDR_bb == 0){// wait for address transfer
				if(I2C2_SR1_AF_bb){
					 I2C2_SR1_AF_bb	= 0;			// must be
					break;
				}	
			}
			
			if(I2C2_SR1_ADDR_bb) {
				*buf_adr=adr;
				*c_adr += 1;
				*buf_adr++;
				//return ;
			}
			
			dummy = I2C2->SR1;						// must be - clear the flag
			dummy = I2C2->SR2;						// must be - clear the flag
			
			//Delay_ms(1);
		}
	}

void I2C2_READ_REG( u08 adres, u08 reg_adres, u08 * dane, u08 len ) {
	//uint32_t dummy;
	I2C2->CR1 |= I2C_CR1_START;
	while( !( I2C2->SR1 & I2C_SR1_SB ));
	I2C2->DR = adres;
	while( !( I2C2->SR1 & I2C_SR1_ADDR ));
	//dummy = I2C2->SR2;
	while( !( I2C2->SR1 & I2C_SR1_TXE ));
	I2C2->DR = reg_adres;
	while( !( I2C2->SR1 & I2C_SR1_BTF ));
	I2C2->CR1 |= I2C_CR1_START;
	while( !( I2C2->SR1 & I2C_SR1_SB ));
	I2C2->DR = adres | 0x01;
	while( !( I2C2->SR1 & I2C_SR1_ADDR ));
	//dummy = I2C2->SR2;

	while( len )
	{
		if( len == 1 )
			I2C2->CR1 &= ~I2C_CR1_ACK;
		while( !( I2C2->SR1 & I2C_SR1_RXNE ));
		*( dane++ ) = I2C2->DR;
		len--;
	}
	I2C2->CR1 |= I2C_CR1_STOP;
}
// I2C from SQUEEZ https://microgeek.eu/viewtopic.php?t=505
	/*
	typedef struct {
   uint8_t addr;
   uint8_t *data;
   uint8_t len;
   uint8_t state;
} T_i2c;

volatile T_i2c I2C_host;

void I2C_transmit(uint8_t addr, uint8_t *data, uint8_t len)
{
   I2C_host.addr = addr;
   I2C_host.data = data;
   I2C_host.len = len;
   I2C_host.state = 1;

   I2C1->CR2 |= I2C_CR2_ITEVTEN;
   I2C1->CR1 |= I2C_CR1_START;
}

void ssd1306_init(void)
{
   static const uint8_t buff[] = {0x00, 0xFE, 0x40}; // itp.   
   I2C_transmit(SSD1306_I2C_ADDRESS, buff, (sizeof buff / sizeof *buff) );
}

void I2C1_EV_IRQHandler(void)
{
   uint16_t tsr;

   if( (I2C1->SR1 & I2C_SR1_SB) )   //EV5
   {
      tsr = I2C1->SR1;
      I2C1->DR = I2C_host.addr;
      I2C_host.state = 2;
      return;
   }

   if( (I2C1->SR1 & I2C_SR1_ADDR) )   //EV6
   {
      tsr = I2C1->SR1;
      tsr = I2C1->SR2;
      I2C1->DR = I2C_host.data[0];   //EV8_1
      I2C_host.state = 3;
      return;
   }

   if( (I2C1->SR1 & I2C_SR1_TXE) )
   {
      if( (I2C_host.len+2)-I2C_host.state != 0 )
      {
         I2C1->DR = *(I2C_host.data + (I2C_host.state - 2));
         I2C_host.state++;
      } else {
            tsr = I2C1->SR1;
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->CR2 &= ~I2C_CR2_ITEVTEN;
            I2C_host.state = 0;
      }
   }
	}
*/
