//#include "includes.h"
#include "enc28j60.h"
#include "inc/spi/spi.h"
#include <stdio.h>
//#include "inc/tool/UARTLib.h"
#include "inc/tool/delay.h"

static u08 Enc28j60Bank;
static int16_t gNextPacketPtr;	// Wskaznik pakietu


u08 enc28j60ReadOp(u08 op, u08 address)
	{
		u08 dat;
		ENC28J60_CSL();
		
		//dat = op | (address & ADDR_MASK);
		SPI1_ReadWrite(op | (address & ADDR_MASK));
		dat = SPI1_ReadWrite(0x00);
		// do dummy read if needed (for mac and mii, see datasheet page 29)
		if(address & 0x80)
		{
			dat = SPI1_ReadWrite(0x00);
		}
		// release CS
		ENC28J60_CSH();
		return dat;
	}

void enc28j60WriteOp(u08 op, u08 address, u08 data)
	{
		//u08 dat;
			
		ENC28J60_CSL();
		// issue write command
		//dat = op | (address & ADDR_MASK);
		SPI1_ReadWrite(op | (address & ADDR_MASK));
		// write data
		//dat = data;
		SPI1_ReadWrite(data);
		ENC28J60_CSH();
	}

void enc28j60_spi_Init(void)
	{
		GPIOA->CRL = (GPIOA->CRL & 0x000fffff) | 0xb4b00000;	// PA5~7
		GPIOC->CRL = (GPIOC->CRL & 0xfff0ffff) | 0x00030000;	// PC4 enc28_CS
		GPIOE->CRL = (GPIOE->CRL & 0xfffffff0) | 0x00000004;	// PE0 enc28_INT
		
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_BR_2|	SPI_CR1_MSTR;	// 0x037c
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_MSTR;	
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_0|SPI_CR1_MSTR;	
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_MSTR;	
		//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_MSTR;	// 10 MHz max
		SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2|SPI_CR1_MSTR;	// 10 MHz max
		SPI1->CR1 |= SPI_CR1_SPE;		
	}
void enc28j60PowerDown() 
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
		while(enc28j60Read(ESTAT) & ESTAT_RXBUSY);
		while(enc28j60Read(ECON1) & ECON1_TXRTS);
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PWRSV);
	}
void enc28j60PowerUp() 
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
		while(!enc28j60Read(ESTAT) & ESTAT_CLKRDY);
	}
void enc28j60ReadBuffer(uint16_t len, u08* data)
	{
		ENC28J60_CSL();
		// issue read command
		SPI1_ReadWrite(ENC28J60_READ_BUF_MEM);
		while(len--)
			{
					//len--;
					// read data
					*data++ = (u08)SPI1_ReadWrite(0x00);
					//data++;
			}
		*data='\0';
		ENC28J60_CSH();
	}

void enc28j60WriteBuffer(uint16_t len, u08* data)
	{
		ENC28J60_CSL();
		// issue write command
		SPI1_ReadWrite(ENC28J60_WRITE_BUF_MEM);
		
		while(len--)		{
				//len--;
				SPI1_ReadWrite(*data++);
				//data++;
			}
		ENC28J60_CSH();
	}

void enc28j60SetBank(u08 address)
	{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
		{
        // set the bank
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
        Enc28j60Bank = (address & BANK_MASK);
		}
	}

u08 enc28j60Read(u08 address)
	{
		// set the bank
		enc28j60SetBank(address);
		// do the read
		return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
	}

void enc28j60Write(u08 address, u08 data)
	{
		// set the bank
		enc28j60SetBank(address);
		// do the write
		enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
	}

void enc28j60PhyWrite(u08 address, uint16_t data)
	{
		// set the PHY register address
		enc28j60Write(MIREGADR, address);
		// write the PHY data
		enc28j60Write(MIWRL, data);
		enc28j60Write(MIWRH, data>>8);
		// wait until the PHY write completes
		while(enc28j60Read(MISTAT) & MISTAT_BUSY)
		{
			Delay_us(10);
			//_nop_();
		}
	}

u16 enc28j60PhyRead(u08 address)
	{
		
		// set the PHY register address
		enc28j60Write(MIREGADR, address);
		enc28j60Write(MICMD, MICMD_MIIRD);
		
		Delay_us(15);
		// set MICMD.MIIRD bit
		//enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MICMD, (MICMD_MIIRD));
		
		while(enc28j60Read(MISTAT) & MISTAT_BUSY);
		
		//enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, MICMD, (MICMD_MIIRD));
		enc28j60Write(MICMD, 0x00);
		
		return ((enc28j60Read(MIRDH)<<8)|enc28j60Read(MIRDL));
		//return (enc28j60Read(MIRDH));
	}

void enc28j60clkout(u08 clk)
	{
    //setup clkout: 2 is 12.5MHz:
	enc28j60Write(ECOCON, clk & 0x7);
	}

void enc28j60Init(u08* macaddr)
	{
		
		GPIOE->CRL = (GPIOE->CRL & 0xffffff0f) | 0x00000020;	// PE1 enc28_reset
		enc28j60_spi_Init();
		ENC28J60_CSH();	      
		
		ENC28J60_RES = 0;
		Delay_ms(1);
		ENC28J60_RES = 1;
		Delay_ms(5);
		// perform system reset
		enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
		Delay_ms(20);
		// check CLKRDY bit to see if reset is complete
		// The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
		//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
		// do bank 0 stuff
		// initialize receive buffer
		// 16-bit transfers, must write low byte first
		// set receive buffer start address
		
		gNextPacketPtr = RXSTART_INIT;
		// Rx start
		enc28j60Write(ERXSTL,  RXSTART_INIT&0xFF);	 //
		enc28j60Write(ERXSTH,  RXSTART_INIT>>8);
		// set receive pointer address
		enc28j60Write(ERXRDPTL,RXSTART_INIT&0xFF);
		enc28j60Write(ERXRDPTH,RXSTART_INIT>>8);
			// RX end
		enc28j60Write(ERXNDL,  RXSTOP_INIT&0xFF);
		enc28j60Write(ERXNDH,  RXSTOP_INIT>>8);
			// TX start
		enc28j60Write(ETXSTL,  TXSTART_INIT&0xFF);
		enc28j60Write(ETXSTH,  TXSTART_INIT>>8);
			// TX end
		enc28j60Write(ETXNDL,  TXSTOP_INIT&0xFF);
		enc28j60Write(ETXNDH,  TXSTOP_INIT>>8);
		// do bank 1 stuff, packet filter:
        // For broadcast packets we allow only ARP packtets
        // All other packets should be unicast only for our mac (MAADR)
        //
        // The pattern to match on is therefore
        // Type     ETH.DST
        // ARP      BROADCAST
        // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
        // in binary these poitions are:11 0000 0011 1111
        // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
		enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
		enc28j60Write(EPMM0,   0x3f);
		enc28j60Write(EPMM1,   0x30);
		enc28j60Write(EPMCSL,  0xf9);
		enc28j60Write(EPMCSH,  0xf7);
		enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
		enc28j60Write(MACON2,  0x00);
		
		//enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX);
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
		// set inter-frame gap (non-back-to-back)
		enc28j60Write(MAIPGL,  0x12);
		enc28j60Write(MAIPGH,  0x0C);
		// set inter-frame gap (back-to-back)
		enc28j60Write(MABBIPG, 0x12);
		enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);	
		enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
			// do bank 3 stuff
      // write MAC address
      // NOTE: MAC address in ENC28J60 is byte-backward
		enc28j60Write(MAADR5,  macaddr[0]);	
		enc28j60Write(MAADR4,  macaddr[1]);
		enc28j60Write(MAADR3,  macaddr[2]);
		enc28j60Write(MAADR2,  macaddr[3]);
		enc28j60Write(MAADR1,  macaddr[4]);
		enc28j60Write(MAADR0,  macaddr[5]);
		
		
		
		//enc28j60PhyWrite(PHCON1, PHCON1_PDPXMD);
		// no loopback of transmitted frames
		enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
		// switch to bank 0
		enc28j60SetBank(ECON1);
		// enable interrutps
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
		// enable packet reception
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
		
		// LEDA=Green, LEDB=Orange
		 enc28j60PhyWrite(PHLCON, (0b0000<<12)|(0b1100<<8)|(0b0111<<4)|(0b0110) );
		
	}

u08 enc28j60getrev(void)
	{
		uint8_t rev;
    rev=enc28j60Read(EREVID);
    // microchip forgott to step the number on the silcon when they
    // released the revision B7. 6 is now rev B7. We still have
    // to see what they do when they release B8. At the moment
    // there is no B8 out yet
    if (rev>5) rev++;
		return(rev);
		//return(enc28j60Read(EREVID));
	}

// dhcp_client.c needs general broadcast
#ifdef ENC28J60_BROADCAST
// A number of utility functions to enable/disable general broadcast (not just arp)
void enc28j60EnableBroadcast( void ) {
    enc28j60Write(ERXFCON, (uint8_t)((enc28j60Read(ERXFCON) | ERXFCON_BCEN)));
	}
void enc28j60DisableBroadcast( void ) {
    enc28j60Write(ERXFCON, enc28j60Read(ERXFCON) & (0xff ^ ERXFCON_BCEN));
	}
#endif
// link status
uint8_t enc28j60linkup(void)
	{
    // PHSTAT1 LLSTAT (= bit 2 in lower reg), PHSTAT1_LLSTAT
    // LLSTAT is latching, that is: if it was down since last
    // calling enc28j60linkup then we get first a down indication
    // and only at the next call to enc28j60linkup it will come up.
    // This way we can detect intermittened link failures and
    // that might be what we want.
    // The non latching version is LSTAT.
    // PHSTAT2 LSTAT (= bit 10 in upper reg)
    if (enc28j60PhyRead(PHSTAT2) & (1<<10) ){
    //if (enc28j60PhyRead(PHSTAT1) & PHSTAT1_LLSTAT){
      return(1);
    }
    return(0);
	}


void enc28j60PacketSend(uint16_t len, uint8_t* packet)
	{
    // Check no transmit in progress
    while (enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS);
		// 
		// Reset the transmit logic problem. Unblock stall in the transmit logic.
    // See Rev. B4 Silicon Errata point 12.
		if( (enc28j60Read(EIR) & EIR_TXERIF) ) {
			enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
			//enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF); 
		}
		// Set the write pointer to start of transmit buffer area
		enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
		enc28j60Write(EWRPTH, TXSTART_INIT>>8);
		// Set the TXND pointer to correspond to the packet size given
		enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
		enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);
		// write per-packet control byte (0x00 means use macon3 settings)
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
		// copy the packet into the transmit buffer
		enc28j60WriteBuffer(len, packet);
		// send the contents of the transmit buffer onto the network
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
	}

// just probe if there might be a packet
uint8_t enc28j60hasRxPkt(void)
	{
		if( enc28j60Read(EPKTCNT) ==0 ){
			return(0);
		}
		return(1);
	}
// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
// maxlen  The maximum acceptable length of a retrieved packet.
// packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60PacketReceive(uint16_t maxlen, u08* packet)
	{
		unsigned int rxstat;
		unsigned int len;
		
		// check if a packet has been received and buffered
		//if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
		// The above does not work. See Rev. B4 Silicon Errata point 6.
		if( enc28j60Read(EPKTCNT) ==0 )  
		{
			return(0);
		}
		
		// Set the read pointer to the start of the received packet
		enc28j60Write(ERDPTL, (gNextPacketPtr &0xFF));
		enc28j60Write(ERDPTH, (gNextPacketPtr)>>8);
		
		// read the next packet pointer
		gNextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
		gNextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
		
		// read the packet length (see datasheet page 43)
		len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
		len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
		
		len-=4; //remove the CRC count
		// read the receive status (see datasheet page 43)
		rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
		rxstat |= ((uint16_t)enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0))<<8;
		
		// limit retrieve length
    if (len>maxlen-1)
		{
      len=maxlen-1;
    }
		
    // check CRC and symbol errors (see datasheet page 44, table 7-3):
    // The ERXFCON.CRCEN is set by default. Normally we should not
    // need to check this.
    if ((rxstat & 0x80)==0)
		{
		  len=0;
		}
		else
		{
      // copy the packet from the receive buffer
      enc28j60ReadBuffer(len, packet);
    }
		// Move the RX read pointer to the start of the next received packet
		// This frees the memory we just read out
		
		
		if (gNextPacketPtr -1 > RXSTOP_INIT){ // RXSTART_INIT is zero, no test for gNextPacketPtr less than RXSTART_INIT.
      enc28j60Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
      enc28j60Write(ERXRDPTH, (RXSTOP_INIT)>>8);
    } else {
      enc28j60Write(ERXRDPTL, (gNextPacketPtr-1)&0xFF);
      enc28j60Write(ERXRDPTH, (gNextPacketPtr-1)>>8);
    }
		
		// decrement the packet counter indicate we are done with this packet
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
		return(len);
	}



