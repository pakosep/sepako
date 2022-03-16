#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"
#include "softspi.h"
#include "inc/tool/delay.h"


/* a byte transfer in (0,0) mode */
uint8_t softSPI_sr0(uint8_t byte)
	{
		uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			if (byte & 0x80)			sMOSI = 1;
			else			sMOSI = 0;
			
			byte <<= 1;
			sSCK = 1; /* a slave latches input data bit */
			Delay_us(10);
			if (sMISO)			byte |= 0x01;
			
			sSCK = 0; /* a slave shifts out next output data bit */
			Delay_us(10);
		}
		return(byte);
	}

/* a byte transfer in (1,1) mode */
uint8_t softSPI_sr1(uint8_t byte)
	{
		uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			if (byte & 0x80)			sMOSI = 1;
			else				sMOSI = 0;
			
			sSCK = 0; /* a slave shifts out output data bit */
			byte <<= 1;
			
			if (sMISO)		byte |= 0x01;
			sSCK = 1; /* a slave latches input data bit */
		}
		return(byte);
	}
	
	uint8_t inverse_byte(uint8_t byte)
	{
		uint8_t mask = 1, result = 0;
		while(mask)
		{
			if (byte & 0x80)
			result |= mask;
			mask <<= 1;
			byte <<= 1;
		}
		return(result);
	}