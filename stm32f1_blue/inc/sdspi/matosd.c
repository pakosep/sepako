/*-----------------------------------------------------------------------*/
/* MMC/SDSC/SDHC (in SPI mode) control module for STM32 Version 1.1.6    */
/* (C) Martin Thomas, 2010 - based on the AVR MMC module (C)ChaN, 2007   */
/*-----------------------------------------------------------------------*/

/* Copyright (c) 2010, Martin Thomas, ChaN
   All rights reserved.
*/

#include "inc/stm32f10x.h"
#include "ffconf.h"
#include "matosd.h"
#include "hdr/hdr_bitband.h"
#include "inc/tool/UARTLib.h"
#include "inc/tool/delay.h"

#define USE_MINI_STM32

 #define SPI_SD                   SPI1
	
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      0
 #define SOCKET_CP_CONNECTED      0
 
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOC
 #define GPIO_Pin_CS              GPIO_Pin_13
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel2
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_FLAG_TC2
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_FLAG_TC3
 #define GPIO_SPI_SD              GPIOA
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB2PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB2Periph_SPI1
 /* - for SPI1 and full-speed APB2: 72MHz/4 */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_4

static volatile DSTATUS Stat = STA_NOINIT;		/* Disk status 	if 0 OK */
volatile u08 		Timer1, Timer2, SPI_CLK;						/* 1000Hz decrement timers 		*/
uint8_t 				CardType;											/* Card type flags 						*/

 #define FCLK_SLOW() 	{ SPI_CLK = 5; }	/* Set SCLK = PCLK / 256 */
 #define FCLK_FAST() 	{ SPI_CLK = 0; }	/* Set SCLK = PCLK / 8 */
 
// u08 BuffeR[512];

static const DWORD socket_state_mask_cp = (1 << 0);
static const DWORD socket_state_mask_wp = (1 << 1);

/*-----------------------------------------------------------------------*/
/* Power Control and interface-initialization (Platform dependent)       */
/*-----------------------------------------------------------------------*/
static void init_spi (void)	{
		/* De-select the Card: Chip Select high */
		CS_HIGH();
		GPIOC->CRH = (GPIOC->CRH & 0xfff00ff0) | 0x00033004;	// 8-MISO,11-SD_CS,12-SCK
		GPIOD->CRL = (GPIOD->CRL & 0xfffff0ff) | 0x00000300;	// 2-MOSI
	}
/*-----------------------------------------------------------------------*/
/* Transmit/Receive a byte to MMC via SPI  (Platform dependent)          */
/*-----------------------------------------------------------------------*/
static inline u08 xchg_spi(u08 byte){ 
		
		uint8_t counter;
		for(counter = 8; counter; counter--)
		{
			
			sMOSI = (byte & 0x80) ? 1 : 0;
			byte <<= 1;
			Delay_us(SPI_CLK);
			sSCK = 1; /* a slave latches input data bit */
			if (sMISO)			byte |= 0x01;
			Delay_us(SPI_CLK);
			sSCK = 0; /* a slave shifts out next output data bit */
			
		}
		return(byte);
	}

/* Alternative macro to receive data fast */
#define xmit_spi(dat)  xchg_spi(dat)
/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/

/* Alternative macro to receive data fast */
//#define rcvr_spi()  xchg_spi(0xff)

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/
static u08 wait_ready (UINT wt	) /* Timeout [ms]  Zwiekszono w stosunku do oryginalu x10 */
	{
		BYTE d;
		Timer2 = wt/500;	  /* Wait for ready in timeout  x10 of 500ms  (1000ms) */
		do {
		d = xchg_spi(0xFF);
		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
		} while (d != 0xFF && Timer2);	/* Wait for card goes ready or timeout */
		
		return (d == 0xFF) ? 1 : 0;			/* 1:Ready, 0:Timeout */
	}

/* Receive multiple byte */
static void rcvr_spi_multi (
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive (even number) */
) {
		do {																		/* Receive the data block into buffer */
			*buff++ = xchg_spi(0xff);
		} while (btr--);
	}
#if _USE_WRITE
/* Send multiple byte */
	
static void xmit_spi_multi (
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send (even number) */
)
	{
		u08 d;
		do {																		/* Receive the data block into buffer */
			d = *buff++;
			xchg_spi(d);
		} while (btx--);
	}
#endif
/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)  *(dst)=xchg_spi(0xff)
/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static
void deselect (void)
	{
		CS_HIGH();		/* Set CS# high */
		xchg_spi(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
	}
/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/
static
int select (void)	/* 1:OK, 0:Timeout */
	{
		//Delay_us(2);			// 2us
		CS_LOW();		/* Set CS# low */
		//__NOP(); __NOP();__NOP();__NOP();
		xchg_spi(0xFF);	/* Dummy clock (force DO enabled) */
		if (wait_ready(500)) return 1;	/* Wait for card ready */
		
		deselect();
		return 0;	/* Timeout */
	}

/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/
static
int rcvr_datablock (	/* 1:OK, 0:Error */
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
	{
		BYTE token;
		Timer1 = 2;							/* Wait for DataStart token in timeout of 200ms (1s) */
		do {							
			token = xchg_spi(0xFF);
			/* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */
		} while ((token == 0xFF) && Timer1);
		if(token != 0xFE) return 0;		/* Function fails if invalid DataStart token or timeout */
		
		rcvr_spi_multi(buff, btr);		/* Store trailing data to the buffer */
		xchg_spi(0xFF); xchg_spi(0xFF);			/* Discard CRC */
		
		return 1;						/* Function succeeded */
	}

/*-----------------------------------------------------------------------*/
/* Send a data packet to SD                                             */
/*-----------------------------------------------------------------------*/
#if _USE_WRITE
static
int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* Ponter to 512 byte data to be sent */
	BYTE token			/* Token */
)
	{
		BYTE resp,cnt=4;
		
		if (!wait_ready(500)) return 0;		/* Wait for card ready */
		
		xchg_spi(token);					/* Send token */
		if (token != 0xFD) {				/* Send data if token is other than StopTran */
			xmit_spi_multi(buff, 512);			/* Data */
			
			do{
				resp = xchg_spi(0xFF);				/* Receive data resp */
				if(!cnt--) return 0;
			}
			while ((resp & 0x1F) != 0x05);		/* Function fails if the data packet was not accepted */
			
		}
		return 1;
	}
#endif

/*========= Public Functions ============================================*/

/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (		/* Return value: R1 resp (bit7==1:Failed to send) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
	{
		BYTE n, res;
		if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
			cmd &= 0x7F;
			res = send_cmd(CMD55, 0);
			if (res > 1) return res;		// return res
		}
		/* Select the card and wait for ready except to stop multiple block read */
		if (cmd != CMD12) {
			deselect();
			if (!select()) return 0xFF; // return 0xFF
		}
		/* Send command packet */
		xchg_spi(0x40 | cmd);						/* Start + command index */
		xchg_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
		xchg_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
		xchg_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
		xchg_spi((BYTE)arg);						/* Argument[7..0] */
		n = 0x01;												/* Dummy CRC + Stop */
		if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
		if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
		xchg_spi(n);

		/* Receive command resp */
		if (cmd == CMD12) xchg_spi(0xFF);	/* Diacard following one byte when CMD12 */
		n = 10;								/* Wait for response (10 bytes max) */
		do
			res = xchg_spi(0xFF);
		while ((res & 0x80) && --n);
		return res;							/* Return received response */
	}

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_init (
	BYTE drv		/* Physical drive number (0) */
)
	{
		BYTE n, cmd, ty, ocr[4];
		
		if (drv) return STA_NOINIT;			/* Supports only drive 0 */
		FCLK_SLOW();
		init_spi();											/* Initialize SPI */
		
		// if (Stat & STA_NODISK) return Stat;	/* Is card existing in the soket? */
		//CS_LOW();
		for (n = 10; n; n--) xchg_spi(0xFF);	/* Send 80 dummy clocks */
		//for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */
		//Delay_us(50);
		
		ty = 0;
		if (send_cmd(CMD0, 0) == 1) {			/* Put the card SPI/Idle state */
			Timer1 = 1;						/* Initialization timeout = 1 sec */
			if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
				for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);	/* Get 32 bit return value of R7 resp */
				if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* Is the card supports vcc of 2.7-3.6V? */
					while (Timer1 && send_cmd(ACMD41, 1UL << 30)) ;	/* Wait for end of initialization with ACMD41(HCS) */
					if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
						for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
						ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* Card id SDv2 */
					}
				}
			} else {	/* Not SDv2 card */
				if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMC? */
					ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
				} else {
					ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
				}
				while (Timer1 && send_cmd(cmd, 0)) ;		/* Wait for end of initialization */
				if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
					ty = 0;
			}
		}
		CardType = ty;	/* Card type */
		deselect();
		
		if (ty) {			/* OK */
			FCLK_FAST();			/* Set fast clock */
			Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
		} else {			/* Failed */
			Stat = STA_NOINIT;
		}
		
		return ty;
	}
/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (	u08 drv	)	/* Physical drive number (0) */
	{
		if (drv) return STA_NOINIT;		/* Supports only single drive */
		return Stat;
	}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,		/* Physical drive number (0) */
	BYTE *buff,		/* Pointer to the data buffer to store read data */
	DWORD sector,	/* Start sector number (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
	{
		if (drv || !count) return RES_PARERR;		/* Check parameter */
		if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */
		
		if (!(CardType & CT_BLOCK)) sector *= 512;	/* LBA ot BA conversion (byte addressing cards) */
		
		if (count == 1) {	/* Single sector read */
			
			if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512))
			count = 0;
		}
		else {				/* Multiple sector read */
			if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
				do {
					if (!rcvr_datablock(buff, 512)) break;
					buff += 512;
				} while (--count);
				send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
			}
		}
		deselect();
		
		return count ? RES_ERROR : RES_OK;	/* Return result */
	}
/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE drv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Ponter to the data to write */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
	{
		if (drv || !count) return RES_PARERR;		/* Check parameter */
		if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check drive status */
		if (Stat & STA_PROTECT) return RES_WRPRT;	/* Check write protect */
		
		if (!(CardType & CT_BLOCK)) sector *= 512;	/* LBA ==> BA conversion (byte addressing cards) */
		
		if (count == 1) {	/* Single sector write */
			
			if ((send_cmd(CMD24, sector) == 0) && xmit_datablock(buff, 0xFE))
			count = 0;	
		}
		else {				/* Multiple sector write */
			if (CardType & CT_SDC) send_cmd(ACMD23, count);	/* Predefine number of sectors */
			if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
				do {
					if (!xmit_datablock(buff, 0xFC)) break;
					buff += 512;
				} while (--count);
				if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
					count = 1;
			}
		}
		deselect();

		return count ? RES_ERROR : RES_OK;	/* Return result */
	}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
#define _USE_IOCTL 1
#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive number (0) */
	BYTE cmd,		/* Control command code */
	void *buff		/* Pointer to the conrtol data */
)
	{
		DRESULT res;
		BYTE n, csd[16], *ptr=buff;
		DWORD *dp, st, ed, csize;
		
		if (drv) return RES_PARERR;					/* Check parameter */
		if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */
		
		res = RES_ERROR;
		
		switch (cmd) {
			case CTRL_SYNC :		/* Wait for end of internal write process of the drive */
			if (select()) res = RES_OK;
			break;
			
			case GET_SD_STATUS :												/* Get erase block size in unit of sector (DWORD) */
				if (CardType & CT_SD2) {									/* SDC ver 2.00 */
					if (send_cmd(ACMD13, 0) == 0) {					/* Read SD status */
						xchg_spi(0xFF);
						if (rcvr_datablock(buff, 64)) {				/* Read partial block */
							res = RES_OK;
						}
					}
				}
			break;
			case GET_SD_CMD9 :												/* Get erase block size in unit of sector (DWORD) */
				if (send_cmd(CMD9, 0) == 0 && rcvr_datablock(buff, 16) ) {					/* Read SD status */
					
					res = RES_OK;
				}
			break;
			case GET_SECTOR_COUNT :	/* Get drive capacity in unit of sector (DWORD) */
				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
					if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
						csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
						*(DWORD*)buff = csize << 10;
					} else {					/* SDC ver 1.XX or MMC ver 3 */
						n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
						csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
						*(DWORD*)buff = csize << (n - 9);
					}
					res = RES_OK;
				}
			break;
			
			case GET_BLOCK_SIZE :															// Get erase block size in unit of sector (DWORD) 
				if (CardType & CT_SD2) {												// SDC ver 2.00 
					if (send_cmd(ACMD13, 0) == 0) {								// Read SD status 
						xchg_spi(0xFF);
						if (rcvr_datablock(csd, 16)) {							// Read partial block 
							for (n = 64 - 16; n; n--) xchg_spi(0xFF);	// Purge trailing data 
							*(DWORD*)buff = 16UL << (csd[10] >> 4);
							res = RES_OK;
						}
					}
				} else {																				 							// SDC ver 1.XX or MMC 
					if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	// Read CSD */
						if (CardType & CT_SD1) {																	// SDC ver 1.XX 
							*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
						} else {																									// MMC 
							*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
						}
						res = RES_OK;
					}
				}
			break;
			case MMC_GET_CSD :							/* Receive CSD as a data block (16 bytes) */
				if ( send_cmd(CMD9, 0) == 0		/* READ_CSD */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;
			
			case CTRL_TRIM :																/* Erase a block of sectors (used when _USE_ERASE == 1)  48504 */
			if (!(CardType & CT_SDC)) break;								/* Check if the card is SDC */
			if (disk_ioctl(drv, MMC_GET_CSD, csd) ) break;	/* Get CSD */
			if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;	/* Check if sector erase can be applied to the card */
			dp = buff; st = dp[0]; ed = dp[1];							/* Load sector block */
			if (!(CardType & CT_BLOCK)) {
				st *= 512; ed *= 512;
			}
			UaPutK("\r\n s = "); hex2uart(st,8);
			UaPutK("\r\n e = "); hex2uart(ed,8);
			if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(3000))	/* Erase sector block */
				res = RES_OK;																	/* FatFs does not check result of this command */
			break;
			
			default:
			res = RES_PARERR;
		}
		deselect();
		return res;
	}
#endif
/*-----------------------------------------------------------------------*/
/* Device Timer Interrupt Procedure  (Platform dependent)                */
/*-----------------------------------------------------------------------*/
DRESULT SD_writeSingleBlock(	u08 *buff, DWORD sector){
		return disk_write (	0,buff, sector,	1);
	}
DRESULT SD_readSingleBlock(	u08 *buff, DWORD sector){
		return disk_read (	0,buff, sector,	1);
	}