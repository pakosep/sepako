/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SDIO_STM32F103.c
 *      Purpose: SD/SDIO MMC Interface Driver for ST STM32F103
 *      Rev.:    V4.13
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2010 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/


#include <stddef.h>
//#include <stm32f10x_lib.h>           /* STM32F10x Definitions                */
#include "inc/stm32f10x.h" 
//#include "File_Config.h"
#include "SDIO.h"
#include "inc/tool/delay.h"
#include "inc/tool/UARTLib.h"

/* SD Card communication speed                                               */
#define SD_CLK            24000000

/* Timeouts, in multiples of 6 byte send over SDIO (for 1 bit mode)          */
#define WR_TOUT           1000000     /* ~ 200 ms with SDIO clk 24MHz         */

#define DATA_RD_TOUT_VALUE  ( 10*(SD_CLK/1000)) /*  ~10ms at 24MHz SD clock  */
#define DATA_WR_TOUT_VALUE  (200*(SD_CLK/1000)) /* ~200ms at 24MHz SD clock  */

/* Wait time in for loop cycles                                              */
#define DMA_TOUT          100000	// 11ms dla F_CPU=128 MHz

#define SDIO_STA_RXDAVL_bb			bitband_t m_BITBAND_PERIPH(&SDIO->STA, 21)
#define SDIO_STA_RXFIFOHF_bb		bitband_t m_BITBAND_PERIPH(&SDIO->STA, 15)
#define SD_HALFFIFO 						((uint32_t)0x00000008)

#define SDIO_STATIC_FLAGS						0x00C007FF //0xC007ff

/* Local variables */
u08  CardType;
static u16 CardRCA;
static u32 sernum;

/*----------------------------------------------------------------------------
 *      MMC Driver Functions
 *----------------------------------------------------------------------------
 *  Required functions for MMC driver module:
 *   - bool mci_init        ()
 *   - bool mci_read_sect   (u32 sect, u08 *buf, u32 cnt)
 *   - bool mci_write_sect  (u32 sect, u08 *buf, u32 cnt)
 *   - bool mmc_read_config (MMCFG *cfg)
 *---------------------------------------------------------------------------*/

/* Local Function Prototypes */
static void mci_set_speed       (u32 speed);
static void mci_bus_mode        (u32 mode);
static bool mci_send_acmd       (void);
static bool mci_set_address     (void);
static bool mci_read_cid        (void);
static bool mci_select_card     (void);
static bool mci_set_bus_4bit    (void);
static bool mci_set_block_len   (void);
static bool mci_cmd_read_block  (u32 block, u32 cnt);
static bool mci_cmd_write_block (u32 block, u32 cnt);
static u32  mci_read_status     (void);
static bool mci_send_stop       (void);
static bool mci_wait_for_tran   (void);
static u32  mci_command         (u08 cmd, u32 arg, u32 resp, u32 *rp);
static void mci_dma_start       (u32 mode, u08 *buf, u32 cnt);

/*--------------------------- mci_init --------------------------------------*/

bool mci_init (void) {
		/* Initialize and enable the Flash Card. */
		u32 i,rstat,rval[4];
		
		RCC->AHBENR |= RCC_AHBENR_SDIOEN | RCC_AHBENR_DMA2EN;		// AHB clock enable SDIO, DMA2       
		
		//UaPutS(" ");								// Zapobiega wieszaniu gdy uruchomiony flash
		/* PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */ 
		GPIOC->CRH   = (GPIOC->CRH & 0xFFF00000) | 0x000BBBBB;   /* Alternate Function PushPull 50MHz */
		
		/* Configure PD.02 CMD line */
		GPIOD->CRL   = (GPIOD->CRL & 0xFFFFF0FF) | 0x00000B00; /* Alternate Function PushPull 50MHz */
		
		/* Clear all pending interrupts. */
		SDIO->CMD     = 0;   
		SDIO->DCTRL   = 0;
		SDIO->ICR     = 0x00C007FF; 
		
		/* Power up, switch on VCC for the Flash Card. */
		SDIO->POWER   = 0x03;
		for (i = 0; i < 50000; i++);
		
		mci_set_speed (LOW_SPEED);
		
		/* Reset the card, send CMD0. */
		mci_command (GO_IDLE_STATE, 0, RESP_NONE, NULL);
		
		/* Initialize CardType as unknown. */
		CardType = CARD_NONE;
		CardRCA  = 0;
		
		/* Check for SD card, clear Open Drain output control. */
		mci_bus_mode (PUSH_PULL_MODE);
		
		/* Check if card is V2.00 or later SD memory card (CMD8). */
		rstat = mci_command (SEND_IF_COND, 0x1AA, RESP_SHORT, &rval[0]);
		if (!(rstat & SDIO_STA_CTIMEOUT) && (rval[0] == 0x1AA)) {
			CardType = CARD_SDV2;
		}
		
		/* Check if SD card, send ACMD41. */
		for (i = 0; i < 500; i++) {
			if (mci_send_acmd () == __TRUE) {
				rstat = mci_command (SEND_APP_OP_COND, OCR_INDEX | ((CardType == CARD_SDV2) << 30), RESP_SHORT, &rval[0]);
				if (!(rstat & SDIO_STA_CTIMEOUT) && (rval[0] & 0x80000000)) {
					/* OK, SD card initialized. */
					if (CardType != CARD_SDV2)
						CardType = CARD_SD;
					break;
				}
			}
		}
		
		/* Check if card is SDHC card (Read OCR (CMD58)). */
		if (CardType == CARD_SDV2) {
			rstat = mci_command (READ_OCR, 0, RESP_SHORT, &rval[0]);
			/* Check Card Capacity Status (CCS). */
			if (/*!(rstat & SDIO_STA_CTIMEOUT) && */(rval[0] & (1 << 30))) {
				CardType = CARD_SDV2_HC;
			}
		}
		
		/* If card is not SD check if it is MMC card. */
		if (CardType == CARD_NONE) {
			/* Set Open Drain output control for MMC. */
			mci_bus_mode (OPEN_DRAIN_MODE);
			
			/* Check for MMC, send CMD1. */
			for (i = 0; i < 100; i++) {
				rstat = mci_command (SEND_OP_COND, OCR_INDEX, RESP_SHORT, &rval[0]);
				if (!(rstat & SDIO_STA_CTIMEOUT) && (rval[0] & 0x80000000)) {
					CardType = CARD_MMC;
					break;
				}
			}
		}
		
		if (CardType == CARD_NONE) {
			/* Failed, no card found. */
			goto fail;
		}
		
		/* Initialize the Card to SD/MMC mode. */
		if (mci_read_cid () == __FALSE) {
			goto fail;
		}
		if (mci_set_address () == __FALSE) {
			goto fail;
		}
		
		/* Disable Open Drain mode for MMC. */
		if (CardType == CARD_MMC) {
			mci_bus_mode (PUSH_PULL_MODE);
		}
		
		/* Data Transfer Mode, end of Card-Identification Mode. */
		mci_set_speed (HIGH_SPEED);
		
		if (mci_select_card () == __FALSE) {
			goto fail;
		}
		
		if (CardType >= CARD_SD) {
			/* Use wide 4-bit bus for SD */
			SDIO->CLKCR |= 0x0800;
			for (i = 0; i < 100; i++);
			if (mci_set_bus_4bit () == __FALSE) {
				/* Failed to enable 4-bit bus. */
				goto fail;
			}
		}
		/* Set block length to 512 bytes. */
		if (mci_set_block_len () == __FALSE) {
	 fail:   SDIO->POWER = 0x00;
			return (__FALSE);
		}
		
		/* Success, card initialized. */
		return (__TRUE);
	}


/*--------------------------- mci_set_speed ---------------------------------*/

static void mci_set_speed (u32 speed) {
		/* Set SDIO clock speed to desired value. SDIO clock = SDIOCLK/[CLKDIV+2] */
		u32 i,clkdiv;
		
		if (speed == HIGH_SPEED) {
			/* Max. 24 MBit used for Data Transfer. */
			/*clkdiv =   1;*/                        /* 72MHz  / (  1 + 2) =  24MHz  */
			clkdiv =   3;                            /* 120MHz / (  3 + 2) =  24MHz  */
		}
		else {
			/* Max. 400 kBit used in Card Initialization. */
			/* clkdiv = 178; */                      /* 72MHz  / (178 + 2) = 400KHz  */
			clkdiv = 255;                            /* 120MHz / (255 + 2) = 400KHz  */
		}
		SDIO->CLKCR = (SDIO->CLKCR & ~0xFF) |
															 (1 << 8) |     /* enable PWRSAV                */
															 (1 << 9) |     /* enable CLKEN                 */
																clkdiv;
		
		/* Short delay before next write (7 HCLK) */
		for (i = 0; i < 1000; i++);
	}


/*--------------------------- mci_bus_mode ----------------------------------*/

static void mci_bus_mode (u32 mode) {
		/* Set MCI Bus mode to Open Drain or Push Pull. */
		u32 i;
		
		if (mode == OPEN_DRAIN_MODE) {
			;  /* not configurable */
		}
		else {
			;  /* not configurable */
		}
		/* A short delay after switching mode. */
		for (i = 0; i < 100; i++);
	}

/*--------------------------- mci_send_acmd ---------------------------------*/

static bool mci_send_acmd (void) {
		/* Send CMD55 to enable ACMD */
		u32 arg,rstat,rval;
		
		arg = 0;
		if (CardType >= CARD_SD) {
			/* Use address from SET_RELATIVE_ADDR. */
			arg = CardRCA << 16;
		}
		rstat = mci_command (APP_CMD, arg, RESP_SHORT, &rval);
		if ((rstat == 0) && (rval & STAT_ACMD_ENABLE)) {
			return (__TRUE);
		}
		return (__FALSE);
	}

/*--------------------------- mci_set_address -------------------------------*/

static bool mci_set_address (void) {
		/* Set Relative Address, send CMD3 after CMD2. */
		u32 i,arg,rstat,rval;
		
		arg = 0;
		if (CardType == CARD_MMC) {
			/* Fix the RCA address for MMC card. */
			arg = 0x00010000;
		}
		for (i = 0; i < 50; i++) {
			rstat = mci_command (SET_RELATIVE_ADDR, arg, RESP_SHORT, &rval);
			if (!(rstat & SDIO_STA_CTIMEOUT) && ((rval & 0x0F00) == 0x0500)) {
				/* Response is back and correct. */
				CardRCA = rval >> 16;
				return (__TRUE);
			}
		}
		return (__FALSE);
	}


/*--------------------------- mci_read_cid ----------------------------------*/

static bool mci_read_cid (void) {
  /* Check CID, send CMD2 after CMD1 (MMC) or ACMD41 (SD). */
  u32 i,rstat,rval[4];

  for (i = 0; i < 50; i++) {
    rstat = mci_command (ALL_SEND_CID, 0, RESP_LONG, &rval[0]);
    if (!(rstat & SDIO_STA_CTIMEOUT)) {
      /* Response is back and correct. */
      if (CardType >= CARD_SD) {
        /* Serial Number for SD Card. */
        sernum = (rval[2] << 8)  | (rval[3] >> 24);
      }
      else {
        /* Serial Number for MMC Card. */
        sernum = (rval[2] << 16) | (rval[3] >> 16);
      }
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*--------------------------- mci_select_card -------------------------------*/

static bool mci_select_card (void) {
  /* Select the Card, send CMD7 after CMD9, inter-change state */
  /* between STBY and TRANS after this command. */
  u32 i,arg,rstat,rval;

  arg = 0x00010000;
  if (CardType >= CARD_SD) {
    /* Use address from SET_RELATIVE_ADDR. */
    arg = CardRCA << 16;
  }

  for (i = 0; i < 200; i++) {
    rstat = mci_command (SELECT_CARD, arg, RESP_SHORT, &rval);
    if ((rstat == 0) && ((rval & 0x0F00) == 0x0700)) {
      /* Should be in STBY state now and ready. */
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*--------------------------- mci_set_bus_4bit ------------------------------*/

static bool mci_set_bus_4bit (void) {
  /* Select 4-bit bus width for SD Card. */
  u32 i,rstat,rval;

  for (i = 0; i < 50; i++) {
    if (mci_send_acmd () == __FALSE) {
      continue;
    }
    /* Send ACMD6 command to set the bus width. */
    rstat = mci_command (SET_ACMD_BUS_WIDTH, BUS_WIDTH_4BITS, RESP_SHORT, &rval);
    if ((rstat == 0) && ((rval & 0x0F00) == 0x0900)) {
      /* Response is back and correct. */
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*--------------------------- mci_set_block_len -----------------------------*/

static bool mci_set_block_len (void) {
  /* Set block length to 512 bytes. */
  u32 i,rstat,rval;

  for (i = 0; i < 50; i++) {
    /* Send ACMD6 command to set the bus width. */
    rstat = mci_command (SET_BLOCK_LEN, 512, RESP_SHORT, &rval);
    if ((rstat == 0) && ((rval & 0x0F00) == 0x0900)) {
      /* Response is back and correct. */
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*--------------------------- mci_cmd_read_block ----------------------------*/

static bool mci_cmd_read_block (u32 block, u32 cnt) {
		/* Send a command to Read Single/Multiple blocks. */
		u32 i,rstat,rval;
		u08  cmd;
		
		cmd = READ_BLOCK;
		if (cnt > 1) {
			cmd = READ_MULT_BLOCK;
		}
		if (CardType < CARD_SDV2_HC) 
		block <<= 9;
		
		for (i = 0; i < 50; i++) {
			rstat = mci_command (cmd, block, RESP_SHORT, &rval);
			if (rstat == 0 && (rval & 0x0F00) == 0x0900) {
				/* Ready and in TRAN state. */
				return (__TRUE);
			}
		}
		return (__FALSE);
	}

/*--------------------------- mci_cmd_write_block ---------------------------*/

static bool mci_cmd_write_block (u32 block, u32 cnt) {
		/* Send a command to Write Single/Multiple block. */
		u32 i,rstat,rval;
		u08  cmd;
		
		cmd = WRITE_BLOCK;
		if (cnt > 1) {
			cmd = WRITE_MULT_BLOCK;
		}
		if (CardType < CARD_SDV2_HC) 
			block <<= 9;
		
		for (i = 0; i < 50; i++) {
			rstat = mci_command (cmd, block, RESP_SHORT, &rval);
			if (rstat == 0 && (rval & 0x0F00) == 0x0900) {
				/* Ready and in TRAN state. */
				return (__TRUE);
			}
		}
		return (__FALSE);
	}

/*--------------------------- mci_read_status -------------------------------*/

static u32 mci_read_status (void) {
  /* Read the status of Flash Card. */
  u32 i,arg,rstat,rval;

  arg = 0x00010000;
  if (CardType >= CARD_SD) {
    /* Use address from SET_RELATIVE_ADDR. */
    arg = CardRCA << 16;
  }

  for (i = 0; i < 200; i++) {
    rstat = mci_command (SEND_STATUS, arg, RESP_SHORT, &rval);
    if ((rstat == 0) && (rval & 0x0100)) {
      /* The Ready bit should be set, state TRAN or RCV. */
      return (rval);
    }
  }
  return (SDIO_STA_RESP_INVALID);
}


/*--------------------------- mci_send_stop ---------------------------------*/

static bool mci_send_stop (void) {
  /* Stop transmission, Flash Card is in wrong state. */
  u32 i,rstat,rval;

  for (i = 0; i < 20; i++) {
    rstat = mci_command (STOP_TRANSMISSION, 0, RESP_SHORT, &rval);
    if ((rstat == 0) && (rval & 0x0100)) {
      /* The Ready bit should be set. */
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*--------------------------- mci_wait_for_tran -----------------------------*/

static bool mci_wait_for_tran (void) {
  /* Wait for Card state TRAN. */
  u32 i;

  for (i = WR_TOUT; i; i--) {
    /* Wait for Card state TRAN to continue. */
    if ((mci_read_status () & 0x0F00) == 0x0900) {
      break;
    }
  }
  if (i == 0) {
    /* Previous request has Failed. */
    mci_send_stop ();
    return (__FALSE);
  }
  return (__TRUE);
}


/*--------------------------- mmc_command -----------------------------------*/

static u32 mci_command (u08 cmd, u32 arg, u32 resp_type, u32 *rp) {
		/* Send a Command to Flash card and get a Response. */
		u32 cmdval,stat;
		
		cmd   &= 0x3F;
		cmdval = 0x400 | cmd;
		switch (resp_type) {
			case RESP_SHORT:
				cmdval |= 0x40;
				break;
			case RESP_LONG:
				cmdval |= 0xC0;
				break;
		}
		/* Send the command. */
		SDIO->ARG = arg;
		SDIO->CMD = cmdval;
		
		if (resp_type == RESP_NONE) {
			/* Wait until command finished. */
			while (SDIO->STA & SDIO_STA_CMDACT);
			SDIO->ICR   = 0x00C007FF; 
			return (0);
		}
		
		for (;;) {
			stat = SDIO->STA;
			if (stat & SDIO_STA_CTIMEOUT) {
				SDIO->ICR = stat & SDIO_STA_CLEAR_MASK;
				return (stat);
			}
			if (stat & SDIO_STA_CCRCFAIL) {
				SDIO->ICR = stat & SDIO_STA_CLEAR_MASK;
				if ((cmd == SEND_OP_COND)      ||
						(cmd == SEND_APP_OP_COND)  ||
						(cmd == STOP_TRANSMISSION)) {
					SDIO->CMD = 0;
					break;
				}
				return (stat);
			}
			if (stat & SDIO_STA_CMDREND) {
				SDIO->ICR = stat & SDIO_STA_CLEAR_MASK;
				break;
			}
		}
		if ((SDIO->RESPCMD & 0x3F) != cmd) {
			if ((cmd != SEND_OP_COND)     &&
					(cmd != SEND_APP_OP_COND) &&
					(cmd != ALL_SEND_CID)     &&
					(cmd != SEND_CSD))         {
				return (SDIO_STA_RESP_INVALID);
			}
		}
		if (rp == NULL) {
			/* Response pointer undefined. */
			return (SDIO_STA_RESP_INVALID);
		}
		
		/* Read MCI response registers */
		rp[0] = SDIO->RESP1;
		if (resp_type == RESP_LONG) {
			rp[1] = SDIO->RESP2;
			rp[2] = SDIO->RESP3;
			rp[3] = SDIO->RESP4;
		}
		
		return (0);
	}


/*--------------------------- mci_dma_start ---------------------------------*/

static void mci_dma_start (u32 mode, u08 *buf, u32 cnt) {
		/* configure DMA2 controller channel4 for read or write */ 
		
		DMA2_Channel4->CCR &= ~(1 << 0);             /* disable channel4          */
		DMA2_Channel4->CPAR = (u32)&(SDIO->FIFO);
		DMA2_Channel4->CMAR = (u32)buf;
		/* The burst size set to 8, transfer size cnt*512 bytes. */
		DMA2_Channel4->CNDTR = ((cnt*512)/4);     /* trans. size cnt*512 bytes */
		
		if (mode == DMA_READ) {
			/* Transfer from SDIO-FIFO to memory. */
			DMA2_Channel4->CCR = (2<<12) |             /* Channel Priority High     */
													 (2<<10) |             /* Memory     size  32-bits  */
													 (2<< 8) |             /* Peripheral size  32-bits  */
													 (1<< 7) |             /* Memory increment enabled  */
													 (0<< 4);              /* Read from peripheral      */
		}
		else {
			/* Transfer from memory to SDIO-FIFO. */
			DMA2_Channel4->CCR = (2<<12) |             /* Channel Priority High     */
													 (2<<10) |             /* Memory     size  32-bits  */
													 (2<< 8) |             /* Peripheral size  32-bits  */
													 (1<< 7) |             /* Memory increment enabled  */
													 (1<< 4);              /* Read from memory          */
		}
		/* Enable DMA channels, little endian */
		DMA2->IFCR = (0xF<<12);                      /* clear all channel IRQs    */
		DMA2_Channel4->CCR |= (1 << 0);              /* enable channel4           */
	}


/*--------------------------- mci_read_sect ---------------------------------*/

bool mci_read_sect (u32 sect, u08 *buf, u32 cnt) {
		/* Read one or more 512 byte sectors from Flash Card. */
		u32 i;
		
		if (mci_wait_for_tran () == __FALSE) {
			/* Card not in TRAN state. */
			return (__FALSE);
		}
		
		if (mci_cmd_read_block (sect, cnt) == __FALSE) {
			/* Command Failed. */
			return (__FALSE);
		}
		
		/* Set SDIO Transfer registers. */
		SDIO->DTIMER  = DATA_RD_TOUT_VALUE;
		SDIO->DLEN    = cnt * 512;
		
		/* Start DMA Peripheral to Memory transfer. */
		mci_dma_start (DMA_READ, buf, cnt);
		
		SDIO->DCTRL = 0x9B;
		
		for (i = DMA_TOUT; i; i--) {
			if (DMA2->ISR & (1<<13)) {
				DMA2->IFCR = (0x0f<<12);          // clear all channel IRQs    
				break;														// Data transfer finished. 
			}
		}
		
		
		if (i == 0) {
			return (__FALSE);										// DMA Transfer timeout. 
		}
		
		
		if (cnt > 1) {
			/* Stop reading Multiple sectors. */
			mci_send_stop ();
		}
		return (__TRUE);
	}


/*--------------------------- mci_read_sect_without-dma ---------------------------------*/
bool mci_read_sect_(u32 sect, u08 *buf, u32 cnt) {
		/* Read one or more 512 byte sectors from Flash Card. */
		
		uint32_t *dest_buff = (uint32_t *) buf;
		uint32_t flags,sta,count;
		
		if (mci_wait_for_tran () == __FALSE) {
			/* Card not in TRAN state. */
			return (__FALSE);
		}
		
		if (mci_cmd_read_block (sect, cnt) == __FALSE) {
			/* Command Failed. */
			return (__FALSE);
		}
		
		/* Set SDIO Transfer registers. */
		SDIO->DTIMER  = DATA_RD_TOUT_VALUE;
		//SDIO->DTIMER = 0xffffff;
		SDIO->DLEN   = cnt * 512;
		
		/* Start DMA Peripheral to Memory transfer. */
		//mci_dma_start (DMA_READ, buf, cnt);
		
		SDIO->DCTRL = (9 << 4) | SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR ;
		
		/*
		//UaPutK("\r\n flag1 = "); uint2uart(flags );
		while (0==(flags = (SDIO->STA & (SDIO_STA_RXOVERR | SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_DATAEND | SDIO_STA_STBITERR))))
    {
      if (SDIO_STA_RXDAVL_bb)
      {
        *dest_buff++ = SDIO->FIFO;
      }
      
    }
		*/
		
		while (!(flags = ( SDIO->STA & (SDIO_STA_RXOVERR | SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_DBCKEND | SDIO_STA_STBITERR))))
		{
			if (SDIO_STA_RXFIFOHF_bb)
			{
				for (count = 0; count < SD_HALFFIFO; count++)
				{
					*(dest_buff + count) = SDIO->FIFO;
				}
				dest_buff += SD_HALFFIFO;
			}
		}
		
		
    if (flags & SDIO_STA_DATAEND)
    {
      //flags = send_cmd_short_resp(0,CMD12);
			mci_send_stop ();
			flags = SDIO->STA;
      if(SDIO->STA & SDIO_STA_CTIMEOUT)
      {
        //sd_err=5;
        return __FALSE;
      }
    }
    else
    {
      //sd_err=6;
      return __FALSE;
    }
		
		SDIO->ICR = SDIO_STATIC_FLAGS;
		/*
		if (cnt > 0) {
			// Stop reading Multiple sectors. 
			mci_send_stop ();
		} */
		return (__TRUE);
		
	}


/*--------------------------- mci_write_sect --------------------------------*/

bool mci_write_sect (u32 sect, u08 *buf, u32 cnt) {
		/* Write a cnt number of 512 byte sectors to Flash Card. */
		u32 i,j;
		
		if (mci_wait_for_tran () == __FALSE) {
			/* Card not in TRAN state. */
			return (__FALSE);
		}
		
		if (mci_cmd_write_block (sect, cnt) == __FALSE) {
			/* Command Failed. */
			return (__FALSE);
		}
		
		for (j = 0; j < cnt; buf += 512, j++) {
			/* Set SDIO Transfer registers. */
			SDIO->DTIMER  = DATA_WR_TOUT_VALUE;
			SDIO->DLEN    = 512;
			
			/* Start DMA Memory to Peripheral transfer. */
			mci_dma_start (DMA_WRITE, buf, 1);
			SDIO->DCTRL = 0x99;
			
			for (i = DMA_TOUT; i; i--) {
				if (DMA2->ISR & (1<<13)) {
						DMA2->IFCR = (0xF<<12);             /* clear all channel IRQs    */
					/* Data transfer finished. */
					break;
				}
			}
			
			if (i == 0) {
				/* DMA Data Transfer timeout. */
				mci_send_stop ();
				/* Write request Failed. */
				return (__FALSE);
			}
			
			if (cnt == 1) {
				return (__TRUE);
			}
			
			/* Wait until Data Block sent to Card. */
			while (SDIO->STA != (SDIO_STA_DATAEND | SDIO_STA_DBCKEND)) {
				if (SDIO->STA & (SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT)) {
					/* If error while Data Block sending occured. */
					mci_send_stop ();
					/* Write request Failed. */
					return (__FALSE);
				}
			}
		}
		mci_send_stop ();
		return (__TRUE);
	}


/*--------------------------- mci_read_config -------------------------------*/

bool mci_read_config (MMCFG *cfg) {
  /* Read MMC/SD Card device configuration. */
  u32 i,rstat,arg,v,m,rval[4];

  /* Wait if potential Write in progress. */
  mci_wait_for_tran ();

  /* Deselect the Card, transit to STBY state. */
  mci_command (SELECT_CARD, 0, RESP_NONE, NULL);

  /* Read the CID - Card Identification. */
  cfg->sernum = sernum;

  /* Read the CSD - Card Specific Data. */
  arg = 0x00010000;
  if (CardType >= CARD_SD) {
    /* Use address from SET_RELATIVE_ADDR. */
    arg = CardRCA << 16;
  }

  for (i = 20; i; i--) {
    rstat = mci_command (SEND_CSD, arg, RESP_LONG, &rval[0]);
    if (rstat == 0) {
      /* Response is back and correct. */
      break;
    }
  }
  if (i == 0) {
    /* Read CSD failed. */
    return (__FALSE);
  }

  /* Read Block length. */
  v = (rval[1] >> 16) & 0x0F;
  cfg->read_blen = 1 << v;

  /* Write Block length */
  v = (rval[3] >> 22) & 0x0F;
  cfg->write_blen = 1 << v;

  /* Total Number of blocks */
  switch ((rval[0] >> 30) & 3) {        /* Bits 126..127      */
    case 0:                             /* SD card,  CSD v1.0 */
    case 2:                             /* MMC card, CSD v1.2 */
      v = ((rval[1] << 2) | (rval[2] >> 30)) & 0x0FFF;
      m =  (rval[2] >> 15) & 0x07;
      cfg->blocknr = (v + 1) << (m + 2);
      break;
    case 1:                             /* SD card,  CSD v2.0 */
      v = ((rval[1] << 16) | (rval[2] >> 16)) & 0xFFFF;
      cfg->blocknr = (v + 1) << 10;
      break;
    default:
      return (__FALSE);
  }

  /* Re-select the Card, back to TRAN state. */
  return (mci_select_card ());
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
