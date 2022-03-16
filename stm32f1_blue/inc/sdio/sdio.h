/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SDIO_STM32F103.h
 *      Purpose: SD/SDIO MMC Interface Driver for ST STM32F103 Definitions
 *      Rev.:    V4.13
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2010 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/
//#include <stdbool.h>
#include "inc/sdspi/integer.h"

#ifndef __SDIO_STM32X_H
#define __SDIO_STM32X_H

extern U8  CardType;

/* Macros for compatibility with existing SPI interface. */
#define mci_init            mmc_init
#define mci_read_sect       mmc_read_sect
#define mci_write_sect      mmc_write_sect
#define mci_read_config     mmc_read_config

/* Expected Response from mci_command */
#define RESP_NONE           0
#define RESP_SHORT          1
#define RESP_LONG           2

/* MMC/SD Commands */
#define GO_IDLE_STATE       0       /* Resetthe card to idle state  (MMC,SD) */
#define SEND_OP_COND        1       /* Send Op.Cond. Register       (MMC)    */
#define ALL_SEND_CID        2       /* Send Card CID number         (MMC,SD) */
#define SET_RELATIVE_ADDR   3       /* Set Relative Address         (MMC,SD) */
#define SET_ACMD_BUS_WIDTH  6
#define SELECT_CARD         7       /* Select/Deselect the Card     (MMC,SD) */
#define SEND_IF_COND        8       /* Send Interface Condition         (SD) */
#define SEND_CSD            9       /* Send Card Specific Data      (MMC,SD) */
#define SEND_CID            10      /* Send Card Identification Data(MMC,SD) */
#define STOP_TRANSMISSION   12      /* Stop Read or Write transm.   (MMC,SD) */
#define SEND_STATUS         13      /* Send Status Resiger          (MMC,SD) */
#define SET_BLOCK_LEN       16      /* Set Block Length in bytes    (MMC,SD) */
#define READ_BLOCK          17      /* Read a Single Block          (MMC,SD) */
#define READ_MULT_BLOCK     18      /* Read Multiple Blocks         (MMC,SD) */
#define WRITE_BLOCK         24      /* Write a Block                (MMC,SD) */
#define WRITE_MULT_BLOCK    25      /* Write Multiple Blocks        (MMC,SD) */
#define SEND_APP_OP_COND    41      /* Send App. Op.Cond Register       (SD) */
#define APP_CMD             55      /* App.Specific Command follow  (MMC,SD) */
#define READ_OCR            58      /* Read OCR Register            (MMC,SD) */

/* Card Status definitions */
#define STAT_ACMD_ENABLE    0x00000020
#define STAT_RDY_DATA       0x00000100
#define STAT_CURRENT_STATE  0x00001E00
#define STAT_ERASE_RESET    0x00002000

#define CARD_NONE       0
#define CARD_MMC        1
#define CARD_SD         2
#define CARD_SDV2       3
#define CARD_SDV2_HC    4

/* SD Card Bus Width definitions */
#define BUS_WIDTH_1BIT      0
#define BUS_WIDTH_4BITS     10

/* MMC/SD definitions */
#define OCR_INDEX           0x00FF8000

/* SDIO Status register bit information */
//#define SDIO_STA_CCRCFAIL       0x00000001  /* Command response received (CRC check failed)  */
//#define SDIO_STA_DCRCFAIL       0x00000002  /* Data block sent/received (CRC check failed)   */
//#define SDIO_STA_CTIMEOUT       0x00000004  /* Command response timeout.                     */
//#define SDIO_STA_DTIMEOUT       0x00000008  /* Data timeout                                  */
//#define SDIO_STA_TXUNDERR       0x00000010  /* Transmit FIFO underrun error                  */
//#define SDIO_STA_RXOVERR        0x00000020  /* Received FIFO overrun error                   */
//#define SDIO_STA_CMDREND        0x00000040  /* Command response received (CRC check passed)  */
//#define SDIO_STA_CMDSENT        0x00000080  /* Command sent (no response required)           */
//#define SDIO_STA_DATAEND        0x00000100  /* Data end (data counter, SDIDCOUNT, is zero)   */
//#define SDIO_STA_STBITERR       0x00000200  /* Start bit not detected                        */
//#define SDIO_STA_DBCKEND        0x00000400  /* Data block sent/received (CRC check passed)   */
//#define SDIO_STA_CMDACT         0x00000800  /* Command transfer in progress                  */
//#define SDIO_STA_TXACT          0x00001000  /* Data transmit in progress                     */
//#define SDIO_STA_RXACT          0x00002000  /* Data receive in progress                      */
//#define SDIO_STA_TXFIFOHE       0x00004000  /* Transmit FIFO Half Empty                      */
//#define SDIO_STA_RXFIFOHF       0x00008000  /* Receive FIFO Half Full                        */
//#define SDIO_STA_TXFIFOF        0x00010000  /* Transmit FIFO full                            */
//#define SDIO_STA_RXFIFOF        0x00020000  /* Receive FIFO full                             */
//#define SDIO_STA_TXFIFOE        0x00040000  /* Transmit FIFO empty                           */
//#define SDIO_STA_RXFIFOE        0x00080000  /* Receive FIFO empty                            */
//#define SDIO_STA_TXDAVL         0x00100000  /* Data available in transmit FIFO               */
//#define SDIO_STA_RXDAVL         0x00200000  /* Data available in receive FIFO                */
//#define SDIO_STA_SDIOIT         0x00400000  /*  SDIO interrupt received                      */
//#define SDIO_STA_CEATAEND       0x00800000  /* CE-ATA cmd completion sig. received for CMD61 */

#define SDIO_STA_RESP_INVALID   0xFFFFFFFF
#define SDIO_STA_CLEAR_MASK     0x00C007FF

/* General mode definitions */
#define LOW_SPEED           0
#define HIGH_SPEED          1
#define OPEN_DRAIN_MODE     0
#define PUSH_PULL_MODE      1
#define DMA_READ            0
#define DMA_WRITE           1

/* MMC device configuration */ 
typedef struct { 
   U32 sernum; 
   U32 blocknr; 
   U16 read_blen; 
   U16 write_blen; 
} MMCFG; 

bool mci_init (void);
bool mci_read_sect 		(U32 sect, U8 *buf, U32 cnt);
bool mci_write_sect		(U32 sect, U8 *buf, U32 cnt);
bool mci_read_config 	(MMCFG *cfg);

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
