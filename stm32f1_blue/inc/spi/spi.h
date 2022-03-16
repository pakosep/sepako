#ifndef __SPI_H
#define __SPI_H

#include "inc/stm32f10x.h"
#include "config.h"

typedef struct ws2812b_color {
	uint8_t 	red;
	uint8_t green;
	uint8_t  blue;
} ws2812b_color;

void 	SPI1_init(void);
void  SPI1_init_(void);
void 	SPI2_init(void);
u16 	SPI1_master(u16 cmd); 
u08 	SPI2_slave(u08 cmd);
void 	SPI2MemDMAInit(void *dst_buf);

uint16_t spi1_rw(uint16_t data);
uint16_t spi2_rw(uint16_t data);

u08 SPI1_ReadWrite(u08 data);

void crc16_spi2_init(void);
void crc16_spi2(u16 * data,u16 len,u16 *crc );

void crc16_spi1_init(void);
void crc16_spi1(u16 * data,u16 len, u16 *crc );

//void crc8_spi2_init(void);
//void crc8_spi2 (u08 * data,u08 len,u08 *crc );

#endif 