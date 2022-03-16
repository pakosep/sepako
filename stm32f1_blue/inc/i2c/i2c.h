#ifndef __I2C_H
#define __I2C_H

#include "inc/stm32f10x.h"
#include "config.h"

#define I2C1_CR1_SWRST_bb			bitband_t m_BITBAND_PERIPH(&I2C1->CR1,15)
#define I2C1_CR1_PE_bb				bitband_t m_BITBAND_PERIPH(&I2C1->CR1,0)
#define I2C1_CR1_START_bb			bitband_t m_BITBAND_PERIPH(&I2C1->CR1,8)
#define I2C1_SR1_SB_bb			  bitband_t m_BITBAND_PERIPH(&I2C1->SR1,0)
#define I2C1_SR1_ADDR_bb			bitband_t m_BITBAND_PERIPH(&I2C1->SR1,1)
#define I2C1_SR1_TxE_bb			  bitband_t m_BITBAND_PERIPH(&I2C1->SR1,7)
#define I2C1_SR1_BTF_bb			  bitband_t m_BITBAND_PERIPH(&I2C1->SR1,2)
#define I2C1_CR1_STOP_bb			bitband_t m_BITBAND_PERIPH(&I2C1->CR1,9)
#define I2C1_SR1_AF_bb			  bitband_t m_BITBAND_PERIPH(&I2C1->SR1,10)

#define I2C2_CR1_SWRST_bb			bitband_t m_BITBAND_PERIPH(&I2C2->CR1,15)
#define I2C2_CR1_PE_bb				bitband_t m_BITBAND_PERIPH(&I2C2->CR1,0)
#define I2C2_CR1_START_bb			bitband_t m_BITBAND_PERIPH(&I2C2->CR1,8)
#define I2C2_SR1_SB_bb			  bitband_t m_BITBAND_PERIPH(&I2C2->SR1,0)
#define I2C2_SR1_ADDR_bb			bitband_t m_BITBAND_PERIPH(&I2C2->SR1,1)
#define I2C2_SR1_TxE_bb			  bitband_t m_BITBAND_PERIPH(&I2C2->SR1,7)
#define I2C2_SR1_BTF_bb			  bitband_t m_BITBAND_PERIPH(&I2C2->SR1,2)
#define I2C2_CR1_STOP_bb			bitband_t m_BITBAND_PERIPH(&I2C2->CR1,9)
#define I2C2_SR1_AF_bb			  bitband_t m_BITBAND_PERIPH(&I2C2->SR1,10)

#define I2C_CR2_FREQ_36MHz		0b00100100
#define I2C_WR_ADDR						0x3C
#define I2C_RD_ADDR						0x3D

#define DS3231_ADDR						0x68<<1	 // 0x68<<1 = 0xd0
#define INA219_ADDR						0x40<<1  // w dokumentacji domyslny adres 0x40

void i2c1r_init(void);
void i2c1_init(void);
void i2c1_read( u08 dev_addr, u08 reg_addr, u08 * data, u08 len );
void i2c1_write(u08 dev_addr, u08* data, uint32_t len);

void i2c2_init(void);
void i2c2_read( u08 dev_addr, u08 reg_addr, u08 * data, u08 len );
void i2c2_write(u08 dev_addr, u08* data, uint32_t len);

void i2c2_scan(u08 * buf_adr,u08 *);

#endif 
