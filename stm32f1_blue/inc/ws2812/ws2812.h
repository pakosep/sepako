/*
 * light weight WS2812 lib - ARM Cortex M0/M0+ version
 *
 * Created: 07.07.2013
 *  Author: Tim (cpldcpu@gmail.com)
 */

#ifndef LIGHT_WS2812_H_
#define LIGHT_WS2812_H_

///////////////////////////////////////////////////////////////////////
// User defined area: Select CPU type
///////////////////////////////////////////////////////////////////////
//#define LIGHT_WS2812_LPC8XX
//#define LIGHT_WS2812_STM32L0XX
#define LIGHT_WS2812_STM32F1XX

///////////////////////////////////////////////////////////////////////
// User defined area: CPU specific CMSIS include
///////////////////////////////////////////////////////////////////////

#ifdef LIGHT_WS2812_STM32L0XX
  #include "stm32l0xx_hal.h"
  #define LIGHT_WS2812_STM32
#endif
#ifdef LIGHT_WS2812_STM32F1XX
  #include "inc/stm32f10x.h"
  #define LIGHT_WS2812_STM32
#endif

///////////////////////////////////////////////////////////////////////
// User defined area: Define I/O pin
///////////////////////////////////////////////////////////////////////

#define 	GPIO_PIN_0   		((uint16_t)0x0001)
#define 	GPIO_PIN_1   		((uint16_t)0x0002)
#define 	GPIO_PIN_2   		((uint16_t)0x0004)
#define 	GPIO_PIN_3   		((uint16_t)0x0008)
#define 	GPIO_PIN_4   		((uint16_t)0x0010)
#define 	GPIO_PIN_5   		((uint16_t)0x0020)
#define 	GPIO_PIN_6   		((uint16_t)0x0040)
#define 	GPIO_PIN_7   		((uint16_t)0x0080)
#define 	GPIO_PIN_8   		((uint16_t)0x0100)
#define 	GPIO_PIN_9   		((uint16_t)0x0200)
#define 	GPIO_PIN_10   	((uint16_t)0x0400)
#define 	GPIO_PIN_11   	((uint16_t)0x0800)
#define 	GPIO_PIN_12   	((uint16_t)0x1000)
#define 	GPIO_PIN_13   	((uint16_t)0x2000)
#define 	GPIO_PIN_14   	((uint16_t)0x4000)
#define 	GPIO_PIN_15   	((uint16_t)0x8000)
#define 	GPIO_PIN_All  	((uint16_t)0xFFFF)
#define 	IS_GPIO_PIN(PIN)   ((((PIN) & (uint16_t)0x00) == 0x00) && ((PIN) != (uint16_t)0x00))
#define 	IS_GET_GPIO_PIN(PIN)

// Data port and pin definition for your CPU. Depending on the way
// port access is implemented in your Cortex CPU, the set and clr
// mask and addresses may be the same.

#ifdef LIGHT_WS2812_STM32F1XX
  // This example is for STM32 family
  #define ws2812_port_set ((uint32_t*)&GPIOA->BSRR)  // Address of the data port register to set the pin
  #define ws2812_port_clr ((uint32_t*)&GPIOA->BRR) // Address of the data port register to clear the pin

  #define ws2812_mask_set  GPIO_PIN_7   // Bitmask to set the data out pin
  #define ws2812_mask_clr  GPIO_PIN_7   // Bitmask to clear the data out pin
#endif
///////////////////////////////////////////////////////////////////////
// User defined area: Define CPU clock speed
//
// The current implementation of the sendarray routine uses cycle accurate
// active waiting. The routine is automatically adjusted according to
// the clockspeed defined below. Only values between 8 Mhz and 60 Mhz
// are allowable.
//
// Important: The timing calculation assumes that there are no waitstates
// for code memory access. If there are waitstates you may have to reduce
// the value below until you get acceptable timing. It is highly recommended
// to use this library only on devices without flash waitstates and
// predictable code execution timing.
//
///////////////////////////////////////////////////////////////////////

#define ws2812_cpuclk 32000000

///////////////////////////////////////////////////////////////////////
// End user defined area
///////////////////////////////////////////////////////////////////////

#if (ws2812_cpuclk<8000000)
	#error "Minimum clockspeed for ARM ws2812 library is 8 Mhz!"
#endif

#if (ws2812_cpuclk>60000000)
	#error "Maximum clockspeed for ARM ws2812 library is 60 Mhz!"
#endif


///////////////////////////////////////////////////////////////////////
// Main function call
//
// Call with address to led color array (order is Green-Red-Blue)
// Numer of bytes to be transmitted is leds*3
///////////////////////////////////////////////////////////////////////

void ws2812_sendarray(uint8_t *ledarray,int length);

#endif /* LIGHT_WS2812_H_ */
