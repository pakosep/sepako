/** \file gpio.h
 * \brief Header for gpio.c
 * \details Header for gpio.c
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2012-01-07
 */

/******************************************************************************
* chip: STM32F1x
* compiler: arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include "inc/stm32f10x.h"


/*
+=============================================================================+
| global definitions
+=============================================================================+
C1 C0 M1 M0		HEX
0	 0	0	 0		0x0		Input Analog 
0	 0	0	 1		0x1		Output Push-pull	10MHz
0	 0	1	 0		0x2		Output Push-pull	2MHz
0	 0	1	 1		0x3		Output Push-pull	50MHz

0	 1	0	 0		0x4		Input floating 
0	 1	0	 1		0x5		Output Open Drain	10MHz
0	 1	1	 0		0x6		Output Open Drain	2MHz
0	 1	1	 1		0x7		Output Open Drain	50MHz

1	 0	0	 0		0x8		Input  Pull Up/Down 
1	 0	0	 1		0x9		Output Alternate Push-pull	10MHz
1	 0	1	 0		0xA		Output Alternate Push-pull	2MHz
1	 0	1	 1		0xB		Output Alternate Push-pull	50MHz

1	 1	0	 0		0xC		Reserved 
1	 1	0	 1		0xD		Output Alternate Open-drain	10MHz
1	 1	1	 0		0xE		Output Alternate Open-drain	2MHz
1	 1	1	 1		0xF		Output Alternate Open-drain	50MHz

*/

/*
+=============================================================================+
| strange variables
+=============================================================================+
*/

/*
+=============================================================================+
| global variables
+=============================================================================+
*/

/*
+=============================================================================+
| global functions' declarations
+=============================================================================+
*/

void gpio_init(void);
void gpio_pin_cfg(GPIO_TypeDef *port_ptr, uint32_t pin, uint32_t mode_cnf_value);

/******************************************************************************
* END OF FILE
******************************************************************************/
#endif /* GPIO_H_ */
