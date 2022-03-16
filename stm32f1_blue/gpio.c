/** \file gpio.c
 * \brief GPIO driver.
 * \details GPIO default configuration and function for configuring a pin
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2012-01-07
 */

/******************************************************************************
* chip: STM32F1x
* compiler: arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
*
* prefix: gpio_
*
* available global functions:
* 	void gpio_init(void)
* 	void gpio_pin_cfg(GPIO_TypeDef *port_ptr, uint32_t pin, uint32_t mode_cnf_value)
*
* available local functions:
*
* available interrupt handlers:
******************************************************************************/

/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdint.h>
#include "inc/stm32f10x.h"
#include "hdr/hdr_gpio.h"
#include "gpio.h"

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

/*
+=============================================================================+
| global functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief GPIO initialization.
* \details Enables AFIO, all GPIO ports and sets them to input with pull-downs.
*//*-------------------------------------------------------------------------*/

void gpio_init(void)
{
	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
			RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN | RCC_APB2ENR_AFIOEN;	// enable all GPIOs and AFIO

	// set all ports to input with pull-down
	GPIOA->CRL = 0x88888888;
	GPIOA->CRH = 0x88888888;
	GPIOA->ODR = 0;
	GPIOB->CRL = 0x88888888;
	GPIOB->CRH = 0x88888888;
	GPIOB->ODR = 0;
	GPIOC->CRL = 0x88888888;
	GPIOC->CRH = 0x88888888;
	GPIOC->ODR = 0;
	GPIOD->CRL = 0x88888888;
	GPIOD->CRH = 0x88888888;
	GPIOD->ODR = 0;
	GPIOE->CRL = 0x88888888;
	GPIOE->CRH = 0x88888888;
	GPIOE->ODR = 0;
}

/*------------------------------------------------------------------------*//**
* \brief Configures pin.
* \details Configures one pin in one port.
*
* \param [in] port_ptr points to the configuration structure of desired port
* \param [in] pin selects one pin, [0,15]
* \param [in] mode_cnf_value is a value of MODE and CNF which will be set for
* selected pin, {GPIO_IN_ANALOG,
* GPIO_IN_FLOATING, GPIO_IN_PULL_U_D,
* GPIO_OUT_PP_2M,   GPIO_OUT_PP_10M,
* GPIO_OUT_PP_50M,  GPIO_OUT_OD_2M,
* GPIO_OUT_OD_10M,  GPIO_OUT_OD_50,
* GPIO_ALT_PP_2M,   GPIO_ALT_PP_10M,
* GPIO_ALT_PP_50M,  GPIO_ALT_OD_2M,
* GPIO_ALT_OD_10M,  GPIO_ALT_OD_50M} or
* use m_GPIO_MODE_CNF_value(mode, cnf) macro
*//*-------------------------------------------------------------------------*/

void gpio_pin_cfg(GPIO_TypeDef *port_ptr, uint32_t pin, uint32_t mode_cnf_value)
{
	volatile uint32_t *cr_ptr;
	uint32_t cr_value;

	cr_ptr = &port_ptr->CRL;				// configuration of pins [0,7] is in CRL

	if (pin >= 8)							      // is pin in [8; 15]?
	{										            // configuration of pins [8,15] is in CRH
		cr_ptr++;							        // advance to next struct element CRL -> CRH
		pin -= 8;							        // crop the pin number
	}

	cr_value = *cr_ptr;						  // localize the CRL / CRH value
	cr_value &= ~(GPIO_CRx_MODE_CNF_mask << (pin * 4));	// clear the MODE and CNF fields (now that pin is an analog input)
	cr_value |= (mode_cnf_value << (pin * 4));	// save new MODE and CNF value for desired pin

	*cr_ptr = cr_value;						  // save localized value to CRL / CRL
}

/*tmf obsluga enkodera
unsigned char i=0;
   
    if((PINC & _BV(PC0))==0) i++;  //PC0 i PE7 to piny do których pod³¹czony jest enkoder
    if((PINE & _BV(PE7))==0) i^=3; // Konwersja kodu Graya do zwyk³ego binarnego
   
    unsigned char enc_last=EncoderState & 3; //Tylko 2 bity s¹ istotne
   i-=enc_last;   //Poprzednia pozycja enkodera, potrzebne do okreœlenia kierunku
    enc_last+=i; //Do ostatniej dodajemy bie¿¹c¹
        EncoderState&=0xFC;
    EncoderState|=enc_last; //EncoderState by³o potrzebne w moim programie, tu mo¿na to wywaliæ i wykorzystaæ enc_last deklaruj¹c j¹ koniecznie jako static
   
    if(i & 1)
    { // bit 0 zawiera wartoœæ przesuniêcia (0 brak, 1 zmiana o jedn¹ pozycjê
    if(i & 2) NoOfPulses--; // bit 1 - okreœla kierunek, lewo/prawo
    else NoOfPulses++; //NoOfPulses zawiera sumaryczn¹ pozycjê enkodera od ostatniego odczytu. Ka¿dy odczyt powinien zerowaæ t¹ zmienn¹, ¿eby mieæ deltê
    } */
		
/*
+=============================================================================+
| local functions
+=============================================================================+
*/

/*
+=============================================================================+
| ISRs
+=============================================================================+
*/

/******************************************************************************
* END OF FILE
******************************************************************************/
