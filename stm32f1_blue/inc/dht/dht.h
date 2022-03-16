#include "inc/stm32f10x.h"
#include "hdr/hdr_bitband.h"

#define AM2302_pin					bitband_t m_BITBAND_PERIPH(&GPIOA->IDR, 15)
#define AM2302_prt					bitband_t m_BITBAND_PERIPH(&GPIOA->ODR, 15)

typedef enum {DHT22, AM2302} DHT;

typedef struct {
	volatile u32 * pin;
	volatile u32 * prt;
	u16 AMh;
	s16 AMt;
	DHT	 typ;
	
} AM23_hw_t;

void DHT_Init(void);
void DHT_Start(void);
u08  DHT_read(u16* AMh, s16* AMt, DHT czuj, volatile long unsigned int * P_PIN);

u08  AM23_read(AM23_hw_t * hw);