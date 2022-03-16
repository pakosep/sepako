#ifndef __UARTLIB_H
#define __UARTLIB_H

#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

void UART_Send(uint8_t *buf, uint32_t cnt);
void UART_Receive(uint8_t *buf, uint32_t cnt);
void UART_SendLine(char *buf);
void U1PutC(char p);
void U1PutS(char *s);

#ifdef __cplusplus
}
#endif
#endif //__UARTLIB_H
