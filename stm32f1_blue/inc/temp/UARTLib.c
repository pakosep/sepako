#include "inc/stm32f10x.h"
#include "UARTLib.h"

#define USARTx      USART1

//============================================================================
void U1PutC(char p){		
		//while (!((USART1->SR) & (1 << TC))); 
		while ((USARTx->SR & USART_SR_TC)==0);
		USARTx->DR=p & 0xff;
	}
//============================================================================
void U1PutS(char *s)	{
		//  loop until *s != NULL
		while (*s)		
		{
			U1PutC(*s);
			s++;
		}
	}

void UART_Send(uint8_t *buf, uint32_t cnt){
  while(cnt>1) {  
    //Write data to DR
    USARTx->DR = *buf;
    buf++;    
    //Wait for DR ready
    while (!(USARTx->SR & USART_SR_TXE)) {};
    cnt--;
  }
  //Send the last byte
  USARTx->DR = *buf ;
  
  //Wait for the end of actual transmission
  while (!(USARTx->SR & USART_SR_TC)) {};   //Not this will be cleared only after a read from SR and then write to DR. (Therefore after this operation, it will still be set)
}

void UART_SendLine(char *buf){
/*
  uint32_t i=256; //Max line length
  while(i>0 & *buf!='\n') {  
    //Write data to DR
    USARTx->DR = *buf;
    buf++;
    
    //Wait for DR ready
    while (!(USARTx->SR & USART_SR_TXE)) {};
    i--;
  }
  
  //Send the last byte
  USARTx->DR = *buf;
  //Wait for the end of actual transmission
  while (!(USARTx->SR & USART_SR_TC)) {};   //Not this will be cleared only after a read from SR and then write to DR. (Therefore after this operation, it will still be set)
	*/
	U1PutS(buf);
	
}


void UART_Receive(uint8_t *buf, uint32_t cnt) {
  while(cnt>0) {
    //Wait for new data
    while (!(USARTx->SR & USART_SR_RXNE)) {};
    //Read the data
    *buf++=USARTx->DR;
    
    cnt--;
  }
}

