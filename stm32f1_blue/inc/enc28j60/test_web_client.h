#ifndef _TCPIP_H
#define _TCPIP_H

#include  "ENC28J60.h"

//#define BUFFER_SIZE 1500 //1500 or 400

//extern unsigned char buf[BUFFER_SIZE+1];
//extern unsigned int plen;

void (*EthEventCallback)(void);
void RegisterEthEventCallback(void (*callback)(void));
void eth_event(void);
void zapytanie_http(void);

void simple_server_client(void);
//void init_simple_server(void);
void init_simple_client(void);

#endif


