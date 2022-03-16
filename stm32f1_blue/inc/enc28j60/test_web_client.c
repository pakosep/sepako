/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher
 * Copyright: GPL V2
 * See http://www.gnu.org/licenses/gpl.html
 *
 * Note: This software implements a web server and a web browser.
 * The web server is at "myip" and the browser tries to access 
 * the webserver at WEBSERVER_VHOST (tuxgraphics.org)
 *
 * Here is how to use this test software:
 * test_web_client.hex is a web client that uploads data to 
 * http://tuxgraphics.org/cgi-bin/upld when you ping it once
 * This is what you need to do:
 * - Change above the myip and gwip paramters 
 * - Compile and load the software
 * - ping the board once at myip from your computer (this will trigger the web-client)
 * - go to http://tuxgraphics.org/cgi-bin/upld and it will tell
 *   you from which IP address the ethernet board was ping-ed.
 *
 * Chip type           : Atmega168 or Atmega328 or Atmega644 with ENC28J60
 *********************************************/
#include <stdlib.h>
#include <string.h>
// http://www.nongnu.org/avr-libc/changes-1.8.html:
#define __PROG_TYPES_COMPAT__

#include "ip_arp_udp_tcp.h"
#include "websrv_help_functions.h"
#include "enc28j60.h"
#include "test_web_client.h"
#include "net.h"
#include "dnslkup.h"
#include "inc/tool/delay.h"
#include "inc/tool/UARTLib.h"

#include "inc/tool/itoa.h"
#include "config.h"
#include "inc/glcd/glcd.h"

char BuforURL[40];	// Bufor do zapisu na karcie SD - tutaj wpisujemy co ma byc zapisane przed wejsciem do petli while
// Please modify the following lines. mac and ip have to be unique
// in your local area network. You can not have the same numbers in
// two devices:
static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x29};
// how did I get the mac addr? Translate the first 3 numbers into ascii is: TUX
// This web server's own IP.
static uint8_t myip[4] = {10,74,164,1};
//static uint8_t myip[4] = {192,168,255,100};
//
// The name of the virtual host which you want to 
// contact at websrvip (hostname of the first portion of the URL):
#define WEBSERVER_VHOST "localhost"
// listen port for tcp/www:
#define MYWWWPORT 80

// Default gateway. The ip address of your DSL router. It can be set to 
// the same as the web server in the case where there is no default GW to access the 
// web server (=web server is on the same lan as this host) 
static uint8_t gwip[4] = {10,74,164,101};
//
// --- there should not be any need to changes things below this line ---
#define TRANS_NUM_GWMAC 1

static uint8_t gwmac[6] = {0x0,0x21,0x70,0x6f,0x8c,0xf1}; 
//static uint8_t gwmac[6]; 
//static uint8_t otherside_www_ip[4];
//static uint8_t otherside_www_ip[4] = {10,74,164,101}; // will be filled by dnslkup
//static char urlvarstr[21];
//
//
#define BUFFER_SIZE 1500
static uint8_t buf[BUFFER_SIZE+1];
static uint8_t pingsrcip[4];
static uint8_t start_web_client=0;
static uint8_t web_client_attempts=0;
static uint16_t web_client_sendok=0;
volatile uint8_t sec=0;
static volatile uint8_t cnt2step=0;
static int8_t dns_state=0;
static int8_t gw_arp_state=0;

// set output to VCC, red LED off
#define LEDOFF LED1=0
// set output to GND, red LED on
#define LEDON LED1=1
// to test the state of the LED
#define LEDISOFF LED1
// 

uint16_t http200ok(void)
	{
    return(fill_tcp_data_p(buf,0,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
	}

// prepare the webpage by writing the data to the tcp send buffer
uint16_t print_webpage(uint8_t *buf)
	{
		uint16_t plen;
		char vstr[17];
		uint8_t err;
		plen=http200ok();
		plen=fill_tcp_data_p(buf,plen,"<h2>web client status</h2>\n<pre>\n");
		if (gw_arp_state==1){
			plen=fill_tcp_data_p(buf,plen,"waiting for GW ");
			mk_net_str(vstr,gwip,4,'.',10);
			plen=fill_tcp_data(buf,plen,vstr);
			plen=fill_tcp_data_p(buf,plen," to answer arp.\n");
			return(plen);
		}
		if (dns_state==1){
		 plen=fill_tcp_data_p(buf,plen,"waiting for DNS answer.\n");
		 err=dnslkup_get_error_info();
		 plen=fill_tcp_data_p(buf,plen,"Error code: ");
		 itoa(err,vstr,10);
		 plen=fill_tcp_data(buf,plen,vstr);
		 plen=fill_tcp_data_p(buf,plen," (0=no error)\n");
		 return(plen);
		}
		plen=fill_tcp_data_p(buf,plen,"Number of data uploads started by ping: ");
		// convert number to string:
		itoa(web_client_attempts,vstr,10);
		plen=fill_tcp_data(buf,plen,vstr);
		plen=fill_tcp_data_p(buf,plen,"\nNumber successful data uploads to web: ");
		// convert number to string:
		itoa(web_client_sendok,vstr,10);
		plen=fill_tcp_data(buf,plen,vstr);
		plen=fill_tcp_data_p(buf,plen,"\ncheck result: <a href=http://tuxgraphics.org/cgi-bin/upld>http://tuxgraphics.org/cgi-bin/upld</a>");
		plen=fill_tcp_data_p(buf,plen,"\n</pre><br><hr>");
		return(plen);
	}


// we were ping-ed by somebody, store the ip of the ping sender
// and trigger an upload to http://tuxgraphics.org/cgi-bin/upld
// This is just for testing and demonstration purpose
void ping_callback(uint8_t *ip){
		uint8_t i=0;
		// trigger only first time in case we get many ping in a row:
		if (start_web_client==0){
			start_web_client=1;
			// save IP from where the ping came:
			while(i<4){
				pingsrcip[i]=ip[i];
				i++;
			}
		}
	}

// the __attribute__((unused)) is a gcc compiler directive to avoid warnings about unsed variables.
void browserresult_callback(uint16_t webstatuscode,uint16_t datapos __attribute__((unused)), uint16_t len __attribute__((unused))){
    char strMoc[16];
		char strVal[8];
		char strAll[60];
		//u08 lenVal=0;
		uint8_t *ptr_buf = buf;
		if (webstatuscode==200){
      web_client_sendok++;
			
			UaPutS("\r\n--> ");	
			//sint2uart(web_client_sendok);
			//sint2uart(datapos);
			//sint2uart(len);
			//UaPutS(" ");
			
			//sint2uart(buf[datapos+0]);
			//sint2uart(buf[datapos+1]);
			//UaPutS((char*)(buf+datapos));	
			//UaPutS("\r\n");
			memset ( strVal,0,8);
			memset ( strAll,0,60);
			//char *unt2str (u32 val,char *s,u08 digit);
			strcat (strAll,int2str(web_client_sendok,strVal,5,0));
			strcat (strAll," ");
			
			for(u08 i=0;i<3;i++){
				ptr_buf = strstr((char*)(ptr_buf+datapos),"pvalue vok"); //"pvalue vok\">208 W</d"
				strncpy (strMoc, (char*)(ptr_buf+12), 8);
				
				memset ( strVal,0,8);
				strncpy(strVal, strMoc, strlen(strMoc)-strlen(strchr(strMoc,' ')));
				//UaPutS(strVal);
				//UaPutS(" ");
				
				strcat (strAll,strVal);
				strcat (strAll," ");
			}
			
			UaPutS (strAll);
			//TFT_txt(sint2str(web_client_sendok,strVal),0, 200,YELLOW);
			TFT_txt(strAll,0, 200,YELLOW);
			
			//char* strstr (const char* str1, const char* str2);
			//LED1 = 0;
			//www_server_reply(buf,len)
			
			//buf[TCP_FLAGS_P]=TCP_FLAGS_ACK_V|TCP_FLAGS_PUSH_V|TCP_FLAGS_FIN_V;
			//make_tcp_synack_from_syn(buf);
			//buf[TCP_FLAGS_P]=TCP_FLAGS_ACK_V|TCP_FLAGS_PUSH_V|TCP_FLAGS_FIN_V;
			//make_tcp_ack_with_data_noflags(buf,0); // send data
    }
	}

// the __attribute__((unused)) is a gcc compiler directive to avoid warnings about unsed variables.
void arpresolver_result_callback(uint8_t *ip __attribute__((unused)),uint8_t transaction_number,uint8_t *mac){
    uint8_t i=0;
    if (transaction_number==TRANS_NUM_GWMAC){
			// copy mac address over:
      while(i<6){gwmac[i]=mac[i];i++;}
    }
	}


void init_simple_client(void)
	{
		// Set the clock speed to "no pre-scaler" (8MHz with internal osc or 
		// full external speed)
		// set the clock prescaler. First write CLKPCE to enable setting 
		// of clock the next four instructions.
		// Note that the CKDIV8 Fuse determines the initial
		// value of the CKKPS bits.
		
		//Delay_us(60); // 60us
		
		/*initialize enc28j60*/
		
		enc28j60Init(mymac);
		//enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
		enc28j60clkout(5); //  clkout is disabled. The pin is driven low
		Delay_ms(60); // 60us
		
		//init_cnt2();
		//sei();
		
		/* Magjack leds configuration, see enc28j60 datasheet, page 11 */
		// LEDB=yellow LEDA=green
		//
		// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
		//enc28j60PhyWrite(PHLCON,0b0000010001110110);
		//enc28j60PhyWrite(PHLCON,0x476);
		
		//init the web server ethernet/ip layer:
		init_udp_or_www_server(mymac,myip);
		//www_server_port(MYWWWPORT);
		
		// register to be informed about incomming ping:
		register_ping_rec_callback(&ping_callback);
		UaPutS("\r\n End init.");
	}
	
void simple_server_client(void)
	{	
		static u08 raz[4];
		uint16_t dat_p = 0,plen = 0;
		
		
		//client_browser_url("/enc28j60",NULL,"localhost",&browserresult,gwip,gwmac);
		//client_browse_url("/cgi-bin/upld?pw=sec&pingIP=",urlvarstr,WEBSERVER_VHOST,&browserresult_callback,otherside_www_ip,gwmac);
		//client_browse_url("/enc28j60/index.html",urlvarstr,WEBSERVER_VHOST,&browserresult_callback,otherside_www_ip,gwmac);
		//client_browse_url(PSTR("/cgi-bin/checkip"),NULL,"tuxgraphics.org",&browserresult,other_side_ip,gwmac);
		start_web_client=1;
		
		
		while(1){
			// handle ping and wait for a tcp packet
			plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
			dat_p=packetloop_arp_icmp_tcp(buf,plen);
			if(plen==0){				
				
				// we are idle here trigger arp and dns stuff here
				if (gw_arp_state==0){
					// find the mac address of the gateway (e.g your dsl router).
					get_mac_with_arp(gwip,TRANS_NUM_GWMAC,&arpresolver_result_callback);
					gw_arp_state=1;
					UaPutK("\r\n gw_arp_state=1 ");	
					//arpip_state
				}
				
				if (get_mac_with_arp_wait()==0 && gw_arp_state==1){
					// done we have the mac address of the GW
					gw_arp_state=2;
					UaPutK("\r\n gw_arp_state=2");
				}
				
				/*
				if (dns_state==0 && gw_arp_state==2){
					if (!enc28j60linkup()) //continue; // only for dnslkup_request we have to check if the link is up. 
					sec=0;
					dns_state=1;
					dnslkup_request(buf,WEBSERVER_VHOST,gwmac);
					//continue;
				}
				
				if (dns_state==1 && dnslkup_haveanswer()){
					dns_state=2;
					dnslkup_get_ip(otherside_www_ip);
				}
				if(!raz[1]){	UaPutK("\r\n dns_state");	sint2uart(dns_state);	raz[1]=1;	}
				
				if (dns_state!=2){
					// retry every minute if dns-lookup failed:
					if (sec > 60){
						dns_state=0;
					}
					// don't try to use web client before	 we have a result of dns-lookup
					//continue;
				}
				*/
				//----------
				if (start_web_client==1){
					//LEDON;
					if(!raz[0]){	UaPutK("\r\n start_web_client_0");	sint2uart(start_web_client);	raz[0]=1;	}
					sec=0;
					start_web_client=2;
					web_client_attempts++;
					//mk_net_str(str,pingsrcip,4,'.',10);
					//urlencode(str,urlvarstr);
					//UaPutK("\r\n urlvarstr="); UaPutS(urlvarstr);
					//client_browse_url("/cgi-bin/upld?pw=sec&pingIP=",urlvarstr,WEBSERVER_VHOST,&browserresult_callback,otherside_www_ip,gwmac);
					client_browse_url("/flara/dat512.html",BuforURL,WEBSERVER_VHOST,&browserresult_callback,gwip,gwmac);
				}
			}
			if(dat_p==0){ // plen!=0
				// check for incomming messages not processed;
				// as part of packetloop_arp_icmp_tcp, e.g udp messages
				udp_client_check_for_dns_answer(buf,plen);
				//continue;
			}
		}
	}

void RegisterEthEventCallback(void (*callback)(void))
	{
		EthEventCallback = callback;
	}

void eth_event(void)
	{
		uint16_t dat_p = 0,plen = 0;
		// handle ping and wait for a tcp packet
		LED1 = 1;
	  plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
		LED1 = 0;
		
		if(plen != 0){
			//LED2 = 1;
		} //else LED2 = 0;
		
	  dat_p=packetloop_arp_icmp_tcp(buf,plen);
		
	  if(plen==0){
			
			/*
			// we are idle here trigger arp and dns stuff here
			if (gw_arp_state==0){
				// find the mac address of the gateway (e.g your dsl router).
				get_mac_with_arp(gwip,TRANS_NUM_GWMAC,&arpresolver_result_callback);
				gw_arp_state=1;
				UaPutK("\r\n get_mac_with_arp start...");	
				//sint2uart(arpip_state);
				
			}
			if (get_mac_with_arp_wait()==0 && gw_arp_state==1){
				// done we have the mac address of the GW
				gw_arp_state=2;
				UaPutK("\r\n get_mac_with_arp done");	
				
			}
			*/
			/*
			if (dns_state==0 && gw_arp_state==2){
				if (!enc28j60linkup()) //continue; // only for dnslkup_request we have to check if the link is up.
				sec=0;
				dns_state=1;
				dnslkup_request(buf,WEBSERVER_VHOST,gwmac);
				//continue;
			}
			if (dns_state==1 && dnslkup_haveanswer()){
				dns_state=2;
				dnslkup_get_ip(otherside_www_ip);
			}
			
			if (dns_state!=2){
				// retry every minute if dns-lookup failed:
				if (sec > 60){
					dns_state=0;
				}
				// don't try to use web client before
				// we have a result of dns-lookup
				//continue;
			}
			*/
			
			//----------
			// CALLBACK
			
			if(EthEventCallback) (*EthEventCallback)();
			
			// reset after a delay to prevent permanent bouncing
			// continue;
		}
		if(dat_p==0){ // plen!=0
			// check for incomming messages not processed
			// as part of packetloop_arp_icmp_tcp, e.g udp messages
			// udp_client_check_for_dns_answer(buf,plen);
			// continue;
		}
	}
	
void zapytanie_http(void){
		if(f5_eth==1){
			LED2=1;
			client_browse_url("/flara/dat.html",BuforURL,"localhost",&browserresult_callback,gwip,gwmac);	
			LED2=0;
			f5_eth = 0;
			//LED1 = 1;
		}
	}
