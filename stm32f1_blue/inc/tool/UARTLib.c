#include "inc/stm32f10x.h"
#include "UARTLib.h"

#define uxbuf_ptr	u1buf_ptr

static const u08 cyfhex[17] = 	{'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};

static volatile T_FIFO Fifo1,Fifo2,Fifo3;
volatile T_FIFO *p_Fifo1 = &Fifo1;
volatile T_FIFO *p_Fifo2 = &Fifo2;
volatile T_FIFO *p_Fifo3 = &Fifo3;

void Text1(u08 addr){	UaPutS((char*)StrP122+addr*32);	 } // stringi co 32 znaki
void StrFF(u16 addr){	UaPutS((char*)StrP119+addr*16);	 } // Stringi co 16 znakow

//void my_callback_function(void (*ptr)()) { (*ptr)(); //calling the callback function}
//============================================================================
//void UaPutC(char p){		
//	while ((USARTx->SR & USART_SR_TC)==0);
//	USARTx->DR=p & 0xff;
//	}
void UART1_putc(char p){
		while ((USART1->SR & USART_SR_TC)==0);
		USART1->DR=p;
	}

void UART2_putc(char p){
		while ((USART2->SR & USART_SR_TC)==0);
		USART2->DR=p;
	}

void UART3_putc(char p){
		while ((USART3->SR & USART_SR_TC)==0);
		USART3->DR=p;
	}

void UaPutC(char p){
			PutChar(p);
	}

//============================================================================
char *strrev(char *str){
  char *p1, *p2;
  if (! str || ! *str)  return str;
  for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2)
  {
    *p1 ^= *p2;
    *p2 ^= *p1;
    *p1 ^= *p2;
  }
  return str;
}
//============================================================================
void reverse(char *s)  {
    int i, j;      char c;
    for (i = 0, j = strlen(s)-1; i<j; i++, j--) 
		{
      c = s[i];
      s[i] = s[j];
      s[j] = c;
    }
  }
//============================================================================
//void register_putc_callback(void (*callback)(char data)){	 // Definicja funkcji rejestrujacej callback
//    u_putc = callback;  }
 
void UaPutS(char *s)	{
		//  loop until *s != NULL
		while (*s)		
		{
			UaPutC(*s);
			s++;
		}
	}
//============================================================================
void UaPutK(const char *s)	{
		//  loop until *s != NULL
		while (*s)		
		{
			UaPutC((char) *s);
			s++;
		}
	}
//============================================================================
void UART_getChar( char *Char )				{ 
		u08 i;
		i = Fifo1.rri;
		if(Fifo1.rct)		
		{
			*Char = Fifo1.rbuf[i];
			Fifo1.rri = ++i % UART1_RXB;
			__disable_irq();
			Fifo1.rct--; 
			__enable_irq();
		}		
	}

void UART_getChar2( char *Char )				{ 
		u08 i;
		i = Fifo2.rri;
		if(Fifo2.rct)		
		{
			*Char = Fifo2.rbuf[i];
			Fifo2.rri = ++i % UART2_RXB;
			__disable_irq();
			Fifo2.rct--; 
			__enable_irq();
		}		
	}
	
//============================================================================
char *UART_getStr(char *Str){
		u16 i; char d=0;
		i = Fifo1.rri;
		do{
			while (Fifo1.rct)		
			{
				d = Fifo1.rbuf[Fifo1.rri];
				*Str++ = d;
				Fifo1.rri = ++i % UART1_RXB;
				__disable_irq();
				Fifo1.rct--;
				__enable_irq();
				if(d==0x0d) break;
			}
		} while(d != 0x0d);
		*(--Str) = 0;
		return Str;
	}
//============================================================================
u32 UART_getNum (void){ 
		u16 i; u08 d=0;
		char str[9], *wstr=str;
		i = Fifo1.rri;
		//str = (char*)Fifo2.rbuf+i;
		do{
			while (Fifo1.rct)		
			{
				d = Fifo1.rbuf[Fifo1.rri];
				*(wstr++) = d; 
				Fifo1.rri = ++i % UART1_RXB;
				__disable_irq();
				Fifo1.rct--;
				__enable_irq();
			}
		} while(d!=0x0d);
		*(--wstr) = 0;
		return str2int(str);	
	}
//============================================================================
u32 UART_getHex (void){
		u16 i; u08 d=0;
		char str[9], *wstr=str;
		i = Fifo1.rri;
		//str = (char*)Fifo2.rbuf+i;
		do{
			while (Fifo1.rct)
			{
				d = Fifo1.rbuf[Fifo1.rri];
				*(wstr++) = d;
				Fifo1.rri = ++i % UART1_RXB;
				__disable_irq();
				Fifo1.rct--;
				__enable_irq();
			}
		} while(d!=0x0d);
		*(--wstr) = 0;
		return str2hex(str);	
	}
//============================================================================
char *uint2str (u32 val, char *str)  {  // Range from 0 to 4294967295(2^32)
    u08 i=0;
    do 
		{      
      str[i++] = val % 10 + '0';   
    } while ((val /= 10) > 0);     
    str[i] = '\0';
    reverse(str);
    return str;
  }
//============================================================================
char *sint2str (s32 val, char *str)  {  // Range from -2147483647 to 2147483647(2^31)
    int sign;
		u08 i=0;
    if ((sign = val) < 0)  /* record sign */
    val = -val;          /* make n positive */
    do 
		{       /* generate digits in reverse order */
      str[i++] = val % 10 + '0';   /* get next digit */
    } while ((val /= 10) > 0);     /* delete it */
    if (sign < 0)    str[i++] = '-';
		else str[i++] = ' ';
    str[i] = '\0';
    reverse(str);
    return str;
  }
//============================================================================
char *unt2str(u32 val, char *str, u08 fw)  {  // with first zero "0012"
    u08 i = 0;
		u32 dpow = 1;
		for(u08 k=0;k<fw;k++) dpow *= 10;
		if ( val < dpow ){
			do 
			{ 
      	if(val>0 || i==0)												/* generate digits in reverse order */
				str[i++] = val % 10 + '0';   						/* get next digit */
				else
				str[i++] = '0';
				
				fw--;
			} while (((val /= 10) > 0) || (fw>0) );   /* delete it */
			str[i] = '\0';
			reverse(str);
		}
		else { 
			str[0]=' ';
			for(i=0;i<fw;i++)	str[i+1]='x';	
			str[i+1]='\0';
		}
    return str;
  }
//============================================================================
char *int2str (s32 val, char *str,  u08 fw, u08 k) {// without first zeros 
    u08 i = 0;
		u32 dpow = 1;
		static uint8_t sign;
		char zero;
		
		if (val < 0) { val = -val; sign=1;} 
		else sign=0;
		
		for(u08 k=0;k<fw;k++) dpow *= 10;
		if ( (u32)val < dpow && fw >= k ){
			
			do 
			{ 																	// generate digits in reverse order 			
				if(i<(k+1)) zero='0'; 
				else zero = ' ';									// krotki znak spacji
				str[i++]=val>0?val%10+'0':zero;		// get next digit '\x1f'
				
				if(i==k) {
					str[i++] = '.';
				}
				fw--;				
			} while (((val /= 10) > 0) || (fw>0) );   /* delete it */
			
			if(sign)  str[i++] = '-'; 
			else		 	str[i++] = ' ';				// spacja zamiast znaku minus
			str[i] = '\0';
			reverse(str);
			//strrev(str);
		}
		else { 
			str[0]='-';												// spacja z przodu
			for(i=1;i<fw;i++)	str[i]='x';	
			str[i]='\0';
			//strcpy  ((char *)str,"max");
		}
		return str;
  }
//
char *int2str_z (s32 val, char *str,  u08 fw, u08 k) {// with first zeros 
    u08 i = 0;
		u32 dpow = 1;
		static uint8_t sign;
		char zero;
		
		if (val < 0) { val = -val; sign=1;} 
		else sign=0;
		
		for(u08 k=0;k<fw;k++) dpow *= 10;
		if ( (u32)val < dpow && fw >= k ){
			
			do 
			{ 																	// generate digits in reverse order 			
				if(i<(k+1)) zero='0'; 
				else zero = '0';									// krotki znak spacji
				str[i++]=val>0?val%10+'0':zero;		// get next digit '\x1f'
				
				if(i==k) {
					str[i++] = '.';
				}
				fw--;				
			} while (((val /= 10) > 0) || (fw>0) );   /* delete it */
			
			if(sign)  str[i++] = '-'; 
			else		 	str[i++] = ' ';				// spacja zamiast znaku minus
			str[i] = '\0';
			reverse(str);
			//strrev(str);
		}
		else { 
			str[0]='-';												// spacja z przodu
			for(i=1;i<fw;i++)	str[i]='x';	
			str[i]='\0';
			//strcpy  ((char *)str,"max");
		}
		return str;
  }
//============================================================================
char *num2str (u32 val, char *str, u08 fw, u08 k) { // without first zero 
    u08 i = 0;
		u32 dpow = 1;
		for(u08 k=0;k<fw;k++) dpow *= 10;
		if ( val < dpow ){
			do 
			{ 
				if(val>0 || i==0)
				str[i++] = val % 10 + '0';   					/* get next digit */
				else
				str[i++] = ' ';
				
				if(i==k) str[i++] = '.';
				fw--;				
				
			} while (( (val /= 10) > 0) || (fw>0) );   /* delete it */
			str[i] = '\0';
			reverse(str);
			//strrev(s);
		}
		else { 
			str[0]=' ';
			for(i=0;i<fw;i++)	str[i+1]='x';	
			str[i+1]='\0';
		}
		return str;
  }
//============================================================================
char *hex2str(u32 hex, char *xstr,u08 fw){ 					//Convert HEX number to string
		
		if ( hex <= (0xffffffff>>(32-fw*4)) )  {
			char *strp = xstr;
			*strp++ = ' ';
			do{
				*strp++ = cyfhex[hex & 0x0f];
				hex >>= 4;
				fw--;
			}
			while((hex > 0) || (fw>0));
			*strp = '\0';
			strrev(xstr);
		}
		else {u08 i; for(i=0;i<fw;i++)	xstr[i]='x';	xstr[i]=0; }
		return xstr;
	}
//============================================================================
int  str2heX(char * str){														//Convert HEX string to number
  int i, len, val_p;
  //  char c;
  val_p = 0;
  len = strlen(str);
  for(i = 0; i < len; i++)
  {
    if(str[i]>='0'&&str[i]<='9')    {
	(val_p)+=((str[i]-'0')<<(4*(len-i-1)));
    }
    else if(str[i]>='a'&&str[i]<='f')    {
	(val_p)+=((str[i]-'a'+0xa)<<(4*(len-i-1)));
    }
    else    {
	return 0;
    }
  }
  return val_p;
}
//============================================================================
int  str2int(char * str){
		int i, len, val_p, pot=1,mn=1;
		//  char c;
		reverse(str);
		val_p = 0;
		len = strlen(str);
		for(i = 0; i < len; i++)
		{
			if(str[i]>='0'&&str[i]<='9')    {
		 (val_p)+=((str[i]-'0')*pot);
		 pot *= 10;
			}
			else if(str[i]=='-')  {
				mn = -1;
			}
			else if(str[i]=='+'|| str[i]=='d')    {
			 ;
			}
			else    {
				return 0;	
			}
		}
		return val_p*mn;
	}
//============================================================================
u32  str2hex(char *Buf) {														//Convert HEX string to number
		u32 temp=0;
		u08 mn = 0;
		char *str = (char *) Buf;
		reverse(str);													// reverse string
		while (*str)
		{
			if(*str<58)
			temp |= (u32)(*str-48)<<4*mn;
			else
			temp |= (u32)(*str-87)<<4*mn;
			mn++;
			str++;
		}
		return temp;
	}

//============================================================================
char *PSTR(char *xstr)					{	return xstr;	}
//============================================================================
void sint2uart(s32 n)  {  		
	 char buf[12];
	 UaPutS(sint2str (n, buf));
  }
//============================================================================
void uint2uart(u32 n)  {  			
	 char buf[12];
	 UaPutS(uint2str (n, buf));
  }
//============================================================================
void unt2uart(u32 n, u08 dig)  {  			
	 char buf[12];
	 UaPutS(unt2str (n, buf, dig));
  }
//============================================================================
void int2uart(s32 val,u08 fw,u08 k)	{	
		char buf[11];
    UaPutS(int2str (val, buf, fw, k));
	}

void int2uarz(s32 val,u08 fw,u08 k)	{	
		char buf[11];
    UaPutS(int2str_z (val, buf, fw, k));
	}
//============================================================================
void num2uart(u32 val,u08 fw,u08 k)	{	
		char buf[11];
    UaPutS(num2str (val, buf, fw, k));
	}
//============================================================================
void hex2uart(u32 hex,u08 fw){ //Wyswietl liczbe w formacie HEX
		char buf[10];
		UaPutS(hex2str(hex, buf,fw) );
	}
//============================================================================
void USART1_IRQHandler(void)		{
    u08 d,i;
		//u16 i;
		
		if (USART1->SR & USART_SR_RXNE) {     // indicate interrupt from RX
			//bUSART1_SR_RXNE = 0;              // clear interrupt flag
			d = USART1->DR;			
			i = Fifo1.rct;
			if (i < UART1_RXB) {	
				Fifo1.rct = ++i;
				i = Fifo1.rwi;
				Fifo1.rbuf[i] = d;
				Fifo1.rwi = ++i % UART1_RXB;
			}
			USART1->DR = d;		//echo
    }
	}

void USART2_IRQHandler(void)		{
    u08 d;
		int i;
		
		if (USART2->SR & USART_SR_RXNE) {     // indicate interrupt from RX
			//bUSART2_SR_RXNE = 0;              // clear interrupt flag
			d = USART2->DR;						/* Get received byte */
			i = Fifo2.rct;
			if (i < UART2_RXB) {			/* Store it into the rx fifo if not full */
				Fifo2.rct = ++i;
				i = Fifo2.rwi;
				Fifo2.rbuf[i] = d;
				Fifo2.rwi = ++i % UART2_RXB;
			}
			USART2->DR = d;		//echo
    }
		
	}
