//ADS7843/7846/UH7843/7846/XPT2046/TSC2046 
//2010/6/13
//V1.3

#include <math.h>
#include "touch.h" 
#include "inc/glcd/GLCD.h"
//#include "hdr/hdr_gpio.h"
#include "inc/tool/delay.h"
//#define SPI_DISABLE

#define SPI_ENABLE

//Pen_Holder Pen_Point;

void Touch_Init(void)
	{			    		   
		#ifdef SPI_DISABLE
			GPIOA->CRL = (GPIOA->CRL & 0x0000ffff) | 0x38330000;		
			GPIOA->ODR |=   0x00F0;    	//PA4
			GPIOC->CRH = (GPIOC->CRH & 0xff0fffff) | 0x00800000; //PC13
			GPIOC->ODR |=   0x1<<13;   	//PC13 
			
		// 	Read_ADS(&Pen_Point.X,&Pen_Point.Y);
		//	MY_NVIC_Init(2,0,EXTI15_10_IRQChannel,2);	  
			RCC->APB2ENR|=0x01;    
			AFIO->EXTICR[3]|=0x0020; //EXTI13
			EXTI->IMR|=1<<13;        
			EXTI->EMR|=1<<13;        
			EXTI->FTSR|=1<<13;       //line13
		#endif		 
		#ifdef SPI_ENABLE
		GPIOA->CRL = (GPIOA->CRL & 0x0000ffff) | 0xb4b10000;		
		GPIOB->CRL = (GPIOB->CRL & 0xf0ffffff) | 0x04000000;		
				//TCS = 1;
				//GPIOA->ODR|=0x00F0;		
			
			RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;
			//SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2|SPI_CR1_BR_1|SPI_CR1_MSTR;
			//SPI1->CR2 = SPI_CR2_SSOE;
			SPI1->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2|SPI_CR1_BR_1|SPI_CR1_BR_0|	SPI_CR1_MSTR;	// 0x037c
			SPI1->CR1 |= SPI_CR1_SPE;
		#endif		 
		/*#ifdef ADJ_SAVE_ENABLE	  
			AT24CXX_Init();
			if(Get_Adjdata())return;
			else			  
			{ 										    
				LCD_Clear(Black);
				Touch_Adjust();  
				Save_Adjdata();	 
			}			
			Get_Adjdata();	 
		#else	 */
		//	LCD_Clear(Black);
		//  Touch_Adjust();
	}

uint8_t SPI_rw(u08 cmd) { 
		u08 spi_out=0;    		
		// GPIOC->BSRRH = GPIO_Pin_4;
		while ((SPI1->SR & SPI_SR_TXE) == 0);		/// while TX buffer not empty
		SPI1->DR = cmd;	
		while ((SPI1->SR & SPI_SR_RXNE) == 0);	/// while RX buffer empty
		spi_out = SPI1->DR;		
		//GPIOC->BSRRL = GPIO_Pin_4;		
		return spi_out;
	} 

static __inline u16 ADS_Read(u08 CMD)	  
	{ 
		u16 Num=0; 	 
		//SPI1->CR1 &= ~( SPI_CR1_BR_1); 	
		TCS=0;
		
		//Delay_us(100);
		SPI_rw(CMD);
		Num = SPI_rw(0x00);
		Num <<= 8;
		Num = (Num + SPI_rw(0x00))>>3;
		Num &= 0x0fff;
		
		TCS = 1;
		return(Num);   
	}
	
static __inline void ADS_Write_Byte(u08 CMD)    
	{ 	
		#ifdef SPI_DISABLE
		u08 count = 0;    
		for(count = 0;count<8;count++)  
		{ 	  
			if(cmd & 0x80)TDIN=1;  
			else TDIN=0;   
			cmd <<= 1;    
			TCLK = 0;
			TCLK = 1;      
		} 	
		#endif		    
		#ifdef SPI_ENABLE	
		//u32 dummy = 0;    
		while (!(SPI1->SR & SPI_SR_TXE));		//Next IF TX buffer empty (TXE=1)
		SPI1->DR = CMD;	
		while (!(SPI1->SR & SPI_SR_RXNE));	//Next IF RXNE buffer not empty
		//dummy = SPI1->DR;
		SPI1->DR;
		#endif
	} 		 

static __inline u16 ADS_Read_(u08 CMD)	  
	{ 
		u16 Num=0; 	 
		#ifdef SPI_DISABLE
		u08 count=0; 	  	
		TCLK=0;
		TCS =0; //ADS7843	 
		ADS_Write_Byte(CMD);
		//Delay_us(6);//ADS7846
		TCLK=1;
		TCLK=0; 	 
		for(count=0;count<16;count++)  
		{ 				  
			Num  <<= 1; 	 
			TCLK   = 0;
			TCLK   = 1;
			if(DOUT)Num++; 		 
		}  	
		Num >>= 4; 
		TCS = 1;
		return(Num);   
		#endif
		#ifdef SPI_ENABLE
		TCS=0;
		ADS_Write_Byte(CMD);
		//Delay_us(1);
		while (!(SPI1->SR & SPI_SR_TXE));		//Next IF TX buffer empty (TXE=1)
		SPI1->DR = 0x00;	
		while (!(SPI1->SR & SPI_SR_RXNE));	//Next IF RXNE buffer not empty
		Num = SPI1->DR << 8;
		//Delay_us(1);
		while (!(SPI1->SR & SPI_SR_TXE));		//Next IF TX buffer empty (TXE=1)
		SPI1->DR = 0x00;	
		while (!(SPI1->SR & SPI_SR_RXNE));	//Next IF RXNE buffer not empty
		Num += SPI1->DR;	
		Num >>= 3;
		Num &= 0x0fff;
		TCS = 1;
		return(Num);   
		#endif
	}


#define READ_TIMES 	4						// LOST_VAL ->| PASS |<- READ_TIMES-LOST_VAL
#define LOST_VAL 		1 					// READ_TIMES > 2*LOST_VAL
static __inline u16 ADS_Read_XY(u08 xy)
	{
		u16 i, j;
		u16 buf[READ_TIMES];
		u16 sum=0;
		u16 temp;
		for(i=0;i<READ_TIMES;i++)
		{				 
			buf[i]=ADS_Read(xy);	   	//measurement  READ_TIMES times
		}				    
		for(i=0;i<READ_TIMES-1; i++)	//Sorting ascending
		{
			for(j=i+1;j<READ_TIMES;j++)
			{
				if(buf[i]>buf[j])
				{
					temp=buf[i];
					buf[i]=buf[j];
					buf[j]=temp;
				}
			}
		}	  
		sum=0;
		for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
		temp=sum/(READ_TIMES-2*LOST_VAL);
		return temp;   
	} 

//90*2=180us
int Read_ADS(u32 *x,u32 *y) 					// Check condition
	{
		u32 xtemp,ytemp;			 	 		  
		xtemp=ADS_Read_XY(CMD_RDX);
		ytemp=ADS_Read_XY(CMD_RDY);	  												   
		if(xtemp<128||ytemp<128)   return 0; 	// condition MIN edge area
		if(xtemp>4096||ytemp>4096) return 0;	// condition MAX edge area
		*x=xtemp;
		*y=ytemp;
		return 1;
	}	

int Read_ADS1(u32 *x,u32 *y) 					// Check condition
	{
		u32 xtemp=0,ytemp=0;
		//while ((xtemp<128) || (xtemp>4096))	xtemp=ADS_Read(CMD_RDX);
		//while ((ytemp<128) || (xtemp>4096))	ytemp=ADS_Read(CMD_RDY);
		
		xtemp=ADS_Read(CMD_RDX);
		ytemp=ADS_Read(CMD_RDY);
		
		//if(xtemp<128||ytemp<128)   return 0; 	// condition MIN edge area
		//if(xtemp>4096||ytemp>4096) return 0;	// condition MAX edge area
		*x=xtemp;
		*y=ytemp;
		return 1;
	}	

//180*2=360us;
#define ERR_RANGE 50 

int Read_ADS2(u32 *x,u32 *y) 						// Check condition
	{
		u32 x1,y1;
		u32 x2,y2;
		u08 flag;    
    flag=Read_ADS(&x1,&y1);   
    if(flag==0) return(0);
    flag=Read_ADS(&x2,&y2);	   
    if(flag==0) return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
      *x=(x1+x2)/2;
      *y=(y1+y2)/2;
      return 1;
    }else return 0;	  									// condition identical measurements
	} 

/*
u08 Read_TP_Once(void)
	{
		u08 t=0;	    
		Pen_Int_Set(0);
		Pen_Point.Key_Sta=Key_Up;
		Read_ADS2(&Pen_Point.X,&Pen_Point.Y);
		while(PEN==0&&t<=250)
		{
			t++;
			Delay_ms(10);
		}
		Pen_Int_Set(1);
		if(t>=250)return 0;
		else return 1;	
	}

u16 T_abs(u16 m,u16 n)
	{
		if(m>n)return m-n;
		else return n-m;
	}

#define PT_RANGE 3
u08 AI_Read_TP(void)
{
	static u08 LSTA=0;
	static u16 fx=0,fy=0;
	u08 times;
	u16 tempx,tempy;
	switch(LSTA)
	{
		case 0:	  
		if(Pen_Point.Key_Sta==Key_Down)
		{				 
			Pen_Int_Set(0);  
			Convert_Pos();	 
			fx=Pen_Point.X0;
			fy=Pen_Point.Y0;
			times=0;			 
			do
			{
				Convert_Pos();	   
				tempx=Pen_Point.X0;
				tempy=Pen_Point.Y0;									 
				if((T_abs(tempx,fx)>PT_RANGE)||(T_abs(tempy,fy)>PT_RANGE))
				{					 				 
					LSTA=1;
					return 1;
				}else times++;	    
				if(times>20)
				{					 			 
					LSTA=2;
					return 2;
				}
			}while(PEN==0);
			LSTA=2;
			return 2;
		}
		break;	  
		case 1:
		if(PEN==0)
		{					    
			Convert_Pos(); 
			Delay_us(100); 
			return LSTA;   
		}
		Pen_Int_Set(1); 
	    Pen_Point.Key_Sta=Key_Up; 
		break;		 
		case 2:
		if(PEN==0)
		{					    
			Convert_Pos(); 
			Delay_us(100);
			tempx=Pen_Point.X0;
			tempy=Pen_Point.Y0;	  
			if((T_abs(tempx,fx)>PT_RANGE)||(T_abs(tempy,fy)>PT_RANGE))//移动距离大于PT_RANGE
			{
				LSTA=1;
				return 1;
			}
			return 2;  
		}
		Pen_Int_Set(1); 
	    Pen_Point.Key_Sta=Key_Up;  
		break;
	}						    
	LSTA=0;	   
	return 0;  	 
}

u08 Is_In_Area(u08 x1,u16 y1,u08 x2,u16 y2)
	{
		if(Pen_Point.X0<=x2&&Pen_Point.X0>=x1&&Pen_Point.Y0<=y2&&Pen_Point.Y0>=y1)return 1;
		else return 0;
	}  

void Draw_Touch_Point(u08 x,u16 y)
	{
		LCD_DrawLine(x-12,y,x+13,y,White);
		LCD_DrawLine(x,y-12,x,y+13,White);
		LCD_DrawPoint(x+1,y+1,White);
		LCD_DrawPoint(x-1,y+1,White);
		LCD_DrawPoint(x+1,y-1,White);
		LCD_DrawPoint(x-1,y-1,White);
		//Draw_Circle(x,y,6);
	}	  

void Draw_Big_Point(u08 x,u16 y)
	{	    
		LCD_DrawPoint(x,y,White);
		LCD_DrawPoint(x+1,y,White);
		LCD_DrawPoint(x,y+1,White);
		LCD_DrawPoint(x+1,y+1,White);	 	  	
	}

void Convert_Pos(void)
	{		 	  
		if(Read_ADS2(&Pen_Point.X,&Pen_Point.Y))
		{
			Pen_Point.X0 = Pen_Point.xfac * Pen_Point.X + Pen_Point.xoff;
			Pen_Point.Y0 = Pen_Point.yfac * Pen_Point.Y + Pen_Point.yoff;  
		}
	}	   

void EXTI15_10_IRQHandler(void)
	{ 		   			 
		Pen_Point.Key_Sta=Key_Down; 		  				 
		EXTI->PR=1<<13;  
	} 

void Pen_Int_Set(u08 en)
	{
		if(en)EXTI->IMR|=1<<13;   
		else EXTI->IMR&=~(1<<13); 
	}	  

#ifdef ADJ_SAVE_ENABLE
//(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE 40

void Save_Adjdata(void)
{
	s32 temp;			 

	temp=Pen_Point.xfac*100000000;
  //  AT24CXX_WriteLenByte(SAVE_ADDR_BASE,temp,4);   
	temp=Pen_Point.yfac*100000000;
  //  AT24CXX_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);
  //  AT24CXX_WriteLenByte(SAVE_ADDR_BASE+8,Pen_Point.xoff,2);		    
	// 	AT24CXX_WriteLenByte(SAVE_ADDR_BASE+10,Pen_Point.yoff,2);	
	//	temp=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+12);
	temp&=0XF0;
	temp|=0X0A;
	//	AT24CXX_WriteOneByte(SAVE_ADDR_BASE+12,temp);			 
}

u08 Get_Adjdata(void)
{					  
	s32 tempfac;
	//tempfac=AT24CXX_ReadOneByte(52);
	if((tempfac&0X0F)==0X0A)
	{    												 
		//tempfac=AT24CXX_ReadLenByte(40,4);		   
		Pen_Point.xfac=(float)tempfac/100000000;//得到x校准参数
		//tempfac=AT24CXX_ReadLenByte(44,4);			          
		Pen_Point.yfac=(float)tempfac/100000000;//得到y校准参数
	    
		//tempfac=AT24CXX_ReadLenByte(48,2);			   	  
		Pen_Point.xoff=tempfac;					 
	    
		//tempfac=AT24CXX_ReadLenByte(50,2);				 	  
		Pen_Point.yoff=tempfac;					 
		return 1;	 
	}
	return 0;
}
#endif		 

void Touch_Adjust(void)
{								 
	u16 pos_temp[4][2];
	u08  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	float fac; 	   
	cnt=0;				
	//POINT_COLOR=Blue;
	//BACK_COLOR =White;
	//LCD_Clear(Black);
	//POINT_COLOR=Red;
	//LCD_Clear(Black);
	Draw_Touch_Point(20,20);
	Pen_Point.Key_Sta=Key_Up;
	Pen_Point.xfac=0;
	while(1)
	{
		if(Pen_Point.Key_Sta==Key_Down)
		{
			if(Read_TP_Once())
			{  								   
				pos_temp[cnt][0]=Pen_Point.X;
				pos_temp[cnt][1]=Pen_Point.Y;
				cnt++;
			}			 
			switch(cnt)
			{			   
				case 1:
					LCD_Clear(Black);
					Draw_Touch_Point(220,20);
					break;
				case 2:
					LCD_Clear(Black);
					Draw_Touch_Point(20,300);
					break;
				case 3:
					LCD_Clear(Black);
					Draw_Touch_Point(220,300);
					break;
				case 4:
	    		 
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1 *= tem1;
					tem2 *= tem2;
					d1=sqrt(tem1+tem2);
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//
					{
						cnt=0;
						LCD_Clear(Black);
						Draw_Touch_Point(20,20);
						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)
					{
						cnt=0;
						LCD_Clear(Black);
						Draw_Touch_Point(20,20);
						continue;
					}
					
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);
					
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)
					{
						cnt=0;
						LCD_Clear(Black);
						Draw_Touch_Point(20,20);
						continue;
					}
					
					Pen_Point.xfac=(float)200/(pos_temp[1][0]-pos_temp[0][0]);
					Pen_Point.xoff=(240-Pen_Point.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;
					
					Pen_Point.yfac=(float)280/(pos_temp[2][1]-pos_temp[0][1]);
					Pen_Point.yoff=(320-Pen_Point.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;
					//POINT_COLOR=Blue;
					LCD_Clear(Black);
					GUI_Text(35,110,"Touch Screen Adjust OK!",White,Black);
					Delay_ms(1000);
					LCD_Clear(Black);
					return;				 
			}
		}
	} 
}		    

*/

/* 
================================================================================
======= Wyznaczanie wspolczynnikow okreslajacych wspolrzedne ekranu ============
================================================================================

		#define tpDIV	1024
		u32 Xo,Yo,Kx,Ky,x1,x2,y1,y2,A1=50,B1=50,A2=200,B2=180;
		U1PutS("\r\nWybierz Punkt "); 
		
		//LCD_DrawVerLine(A1,B1,B1+100,White);
		LCD_DrawLine(A1,  B1-5,A1  ,B1+5,White);
		LCD_DrawLine(A1-5,B1	,A1+5,B1  ,White);
		//LCD_DrawPoint(A1,B1,Yellow);
		
		while(!f1_tpirq);		//dopuki pioro podniesione
		while(f1_tpirq && (tst<1024) ) {
			if(Read_ADS1(&Ty,&Tx)){
				px += Tx;
				py += Ty;
				tst++;
			}
		}
		x1 =px/tpDIV;
		y1 =py/tpDIV;
		px=0; py=0;	tst=0;
		
		GUI_Text(250,220, num2str(x1,tbuf,5,0),White,Black);		
		GUI_Text(250,200, num2str(y1,tbuf,5,0),White,Black);
		
		//LCD_DrawPoint(A1,B1,Black);
		LCD_DrawLine(A1,  B1-5,A1  ,B1+5,Black);
		LCD_DrawLine(A1-5,B1	,A1+5,B1  ,Black);
		while(f1_tpirq);		//dopuki pioro opuszczone
		
		LCD_DrawLine(A2,  B2-5,A2  ,B2+5,White);
		LCD_DrawLine(A2-5,B2	,A2+5,B2  ,White);
		//LCD_DrawPoint(A2,B2,Yellow);
		
		while(!f1_tpirq);		//dopuki pioro podniesione
		while(f1_tpirq && (tst<1024) ) {
			if(Read_ADS1(&Ty,&Tx)){
				px += Tx;
				py += Ty;
				tst++;
			}
		}
		x2 =px/tpDIV;
		y2 =py/tpDIV;
		px=0; py=0; tst=0;
		
		//U1PutS("x1="); 		U1PutS(num2str(x1, u1buf, 5, 0)); 
		GUI_Text(250,220, num2str(x2,tbuf,5,0),White,Black);		
		GUI_Text(250,200, num2str(y2,tbuf,5,0),White,Black);
		
		LCD_DrawLine(A2,  B2-5,A2  ,B2+5,Black);
		LCD_DrawLine(A2-5,B2	,A2+5,B2  ,Black);
		
		while(f1_tpirq);		//dopuki pioro opuszczone
		
		Xo=(A2*x1-A1*x2)/(x1-x2);
		Kx=(16384*(A2-A1))/(x1-x2);
		
		Yo=(B2*y1-B1*y2)/(y1-y2);
		Ky=(16384*(B2-B1))/(y1-y2);
		
		GUI_Text(250,220, num2str(Xo,tbuf,5,0),White,Black);		
		GUI_Text(250,200, num2str(Yo,tbuf,5,0),White,Black);
		GUI_Text(250,180, num2str(Kx,tbuf,5,0),White,Black);
		GUI_Text(250,160, num2str(Ky,tbuf,5,0),White,Black);
		
		A1 = Xo - kx*x1
		B1 = Yo - ky*y1
		A2 = Xo - kx*x2
		B2 = Yo - ky*y2
		
		Xo=(A1*x2+A2*x1)/(x1+x2)
		kx=(A1-A2)/(x1+x2)
		
		Yo=(B1*y2+B2*y1)/(y1+y2)
		ky=(B1-B2)/(y1+y2)
		*/