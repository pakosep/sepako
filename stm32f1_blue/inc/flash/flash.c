#include "inc/stm32f10x.h"
#include "inc/tool/UARTLib.h"
#include "flash.h"

#define FLASH_KEY1  0x45670123
#define FLASH_KEY2  0xCDEF89AB
#define Page_ADR		0x08007C00  // Page 31

// high density 2Kbyte per Page
// !!! Uwaga, przy zapisie brana jest pod uwage tylko m³odsza 16 bitowa czêœæ adresu 
// natomiast przy odczycie nal¿y podaæ pe³ny 32 bitowy adres komórki.
// kasujemy ca³a stronê, zapisujemu conajmniej s³owo (2 bajty)
// Przed zapisem lub kasowaniem nale¿y "odkluczyæ" pamiêæ Flash funkcji¹ FlashUnlock()

u16 licznik=1;

void FLASH_write(uint16_t* data, uint16_t size)
	{
		uint16_t i;
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
		//FLASH_Erase Page
		while((FLASH->SR&FLASH_SR_BSY));
		FLASH->CR |= FLASH_CR_PER; //Page Erase Set
		FLASH->AR = Page_ADR; //Page Address
		FLASH->CR |= FLASH_CR_STRT; //Start Page Erase
		while((FLASH->SR&FLASH_SR_BSY));
		FLASH->CR &= ~FLASH_CR_PER; //Page Erase Clear
		//FLASH_Program HalfWord
		FLASH->CR |= FLASH_CR_PG;
		for(i=0; i<size; i++){
			while((FLASH->SR&FLASH_SR_BSY));
			*(__IO uint16_t*)(Page_ADR + i*2) = data[i];
		}
		FLASH->CR &= ~FLASH_CR_PG;
		FLASH->CR |= FLASH_CR_LOCK;
	}

void FLASH_read(uint16_t* data, uint16_t size)
	{
		uint16_t i;
		for(i=0; i<size; i++){
			data[i] = *(__IO uint16_t*)(Page_ADR + i*2);
		}	 
	}

int  FlashUnlock(void) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
		if ((FLASH->CR & FLASH_CR_LOCK) == FLASH_CR_LOCK)	return -1;
		return 0;
	}
 
void FlashLock  (void) {
	FLASH->CR |= FLASH_CR_LOCK;
}
 
void ProgramFlashDataFromAddr(u32 addr, u32* data){
		u16 dat_m;	// dane rozd3zielone na dwie czêœci
		u16 dat_l;
		//u16 check;
		dat_l = (u16)( *data &  0xFFFF);								// 16 LSB
		dat_m = (u16)((*data & (0xFFFF << 16)) >> 16);	// 16 MSB
		
		FlashUnlock();
		FLASH->CR |= FLASH_CR_PG;			// w³¹czenie zapisu
		if ((FLASH->CR & FLASH_CR_LOCK) == 0 && (FLASH->SR & FLASH_SR_BSY) == 0) {
			*(u16*) addr = dat_l;					// zapis LSB
			while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
			//check = *(u16*)addr;
		}
		if ((FLASH->CR & FLASH_CR_LOCK) == 0 && (FLASH->SR & FLASH_SR_BSY) == 0) {
			addr += 2;
			*(u16*) addr = dat_m;					// zapis MSB
			while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
			//check = *(u16*)addr;
		}
		FLASH->CR &= (0xFFFFFFFF ^ FLASH_CR_PG);		// wy³¹czenie zapisu
		//check = *(u16*)addr;
		FlashLock  ();
	}
 

void ProgramFlashFromAddr(u32 addr, u16 *data, u16 size){
		
		FlashUnlock();
		FLASH->CR |= FLASH_CR_PG;								  // w³¹czenie zapisu
		if ( (FLASH->CR & FLASH_CR_LOCK) == 0 && (FLASH->SR & FLASH_SR_BSY) == 0){
			
			while(size--){ // romiar w s³owach a nie w bajtach
				*( u16*) addr = *(data++);
				addr += 2;
				//data++;
				while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
			}
		}	
		FLASH->CR &= (0xFFFFFFFF ^ FLASH_CR_PG);	// wy³¹czenie zapisu
		FlashLock  ();
	}

void EraseFlashPage(unsigned int addr) {
		FlashUnlock();
		
		if((FLASH->SR & FLASH_SR_BSY) == 0) {
			FLASH->CR |= FLASH_CR_PER;
			FLASH->AR = addr;
			FLASH->CR |= FLASH_CR_STRT;
			while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
			FLASH->CR &= (0xFFFFFFFF ^ FLASH_CR_PER); 
		}
		FlashLock  ();
	}

uint16_t Fl(u16 addr, char* str){
		u16 slen;
		//==== Odznacz komentarz poni¿ej fdy zapis do FLASH ====
		//ProgramFlashFromAddr(StrP122 + 32*addr, (u16*)str, (strlen(str)+2)/2 ); // +2 = + znak konca i + poniewaz dopelnienie reszty
		//ProgramFlashFromAddr(StrP119 + 16*addr, (u16*)str, (strlen(str)+2)/2 );
		slen = strlen(str)+1;
		UaPutS("\r\n ");	
		num2uart(licznik++, 3, 0);
		UaPutS(" 0x");	
		hex2uart(addr, 2);
		
		/*UaPutS(" Length ");	
		UART_sendNum(slen, 3, 0);
		UaPutS(" /16=");	
		UART_sendNum(slen/16, 3, 0); */
		//UaPutS((char*)StrPage+addr*32);	
		return addr + 1 + slen/16;
	}

void WriteStringToFlash(void){
//						    1				  2				  3				  4				  5				  6
//			 		123456789012345678901234567890123456789012345678901234567890123
u16 adr;
FlashUnlock();
/*0x00*/adr=Fl(0x00,"\r\n Time = ");
/*0x01*/adr=Fl(adr,"\r\n Start Programu ...\r\n");
/*0x03*/adr=Fl(adr," Init Card Error!");
/*0x05*/adr=Fl(adr," Init Card ok!");
/*0x06*/adr=Fl(adr,"\r\n List File and Directory "); 
/*0x08*/adr=Fl(adr," FAT32 not found!");
/*0x0a*/adr=Fl(adr," FAT32 ok!"); 
/*0x0b*/adr=Fl(adr,"\r\n Statistic Card  ");
/*0x0d*/adr=Fl(adr,"\r\n Erase block ");
/*0x0f*/adr=Fl(adr,"\r\n StartBlock   = ");
/*0x11*/adr=Fl(adr,"Other ERROR !!!");
/*0x13*/adr=Fl(adr,"\r\n How many blocks = ");	//adr=Fl(adr,"\r\n Number block = ");	
/*0x15*/adr=Fl(adr,"\r\n Error !!! ");
/*0x16*/adr=Fl(adr,"\r\n Block Erased ");
/*0x15*/adr=Fl(adr,"\r\n Read File ");										
/*0x18*/adr=Fl(adr,"\r\n File Name  = ");
/*0x19*/adr=Fl(adr,"\r\n Create File ");										
/*0x1d*/adr=Fl(adr,"\r\n File Name  = ");
/*0x1f*/adr=Fl(adr,"\r\n Delete file ");
/*0x21*/adr=Fl(adr,"\r\n Start Cluster = ");
/*0x23*/adr=Fl(adr,"\r\n Cluster (");	
/*0x24*/adr=Fl(adr,"\r\n Cluster Start  = ");
/*0x26*/adr=Fl(adr,"\r\n Cluster End = ");
/*0x28*/adr=Fl(adr,"\r\n Fill sector");
/*0x29*/adr=Fl(adr,"\r\n sectorNumber = ");
/*0x2b*/adr=Fl(adr,"\r\n Fill error");
/*0x2c*/adr=Fl(adr,"\r\n Fill OK");
/*0x2d*/adr=Fl(adr,"\r\n Modify memory");
/*0x2f*/adr=Fl(adr,"\r\n Memory Addr = ");
/*0x31*/adr=Fl(adr,"\r\n Memory Data = ");
/*0x33*/adr=Fl(adr,"\r\n New Data    = ");
/*0x35*/adr=Fl(adr,"\r\n Modify OK");
/*0x36*/adr=Fl(adr,"\r\n Copy memory");
/*0x37*/adr=Fl(adr,"\r\n Src Memory = ");
/*0x39*/adr=Fl(adr,"\r\n Len Memory = ");
/*0x3b*/adr=Fl(adr,"\r\n Dst Memory = ");
/*0x3d*/adr=Fl(adr,"\r\n Read Error !!!");		
/*0x3f*/adr=Fl(adr,"\r\n View last use sector");			
/*0x41*/adr=Fl(adr,"\r\n First Sector = ");
/*0x43*/adr=Fl(adr,"\r\n Exit Read Error !!!");
/*0x45*/adr=Fl(adr,"\r\n Zero sector = 0x");
/*0x47*/adr=Fl(adr,"\r\n getSetNextCluster Get, Set or Write ? ");
/*0x4a*/adr=Fl(adr,"\r\n bytesPerSector      = 0x");
/*0x4c*/adr=Fl(adr,"\r\n reservedSectorCount = 0x");
/*0x4e*/adr=Fl(adr,"\r\n numberofFATs        = 0x");
/*0x50*/adr=Fl(adr,"\r\n rootEntryCount      = 0x");
/*0x52*/adr=Fl(adr,"\r\n sectorsPerTrack     = 0x");
/*0x54*/adr=Fl(adr,"\r\n totalSectors_F32    = 0x");
/*0x56*/adr=Fl(adr,"\r\n FATsize_F32         = 0x");
/*0x58*/adr=Fl(adr,"\r\n volumeID            = 0x");
/*0x5a*/adr=Fl(adr,"\r\n bootEndSignature    = 0x");
/*0x5c*/adr=Fl(adr,"\r\n rootCluster         = 0x");
/*0x5e*/adr=Fl(adr,"\r\n dataSectors         = 0x");
/*0x60*/adr=Fl(adr,"\r\n unusedSectors       = 0x");
/*0x62*/adr=Fl(adr,"\r\n firstDataSector     = 0x");
/*0x64*/adr=Fl(adr,"Total Memory: ");
/*0x65*/adr=Fl(adr," Free Memory: ");
/*0x66*/adr=Fl(adr,"Error in getting cluster");
/*0x68*/adr=Fl(adr,"\r\n NextCluster   = 0x");
/*0x6a*/adr=Fl(adr," Creating File..");
/*0x6c*/adr=Fl(adr," No free cluster!");
/*0x6e*/adr=Fl(adr," Enter text (end with ~):");
/*0x70*/adr=Fl(adr," File appended!");
/*0x72*/adr=Fl(adr," File Created! ");
/*0x74*/adr=Fl(adr,"End of Cluster Chain");
/*0x76*/adr=Fl(adr,"Error in getting cluster");
/*0x78*/adr=Fl(adr,"\r\n File already exists, appending data..");
/*0x7b*/adr=Fl(adr,"\r\n appendStartCluster  = ");
/*0x7d*/adr=Fl(adr,"\r\n nextCluster         = ");
/*0x7f*/adr=Fl(adr,"\r\n clusterCount        = ");
/*0x81*/adr=Fl(adr," j(sector)=");
/*0x82*/adr=Fl(adr,"\r\n Number chars = ");
/*0x84*/adr=Fl(adr," File appended!");
/*0x86*/adr=Fl(adr,"End of Cluster Chain");
/*0x88*/adr=Fl(adr,"Error in getting cluster");

/*0x8a*/adr=Fl(adr,"\n\r\n\
 ==== ARM-STM32 ======================\r\n\
 r - Read   t - TR_mode	 m - Mult_rd  \r\n\
 w - Write  n - TX_mode	 s - Reset    \r\n\
 =====================================\r\n # ");
/*0x95*/adr=Fl(adr,"\n\r\n\
 ======= STM32F4 SD/MMC Card ======================\r\n\
 i - Init      r - Read  File  l - List File   	\r\n\
 d - Delete    w - Write File  s - Statistic Card  \r\n\
 m - Modf mem  t - Test  func  n - Next Cluster \r\n\
 o - Open blk  f - Write blk   e - Erase blk    \r\n\
 ==================================================\r\n# ");
// /*0x*/adr=Fl(adr,"");
FlashLock();
}

/*
adr=Fl(0x00,"\r\n Time = ");
adr=Fl(0x01,"\r\n Start Programu ...\r\n");
adr=Fl(0x02," Init Card Error!");
adr=Fl(0x03," Init Card ok!");
adr=Fl(0x04,"\r\n List File and Directory "); 
adr=Fl(0x05," FAT32 not found!");
adr=Fl(0x06," FAT32 ok!"); 
adr=Fl(0x07,"\r\n Statistic Card  ");
adr=Fl(0x08,"\r\n Erase block ");
adr=Fl(0x09,"\r\n StartBlock   = ");
adr=Fl(0x0a,"\r\n How many blocks = ");	//adr=Fl(0x0a,"\r\n Number block = ");	
adr=Fl(0x0b,"\r\n Error !!! ");
adr=Fl(0x0c,"\r\n Block Erased ");
adr=Fl(0x0d,"\r\n Read File ");										
adr=Fl(0x0e,"\r\n File Name  = ");
adr=Fl(0x0f,"\r\n Create File ");										
adr=Fl(0x10,"\r\n File Name  = ");
adr=Fl(0x11,"\r\n Delete file ");
adr=Fl(0x12,"\r\n Start Cluster = ");
adr=Fl(0x13,"\r\n Cluster (");	
adr=Fl(0x14,"\r\n Cluster Start  = ");
adr=Fl(0x15,"\r\n Cluster End = ");
adr=Fl(0x16,"\r\n Fill sector");
adr=Fl(0x17,"\r\n sectorNumber = ");
adr=Fl(0x18,"\r\n Fill error");
adr=Fl(0x19,"\r\n Fill OK");
adr=Fl(0x1a,"\r\n Modify memory");
adr=Fl(0x1b,"\r\n Memory Addr = ");
adr=Fl(0x1c,"\r\n Memory Data = ");
adr=Fl(0x1d,"\r\n New Data    = ");
adr=Fl(0x1e,"\r\n Modify OK");
adr=Fl(0x1f,"\r\n Copy memory");
adr=Fl(0x20,"\r\n Src Memory = ");
adr=Fl(0x21,"\r\n Len Memory = ");
adr=Fl(0x22,"\r\n Dst Memory = ");
adr=Fl(0x23,"\r\n Read Error !!!");		
adr=Fl(0x24,"\r\n View last use sector");			
adr=Fl(0x25,"\r\n First Sector = ");
adr=Fl(0x26,"\r\n Exit Read Error !!!");
adr=Fl(0x27,"\r\n Zero sector = 0x");
adr=Fl(0x28,"\r\n getSetNextCluster Get, Set or Write ? ");
adr=Fl(0x2A,"\n\r\n\

 001 0x00
 002 0x01
 003 0x03
 004 0x05
 005 0x06
 006 0x08
 007 0x0a
 008 0x0b
 009 0x0d
 010 0x0f
 011 0x11
 012 0x13
 013 0x15
 014 0x16
 015 0x18
 016 0x19
 017 0x1b
 018 0x1d
 019 0x1f
 020 0x21
 021 0x23
 022 0x24
 023 0x26
 024 0x28
 025 0x29
 026 0x2b
 027 0x2c
 028 0x2d
 029 0x2f
 030 0x31
 031 0x33
 032 0x35
 033 0x36
 034 0x37
 035 0x39
 036 0x3b
 037 0x3d
 038 0x3f
 039 0x41
 040 0x43
 041 0x45
 042 0x47
 043 0x4a
 044 0x4c
 045 0x4e
 046 0x50
 047 0x52
 048 0x54
 049 0x56
 050 0x58
 051 0x5a
 052 0x5c
 053 0x5e
 054 0x60
 055 0x62
 056 0x64
 057 0x65
 058 0x66
 059 0x68
 060 0x6a
 061 0x6c
 062 0x6e
 063 0x70
 064 0x72
 065 0x74
 066 0x76
 067 0x78
 068 0x7b
 069 0x7d
 070 0x7f
 071 0x81
 072 0x82
 073 0x84
 074 0x86
 075 0x88
 076 0x8a
 077 0x95

*/