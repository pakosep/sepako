#ifndef __FLASH_H
#define __FLASH_H
 
int  FlashUnlock(void);
void FlashLock(void);
void ProgramFlashDataFromAddr	(u32 Flash_addr, u32* data);
void ProgramFlashFromAddr			(u32 Flash_addr, u16* data, u16 size);
void EraseFlashPage(unsigned int addr);
u16  Fl(u16 addr, char* str);
void WriteStringToFlash(void);
 
#endif //__UARTLIB_H