


#ifndef __SECRET_H
#define	__SECRET_H


#include "stm32f10x.h"

typedef enum {  
    FLASH_WRITE_OK = 1,  
    FLASH_WRIKE_NO = 0,  
    FLASH_READ_OK = 1,  
    FLASH_READ_NO = 0  
}FLASH_FLAG ;  
  
extern unsigned char FLASH_WRITE(unsigned short int * memory_data);  
extern unsigned char FLASH_READ(unsigned short int * memory_data,unsigned short int n);  

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	;
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	;

void this_is_a_secret(void);
#endif /* __ADC_H */


