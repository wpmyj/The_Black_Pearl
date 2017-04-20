


#ifndef __SECRET_H
#define	__SECRET_H


#include "stm32f10x.h"

typedef enum {
    FLASH_WRITE_OK = 1,
    FLASH_WRIKE_NO = 0,
    FLASH_READ_OK = 1,
    FLASH_READ_NO = 0
} FLASH_FLAG ;


int Get_ChipID(void);
extern unsigned char FLASH_WRITE(unsigned short int *memory_data, int n);
extern unsigned char FLASH_READ(unsigned short int * memory_data,unsigned short int n);




#endif /* __ADC_H */


