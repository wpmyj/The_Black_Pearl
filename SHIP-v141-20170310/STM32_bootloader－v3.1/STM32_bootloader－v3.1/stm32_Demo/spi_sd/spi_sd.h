#ifndef __SPI_SD_H
#define __SPI_SD_H

#include "stm32f10x.h"


void SPI_SD_Init(void);
void SPI1_SetSpeed(u8 SpeedSet);
unsigned char	SPI1_ReadWrite(u8 writedat);

#endif /* __SPI_SD_H */
