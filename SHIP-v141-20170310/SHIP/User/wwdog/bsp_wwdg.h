#ifndef __WWDG_H
#define	__WWDG_H


#include "stm32f10x.h"
// ���ڼ�����ֵ����ΧΪ��0x40~0x7f��һ�����ó����0X7F
#define WWDG_CNT	0x7f


void WWDG_Config(uint8_t tr, uint8_t wr, uint32_t prv);
void WWDG_Feed(void);
void  Feed_Dog_When_Hungry(void);
#endif /* __IWDG_H */