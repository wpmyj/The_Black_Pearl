#ifndef __IAP_H__
#define __IAP_H__

#include "stm32f10x.h"

typedef  void (*iapfun)(void);				//����һ���������͵Ĳ���.
#define  FLASH_APP_ADDR		0x08006000	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)
void iap_load_app(u32 appxaddr);

#endif 









