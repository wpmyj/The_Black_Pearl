#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"


void ADC1_Init(void);
void ADC1_GPIO_STOP_MODE(void);



/*

ADC_ConvertedValue[4]  :  // ADC ���˳��
ADC_Channel_11 ��ص�ѹ  ADC_ConvertedValue[0]

ADC_Channel_12 �������1 ADC_ConvertedValue[1]

ADC_Channel_13 �������2 ADC_ConvertedValue[2]

ADC_Channel_18 �¶Ȳ���  ADC_ConvertedValue[3]

*/


#endif /* __ADC_H */

