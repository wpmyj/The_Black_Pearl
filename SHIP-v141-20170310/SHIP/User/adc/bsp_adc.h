#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"


void ADC1_Init(void);
void ADC1_GPIO_STOP_MODE(void);



/*

ADC_ConvertedValue[4]  :  // ADC 检测顺序
ADC_Channel_11 电池电压  ADC_ConvertedValue[0]

ADC_Channel_12 电流检测1 ADC_ConvertedValue[1]

ADC_Channel_13 电流检测2 ADC_ConvertedValue[2]

ADC_Channel_18 温度测量  ADC_ConvertedValue[3]

*/


#endif /* __ADC_H */

