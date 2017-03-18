#ifndef TIME_TEST_H
#define TIME_TEST_H

#include "stm32f10x.h"
  #include "misc.h"
	#include "stm32f10x_tim.h"
//#define START_TIME  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
//#define STOP_TIME  TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

void TIM5_NVIC_Configuration(void);
void TIM5_Configuration(void);

#endif	/* TIME_TEST_H */
