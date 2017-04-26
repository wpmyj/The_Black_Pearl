#ifndef TIME_TEST_H
#define TIME_TEST_H

#include "stm32f10x.h"
  #include "misc.h"
	#include "stm32f10x_tim.h"
//#define START_TIME  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
//#define STOP_TIME  TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)






#define             macTIMx                                TIM1
#define             macTIM_APBxClock_FUN                   RCC_APB2PeriphClockCmd
#define             macTIM_CLK                             RCC_APB2Periph_TIM1
#define             macTIM_IRQ                             TIM1_UP_IRQn
#define             macTIM_INT_FUN                         TIM1_UP_IRQHandler



void TIM5_NVIC_Configuration(void);
void TIM5_Configuration(void);

void TIM4_NVIC_Configuration(void);
void TIM4_Configuration(void);


void TIM1_Configuration(void);

#endif	/* TIME_TEST_H */
