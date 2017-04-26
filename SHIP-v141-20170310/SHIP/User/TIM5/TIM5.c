/**
  ******************************************************************************
  * @file    TIM5.c
  * @author  lijinnan
  * @version V1.0
  * @date    2015-01-18
  * @brief   tim5
  ******************************************************************************
  * @attention
  *
  * 实验平台:cyb-bot-smartcar-3
  * 网址    :http://www.cyb-bot.com/
  *
  ******************************************************************************
  */ 
	
#include "TIM5.h"


/*
 * 函数名：TIM2_NVIC_Configuration
 * 描述  ：TIM2中断优先级配置
 * 输入  ：无
 * 输出  ：无	
 */
 
 #ifdef STM32F103VCT6_MCU
void TIM5_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*TIM_Period--1000   TIM_Prescaler--71 -->中断周期为1ms*/
void TIM5_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
    TIM_DeInit(TIM5);
    TIM_TimeBaseStructure.TIM_Period=5000;		 								/* 自动重装载寄存器周期的值(计数值) */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				                /* 时钟预分频数 72M/360 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		                /* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;                   /* 向上计数模式 */
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);							    		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM5, ENABLE);																		/* 开启时钟 */
    
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		/*先关闭等待使用*/    
}

#endif


void TIM4_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*TIM_Period--1000   TIM_Prescaler--71 -->中断周期为1ms*/
void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period=5000;		 								/* 自动重装载寄存器周期的值(计数值) */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				                /* 时钟预分频数 72M/360 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		                /* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;                   /* 向上计数模式 */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);							    		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4, ENABLE);																		/* 开启时钟 */
    
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		/*先关闭等待使用*/    
}


void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = macTIM_IRQ;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM1_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		TIM_DeInit(TIM1);   
		/* 设置TIM2CLK 为 72MHZ */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
    //TIM_DeInit(TIM2);
		
	/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period=5000; // 5000
	
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
	  /* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 72-1; // 72 -1
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //高级定时器需要加上这个，否则会不准确，甚至相差很大
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	
	
		TIMx_NVIC_Configuration();
	
	
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
		
    TIM_Cmd(TIM1, ENABLE);									
  //  RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM1, DISABLE);
}



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
