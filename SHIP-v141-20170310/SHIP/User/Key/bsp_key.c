/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  LJN
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   I/O线中断应用bsp
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 ISO-MINI STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_key.h"
#include "stm32f10x_exti.h"
#include "misc.h"



void uKey_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 


	/* config the extiline clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);


		/* EXTI line gpio config*/	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入
  GPIO_Init(GPIOD, &GPIO_InitStructure);
		/* EXTI line gpio config*/	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	 // 下拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}



void uKey_STOP_MODE(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	/* config the extiline clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);


		/* EXTI line gpio config*/	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 下拉输入
  GPIO_Init(GPIOD, &GPIO_InitStructure);
		/* EXTI line gpio config*/	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	 // 上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}



/*********************************************END OF FILE**********************/
