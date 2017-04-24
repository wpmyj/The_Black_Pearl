/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  lijinnan
  * @version V1.0
  * @date    2015-01-18
  * @brief   测试led
  ******************************************************************************
  * @attention
  *
  * 实验平台:cyb-bot-smartcar-3
  * 网址    :http://www.cyb-bot.com/
  *
  ******************************************************************************
  */ 
	
 
#include "bsp_led.h"
#include "misc.h"
#include "bsp_SysTick.h"
 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启GPIOB和GPIOF的外设时钟*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE); 
		
		/*选择要控制的GPIOB引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIOB*/
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		
		/*选择要控制的GPIOE引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIOE*/
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		
		P_LED_OFF;
		LED1_OFF;
		LED2_OFF;
}


void sysInitIndictor(void)
{

		LED1_ON;
		Delay_ms(30);
		LED2_ON;
		Delay_ms(30);

	
		LED1_OFF;
		Delay_ms(30);
		LED2_OFF;
		Delay_ms(30);

}

void sysIndictor(void)
{
		static u8 sysLedCnt = 0;
	
		if(++sysLedCnt<10)
		{	LED1_ON;}
		else if(sysLedCnt<250)
		{LED1_OFF;}
		else
		{sysLedCnt = 0;}
}


/*********************************************END OF FILE**********************/
