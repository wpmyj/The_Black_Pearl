/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  lijinnan
  * @version V1.0
  * @date    2015-01-18
  * @brief   ����led
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:cyb-bot-smartcar-3
  * ��ַ    :http://www.cyb-bot.com/
  *
  ******************************************************************************
  */ 
	
 
#include "bsp_led.h"
#include "misc.h"
#include "bsp_SysTick.h"
 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void LED_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����GPIOB��GPIOF������ʱ��*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE); 
		
		/*ѡ��Ҫ���Ƶ�GPIOB����*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOB*/
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIOE����*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOE*/
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
