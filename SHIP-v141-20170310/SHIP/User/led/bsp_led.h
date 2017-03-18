#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_8);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_8)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_2);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_2)

					/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)				{p->BRR	=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态


#define LED1_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_8)
#define LED1_OFF			digitalHi(GPIOB,GPIO_Pin_8)
#define LED1_ON				digitalLo(GPIOB,GPIO_Pin_8)

#define LED2_TOGGLE		digitalToggle(GPIOE,GPIO_Pin_2)
#define LED2_OFF			digitalHi(GPIOE,GPIO_Pin_2)
#define LED2_ON				digitalLo(GPIOE,GPIO_Pin_2)


#define P_LED_OFF			digitalHi(GPIOE,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6)
#define P_LED_ON			digitalLo(GPIOE,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6)

#define P_LED1_OFF			digitalHi(GPIOE,GPIO_Pin_3)
#define P_LED1_ON				digitalLo(GPIOE,GPIO_Pin_3)

#define P_LED2_OFF			digitalHi(GPIOE,GPIO_Pin_4)
#define P_LED2_ON				digitalLo(GPIOE,GPIO_Pin_4)

#define P_LED3_OFF			digitalHi(GPIOE,GPIO_Pin_5)
#define P_LED3_ON				digitalLo(GPIOE,GPIO_Pin_5)

#define P_LED4_OFF			digitalHi(GPIOE,GPIO_Pin_6)
#define P_LED4_ON				digitalLo(GPIOE,GPIO_Pin_6)





void LED_GPIO_Config(void);
					
				
void sysInitIndictor(void);
void sysIndictor(void);
#endif /* __LED_H */
