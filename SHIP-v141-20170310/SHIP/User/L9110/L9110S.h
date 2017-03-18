#ifndef __L9110S_H
#define	__L9110S_H

#include "stm32f10x.h"



typedef struct  {
	u8 flag_hc1_on;
	u8 flag_hc2_on;
	u8 flag_hc3_on;
	u8 flag_hc4_on;
	u8 flag_hc5_on;
}Fishhook_Storage_t;


					
#define HC_ON  1
#define HC_OFF 0				
				
#define HC1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_15);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_15)	
#define HC2(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_8);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_8)	
#define HC3(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_9);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_9)	
#define HC4(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_10);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_10)	
#define HC5(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_11);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_11)						
					
				
#define LEDC_ON  0
#define LEDC_OFF 1	

					
#define LEDC1(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_7)	

#define LEDC2(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_8);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_8)	

#define LEDC3(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_9);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_9)	

#define LEDC4(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_10);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_10)						

#define LEDC5(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_11);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_11)		
					
#define LEDC6(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_12);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_12)			

#define LEDC7(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_13);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_13)		
					
#define LEDC8(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_14);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_14)			


#define LEDC9(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_15);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_15)		
					
#define LEDC10(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_14);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_14)	

					
void L9110S_GPIO_Config(void);
void L9110s_LLED_Test(void);					
					
					
void LEDS_ULN2003_GPIO_Config(void);					
void HC_ALL(int cmd);
void HC_MOS_GPIO_Config(void);
void LEDC_ALL(int cmd);		
void LEDC_OTHER(int cmd);

void HC_L9110s_Control(void);			
	
void chooseLED(u8 cmd, u8 s);				
#endif /* __LED_H */
