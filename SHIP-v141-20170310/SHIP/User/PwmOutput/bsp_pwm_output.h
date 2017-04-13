#ifndef __PWM_OUTPUT_H
#define	__PWM_OUTPUT_H

#include "stm32f10x.h"




void TIM3_PWM_Init(void);


void TIM3_Mode_Change(u16 arr,float pulse);
void TIM3_Mode_Test(void);

void TIM2_PWM_Init(void);

void TIM2_Mode_Change(u16 arr,float pulse);
void TIM2_Mode_Test(void);

void TIM2_GPIO_Deinit(void);


	 
typedef enum
{ BEEPOFF = 0,
	BPOWERON,
	BCONTROL,	
	BSTART,		
	BEND,			
	BERROR,		
	BTRUE,
	LOST_CONTROL_BERROR,
	SEEK_STAR_NO,
	RECEIVE_CMD_NOTICE,  //9
	BEEP_ALL_ON_LOW_POWER,
	BEEP_MAX
}beepAction;


typedef enum
{ 
	USTARTSET = 0,
	UENDSET,
	USHIPGO,
	USHIPBACK,
	USHIPCANCEL,
	UWORK
}uCMD;

u8 uCMDbeep(void);	 
u8 uCMDbeep_A1(void);

#endif /* __PWM_OUTPUT_H */

