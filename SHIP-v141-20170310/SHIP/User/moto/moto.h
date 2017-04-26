


#ifndef __MOTO_H
#define __MOTO_H

#include "stm32f10x.h"
#include <stdio.h>

#define AI1 GPIO_Pin_10
#define AI0 GPIO_Pin_11
#define BI1 GPIO_Pin_8
#define BI0 GPIO_Pin_9


#define AENEL GPIO_Pin_14
#define BENEL GPIO_Pin_13

#define APHASE GPIO_Pin_15
#define BPHASE GPIO_Pin_12








#define ONE_STEP 0
#define TWO_STEP 38
#define THREE_STEP 71
#define FOUR_STEP 100




#define DELAY_Moto 30

enum MOTO_TYPE
{
    MOTO_STOP=0,
    MOTO_GO,
    MOTO_BACK,
    MOTO_RIGHT,
    MOTO_LIGHT,
    MOTO_ROLL_RIGHT,
    MOTO_ROLL_LIGHT
};

//#define MOTO_STOP 0
//#define MOTO_GO 1
//#define MOTO_BACK 2
//#define MOTO_RIGHT 3
//#define MOTO_LIGHT 4
//#define MOTO_ROLL_RIGHT 5
//#define MOTO_ROLL_LIGHT 6




#define Moto_Z 1  // sorry i do not know how to speak 正转
#define Moto_F 2  // sorry i do not know how to speak 反转
#define Moto_T 3  // sorry i do not know how to speak 停止



#define MOTO_CONTROL_LED_WIDTH  0x0
#define MOTO_CONTROL_LED_RIGHT 0x2
#define MOTO_CONTROL_LED_LEFT 0x4
#define MOTO_CONTROL_LED_STOP 0




#define Clockwise_Rotation 100											//正转
#define Counter_Clockwise_Rotation 0							//反转


#define RIGHT_DIREXTION  TIM3->CCR4
#define LEFT_DIREXTION   TIM3->CCR3



#define RIGHT_IN1					TIM3->CCR1
#define RIGHT_IN2					TIM3->CCR2


#define LEFT_IN1					TIM4->CCR3
#define LEFT_IN2					TIM4->CCR4



typedef struct
{
    u8 Moto_Right_Control_last_cmd;
    u8 Moto_Right_Control_now_cmd;
    u8 Moto_Right_Control_last_pwm;
    u8 Moto_Right_Control_now_pwm;
    u8 Moto_Left_Control_last_cmd;
    u8 Moto_Left_Control_now_cmd;
    u8 Moto_Left_Control_last_pwm;
    u8 Moto_Left_Control_now_pwm;

} Moto_Change_Cmd_Control_t;


void moto_gpio_init(void);

void TIM4_PWM_Init_Enab(void);
void TIM4_PWM_Change_Enab(int pwm1,int pwm2);


void  Moto_Right_Control(u8 cmd, int pwm);
void  Moto_Left_Control(u8 cmd, int pwm);

void Mos_Moto_Init(void);


void Mos_Moto_Test(void);
void Moto_Direction_Turn( int control, int pwm, int pwm_pro);

//void Mos_Moto_STOP_Mode(void);

void NEW_MOTO_ALL_STOP_RIGHT_NOW(void);


void Moto_Direction_Moto_Change_One_Time(void);


void Moto_Chang_Control_Roll(void);

void MOTO_TEST_CHANGE(void);



#endif
