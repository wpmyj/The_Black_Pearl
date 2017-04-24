#include "moto.h"
#include "stm32f10x_gpio.h"
#include "L9110S.h"
#include "bsp_usart.h"
#include "bsp_SysTick.h"
#include <math.h>
#include <stdlib.h>

extern uint8_t respond[10];
extern u8 flag_cancel_seek;

u8 flag_led_by_car = 0;
u8 flag_adc_mask = 0;


extern int moto_control_led_mode ;
extern int flag_RIGHT_led_toggle_time, flag_LEFT_led_toggle_time,flag_WIDTH_led_toggle_time ;
extern u8 flag_Seek_Route_Check_Ing;
void Mos_Moto_STOP_Mode(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50Mhz
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
//	|GPIO_Pin_9|GPIO_Pin_8;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOD,GPIO_Pin_14);
    GPIO_SetBits(GPIOD,GPIO_Pin_15);
    GPIO_SetBits(GPIOC,GPIO_Pin_7);
    GPIO_SetBits(GPIOC,GPIO_Pin_6);
//		GPIO_SetBits(GPIOC,GPIO_Pin_9);
//		GPIO_SetBits(GPIOC,GPIO_Pin_8);
}

void Moto_Direction_Turn( int control, int pwm, int pwm_pro)
{

    //	printf("\r\n control = %d pwm = %d pwm_pro = %d\r\n",control,pwm, pwm_pro);

    if(control != MOTO_STOP) //�л�����״̬ʱ����ʱ�����adc
    {
        flag_adc_mask = 1;
    }

    switch(control)
    {
    case MOTO_GO:
        moto_control_led_mode =  MOTO_CONTROL_LED_STOP;
        Moto_Right_Control(Moto_F, pwm);
        Moto_Left_Control(Moto_F, pwm_pro);
        //	printf("MOTO_GO --\r\n");
        break;

    case MOTO_BACK:
        moto_control_led_mode =  MOTO_CONTROL_LED_STOP;
        Moto_Right_Control(Moto_Z, pwm);
        Moto_Left_Control(Moto_Z, pwm_pro);
        //	printf("MOTO_BACK --\r\n");
        break;

    case MOTO_STOP:
        TIM3->CCR3=100;
        TIM3->CCR4=100;
        Moto_Right_Control(Moto_T, 0);
        Moto_Left_Control(Moto_T, 0);
        moto_control_led_mode =  MOTO_CONTROL_LED_STOP;
        if(	(flag_cancel_seek !=  1)) //�ų����Զ��������л�״̬ʱ�򣬵��µ��ж϶�ʱ�����
            respond[6] &= 0xf8; //
        //	printf("MOTO_STOP --\r\n");
        break;

    case MOTO_RIGHT:
    case MOTO_LIGHT:
//			printf("MOTO_RIGHT  MOTO_LIGHT --\r\n");
        if(pwm > pwm_pro)
        {
            moto_control_led_mode =  MOTO_CONTROL_LED_RIGHT;
            flag_RIGHT_led_toggle_time = 500;
        } else if(pwm < pwm_pro)
        {
            moto_control_led_mode =  MOTO_CONTROL_LED_LEFT;
            flag_LEFT_led_toggle_time = 500;
        }

        if(	flag_Seek_Route_Check_Ing ==  1) //������ �ҡ� 30��
            moto_control_led_mode =  MOTO_CONTROL_LED_STOP;

        Moto_Right_Control(Moto_F, pwm);
        Moto_Left_Control(Moto_F, pwm_pro);

        break;
    case MOTO_ROLL_RIGHT:
        //	printf("  MOTO_ROLL_RIGHT --\r\n");
        Moto_Right_Control(Moto_Z, pwm);
        Moto_Left_Control(Moto_F, pwm_pro);
        moto_control_led_mode =  MOTO_CONTROL_LED_LEFT;

        break;
    case MOTO_ROLL_LIGHT:
        moto_control_led_mode =  MOTO_CONTROL_LED_RIGHT;
//			printf("  MOTO_ROLL_LIGHT --\r\n");
        Moto_Right_Control(Moto_F, pwm);
        Moto_Left_Control(Moto_Z, pwm_pro);
        break;
    default:
        Moto_Right_Control(Moto_F, 0);
        Moto_Left_Control(Moto_F, 0);
        break;

    }

}


static void TIM4_GPIO_Config_Enab(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ����TIM3CLK Ϊ 72MHZ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* GPIOA and GPIOB clock enable */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_14 | GPIO_Pin_13 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void TIM4_Mode_Config_Enab(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* -----------------------------------------------------------------------
        TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR+1)* 100% = 50%
        TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR+1)* 100% = 37.5%
        TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR+1)* 100% = 25%
        TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR+1)* 100% = 12.5%
      ----------------------------------------------------------------------- */

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 99;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 72;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	   //��������ֵ�������������������ֵʱ����ƽ��������
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��3
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//ʹ��ͨ��4
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM4, ENABLE);
		
}

void TIM4_PWM_Init_Enab(void)
{
    TIM4_GPIO_Config_Enab();
    TIM4_Mode_Config_Enab();
}

void TIM4_PWM_Change_Enab(int pwm1,int pwm2) // Ԥ����·����ӿ�
{
    TIM4->CCR2=pwm1;
    TIM4->CCR3=pwm2;
}


static void Mos_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50Mhz
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}
static void Mos_TIM_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ����TIM3CLK Ϊ 72MHZ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* GPIOA and GPIOB clock enable */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
//	 RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

    /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_8 | GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


}

#define PWM_frequency    10 // khz
static void Mos_TIM_Config_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* Time base configuration */
	
    TIM_TimeBaseStructure.TIM_Period = 99;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 720 / PWM_frequency ; // (72 * 10) ;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	   //��������ֵ�������������������ֵʱ����ƽ��������
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��1
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��3
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);	//ʹ��ͨ��4
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);


    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
}


void Mos_Moto_Init(void)
{
    Mos_GPIO_Init();
    Mos_TIM_GPIO_Init();
    Mos_TIM_Config_Init();
}

void Mos_Moto_Test(void)
{

    Delay_ms(1000);
    Moto_Right_Control(Moto_Z, 100);


    Delay_ms(1000);
    Moto_Right_Control(Moto_F, 100);

    Delay_ms(1000);
    Moto_Right_Control(Moto_F, 0);

    Moto_Direction_Moto_Z_100_pwm();
}


Moto_Change_Cmd_Control_t Moto_Change_Cmd_Control;

void  Moto_Right_Control(u8 cmd, int pwm)
{
//	static u8 last_cmd = 0;


    Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd = cmd;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm = 100 - pwm;



    //	 		printf(  "\r\n  Moto_Right_Controllast_pwm1 %d\r\n", Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm);
// if(cmd  == Moto_T)
//	{
//		GPIO_SetBits(GPIOD,GPIO_Pin_15);
//		GPIO_SetBits(GPIOD,GPIO_Pin_14);
//	}
//
//	pwm = 100 - pwm;
//	if(pwm  > 100)
//	pwm = 100;
//	if(pwm < 0)
//	pwm = 0;
//
//
//
//	TIM3->CCR3=pwm;


//	if(last_cmd != cmd) //�ж��Ƿ�Ϊ����ת ����ͬ��Ϊͬ��
//	{
//		if(cmd  == Moto_Z)
//		{
//				GPIO_SetBits(GPIOD,GPIO_Pin_15);
//				GPIO_ResetBits(GPIOD,GPIO_Pin_14);
//		}
//		else if(cmd  == Moto_F)
//		{
//				GPIO_SetBits(GPIOD,GPIO_Pin_14);
//				GPIO_ResetBits(GPIOD,GPIO_Pin_15);
//		}
//	}

    //last_cmd = cmd;
}






void  Moto_Left_Control(u8 cmd, int pwm)
{
    Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd = cmd;
    Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm =100 - pwm;

// 		printf(  "\r\n  Moto_Right_Controllast_pwm1 %d\r\n", Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm);
//		static u8 last_cmd = 0;
//
//		if(cmd  == Moto_T)
//		{
//			GPIO_SetBits(GPIOC,GPIO_Pin_6);
//			GPIO_SetBits(GPIOC,GPIO_Pin_7);
//		}
//
//		pwm = 100 - pwm;
//		if(pwm  > 100)
//			pwm = 100;
//		if(pwm < 0)
//			pwm = 0;
//
//		TIM3->CCR4=pwm;

//		if(last_cmd != cmd) //�ж��Ƿ�Ϊ����ת ����ͬ��Ϊͬ��
//		{
//			if(cmd  == Moto_Z)
//			{
//					GPIO_SetBits(GPIOC,GPIO_Pin_7);
//					GPIO_ResetBits(GPIOC,GPIO_Pin_6);
//			}
//			else if(cmd  == Moto_F)
//			{
//					GPIO_SetBits(GPIOC,GPIO_Pin_6);
//					GPIO_ResetBits(GPIOC,GPIO_Pin_7);
//			}
//			last_cmd = cmd;
//		}

}



#define MOTO_CHANGE_STEP 1

#define DELAY_SET_0_TIME  0 //���й�� ʱ��  DELAY_SET_0_TIME * 5 ms

void Moto_Chang_Control_Roll(void)
{
    static int flag_right_set_0 = 0, flag_left_set_0 = 0;


    if(Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm > 100)Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm =100;
    if(Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm  > 100)Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm  =100;

    if(Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm < 1)Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm = 0;
    if(Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm  < 1)Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm  = 0;


    /////////////////////////////////////////////////////////////////////////////////////// moto1

    if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd!= Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd) // �µķ��� �� ֮ǰ������ͬ
    {


        if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm <= 100)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm + MOTO_CHANGE_STEP;
            TIM3->CCR3=(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);

        }

        if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm >= 100 - MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = 100;
            TIM3->CCR3=(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);
        }


        if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm == 100)
        {

            if(flag_right_set_0  ==0 )
            {

                GPIO_SetBits(GPIOD,GPIO_Pin_15);
                GPIO_SetBits(GPIOD,GPIO_Pin_14);
							
            }
            if(flag_right_set_0 ++ >= DELAY_SET_0_TIME)
            {

                if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  == Moto_Z)
                {
                    GPIO_SetBits(GPIOD,GPIO_Pin_15);
                    GPIO_ResetBits(GPIOD,GPIO_Pin_14);
                }
                else if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  == Moto_F)
                {
                    GPIO_SetBits(GPIOD,GPIO_Pin_14);
                    GPIO_ResetBits(GPIOD,GPIO_Pin_15);
                }
                else if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  == Moto_T)
                {
                    GPIO_ResetBits(GPIOD,GPIO_Pin_14);
                    GPIO_ResetBits(GPIOD,GPIO_Pin_15);
                }

                flag_right_set_0 = 0;

                //      Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd;  //error
                Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd = Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd;

            }

        }
//		 if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm != 100)
//	 		printf(  "\r\n  --pwm1-- last_pwm1 %d\r\n", Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);

    }
    else   // �µķ��� �� ֮ǰ������ͬ
    {

        if(abs((int)(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm-Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm)) <= MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm;
            TIM3->CCR3=Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm;

        }
        else if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm > Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm - MOTO_CHANGE_STEP;

            TIM3->CCR3=Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm;


        }
        else if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm < Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm)
        {

            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm + MOTO_CHANGE_STEP;

            TIM3->CCR3=Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm;

        }
//		if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm != 100)
//	printf(  "\r\n --pwm1-- last_pwm1 %d\r\n", Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);

    }


    /////////////////////////////////////////////////////////////////////////////////////// moto2



    if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd != Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd) // �µķ��� �� ֮ǰ������ͬ
    {


        if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm <= 100)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm + MOTO_CHANGE_STEP;
            TIM3->CCR4=(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);

        }

        if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm > 100 - MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = 100;
            TIM3->CCR4=(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);
        }



        if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm == 100)
        {


            if(flag_left_set_0  == 0 )
            {
                GPIO_SetBits(GPIOC,GPIO_Pin_7);
                GPIO_SetBits(GPIOC,GPIO_Pin_6);

            }
            if(flag_left_set_0 ++ >= DELAY_SET_0_TIME)
            {

                if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  == Moto_Z)
                {
                    GPIO_SetBits(GPIOC,GPIO_Pin_7);
                    GPIO_ResetBits(GPIOC,GPIO_Pin_6);
                }
                else if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  == Moto_F)
                {
                    GPIO_SetBits(GPIOC,GPIO_Pin_6);
                    GPIO_ResetBits(GPIOC,GPIO_Pin_7);
                }
                else if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  == Moto_T)
                {
                    GPIO_ResetBits(GPIOC,GPIO_Pin_6);
                    GPIO_ResetBits(GPIOC,GPIO_Pin_7);
                }

                Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd = Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd;
                flag_left_set_0 = 0;

            }



        }

//		 if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm != 100)
//		 		printf(  "\r\n --pwm2-- last_pwm2 %d\r\n", Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);

    }
    else   // �µķ��� �� ֮ǰ������ͬ
    {

        if(abs((int)(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm-Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm)) <= MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm;
            TIM3->CCR4=Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm;
            //		printf(  "\r\n 1 last_pwm2 %d\r\n", Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);

        }

        else if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm > Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm - MOTO_CHANGE_STEP;

            TIM3->CCR4=Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm;
            //	printf(  "\r\n 2 last_pwm2 %d\r\n", Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);

        }
        else if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm < Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm)
        {

            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm + MOTO_CHANGE_STEP;

            TIM3->CCR4=Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm;
            //			printf(  "\r\n 3 last_pwm2 %d\r\n", Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);
        }
//		if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm != 100)
//		printf(  "\r\n --pwm2-- last_pwm2 %d\r\n", Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);

    }

}


void Moto_Direction_Moto_Z_100_pwm(void)
{
    u8 i;
    Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm =100;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm = 100;

    Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd  = Moto_Z;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  = Moto_Z;

    Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm = 100;
    Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = 100;

    Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd  = Moto_Z;
    Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  = Moto_Z;


    GPIO_SetBits(GPIOC,GPIO_Pin_7);
    GPIO_ResetBits(GPIOC,GPIO_Pin_6);
    GPIO_SetBits(GPIOD,GPIO_Pin_15);
    GPIO_ResetBits(GPIOD,GPIO_Pin_14);

    for(i = 100; i > 10; i =i -10)
    {
        TIM3->CCR3 = i;
        TIM3->CCR4 = i;
        Delay_ms(10);
    }
    Delay_ms(100);

}


void Moto_Direction_0_pwm(void)
{


    u8 i;

    Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm =0;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm = 0;

    Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd  = Moto_T;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  = Moto_T;

    Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm = 0;
    Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = 0;

    Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd  = Moto_T;
    Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  = Moto_T;

    for(i = 0; i < 100; i = i +10)
    {
        TIM3->CCR3 = i;
        TIM3->CCR4 = i;
        Delay_ms(10);
    }

    GPIO_ResetBits(GPIOC,GPIO_Pin_7);
    GPIO_ResetBits(GPIOC,GPIO_Pin_6);
    GPIO_ResetBits(GPIOD,GPIO_Pin_15);
    GPIO_ResetBits(GPIOD,GPIO_Pin_14);

}

//10.156.20.106
//255.255.255.240
//10.156.20.97
//202.96.209.133
//1500
//LSQ_LGG
//stm32LSQQ
//adminadmin

void Moto_Direction_Moto_F_100_pwm(void)
{
    u8 i;
    Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm =100;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm = 100;

    Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd  = Moto_Z;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  = Moto_Z;

    Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm = 100;
    Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = 100;

    Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd  = Moto_Z;
    Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  = Moto_Z;


    GPIO_SetBits(GPIOC,GPIO_Pin_6);
    GPIO_ResetBits(GPIOC,GPIO_Pin_7);
    GPIO_SetBits(GPIOD,GPIO_Pin_14);
    GPIO_ResetBits(GPIOD,GPIO_Pin_15);

    for(i = 100; i > 10; i =i -10)
    {
        TIM3->CCR3 = i;
        TIM3->CCR4 = i;
        Delay_ms(10);
    }
    Delay_ms(100);

}

/*********************************************END OF FILE**********************/



