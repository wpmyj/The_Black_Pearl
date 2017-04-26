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


#define PWM_frequency    10 // khz


static void TIM3_GPIO_Init(void)
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
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


}

#define START_PWM 100

static void TIM3_Config_Init(void)
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
    TIM_OCInitStructure.TIM_Pulse = START_PWM;	   //��������ֵ�������������������ֵʱ����ƽ��������
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��1
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = START_PWM;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);



    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = START_PWM;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);


    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = START_PWM;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);



    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
}


static void TIM4_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ����TIM3CLK Ϊ 72MHZ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* GPIOA and GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_14 | GPIO_Pin_15 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}


/**
  * @brief  ����TIM3�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
  * @param  ��
  * @retval ��
  */

/*
 * TIMxCLK/CK_PSC --> TIMxCNT --> TIMx_ARR --> TIMxCNT ���¼���
 *                    TIMx_CCR(��ƽ�����仯)
 * �ź�����=(TIMx_ARR +1 ) * ʱ������
 * ռ�ձ�=TIMx_CCR/(TIMx_ARR +1)
 */
void TIM4_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;


    /* Time base configuration */

    TIM_TimeBaseStructure.TIM_Period = 99;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 720 / PWM_frequency ; // (72 * 10) ;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz


    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = START_PWM;	   //��������ֵ�������������������ֵʱ����ƽ��������
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);


    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = START_PWM;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


    TIM_ARRPreloadConfig(TIM4, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM4, ENABLE);
}

void TIM3_Init(void)
{
    TIM3_GPIO_Init();
    TIM3_Config_Init();
}


void TIM4_PWM_Init(void)
{
    TIM4_GPIO_Config();
    TIM4_Mode_Config();
}



void Mos_Moto_Init(void)
{

    TIM3_Init();
    TIM4_PWM_Init();

//		Mos_GPIO_Init();
//		Mos_TIM_GPIO_Init();
//		Mos_TIM_Config_Init();
	
	
	
 RIGHT_DIREXTION  = Clockwise_Rotation;
 LEFT_DIREXTION  = Clockwise_Rotation;


 RIGHT_IN1	= 100;
 RIGHT_IN2	= 100;

 LEFT_IN1	 = 100;
 LEFT_IN2	 = 100;
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



void Mos_Moto_Test(void)
{

}


Moto_Change_Cmd_Control_t Moto_Change_Cmd_Control;

void  Moto_Right_Control(u8 cmd, int pwm)
{
    Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd = cmd;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm = 100 - pwm;
}

void  Moto_Left_Control(u8 cmd, int pwm)
{
    Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd = cmd;
    Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm =100 - pwm;
}


#define MOTO_CHANGE_STEP 5


void Moto_Chang_Control_Roll(void)
{
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
        }

        if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm >= 100 - MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = 100;
        }
				
        if(RIGHT_DIREXTION == Clockwise_Rotation)
        {
            RIGHT_IN2 =(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);

        } else	if(RIGHT_DIREXTION == Counter_Clockwise_Rotation)
        {
            RIGHT_IN1 =(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);
        }



        if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm == 100)
        {

            if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  == Moto_Z)
            {
                RIGHT_DIREXTION = Clockwise_Rotation;
            }
            else if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  == Moto_F)
            {
                RIGHT_DIREXTION = Counter_Clockwise_Rotation;
            }
            else if(Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  == Moto_T)
            {
	
                NEW_MOTO_ALL_STOP_RIGHT_NOW();
            }

            Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd = Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd;

        }

    }
    else   // �µķ��� �� ֮ǰ������ͬ
    {

        if(abs((int)(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm-Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm)) <= MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm;
        }
        else if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm > Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm - MOTO_CHANGE_STEP;
        }
        else if(Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm < Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm)
        {
            Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm + MOTO_CHANGE_STEP;
        }

        if(RIGHT_DIREXTION == Clockwise_Rotation)
        {
            RIGHT_IN2 = (Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);
						RIGHT_IN1  = 100;

        } else	if(RIGHT_DIREXTION == Counter_Clockwise_Rotation)
        {
            RIGHT_IN1 = (Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm);
						RIGHT_IN2 = 100;
        }

    }

    /////////////////////////////////////////////////////////////////////////////////////// moto2


    if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd != Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd) // �µķ��� �� ֮ǰ������ͬ
    {

        if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm <= 100)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm + MOTO_CHANGE_STEP;
        }

        if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm > 100 - MOTO_CHANGE_STEP)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = 100;
        }

        if(LEFT_DIREXTION == Clockwise_Rotation)
        {
         
					  LEFT_IN2 = (Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);
						LEFT_IN1  = 100;

        } else	if(LEFT_DIREXTION == Counter_Clockwise_Rotation)
        {
						LEFT_IN1 = (Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);
						LEFT_IN2  = 100;

        }
				
        if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm == 100)
        {

                if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  == Moto_Z)
                {
												LEFT_DIREXTION = Clockwise_Rotation;
                }
                else if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  == Moto_F)
                {
												LEFT_DIREXTION = Counter_Clockwise_Rotation;
                }
                else if(Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  == Moto_T)
                {
												NEW_MOTO_ALL_STOP_RIGHT_NOW();
                }

                Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd = Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd;

        }

    }
    else   // �µķ��� �� ֮ǰ������ͬ
    {

        if(abs((int)(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm-Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm)) <= MOTO_CHANGE_STEP)
        {
						Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm;
        }

        else if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm > Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm)
        {
            Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm - MOTO_CHANGE_STEP;
        }
        else if(Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm < Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm)
        {
							Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm + MOTO_CHANGE_STEP;
        }
				
        if(LEFT_DIREXTION == Clockwise_Rotation)
        {
            LEFT_IN2 = (Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);
						LEFT_IN1  = 100;

        } else	if(LEFT_DIREXTION == Counter_Clockwise_Rotation)
        {
            LEFT_IN1 = (Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm);
						LEFT_IN2 = 100;
        }

    }

}





void Moto_Direction_Moto_Change_One_Time(void) //ok
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


    /*		���ٶȷ�ת				*/
    TIM3->CCR3= TIM3->CCR4 = 100;

    TIM3->CCR1 =  100;
    TIM4->CCR3 = 100;


    for(i = 100; i >= 10; i = i -10)
    {
        TIM3->CCR2 = i;
        TIM4->CCR4 = i;
        Delay_ms(10);
    }

    for(i = 0; i <= 90; i =i +10)
    {
        TIM3->CCR2 = i;
        TIM4->CCR4 = i;
        Delay_ms(10);
    }


    TIM3->CCR2 =  100;
    TIM4->CCR4 = 100;

    /*		���ٶ���ת				*/
    TIM3->CCR3= TIM3->CCR4 = 0;



    Delay_ms(100);

    for(i = 100; i >= 10; i =i -10)
    {
        TIM3->CCR1 = i;
        TIM4->CCR3 = i;
        Delay_ms(10);
    }

    for(i = 0; i <= 90; i =i +10)
    {
        TIM3->CCR1 = i;
        TIM4->CCR3 = i;
        Delay_ms(10);
    }

    TIM3->CCR1 = 100;
    TIM4->CCR3 = 100;

}



void NEW_MOTO_ALL_STOP_RIGHT_NOW(void) //ok
{

    Moto_Change_Cmd_Control.Moto_Right_Control_last_pwm = 100;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_pwm  = 100;

    Moto_Change_Cmd_Control.Moto_Right_Control_last_cmd  = Moto_T;
    Moto_Change_Cmd_Control.Moto_Right_Control_now_cmd  = Moto_T;

    Moto_Change_Cmd_Control.Moto_Left_Control_now_pwm = 100;
    Moto_Change_Cmd_Control.Moto_Left_Control_last_pwm = 100;

    Moto_Change_Cmd_Control.Moto_Left_Control_last_cmd  = Moto_T;
    Moto_Change_Cmd_Control.Moto_Left_Control_now_cmd  = Moto_T;


    TIM3->CCR1 = 100;
    TIM3->CCR2 = 100;

    TIM4->CCR3 = 100;
    TIM4->CCR4 = 100;

    //TIM3->CCR3= TIM3->CCR4 =0; //����ת�ź�
}



void MOTO_TEST_CHANGE(void)		// ok
{
    uint16_t flag_time2_irq = 0;
    uint16_t flag_direction_irq = 1 ;
    u8 flag_chang_dircetion = 0;


//	/*		���ٶȷ�ת				*/
//
//				TIM3->CCR3= TIM3->CCR4 = 100;
//
//				TIM3->CCR1 =  100;
//				TIM4->CCR3 = 100;
//
//						TIM3->CCR2 =  0;
//				TIM4->CCR4  = 0;
//
//
////	while(1);
//
//
//	/*		���ٶ���ת				*/
//				TIM3->CCR3= TIM3->CCR4 = 0;
//
//				TIM3->CCR1 =  0;
//				TIM4->CCR3 = 0;
//
//				TIM3->CCR2 =  100;
//				TIM4->CCR4 = 100;


    while(1)
    {



        if(flag_time2_irq % 100 == 0)
        {

            if((TIM3->CCR3 == 100)&&(TIM3->CCR4 == 100))
            {
                TIM3->CCR2 =  flag_time2_irq / 10;
                TIM4->CCR4  = flag_time2_irq / 10;


            }
            else
            {
                TIM3->CCR1 = flag_time2_irq / 10;
                TIM4->CCR3 = flag_time2_irq / 10;
            }


            if(flag_time2_irq == 1000)
            {
                flag_direction_irq = 0;

                //        flag_chang_dircetion = !flag_chang_dircetion;

                if(flag_chang_dircetion == 1)
                {
                    flag_chang_dircetion = 0;
                    TIM3->CCR3= TIM3->CCR4 = 100;

                    TIM3->CCR2 =  100;
                    TIM4->CCR4  = 100;
                    TIM3->CCR1 = 100;
                    TIM4->CCR3 = 100;

                } else {
                    flag_chang_dircetion = 1;
                    TIM3->CCR3= TIM3->CCR4 = 0;

                    TIM3->CCR2 =  100;
                    TIM4->CCR4  = 100;
                    TIM3->CCR1 = 100;
                    TIM4->CCR3 = 100;

                }
            }

            if(flag_time2_irq == 0)
            {
                flag_direction_irq = 1;
            }

        }

        Delay_ms(5); // 50  5
        if(flag_direction_irq)
            flag_time2_irq++;
        else
            flag_time2_irq--;


    }

}

/*********************************************END OF FILE**********************/



