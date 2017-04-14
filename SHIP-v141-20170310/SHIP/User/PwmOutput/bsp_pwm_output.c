/**
  ******************************************************************************
  * @file    bsp_pwm_output.c
  * @author  lijinnan
  * @version V1.0
  * @date    2015-01-18
  * @brief   PWM
  ******************************************************************************
  * @attention

  ******************************************************************************
  */ 
  
#include "bsp_pwm_output.h" 
#include "bsp_SysTick.h"
#include "stm32f10x_tim.h"
#include "bsp_usart.h"
 /**
  * @brief  ����TIM2�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
 extern 	u16 CCR1_Val;        
 extern 	u16 CCR2_Val;
 extern 	u16 CCR3_Val;
 extern	  u16 CCR4_Val;


u8 setBeepBlood;
static u8 setBeepBlood_pre = 0;
static int beepNum = 0, beepCnt = 0 ;

static void TIM3_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); 
	
  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void TIM2_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	
  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void TIM3_GPIO_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

void TIM2_GPIO_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}


 void TIM3_Mode_Config(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM�źŵ�ƽ����ֵ */
// 	u16 CCR1_Val = 500;        
// 	u16 CCR2_Val = 375;
// 	u16 CCR3_Val = 250;
// 	u16 CCR4_Val = 125;

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 28887;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  //TIM_OCInitStructure.TIM_Pulse = 10000;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
  //TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  //TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);	//ʹ��ͨ��4
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//��ע���˴��Ķ���һ���֣���ν����
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR

  //TIM_Cmd(TIM4, ENABLE);                   //ʹ�ܶ�ʱ��4	
}

void TIM2_Mode_Config(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM�źŵ�ƽ����ֵ */
// 	u16 CCR1_Val = 500;        
// 	u16 CCR2_Val = 375;
// 	u16 CCR3_Val = 250;
// 	u16 CCR4_Val = 125;

/* ----------------------------------------------------------------------- 
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR+1)* 100% = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR+1)* 100% = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR+1)* 100% = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR+1)* 100% = 12.5%
  ----------------------------------------------------------------------- */

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 28887;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  //TIM_OCInitStructure.TIM_Pulse = 10000;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
  //TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  //TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//��ע���˴��Ķ���һ���֣���ν����

	
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR
}

/**
  * @brief  TIM3 ���PWM�źų�ʼ����ֻҪ�����������
  *         TIM3���ĸ�ͨ���ͻ���PWM�ź����
  * @param  ��
  * @retval ��
  */
void TIM3_PWM_Init(void)
{
	TIM3_GPIO_Config();
	TIM3_Mode_Config();	
}

void TIM2_PWM_Init(void)
{
	TIM2_GPIO_Config();
	TIM2_Mode_Config();	
}



// u16 arr, ���Ƶ�ʣ����㷽ʽ = 72Mhz / arr 
// u16 pulse  ռ�ñȣ�������ֵ
 void TIM3_Mode_Change(u16 arr, float pulse)
 {
	 
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;

		/* Time base configuration */		 
		TIM_TimeBaseStructure.TIM_Period = arr - 1;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
		TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
		//TIM_OCInitStructure.TIM_Pulse = pulse;	   //��������ֵ�������������������ֵʱ����ƽ��������
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
		//TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
		//TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel4 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = pulse;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);	//ʹ��ͨ��4
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
		//��ע���˴��Ķ���һ���֣���ν����


		TIM_ARRPreloadConfig(TIM3, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR

		/* TIM3 enable counter */
		TIM_Cmd(TIM3, ENABLE);                   //ʹ�ܶ�ʱ��4	
			 
 
 }

 
 void TIM2_Mode_Change(u16 arr, float pulse)
 {

		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;

		/* Time base configuration */		 
		TIM_TimeBaseStructure.TIM_Period = arr - 1;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
		TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

		/* PWM1 Mode configuration: Channel4 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = pulse;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
		TIM_OC2Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4
		TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
		//��ע���˴��Ķ���һ���֣���ν����


		TIM_ARRPreloadConfig(TIM2, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR

		/* TIM3 enable counter */
		TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��4	
			 
		 
 }
 
 

//public function. 
u8 uCMDbeep_A1(void)
{
	
if((setBeepBlood == BEEPOFF))
{
		setBeepBlood = setBeepBlood_pre = BEEPOFF;
		beepNum = 0;
		beepCnt = 0;
}

if(beepCnt++ != 1)
		setBeepBlood = setBeepBlood_pre;



if((setBeepBlood != setBeepBlood_pre))
	{
			setBeepBlood_pre = setBeepBlood;	
			
		//	printf("setBeepBlood = %d\r\n",setBeepBlood);
			beepNum = 0;
			beepCnt = 0;
			TIM2_GPIO_Config();
	}
	
	
	if(setBeepBlood != 10)
	{
#ifdef STM32F103VCT6_MCU
			USART_SendData(UART4, setBeepBlood + '0');
			while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
#endif
	}

	
	switch (setBeepBlood)
	{

		case RECEIVE_CMD_NOTICE:

					if(beepCnt==1)
							TIM2_Mode_Change(45000,20000);
					if(beepCnt==20)
							TIM2_Mode_Change(45000,0);
					if(beepCnt==40)
					{
								beepCnt = 0;
								{setBeepBlood = BEEPOFF;}
					}
			
			break;
			
		case BEEPOFF:
//			TIM_Cmd(TIM5, DISABLE);
			TIM2_GPIO_Deinit();
	//		beepCnt = 0;
			break;
		
		case BCONTROL:
			if(!beepCnt)
			{
					beepNum = 1;
			}
			else
			{
					if(beepCnt==1)
							TIM2_Mode_Change(45000,20000);
					if(beepCnt==25)
							TIM2_Mode_Change(45000,0);
					if(beepCnt==50)
					{
							beepCnt = 0;
							if(--beepNum<=0)
								{setBeepBlood = BEEPOFF;}
					}
			}
			break;
//		case BERROR:
//			if(beepCnt==1)
//					TIM2_Mode_Change(20000,50);
//			if(beepCnt==250)
//					TIM2_Mode_Change(20000,0);
//			if(beepCnt==500)
//					beepCnt = 0;
//			break;
			
		case SEEK_STAR_NO:
				if(beepCnt==1)
					TIM2_Mode_Change(35000,2000);
			if(beepCnt==10)
					TIM2_Mode_Change(35000,0);
				if(beepCnt==20)
					TIM2_Mode_Change(35000,2000);
			if(beepCnt==30)
					TIM2_Mode_Change(35000,0);
							if(beepCnt==40)
					TIM2_Mode_Change(35000,2000);
			if(beepCnt==50)
					TIM2_Mode_Change(35000,0);
			if(beepCnt == 750)
					beepCnt = 0;
			break;
	
		case BEEP_ALL_ON_LOW_POWER:  //lost_control
			
			if(beepCnt==1)
					TIM2_Mode_Change(35000,2000);
			if(beepCnt==200)
					TIM2_Mode_Change(35000,0);
			if(beepCnt==910)
					beepCnt = 0;
		
			break;
		case LOST_CONTROL_BERROR:  //lost_control
			if(beepCnt==1)
					TIM2_Mode_Change(35000,2000);
			if(beepCnt==20)
					TIM2_Mode_Change(35000,0);
			if(beepCnt==30)
					TIM2_Mode_Change(35000,2000);
			if(beepCnt==50)
					TIM2_Mode_Change(35000,0);
			if(beepCnt == 410)
					beepCnt = 0;
			break;
		case BEND:
			if(!beepCnt)
			{
					beepNum = 3;
			}
			else
			{
					if(beepCnt==1)
							TIM2_Mode_Change(38800,2000);
					if(beepCnt==25)
							TIM2_Mode_Change(38800,0);
					if(beepCnt==40)
					{
							beepCnt = 0;
							if(--beepNum<=0)
								{setBeepBlood = BEEPOFF;}
					}
			}
			break;
			
		case BSTART:
			if(!beepCnt)
			{
					beepNum = 2;
			}
			else
			{
					if(beepCnt==1)
							TIM2_Mode_Change(38800,2000);
					if(beepCnt==25)
							TIM2_Mode_Change(38800,0);
					if(beepCnt==40)
					{
							beepCnt = 0;
							if(--beepNum<=0)
								{setBeepBlood = BEEPOFF;}
					}
			}
			break;
		default:
			beepCnt = 0;
			beepNum = 0;
			setBeepBlood = setBeepBlood_pre = BEEPOFF;
			break;
	}
	
	
	
	return setBeepBlood;
}

 
 
 void TIM3_Mode_Test(void)
 {

		Delay_ms(2000);
		TIM3_Mode_Change(45000,40000);
		Delay_ms(2000);
		TIM3_Mode_Change(45000,2000);
		Delay_ms(2000);
		TIM3_Mode_Change(45000,0);

 }
 
  void TIM2_Mode_Test(void)
 {

		Delay_ms(2000);
		TIM2_Mode_Change(45000,40000);
		Delay_ms(2000);
		TIM2_Mode_Change(45000,2000);
		Delay_ms(2000);
		TIM2_Mode_Change(45000,0);
	 
 }
/*********************************************END OF FILE**********************/
