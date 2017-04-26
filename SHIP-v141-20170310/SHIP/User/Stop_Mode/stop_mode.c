
#include "stop_mode.h"
#include "bsp_usart.h"
#include "L9110S.h"
#include "bsp_led.h"
#include "oled.h"
#include "moto.h"
#include "bsp_adc.h"
#include "bsp_key.h"
#include "bsp_SysTick.h"

int flag_idle_times = 0;


void USARTS_STOP_Mode(void)
{

		
		GPIO_InitTypeDef GPIO_InitStructure;

    /* config USART1 clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOB, ENABLE);
		
		
    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	

		/* USART2 GPIO config */
		/* Configure USART2 Tx (PA.02) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
				
		/* Configure USART2 Rx (PA.03) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	

		/* USART3 GPIO config */
		/* Configure USART3 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; // as14b io
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		
		
		/* USART4 GPIO config */
		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);


}



void 	IIC_STOP_Mode(void)
{
	
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOE, ENABLE); 														   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9;		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

												   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOB,GPIO_Pin_6);
		GPIO_SetBits(GPIOB,GPIO_Pin_7);
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		GPIO_ResetBits(GPIOE,GPIO_Pin_0);
}


void SD_STOP_Mode(void)
{
	
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD |RCC_APB2Periph_AFIO , ENABLE); 		
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); //PA15Ĭ��jtag
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_4;		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
																   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);
												   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
		GPIO_ResetBits(GPIOA,GPIO_Pin_15);
		GPIO_ResetBits(GPIOD,GPIO_Pin_4);
}
void  Osc_32_Remap(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO , ENABLE); 														   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
		GPIO_ResetBits(GPIOC,GPIO_Pin_14);
		GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}

void Others_STOP_Mode(void)
{
	
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE |RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 														   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_Init(GPIOE, &GPIO_InitStructure);	
																   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	
		// usb
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		// ���
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;	
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOD,GPIO_Pin_0);
		GPIO_ResetBits(GPIOE,GPIO_Pin_1);
		GPIO_ResetBits(GPIOA,GPIO_Pin_12);
		GPIO_ResetBits(GPIOA,GPIO_Pin_11);
		GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	

		// a0 wake_up  **
		// a1 beep  ** 


		// CC1101
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_ResetBits(GPIOC,GPIO_Pin_4);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
		
		
		//uart5
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//u5_int
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		GPIO_ResetBits(GPIOC,GPIO_Pin_12);
		GPIO_ResetBits(GPIOD,GPIO_Pin_2);
		GPIO_ResetBits(GPIOD,GPIO_Pin_1);
	
	// ADC ʹ�ܶ����øߵ�ƽ���޵�ѹ��
		GPIO_SetBits(GPIOB,GPIO_Pin_1);	
		
		// boot 1
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_2);
}

void Enter_Stop_Status_GPIO_Init_All_Same(void) // 23uA
{
		
		
		GPIO_InitTypeDef GPIO_InitStructure;
	

		/*����GPIOB��GPIOF������ʱ��*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD |RCC_APB2Periph_GPIOA 
													|RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE); 
		
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); //PA15Ĭ��jtag
	//GPIO_PinRemapConfig(GPIO_Remap_PD01,ENABLE); //�˴�������һ��1uA 
													   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;

	
		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		GPIO_Init(GPIOC, &GPIO_InitStructure);	
		GPIO_Init(GPIOD, &GPIO_InitStructure);	
		GPIO_Init(GPIOE, &GPIO_InitStructure);	
			
		GPIO_SetBits(GPIOA,GPIO_Pin_All);
		GPIO_SetBits(GPIOB,GPIO_Pin_All);
		GPIO_SetBits(GPIOC,GPIO_Pin_All);
		GPIO_SetBits(GPIOD,GPIO_Pin_All);
		GPIO_SetBits(GPIOE,GPIO_Pin_All);
		
}





 void Enter_Stop_Status_GPIO_Init(void)
{
	
	
	// ���㹳�ȵط��޸�Ϊ   ������� �͵�ƽ
		HC_ALL(HC_OFF);
		
		//  LEDC 1 - 10 �رյ͵�ƽ
		LEDC_ALL(LEDC_OFF);
		
		//�رյ���ʹ�õ�LED
		P_LED_OFF;
		LED1_OFF; // system LED
		LED2_OFF;
		
		// OLED  ����stopģʽ
		OLED_STOP_Mode();
		
		// sd�����֣� ���������򲿷ֳ�ʼ��
		SD_STOP_Mode();
		
		// ������ֽ���stopģʽ
//		Mos_Moto_STOP_Mode();	
		
		// iic ����
		IIC_STOP_Mode();
		
		// adc ����  ��Ҫ������
		ADC1_GPIO_STOP_MODE();
		
		// ���� ��Ϊ���� 
		uKey_STOP_MODE();
		
		USARTS_STOP_Mode();
		
		// ���� ��Ϊ��ͨIO
		Osc_32_Remap();
		// ����IO
		Others_STOP_Mode();
			
}


void Test_Sys_And_Wake_up(void)
{

	
		// ���㹳�ȵط��޸�Ϊ   ������� �͵�ƽ
		HC_ALL(HC_OFF);
		
		//  LEDC 1 - 10 �رյ͵�ƽ
		LEDC_ALL(LEDC_ON);
		
		//�رյ���ʹ�õ�LED
		P_LED_OFF;
		LED1_OFF; // system LED
		LED2_OFF;

			// OLED  ����stopģʽ
		OLED_STOP_Mode();

		// iic ����
		IIC_STOP_Mode();

			// sd�����֣� ���������򲿷ֳ�ʼ��
		SD_STOP_Mode();
	
	
	// ���� ��Ϊ��ͨIO
		Osc_32_Remap();
	
//		Mos_Moto_STOP_Mode();

		Others_STOP_Mode();	
	


}


void SYSCLKConfig_STOP(void)
{
	ErrorStatus HSEStartUpStatus;
  /* ʹ�� HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* �ȴ� HSE ׼������ */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

    /* ʹ�� PLL */ 
    RCC_PLLCmd(ENABLE);

    /* �ȴ� PLL ׼������ */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* ѡ��PLL��Ϊϵͳʱ��Դ */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* �ȴ�PLL��ѡ��Ϊϵͳʱ��Դ */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

					/* ��������Żָ�Ϊ��ǰ����������״̬ */
}



void Close_All_RCC(void)
{


			RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_AFIO| RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB|
			RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE|
			RCC_APB2Periph_GPIOF| RCC_APB2Periph_GPIOG| RCC_APB2Periph_ADC1|
			RCC_APB2Periph_ADC2| RCC_APB2Periph_TIM1| RCC_APB2Periph_SPI1|
			RCC_APB2Periph_TIM8| RCC_APB2Periph_USART1| RCC_APB2Periph_ADC3|
			RCC_APB2Periph_TIM15| RCC_APB2Periph_TIM16| RCC_APB2Periph_TIM17|
			RCC_APB2Periph_TIM9| RCC_APB2Periph_TIM10| RCC_APB2Periph_TIM11 
			, DISABLE);


			RCC_APB1PeriphClockCmd(
			RCC_APB1Periph_TIM2| RCC_APB1Periph_TIM3| RCC_APB1Periph_TIM4|
			RCC_APB1Periph_TIM5| RCC_APB1Periph_TIM6| RCC_APB1Periph_TIM7|
			RCC_APB1Periph_WWDG| RCC_APB1Periph_SPI2| RCC_APB1Periph_SPI3|
			RCC_APB1Periph_USART2| RCC_APB1Periph_USART3| RCC_APB1Periph_UART4| 
			RCC_APB1Periph_UART5| RCC_APB1Periph_I2C1| RCC_APB1Periph_I2C2|
			RCC_APB1Periph_USB| RCC_APB1Periph_CAN1| RCC_APB1Periph_BKP|
			RCC_APB1Periph_PWR| RCC_APB1Periph_DAC| RCC_APB1Periph_CEC|
			RCC_APB1Periph_TIM12| RCC_APB1Periph_TIM13| RCC_APB1Periph_TIM14
			,DISABLE);

			RCC_AHBPeriphClockCmd(
			RCC_AHBPeriph_DMA1
			| RCC_AHBPeriph_DMA2
			| RCC_AHBPeriph_SRAM
			| RCC_AHBPeriph_FLITF
			| RCC_AHBPeriph_CRC
			| RCC_AHBPeriph_FSMC    
			,DISABLE

			);
}

	void STM_EVAL_LEDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); //PA15Ĭ��jtag	
		GPIO_PinRemapConfig(GPIO_Remap_PD01,ENABLE); //
	
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_All);
	GPIO_ResetBits(GPIOB, GPIO_Pin_All);
	GPIO_ResetBits(GPIOC, GPIO_Pin_All);
	GPIO_ResetBits(GPIOD, GPIO_Pin_All);
	GPIO_ResetBits(GPIOE, GPIO_Pin_All);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	
}

void  Enter_Stop_Status(void)
{
	
				//		Close_All_RCC();

				DMA_Cmd (DMA1_Channel6,DISABLE);	// gps dms 

				DMA_Cmd(DMA1_Channel1, DISABLE);
				/* Enable ADC1 DMA */
				ADC_DMACmd(ADC1, DISABLE);
				/* Enable ADC1 */
				ADC_Cmd(ADC1, DISABLE); // adc ��� ��dma

				ADC_SoftwareStartConvCmd(ADC1, DISABLE);

				TIM_Cmd(TIM5, DISABLE);	 // timer
				TIM_Cmd(TIM3, DISABLE);   // moto pwm 
				TIM_Cmd(TIM2, DISABLE);   //beep pwm 
				//
				Delay_us(100000);
	
				/* ʹ�ܵ�Դ����Ԫ��ʱ�� */
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	

				//�ر�����IO�ڵĹ���
				//		Enter_Stop_Status_GPIO_Init();
				//		Enter_Stop_Status_GPIO_Init_All_Same(); //23uA
				//	 STM_EVAL_LEDInit(); //  24uA
				Test_Sys_And_Wake_up();


				//	RCC_LSEConfig(RCC_LSE_OFF);//�ر�RTC�� ���ص���Ҳ����Ҳ��Ӱ��	


				//		printf("\r\n ����ֹͣģʽ \r\n");
				EXIT_USART3_Init();   // uart�жϻ���


				/* ����ֹͣģʽ�����õ�ѹ������Ϊ�͹���ģʽ���ȴ��жϻ���*/
				PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);	

				flag_idle_times = 0;

				/* ��������Żָ�Ϊ��ǰ����������״̬ */
	
}


