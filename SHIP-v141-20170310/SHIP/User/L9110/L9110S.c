#include "L9110S.h"
#include "bsp_SysTick.h"
#include "manager.h"

Fishhook_Storage_t Fishhook_Storage_Control;



extern uint8_t flag_WITH_LIGHT , flag_HEAD_TAIL_LIGHT ,flag_ALL_LIGHT ;
extern u8 flag_led_by_car;
//int flag_hc1_on = 0;
//int flag_hc2_on = 0;
//int flag_hc3_on = 0;
//int flag_hc4_on = 0;
//int flag_hc5_on = 0;


void HC_MOS_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE); 
												   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_11;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOD, &GPIO_InitStructure);	
													   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void LEDS_ULN2003_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE); 
												   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_11
																	|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOE, &GPIO_InitStructure);	
													   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	//2017新pcb暂时未使用
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void HC_ALL(int cmd)
{
			HC1(cmd);
			HC2(cmd);
			HC3(cmd);
			HC4(cmd);
			HC5(cmd);
}



void LEDC_ALL(int cmd)
{
	LEDC1(cmd);
	LEDC2(cmd);
	LEDC3(cmd);
	LEDC4(cmd);
	LEDC5(cmd);
	LEDC6(cmd);
	LEDC7(cmd);
	LEDC8(cmd);
	LEDC9(cmd);
//	LEDC10(cmd);
}

void LEDC_OTHER(int cmd)
{

//	LEDC7(cmd);
//	LEDC8(cmd);
//	LEDC9(cmd);
//	LEDC10(cmd);
}


void L9110S_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE); 
												   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_11;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOE, &GPIO_InitStructure);	
													   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_SetBits(GPIOE,GPIO_Pin_12);  
		GPIO_SetBits(GPIOE,GPIO_Pin_11);  
		GPIO_SetBits(GPIOE,GPIO_Pin_10);  
		GPIO_SetBits(GPIOE,GPIO_Pin_9);  
		GPIO_SetBits(GPIOE,GPIO_Pin_8);  
		GPIO_SetBits(GPIOE,GPIO_Pin_7);  

		GPIO_SetBits(GPIOB,GPIO_Pin_0);  
		GPIO_SetBits(GPIOB,GPIO_Pin_1);  
}


void L9110s_LLED_Test(void)
{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);  
		GPIO_SetBits(GPIOB,GPIO_Pin_1); 
		Delay_ms(2000);
		GPIO_SetBits(GPIOB,GPIO_Pin_0);  
		GPIO_ResetBits(GPIOB,GPIO_Pin_1); 
		Delay_ms(2000);

		GPIO_SetBits(GPIOB,GPIO_Pin_0);  
		GPIO_SetBits(GPIOB,GPIO_Pin_1);  
		Delay_ms(5000);

		GPIO_ResetBits(GPIOB,GPIO_Pin_0);  
		GPIO_SetBits(GPIOB,GPIO_Pin_1); 
		Delay_ms(2000);
		GPIO_SetBits(GPIOB,GPIO_Pin_0);  
		GPIO_ResetBits(GPIOB,GPIO_Pin_1); 
		Delay_ms(2000);

		GPIO_SetBits(GPIOB,GPIO_Pin_0);  
		GPIO_SetBits(GPIOB,GPIO_Pin_1);  
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);  
		GPIO_ResetBits(GPIOB,GPIO_Pin_1); 
		Delay_ms(5000);
}




void HC_L9110s_Control(void)
{
	static int flag_HC1_n_number = 0;
	static int flag_HC2_n_number = 0;
	static int flag_HC3_n_number = 0;
	static int flag_HC4_n_number = 0;
	static int flag_HC5_n_number = 0;
	
	//////////////////////////////////////////////////
	if( Fishhook_Storage_Control.flag_hc1_on == 1)//得到信号的时间
	{
		HC1(HC_ON);
		Fishhook_Storage_Control.flag_hc1_on = 2; //确认信号，并开始计时。
	}else if(Fishhook_Storage_Control.flag_hc1_on == 2)
	{
		if(flag_HC1_n_number ++ >= 100)
		{
			HC1(HC_OFF);
			Fishhook_Storage_Control.flag_hc1_on = 0;
			flag_HC1_n_number = 0;
		}
	}
	/////////////////////////////////////////////////
	if(Fishhook_Storage_Control.flag_hc2_on == 1)//得到信号的时间
	{
		HC2(HC_ON);
		Fishhook_Storage_Control.flag_hc2_on = 2; //确认信号，并开始计时。
	}else if(Fishhook_Storage_Control.flag_hc2_on == 2)
	{
		if(flag_HC2_n_number ++ >= 100)
		{
			HC2(HC_OFF);
			Fishhook_Storage_Control.flag_hc2_on = 0;
			flag_HC2_n_number = 0;
		}
	}	
	
		if(Fishhook_Storage_Control.flag_hc3_on == 1)//得到信号的时间
	{
		HC3(HC_ON);
		Fishhook_Storage_Control.flag_hc3_on = 2; //确认信号，并开始计时。
	}else if(Fishhook_Storage_Control.flag_hc3_on == 2)
	{
		if(flag_HC3_n_number ++ >= 100)
		{
			HC3(HC_OFF);
			Fishhook_Storage_Control.flag_hc3_on = 0;
			flag_HC3_n_number = 0;
		}
	}	
	
	if(Fishhook_Storage_Control.flag_hc4_on == 1)//得到信号的时间
	{
		HC4(HC_ON);
		Fishhook_Storage_Control.flag_hc4_on = 2; //确认信号，并开始计时。
	}else if(Fishhook_Storage_Control.flag_hc4_on == 2)
	{
		if(flag_HC4_n_number ++ >= 100)
		{
			HC4(HC_OFF);
			Fishhook_Storage_Control.flag_hc4_on = 0;
			flag_HC4_n_number = 0;
		}
	}	
	
		if(Fishhook_Storage_Control.flag_hc5_on == 1)//得到信号的时间
	{
		HC5(HC_ON);
		Fishhook_Storage_Control.flag_hc5_on = 2; //确认信号，并开始计时。
	}else if(Fishhook_Storage_Control.flag_hc5_on == 2)
	{
		if(flag_HC5_n_number ++ >= 100)
		{
			HC5(HC_OFF);
			Fishhook_Storage_Control.flag_hc5_on = 0;
			flag_HC5_n_number = 0;
		}
	}	
	
}



void chooseLED(u8 cmd, u8 s)
{
	
	if(flag_WITH_LIGHT|flag_HEAD_TAIL_LIGHT|flag_ALL_LIGHT )
		flag_led_by_car = 1;
	else{
			flag_led_by_car = 0;
	}

	
	if((cmd == HEAD_TAIL_LIGHT)&&(s != 0))
	{
		LEDC3(0);
		LEDC4(0);
		LEDC1(0);
		LEDC2(0);	
		LEDC5(0); // 第一次 第二次不同
		LEDC6(0);
		LEDC7(1);
	}
	else if((cmd == WITH_LIGHT)&&(s != 0))
	{
		LEDC1(1);
		LEDC2(1);	
		LEDC3(0);
		LEDC4(0);
		LEDC5(0); // 第一次 第二次不同
		LEDC6(0);
		LEDC7(0);
		
	}
	else if((cmd == ALL_LIGHT)&&(s != 0))
	{
		LEDC3(1);
		LEDC4(1);
		LEDC1(1);
		LEDC2(1);	
		LEDC5(1); 		// 第一次 第二次不同
		LEDC6(1);
		LEDC7(1);
		LEDC8(1);
		LEDC9(1);
	}
	else
	{
		LEDC3(0);
		LEDC4(0);
		LEDC1(0);
		LEDC2(0);	
		LEDC5(0);		 // 第一次 第二次不同
		LEDC6(0);
		LEDC7(0);
		LEDC8(0);
		LEDC9(0);
	}

}




/*********************************************END OF FILE**********************/
