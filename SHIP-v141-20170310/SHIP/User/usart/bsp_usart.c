
#include "bsp_usart.h"
#include <stdarg.h>
#include "moto.h"
#include "bsp_SysTick.h"
#include "manager.h"
#include "oled.h"
#include "bsp_led.h"
#include "L9110S.h"
#include "stm32f10x_it.h"


extern uint8_t rfrxFlag;
extern char angle_receive_buf[8];
extern char AS14B_receive_buf[128];


extern int flag_compass_adjust;

extern volatile 	u8 flag_enter_in_mode_wwdg;
extern Ship_Board_Data_t Ship_Board_Data;


extern union_GPS_data GPS_dimensionality,GPS_longituder;
extern uint8_t respond[10];

extern volatile int flag_heart_beart ;
extern  u8 flag_rf_stand_by_state;



u8 rf_powerdown = 0;
u16 recycleNum;


 u8 flag_lost_compass_times =0;

extern u8  UART_CHECK_CNT,RXNUM,UART3_RX_RESUME_FLAG;
extern u8 UART3_BUF[120]; 	// �����ж����ݵ���ʱbuf
extern u8 UART3_CODE[120]; // ��������ݵ�buf 
extern u8 flag_4ms_set;

/**
 * @brief  USART1 GPIO ����,����ģʽ���á�115200 8-N-1
 * @param  ��
 * @retval ��
 */

void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* config USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 mode config */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

void  USART1_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



///�ض���c�⺯��printf��USART1
int fputc(int ch, FILE *f)
{
    /* ����һ���ֽ����ݵ�USART1 */
    USART_SendData(USART1, (uint8_t) ch);

    /* �ȴ�������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

    return (ch);
}

///�ض���c�⺯��scanf��USART1
int fgetc(FILE *f)
{
    /* �ȴ�����1�������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

    return (int)USART_ReceiveData(USART1);
}





 /*          
 *          | PA2  - USART2(Tx)   |
 *          | PA3  - USART2(Rx)   |
 */

/*
 * ��������USART2_Config
 * ����  ��USART2 GPIO ����,����ģʽ����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void Ublox_Send_Date(u8* dbuf,u16 len)
{
	u16 j;
	for(j=0;j<len;j++)//ѭ����������
	{ 
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
		// USART_FLAG_TXE
		
		USART_SendData(USART2,dbuf[j]); 
	}	
}

unsigned char rate_set[14] = 						{0Xb5,0X62,0X06,0X08,0X06,0X00,0Xc8,0X00,0X01,0X00,0X01,0X00,0Xde,0X6a}; //14
unsigned char timepulse_set[28] = 			{0Xb5,0X62,0X06,0X07,0X14,0X00,0X40,0X0d,0X03,0X00,0Xa0,0X86,0X01,0X00,0X01,0X00,0X00,0X00,0X34,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0Xd0,0Xbf};//28
unsigned char only_gps_data[20] =				{0XB5,0X62,0X06,0X3E,0X0C,0X00,0X00,0X00,0X00,0X01,0X00,0X00,0X00,0X00,0X01,0X00,0X00,0X01,0X53,0XB8};//20
unsigned char save_to_eeprom[21] =   		{0Xb5,0X62,0X06,0X09,0X0d,0X00,0X00,0X00,0X00,0X00,0Xff,0Xff,0X00,0X00,0X00,0X00,0X00,0X00,0X04,0X1e,0Xac};//21
unsigned char baudrate_115200_set[28] = {0Xb5,0X62,0X06,0X00,0X14,0X00,0X01,0X00,0X00,0X00,0Xd0,0X08,0X00,0X00,0X00,0Xc2,0X01,0X00,0X07,0X00,0X07,0X00,0X00,0X00,0X00,0X00,0Xc4,0X96};

	
void USART2_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART2 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* USART2 GPIO config */
  /* Configure USART2 Tx (PA.02) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	    
  /* Configure USART2 Rx (PA.03) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	
		/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 9600;                //GPSģ��Ĭ��ʹ�ò����ʣ�9600
	//�����������µ�ģ�� ��38400
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure); 
	USART_Cmd(USART2, ENABLE);
	
	
/*
����֮������߼���
1 stm 32 uart 9600

2 rate������Ϊ200ms 5hz // �����������ٵ�

3 timepulse��

4 ���ݷ���ֻ����gps��

5 �������ݵ�eeprom��

6 ��������Ϊ115200��

7 stm 32 uart 115200

8 �������ݵ�eeprom��
*/

Ublox_Send_Date(rate_set, sizeof(rate_set)/sizeof(unsigned char));
Delay_ms(50);				//�ȴ�������� 
Ublox_Send_Date(timepulse_set, sizeof(timepulse_set)/sizeof(unsigned char));
Delay_ms(50);				//�ȴ�������� 
Ublox_Send_Date(only_gps_data, sizeof(only_gps_data)/sizeof(unsigned char));
Delay_ms(50);				//�ȴ�������� 
Ublox_Send_Date(save_to_eeprom, sizeof(save_to_eeprom)/sizeof(unsigned char));
Delay_ms(50);				//�ȴ�������� 
Ublox_Send_Date(baudrate_115200_set, sizeof(baudrate_115200_set)/sizeof(unsigned char));
Delay_ms(50);				//�ȴ�������� 


	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 115200;                //GPSģ��Ĭ��ʹ�ò����ʣ�9600
	//�����������µ�ģ�� ��38400
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure); 

	USART_Cmd(USART2, ENABLE);
	
	Ublox_Send_Date(save_to_eeprom, sizeof(save_to_eeprom)/sizeof(unsigned char));
	Delay_ms(50);				//�ȴ�������� 

	
}



/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART2
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
//int fputc(int ch, FILE *f)
//{
///* ��Printf���ݷ������� */
//  USART_SendData(USART2, (unsigned char) ch);
//  while (!(USART2->SR & USART_FLAG_TXE));
// 
//  return (ch);
//}

/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART2_printf()����
 */
static char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */


#if 1
//�жϻ��洮������
#define UART_BUFF_SIZE      255
volatile    uint8_t uart_p = 0;
uint8_t     uart_buff[UART_BUFF_SIZE];

void bsp_USART2_IRQHandler(void)
{
    if(uart_p<UART_BUFF_SIZE)
    {
        if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
        {
            uart_buff[uart_p] = USART_ReceiveData(USART2);
            uart_p++;
        }
    }
}



//��ȡ���յ������ݺͳ���
char *get_rebuff(uint8_t *len)
{
    *len = uart_p;
    return (char *)&uart_buff;
}

void clean_rebuff(void)
{
    uart_p = 0;
}

#endif



void USART1_printf(USART_TypeDef* USARTx, char *Data,...)
{
	const char *s;
  int d;   
	double f;
  char buf[17];

//  buf_point_after[8];
//	u8 i = 0;

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
          }
					Data++;
          break;

        case 'd':										//ʮ����
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
          }
					Data++;
          break;
					case 'f':										//�����ͽ���
          f = va_arg(ap, double);
				//	printf("\r\n---itoa = %lf\r\n",f);
					
					sprintf(buf,"%f",f);
					
				//	printf("\r\n---itoa = %s\r\n",buf);
					
					
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
	}
}
/*
 * ��������USART2_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���2����USART2
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART2_printf( USART2, "\r\n this is a demo \r\n" );
 *            		 USART2_printf( USART2, "\r\n %d \r\n", i );
 *            		 USART2_printf( USART2, "\r\n %s \r\n", j );
 */
void USART2_printf(USART_TypeDef* USARTx, char *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
          }
					Data++;
          break;

        case 'd':										//ʮ����
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
	}
}




/**
  ******************************************************************************
    bsp_usart3

  ******************************************************************************
  */ 


void	EXIT_USART3_Init(void)
{

		NVIC_InitTypeDef NVIC_InitStructure;
  	EXTI_InitTypeDef EXTI_InitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/* EXTI line mode config */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
	

  
  /* �����ж�Դ EXTI15_10_IRQn */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


}
	
void USART3_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		/* config USART1 clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		
		/* USART1 GPIO config */
		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
			
		/* USART1 mode config */
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART3, &USART_InitStructure); 
		
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART3, ENABLE);
}


void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void Bt_Receive_Init(void)
{
	USART3_Config();
	NVIC_Configuration();

}

/*
 * ��������USART1_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���1����USART1
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART3_printf( USART3, "\r\n this is a demo \r\n" );
 *            		 USART3_printf( USART3, "\r\n %d \r\n", i );
 *            		 USART3_printf( USART3, "\r\n %s \r\n", j );
 */
void USART3_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     // ????????????
	{				                          
		if ( *Data == 0x5c )  //'\'
	{									  
	switch ( *++Data )
	{
		case 'r':							          //???
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          //???
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  //
	switch ( *++Data )
	{				
		case 's':										  //???
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										//???
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} /* end of else if */
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}


 /**
  * @brief  USART4 115200 8-N-1

  */




	
void USART4_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		/* config USART1 clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		
		/* USART1 GPIO config */
		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
			
		/* USART1 mode config */
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(UART4, &USART_InitStructure); 
		
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
		USART_Cmd(UART4, ENABLE);
}


static void  USART4_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void GY_26_Receive_Init(void)
{
	  USART1_Config(); //ʹ��uart1�����洮��4������compass����
		USART1_NVIC_Configuration();
	
	//USART4_Config();
	//USART4_NVIC_Configuration();

}

/*
 * ��������USART1_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���1����USART1
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART3_printf( USART3, "\r\n this is a demo \r\n" );
 *            		 USART3_printf( USART3, "\r\n %d \r\n", i );
 *            		 USART3_printf( USART3, "\r\n %s \r\n", j );
 */
void USART4_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     // ????????????
	{				                          
		if ( *Data == 0x5c )  //'\'
	{									  
	switch ( *++Data )
	{
		case 'r':							          //???
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          //???
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  //
	switch ( *++Data )
	{				
		case 's':										  //???
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										//???
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} /* end of else if */
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}


int GY_26_Get_Angle(void)
{
	
	int Angle = 0;

	
	
	//printf("angle_receive_buf %02x %02x %02x \r\n",angle_receive_buf[0],angle_receive_buf[1],angle_receive_buf[2] );
	if(angle_receive_buf[0] >= 0x30)
					Angle = (angle_receive_buf[0]-0x30) * 100 + (angle_receive_buf[1] - 0x30) * 10 + (angle_receive_buf[2] - 0x30); 
	

			USART_SendData(USART1,0x31);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);


		if(flag_lost_compass_times++ > 250)
		{
			flag_lost_compass_times = 250;
	//		printf("flag_lost_compass_times = %d\r\n",flag_lost_compass_times);
			angle_receive_buf[0] =angle_receive_buf[1] = angle_receive_buf[2] = 0;
		}
	

//	printf("angle_receive_buf %02x %02x %02x \r\n",angle_receive_buf[0],angle_receive_buf[1],angle_receive_buf[2] );
//		USART3_printf(USART3, "0x31");
	return Angle;
}


	void Compass_Adjust_0ver(void)
	{
			if(flag_compass_adjust-- == 0)
			{
				flag_enter_in_mode_wwdg = 1;
				USART_SendData(USART1,0XC1);  //ֹͣУ׼	
					while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				Delay_ms(20);
				USART_SendData(USART1,0XC1);  //ֹͣУ׼	
					while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				Delay_ms(20);
				USART_SendData(USART1,0XC1);  //ֹͣУ׼	
					while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				Delay_ms(20);
				Moto_Direction_Turn( MOTO_STOP, 0, 0);	
				flag_enter_in_mode_wwdg = 0;
			}
			//if(flag_compass_adjust%10 == 0)
			// printf("flag_compass_adjust = %d\r\n",flag_compass_adjust);
			if(flag_compass_adjust <= -10)
				flag_compass_adjust = -10;
	}

	
	
		void Post_Board_Data(double how_dimensionality, double how_longitude,u8 type)
	{

					u8 i = 0;
		
								USART_SendData(USART3, 0x75);
						/* �ȴ�������� */
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
		
						USART_SendData(USART3, Ship_Board_Data.ship_addr[0]); // addr
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, Ship_Board_Data.ship_addr[1]); // addr
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
					
					
						USART_SendData(USART3, 0x04); // addr
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
					
						for(i = 0; i < 8; i ++)
						{
								USART_SendData(USART3, 0);
								while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						}
							for(i = 0; i < 8; i ++)
						{
								USART_SendData(USART3, 0);
								while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						}
	
						USART_SendData(USART3,0x07);// type 07 ��Ӧ����
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, 0);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, 0);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						
						
						USART_SendData(USART3, (uint16_t)(0));  // �Ƕȸ�λ
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, (uint16_t)(0));  // �Ƕȵ�λ
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, (uint16_t)(0));  // �¶�
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
						USART_SendData(USART3, (uint16_t)(0));  // ��ѹ
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
						USART_SendData(USART3, 0);  // �ϲ�״̬
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);						
						USART_SendData(USART3, (uint16_t)(0));  // ����
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
							
							USART_SendData(USART3, (uint16_t)(0));  // ����״̬
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
							
						
						USART_SendData(USART3, 0x0d);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, 0x0a);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, 0xff);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
				
	}
	
	
		void Post_GPS_Data(double how_dimensionality, double how_longitude,u8 type)
	{
			u8 i = 0;
			static u8 how_time_gps_post = 0;
		
		
		
			if(how_time_gps_post ++ == 0)
				{
					how_time_gps_post = 0;
						GPS_dimensionality.d =  how_dimensionality ;
						GPS_longituder.d  = how_longitude ;
						
						USART_SendData(USART3, 0x75);
						/* �ȴ�������� */
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, Ship_Board_Data.ship_addr[0]); // addr
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, Ship_Board_Data.ship_addr[1]); // addr
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
					
					
						USART_SendData(USART3, 0x04); // addr
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
					
						for(i = 0; i < 8; i ++)
						{
								USART_SendData(USART3, GPS_dimensionality.data[i]);
								while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						}
							for(i = 0; i < 8; i ++)
						{
								USART_SendData(USART3, GPS_longituder.data[i]);
								while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						}
	
						USART_SendData(USART3,type);// type  fresh 05
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						
						
						if(type == 5)
						{
							USART_SendData(USART3, Ship_Board_Data.ship_gps_use_star); //��gps��ʱ���ͣ�����������Ŀ
							
							//printf("type == 5\r\n");
						}
						else
						{
									//	printf("type != 5 [%02x]\r\n",AS14B_receive_buf[5]);
										USART_SendData(USART3, AS14B_receive_buf[5]);// ����ͬ����
						}
			
						
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, Ship_Board_Data.ship_gps_seek_star);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						
						USART_SendData(USART3, (uint16_t)((u32)Ship_Board_Data.ship_angle / 256));  // �Ƕȸ�λ
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, (uint16_t)((u32)Ship_Board_Data.ship_angle % 256));  // �Ƕȵ�λ
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						

						USART_SendData(USART3, (uint16_t)(Ship_Board_Data.ship_temperature));  // �¶�
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						
				//		USART_SendData(USART3, (uint16_t)((Ship_Board_Data.ship_moto_current_1 + Ship_Board_Data.ship_moto_current_2)));
					
						USART_SendData(USART3, (uint16_t)(Ship_Board_Data.ship_voltage));  // ��ѹ
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
						
						
						USART_SendData(USART3, Ship_Board_Data.ship_storage);  // �ϲ�״̬
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
												
						USART_SendData(USART3, (uint16_t)(0));  // ����
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
						
						USART_SendData(USART3, (uint16_t)(Ship_Board_Data.ship_status_now));  // ����״̬
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);		
						
						
		//				printf("ship_status=[%02x]\r\n",Ship_Board_Data.ship_status_now);
						
						
						USART_SendData(USART3, 0x0d);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
						USART_SendData(USART3, 0x0a);
						while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
			
				}
				
				
	//			printf("Post_GPS_Data ship_temperature = %d\r\n",(uint16_t)(Ship_Board_Data.ship_temperature));
	//			printf("Post_GPS_Data h ship_angle = %02x\r\n",(uint16_t)((u32)Ship_Board_Data.ship_angle / 256));
	//			printf("Post_GPS_Data l ship_angle = %02x\r\n",(uint16_t)((u32)Ship_Board_Data.ship_angle % 256));
	//			printf("Post_GPS_Data ship_voltage = %d\r\n",(uint16_t)(Ship_Board_Data.ship_voltage));
	}

	
	
//  RF functions       start ----------------------------------------------------------------------------------------------------------------- //
	
	
	void rfUart_SendCMD(u8* Data,u8 len)
{
  u8 i=0;
  for(;i<len;i++){	
		USART_SendData(USART3, Data[i]); 
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
}
	
	
	//===========================================================================================//
//===========================================================================================//
u8 get_rf_state(void) //max 170ms
{
  u32 s;
  u8 j,state = 0;
  u8 buf[26] = {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x06,0x01,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0xff};

 
  for(j=0;j<200;j++)
  {
      rfUart_SendCMD(buf,26);    
      for(s=0;s<60000;s++)
      {
          if(UART3_CODE[0])
          {
              state = AS14B_Get_Buff_Before();
              if(state)
              {
                 if(state == 0x06)
                 {
                    rf_powerdown = TRUE;
                    state = 1;
                 }
                 else
                 {
                    rf_powerdown = FALSE;
                    state = 2;
                 }
                 recycleNum = j;
                 return state;
              }
          }
      }
  }
  return 0;
}


//===========================================================================================//
//===========================================================================================//
u8 wake_up_rf(void)
{
  u32 s;
  u8 j;
  u8 buf[26] = {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x04,0x01,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0xff};
  
  for(j=0;j<50;j++)      //max 160ms
  {

      rfUart_SendCMD(buf,26);
      rfUart_SendCMD(buf,26);
      //rfUart_SendCMD(buf,26);
      rfUart_SendCMD(buf,26);//9MS
      
      for(s=0;s<16000;s++)
      {
          if(UART3_CODE[0])
          {
              if(AS14B_Get_Buff_Before() != 0x06)
              {
                  rf_powerdown = FALSE;
                  recycleNum = j;
                  return 1;
              }
          }
      }
  }
  return 0;
}



//===========================================================================================//
//===========================================================================================//
u8 write_ch_rf(u8 ch)
{
   u32 s;
   u8 j;
   u8 buf[26] = {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0xfb,0x05,0x05,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40};
   
   if(get_rf_state() == 1)
   {
      while((wake_up_rf()!=1));
   }

  buf[14] = ch;
  
  for(j=0;j<255;j++)
  {
      rfUart_SendCMD(buf,26);
      for(s=0;s<16000;s++)
      {
          if(UART3_CODE[0])
          {
              if(AS14B_Get_Buff_Before()==0x07)
              {
                 recycleNum += j;
                 return 1;
              }
          }
      }
  }
  return 0;
}
//===========================================================================================//
//===========================================================================================//
u8 write_address_rf(u8 ad1,u8 ad2)
{
  u32 s;
  u8 j;
  u8 buf[26] = {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x02,0x02,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40};
 
  buf[13] = ad1;
  buf[14] = ad2;

  for(j=0;j<255;j++)
  {
      rfUart_SendCMD(buf,26);
      for(s=0;s<16000;s++)
      {
          if(UART3_CODE[0])
          {
              if(AS14B_Get_Buff_Before()==0x07)
              {
                  recycleNum += j;
                  return 1;
              }
          }
      }
  }
  return 0;
}

//===================RF_CHECK  1105========================================================================//

void	RF_CHECK(void)
{
	u8 flag_get_rf_state; 
	
	flag_get_rf_state = get_rf_state();
	
	if(flag_get_rf_state == 0) //δ��װ�����쳣
	{
//			printf("RF_CHECK  NO device\r\n");
			while(1)
			{
			//		printf("RF_CHECK  NO device\r\n");
					LEDC_ALL(1);
					Delay_ms(200);
					LEDC_ALL(0);	
					Delay_ms(200);	
					Oled_Show_RF_Error();				
			}
	}
	else if(flag_get_rf_state == 1) // �͹���
	{
		
		wake_up_rf();
//		printf("\r\n wake_up_rf SET rf\r\n");
		write_ch_rf(66);
		Delay_ms(6); 
		config_rf_persistent_connection_RX();
		Delay_ms(6);
		write_address_rf(0x00,0x55);
		//Delay_ms(150);  //wait RF reset!

		
	}else //����״̬
	{
//		printf("\r\nSET rf\r\n");
		write_ch_rf(66);
		Delay_ms(6); 
		config_rf_persistent_connection_RX();
		Delay_ms(6);
		write_address_rf(0x00,0x55);
		//Delay_ms(150);  //wait RF reset!
	}

}



//===========================================================================================//
//===========================================================================================//
u8 set_rf_to_sleep(void)
{
  u32 s;
  u8 j;
  u8 buf[26] = {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x05,0x01,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0xff};
     
   if(get_rf_state() == 1)
   {
      while((wake_up_rf()!=1));
   }  
  for(j=0;j<255;j++)
  {
      rfUart_SendCMD(buf,25);
      for(s=0;s<16000;s++)
      {
          if(UART3_CODE[0])
          {
              if(AS14B_Get_Buff_Before()==0x07)
              {
                recycleNum += j;
                rf_powerdown = TRUE;
                return 1;
              }
          }
      }
  }
  return 0;
}

//===========================================================================================//
//===========================================================================================//
u8 config_rf_persistent_connection_RX(void)
{
  u32 s;
  u8 j;
  u8 buf[26] = {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x03,0x02,0x00,0x01,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40};
 
  for(j=0;j<255;j++)
  {
      rfUart_SendCMD(buf,26);
      
      for(s=0;s<16000;s++)
      {
          if(UART3_CODE[0])
          {
              if(AS14B_Get_Buff_Before()==0x07)
              {
                recycleNum += j;
                return 1;
              }
          }
      }
  }
  return 0;
}


//  RF functions       end -----------------------------------------------------------------------------------------------------------------  //



void Respond_Back(void)
{
/*
	u8 i; 
	u8  flag_change_state = 0;
	static uint8_t last_respond[10];
	//respond[0] [1] [2] ��ַ ��ȷ�� 
	//������Ϣ��Ҫ��䣺

	respond[3] = (uint8_t)(Ship_Board_Data.ship_voltage); //����  ADC_ConvertedValueLocal[0]*1100
	respond[4] = Ship_Board_Data.ship_gps_use_star; //gps��Ϣ 0000 0000 ������Ҫ����   ��⵽���Ǹ���

	respond[5] ++ ; // ͬ����

//	respond[6] ; // ����״̬ 0 ֹͣ ; 1 ң����;   2 �Զ����� 
	
	// respond[7]  respond[8] 0x0d 0x0a ���� 
	

	for(i=0;i<10;i++)
	{
		if(i == 3 )  //��ѹ��ֵ
		{
			if(abs(last_respond[i] - respond[i]) > 5)
			{
				
					flag_change_state = 1;
			}
			continue;
		}
	if(i == 4)  // Ѱ�� ����
	{
		if(respond[i] != last_respond[i])
		{
			if((respond[i] > 0)&&(last_respond[i] > 0))
				;
			else
					flag_change_state = 1;
			continue;
		}
	
	}

		if(i == 5 ) // ͬ����
			continue;
		
		
		if(respond[i] != last_respond[i])
		{
	//		printf(" respond[%d] = %d\r\n", i, respond[i]);
			flag_change_state = 1;
		}
			
	}
	
	if((flag_change_state == 1)&&(flag_heart_beart <= 1)) // ���µ����ݣ���û�жϿ���
	{
		if(flag_rf_stand_by_state == 1)
	{
		wake_up_rf();
		flag_rf_stand_by_state = 0;
	}

		USART_SendData(USART3, 0x75);
		// �ȴ�������� 
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
		rfUart_SendCMD(respond,10);  //1105 replace 	

	//	printf("rfUart_SendCMD \r\n");
	}
	
	for(i=0;i<10;i++)
	{
		last_respond[i] = respond[i];
//		printf("%d--", respond[i]);
	}
	
	//		printf("\r\n");

//	printf("\r\n heart beating and Respond_Back----\r\n");
	
	*/
}
	
/*********************************************END OF FILE**********************/
