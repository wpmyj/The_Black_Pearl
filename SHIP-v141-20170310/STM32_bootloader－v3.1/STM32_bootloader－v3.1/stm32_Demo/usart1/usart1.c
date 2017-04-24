#include "usart1.h"
/*******************************************************************************
	��������USART1_Configuration
	��  ��:
	��  ��:
	����˵����
	��ʼ������Ӳ���豸�������ж�
	���ò��裺
	(1)��GPIO��USART1��ʱ��
	(2)����USART1�����ܽ�GPIOģʽ
	(3)����USART1���ݸ�ʽ�������ʵȲ���
	(4)ʹ��USART1�����жϹ���
	(5)���ʹ��USART1����
*/




#ifdef USART1_PRINTF


void USART_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* ��1������GPIO��USART������ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
    	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
    	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*  ��3���Ѿ����ˣ�����ⲽ���Բ���
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    */
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* ��4��������USART1����
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* ���������ݼĴ�������������ж� */
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* ��5����ʹ�� USART1�� ������� */
    USART_Cmd(USART1, ENABLE);

    /* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
//    USART_ClearFlag(USART1, USART_FLAG_TC);     // ���־
}

/*******************************************************************/
/*                                                                 */
/* STM32�򴮿�1����1�ֽ�                                           */
/*                                                                 */
/*                                                                 */
/*******************************************************************/
void Uart1_PutChar(u8 ch)
{
    USART_SendData(USART1, (u8) ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/*******************************************************************/
/*                                                                 */
/* STM32�ڴ���1����1�ֽ�                                           */
/* ˵��������1�����ж�                                             */
/*                                                                 */
/*******************************************************************/
void USART1_IRQHandler(void)            //���жϷ�������У�����������Ӧ�ж�ʱ����֪�����ĸ��ж�Դ�����ж�������˱������жϷ�������ж��ж�Դ�����б�Ȼ��ֱ���д�����Ȼ�����ֻ�漰��һ���ж������ǲ����������б�ġ���������ʲô������������б��Ǹ���ϰ��
{
    u8 dat;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //���������ݼĴ�����
    {
        dat = USART_ReceiveData(USART1);

        if(dat == 0x63)
        {
            dat = 0;

            Uart1_PutChar(0x77);
            Uart1_PutChar(0x97);
        }
    }
}



//---------------------�������-----------------------------------/
int fputc(int ch, FILE *f)
{
    /* ��Printf���ݷ������� */
    USART_SendData(USART1, (unsigned char) ch);
    while (!(USART1->SR & USART_FLAG_TXE));

    return (ch);
}
//----------------------���ճ���-------------------------------//
int GetKey(void)
{
    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int) (USART1->DR & 0x1FF));
}

void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

#else



void USART_Configuration(void)
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

    //	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

/*******************************************************************/
/*                                                                 */
/* STM32�򴮿�1����1�ֽ�                                           */
/*                                                                 */
/*                                                                 */
/*******************************************************************/
void Uart4_PutChar(u8 ch)
{
    USART_SendData(UART4, (u8) ch);
    while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}

//---------------------�������-----------------------------------/
int fputc(int ch, FILE *f)
{
    /* ��Printf���ݷ������� */
    USART_SendData(UART4, (unsigned char) ch);
    while (!(UART4->SR & USART_FLAG_TXE));

    return (ch);
}
//----------------------���ճ���-------------------------------//
int GetKey(void)
{
    while (!(UART4->SR & USART_FLAG_RXNE));

    return ((int) (UART4->DR & 0x1FF));
}

void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}


#endif
