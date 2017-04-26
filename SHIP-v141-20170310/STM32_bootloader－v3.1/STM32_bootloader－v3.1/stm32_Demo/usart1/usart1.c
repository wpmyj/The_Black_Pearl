#include "usart1.h"
/*******************************************************************************
	函数名：USART1_Configuration
	输  入:
	输  出:
	功能说明：
	初始化串口硬件设备，启用中断
	配置步骤：
	(1)打开GPIO和USART1的时钟
	(2)设置USART1两个管脚GPIO模式
	(3)配置USART1数据格式、波特率等参数
	(4)使能USART1接收中断功能
	(5)最后使能USART1功能
*/




#ifdef USART1_PRINTF


void USART_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 第1步：打开GPIO和USART部件的时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
    	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
    	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*  第3步已经做了，因此这步可以不做
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    */
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* 第4步：配置USART1参数
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* 若接收数据寄存器满，则产生中断 */
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 第5步：使能 USART1， 配置完毕 */
    USART_Cmd(USART1, ENABLE);

    /* 如下语句解决第1个字节无法正确发送出去的问题 */
//    USART_ClearFlag(USART1, USART_FLAG_TC);     // 清标志
}

/*******************************************************************/
/*                                                                 */
/* STM32向串口1发送1字节                                           */
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
/* STM32在串口1接收1字节                                           */
/* 说明：串口1接收中断                                             */
/*                                                                 */
/*******************************************************************/
void USART1_IRQHandler(void)            //在中断服务程序中，由于主机响应中断时并不知道是哪个中断源发出中断请求，因此必须在中断服务程序中对中断源进行判别，然后分别进行处理。当然，如果只涉及到一个中断请求，是不用做上述判别的。但是无论什么情况，做上述判别是个好习惯
{
    u8 dat;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //若接收数据寄存器满
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



//---------------------输出程序-----------------------------------/
int fputc(int ch, FILE *f)
{
    /* 将Printf内容发往串口 */
    USART_SendData(USART1, (unsigned char) ch);
    while (!(USART1->SR & USART_FLAG_TXE));

    return (ch);
}
//----------------------接收程序-------------------------------//
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
/* STM32向串口1发送1字节                                           */
/*                                                                 */
/*                                                                 */
/*******************************************************************/
void Uart4_PutChar(u8 ch)
{
    USART_SendData(UART4, (u8) ch);
    while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}

//---------------------输出程序-----------------------------------/
int fputc(int ch, FILE *f)
{
    /* 将Printf内容发往串口 */
    USART_SendData(UART4, (unsigned char) ch);
    while (!(UART4->SR & USART_FLAG_TXE));

    return (ch);
}
//----------------------接收程序-------------------------------//
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
