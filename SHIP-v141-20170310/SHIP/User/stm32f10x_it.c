/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include <stdio.h>
#include "bsp_SysTick.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "bsp_led.h"
#include "nmea/nmea.h"
#include "gps_config.h"
#include "bsp_pwm_output.h"
#include "nmea/nmea.h"
#include "manager.h"
#include "bsp_wwdg.h"


__IO uint16_t IC2Value1 = 0;
__IO uint16_t IC2Value2 = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;


uint8_t rfrxFlag = 0;


extern double new_longitude, new_dimensionality, old_longitude, old_dimensionality;
extern nmeaINFO info;
extern volatile	u8 flag_enter_in_mode_wwdg;
extern u8 debug_num;
extern void TimingDelay_Decrement(void);
extern u8 flag_4ms_set;

extern  u8 flag_lost_compass_times;

char angle_receive_buf[8] = {0x00,0x00,0x00,0x00,0x00,0x00};
char AS14B_receive_buf[64];
int flag_time5 = 0;
int flag_back = 0;


u8  UART_CHECK_CNT,RXNUM,UART3_RX_RESUME_FLAG;
u8 UART3_BUF[120]; 	// 接收中断数据的临时buf
u8 UART3_CODE[120]; // 存搬移数据的buf



// IO 线中断，中断口为PB10   串口
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line11) != RESET) //确保是否产生了EXTI Line中断
    {
        //	NVIC_SystemReset();
        Ship_stop_enter_normal_Init();
    }
}


void GPS_DMA_IRQHANDLER(void)
{
    GPS_ProcessDMAIRQ();
}


void TIM3_IRQHandler(void)
{


    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

    IC2Value2 = TIM_GetCapture2(TIM3);
    IC2Value1 = TIM_GetCapture1(TIM3);

    if (IC2Value2 != 0)
    {
        DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value2;
        Frequency = 72000000 / IC2Value2;
    }
    else
    {
        DutyCycle = 0;
        Frequency = 0;

    }

}


void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

void WWDG_IRQHandler(void)
{
    WWDG_ClearFlag();

    if(flag_enter_in_mode_wwdg == 1)
    {
        Feed_Dog_When_Hungry();
    }
    else
			;
//        printf("r=%d",debug_num);
}

 
 #ifdef STM32F103VCT6_MCU
void TIM5_IRQHandler(void)
{
    if ( TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET )
    {
        flag_4ms_set++;
        TIM_ClearITPendingBit(TIM5, TIM_FLAG_Update);
    }
}
#endif

//void TIM4_IRQHandler(void)
//{
//    if ( TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET )
//    {
//        flag_4ms_set++;
//        TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
//    }
//}



void TIM1_UP_IRQHandler(void)
{	
	if ( TIM_GetITStatus(TIM1 , TIM_IT_Update) != RESET ) 
	{	
		flag_4ms_set++;
		TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);  
	}
}

// 地址       类型  命令  长度  同步  电量  gps   结束      结束后缀
//uint8_t respond[12] ={0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x1f, 0x05,0x0d,0x0a,0xff,0xff};

void USART3_IRQHandler(void)
{
    uint8_t Res;




    UART_CHECK_CNT = 0;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {

        Res = USART_ReceiveData(USART3);

        UART3_BUF[++RXNUM] = Res;
        UART3_BUF[0] = RXNUM;

        if(RXNUM > 119)        //需要确保在绝大部分情况下不会到达此处！！！！！！！
        {
            if(!UART3_CODE[0])
            {
                for(Res=1; Res<(UART3_BUF[0]+1); Res++)
                    UART3_CODE[Res] = UART3_BUF[Res];
                UART3_CODE[0] = UART3_BUF[0];
                RXNUM = 0;
            }
            else
            {
                UART3_RX_RESUME_FLAG = FALSE;   //意味着可能有数据丢失！！！
            }
        }

    }
    else
    {
        if(!UART3_CODE[0])
        {
            UART3_RX_RESUME_FLAG = TRUE;
            for(Res=0; Res<(UART3_BUF[0]+1); Res++)
                UART3_CODE[Res] = UART3_BUF[Res];
            RXNUM = 0;
        }
    }
}

/*
void UART4_IRQHandler(void)
{
    uint8_t ch;
    static int i = 0;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(UART4);

        if((ch != 0x0a) )
            angle_receive_buf[i++] = ch;
        else
            i = 0;
        flag_lost_compass_times = 0;
    }
}
*/


void USART1_IRQHandler(void)
{
    uint8_t ch;
    static int i = 0;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART1);

        if((ch != 0x0a) )
            angle_receive_buf[i++] = ch;
        else
            i = 0;
        flag_lost_compass_times = 0;
    }
}
/********************************************************* end *************************************************************************/
/********************************************************* end *************************************************************************/



/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    printf("\r\n hardfault err\r\n");
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
