#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"
#include <stdio.h>


// UART1  log  调试打印信息

 void USART1_Config(void);
 void  USART1_NVIC_Configuration(void);
 int  fputc(int ch, FILE *f);
 int  fgetc(FILE *f);

 void bsp_USART1_IRQHandler(void);
 char *get_rebuff(uint8_t *len);
 void clean_rebuff(void);
 
 
 

void USART1_printf(USART_TypeDef* USARTx, char *Data,...);



// UART2  GPS_USART_INIT  GPS数据


void USART2_Config(void);
void USART2_printf(USART_TypeDef* USARTx, char *Data,...);


// UART3  AS14B_receive_buf 无线接收


void	EXIT_USART3_Init(void);
void Bt_Receive_Init(void);
void USART3_printf(USART_TypeDef* USARTx, uint8_t *Data,...);



// UART4 compass  磁罗盘数据

void GY_26_Receive_Init(void);
void USART4_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
int GY_26_Get_Angle(void);

void Compass_Adjust_0ver(void);
void Post_Board_Data(double how_dimensionality, double how_longitude,u8 type);
void Post_GPS_Data_Fresh(void);
void Post_GPS_Data(double how_dimensionality, double how_longitude,u8 type);



void USART4_Config(void);


// rf functions

	void rfUart_SendCMD(u8* Data,u8 len);
		u8 get_rf_state(void) ;
		u8 wake_up_rf(void);
		u8 write_ch_rf(u8 ch);
		u8 write_address_rf(u8 ad1,u8 ad2);
		void	RF_CHECK(void);
		u8 set_rf_to_sleep(void);
		u8 config_rf_persistent_connection_RX(void);

#endif /* __USART1_H */
