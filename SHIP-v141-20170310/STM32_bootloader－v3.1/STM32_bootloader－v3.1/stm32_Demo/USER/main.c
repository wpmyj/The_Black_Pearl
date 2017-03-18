/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "delay.h"
#include <stdio.h>
#include "usart1.h"
#include "spi_sd.h"
#include "mmc_sd.h"
#include "ff.h"
#include "stmflash.h"
#include "iap.h"
#include "bsp_led.h"   
#include "secret.h"


int rest,rest1;  
int a;
FIL fsrc,fdst;
FATFS fs;
DIR   dfs;
UINT br, bw;            // File R/W count
BYTE buffer[512];       // file copy buffer
u8   ReadAppBuffer[512];
u16  ChangeBuffer[256];


unsigned short int secret_in[16] = {0xff,0xff,0xff,0x1,0xff,0xff,0xf,0x1,0xf,0xf,0xf,0x1,0x1,0x1,0x1,0x8};
unsigned short int secret_out[16] ;
void Delay(__IO uint32_t nCount)	;



/*************************************************
����: void GPIO_Configuration(void��
����: GPIO����
����: ��
����: ��
**************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;        //����GPIO��ʼ���ṹ��

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

/*
 * ��������Fill_Buffer
 * ����  ���ڻ���������д����
 * ����  ��-pBuffer Ҫ���Ļ�����
 *         -BufferLength Ҫ���Ĵ�С
 *         -Offset ���ڻ������ĵ�һ��ֵ
 * ���  ���� 
 */
void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint16_t index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLength; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}
int main(void)
{
	u32 sd_size;
	u32  i = 0;
	u16  j = 0, k = 0;
	u32  APP_Sector = 0;
	u16  APP_Byte = 0;
	delay_init(72);             //��ʱ��ʼ��
	
		//������
	//*************************???????***********************************************
//  
//if (FLASH_GetReadOutProtectionStatus() == RESET) 
//{
//FLASH_Unlock();
//FLASH_ReadOutProtection(ENABLE); 
//}
// //*************************???????***********************************************
// 
//if (FLASH_GetReadOutProtectionStatus() == SET) 
//{ 

//FLASH_Unlock();
//FLASH_ReadOutProtection(DISABLE); 
//} 
	  
	USART1_Configuration();
	printf("\r\n =============bootloader start\r\n");
	GPIO_Configuration();
	LED_GPIO_Config();
	
	
	// ��ȡid
	Get_ChipID();  		//if ID is unabled, be in while(1);
	
	
	// flash���ݶ�д
	FLASH_WRITE(secret_in, 16);
	FLASH_READ(secret_out, 16);
//	for(i = 0; i < sizeof(secret_out)/sizeof(unsigned short int); i++)
//		printf("\r\n secret_out[%d] = %d\r\n", i, secret_out[i]); 
	
	while(1) 
	{		   
			
			while(SD_Initialize())  //��ⲻ��SD��
				{
					printf("\r\n SD Card Error!");
					printf("\r\n Please Check !"); 
					
					Delay(0X0ffff);
					LED1_TOGGLE;
		
					k++;
					if(k > 10)
						goto exit_fail;
				}
				
					LED1_OFF;
					LED1_ON;
					Delay(0X0fffff);
			
			
				
					sd_size=SD_GetSectorCount(); 		
					printf("\r\n  ��⵽sd�� ");
					printf("\r\n SD����������%d MB\r\n",sd_size>>11);
		if(sd_size != 0)
		{			
f_mount_again:	
					if(f_mount(0,&fs)!= FR_OK) 
					{
							 Delay(0X0ffff);
						goto f_mount_again;
					
					}				

f_open_again:						
					rest = f_open(&fdst,"0:/TEST.bin",FA_OPEN_EXISTING|FA_READ);  //�򿪻��ߴ����ļ�txt		    	
					if ( rest == FR_OK )
					 { 
						/* ��������������д���ļ��� */	 //0:/STM32-DEMO.BIN
							APP_Sector = fdst.fsize / 512;
								APP_Byte = fdst.fsize % 512;
								for(i = 0;i < APP_Sector;i++)
								{
									f_read (&fdst,ReadAppBuffer,512,(UINT *)&bw);
									for(j = 0;j < 256;j++)  
									{
										ChangeBuffer[j] = (ReadAppBuffer[j * 2 + 1] << 8) + ReadAppBuffer[j * 2];	
									}
									LED1_TOGGLE;
									STMFLASH_Write(FLASH_APP_ADDR + i * 512,ChangeBuffer,256);	 
								}
								if(APP_Byte != 0)
								{
									f_read (&fdst,ReadAppBuffer,APP_Byte,(UINT *)&bw);
									for(j = 0;j < (APP_Byte / 2);j++)  
									{
										ChangeBuffer[j] = (ReadAppBuffer[j * 2 + 1] << 8) + ReadAppBuffer[j * 2];	
									}
									STMFLASH_Write(FLASH_APP_ADDR + i * 512,ChangeBuffer,APP_Byte / 2);
								}
						 f_close(&fdst); 
						 printf( "\r\n �ļ������ɹ� \n" );       
						}
					 else{
						 Delay(0X0ffff);
					 
					 goto f_open_again;
					 }
					 
				 }

						printf("\r\n =============bootloader endr\n");
						
	iap_load_app(FLASH_APP_ADDR);
					LED1_ON;
				
						while(1);
			exit_fail:
					LED1_OFF;
		
							printf("\r\n iap_load_app ! update fail \r\n"); 
					iap_load_app(FLASH_APP_ADDR);		
	while(1);
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
