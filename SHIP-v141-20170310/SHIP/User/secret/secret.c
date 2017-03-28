#include "secret.h"
#include "bsp_usart.h"
#include "bsp_SysTick.h"

// 增加sd卡升级会带来的安全问题，防止出现盗版现象。
// 通过stm32的设备id来限制。
// 通过写入到某个flash位置固定的数据，bootloader中写入，然后在app中进行读取，若是失败，则不进行升级。打乱写入
// hex下载后，需要进行加密处理，防止被反编译读取。



unsigned short int secret_in[16] = {0xff,0xff,0xff,0x1,0xff,0xff,0xf,0x1,0xf,0xf,0xf,0x1,0x1,0x1,0x1,0x8};
unsigned short int secret_out[16];


int Get_ChipID(void)
{
    u32 temp0,temp1,temp2;
	u8 i = 0;
		uint16_t temp[12];   //存放芯片ID的临时变量
temp0 = *(__IO u32*)(0x1FFFF7E8);    //产品唯一身份标识寄存器（96位）
    temp1 = *(__IO u32*)(0x1FFFF7EC);
temp2 = *(__IO u32*)(0x1FFFF7F0);
	
	// 由于在内存中是小端存储方式
                                  
//ID码地址： 0x1FFFF7E8   0x1FFFF7EC  0x1FFFF7F0 ，只需要读取这个地址中的数据就可以了。

    temp[0] = (u8)(temp0 & 0x000000FF);
    temp[1] = (u8)((temp0 & 0x0000FF00)>>8);
    temp[2] = (u8)((temp0 & 0x00FF0000)>>16);
    temp[3] = (u8)((temp0 & 0xFF000000)>>24);
    temp[4] = (u8)(temp1 & 0x000000FF);
    temp[5] = (u8)((temp1 & 0x0000FF00)>>8);
    temp[6] = (u8)((temp1 & 0x00FF0000)>>16);
    temp[7] = (u8)((temp1 & 0xFF000000)>>24);
    temp[8] = (u8)(temp2 & 0x000000FF);
    temp[9] = (u8)((temp2 & 0x0000FF00)>>8);
    temp[10] = (u8)((temp2 & 0x00FF0000)>>16);
    temp[11] = (u8)((temp2 & 0xFF000000)>>24);    

		for(i = 0;i < 12;i = i) //数据的个数
		{
				USART_SendData(USART2, temp[i++]); //调用发送函数
				//           uiData++; //发送数据的地址加1，切换到下一个要发送数据的地址。
				while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);//发送缓冲区空状态标志位。只有当缓冲区为空时，才发送下一个数据。
		}
return 0;
}



/*
stm32F4：

u32 mcuID[3];
mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
//printf (" %X %X %X \n",mcuID[0],mcuID[1] ,mcuID[2] );

STM32F1:
SerialID[0] = *(unsigned int*)(0x1FFFF7E8);
    SerialID[1] = *(unsigned int*)(0x1FFFF7EC);
    SerialID[2] = *(unsigned int*)(0x1FFFF7F0);

*/
  

#define  FLASH_START_ADDR 0x801f000

FLASH_FLAG FLASH_STATUS;


static u32 count = 0;
/**---------------------------------------------------------------------------------
  * @brief   FLASH写入必不可少步骤
  * @param  输入数据起始地址
  * @retval 函数有无执行完毕
  --------------------------------------------------------------------------------*/

unsigned char FLASH_WRITE(unsigned short int *memory_data)
{	
	static int t;
	count = 0;
	/*Enables or disables the Internal High Speed oscillator (HSI).*/
	RCC_HSICmd(ENABLE);	
	/*打开FLASH可擦除控制器*/
	FLASH_Unlock();
	/*将flash三个标志位全清*/
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(FLASH_START_ADDR);
	t = sizeof((const char *)memory_data);
	while(count <= t)
	{
		/*flash 为一个字节存储，16位数据必须地址加2*/
		FLASH_ProgramHalfWord((FLASH_START_ADDR +count*2),*(memory_data+count)); 
		count++;
		if(count > t)
		{
			FLASH_STATUS = FLASH_WRIKE_NO;
			return FLASH_STATUS;
		}
	}
	/* Locks the FLASH Program Erase Controller.*/
	FLASH_Lock(); 
	RCC_HSICmd(DISABLE);
	FLASH_STATUS = FLASH_WRITE_OK;
	return FLASH_STATUS;
}

/**---------------------------------------------------------------------------------
  * @brief   FLASH读出函数
  * @param  memory_data：输出存放地址，n 输出个数
  * @retval 函数有无执行完毕
  --------------------------------------------------------------------------------*/
unsigned char FLASH_READ(unsigned short int * memory_data,unsigned short int n)
{	
	count = 0;
	while(count < n)
	{
	  *(memory_data+count) = *(u16 *)(FLASH_START_ADDR + count*2);
//		printf("\r %d \r",*(memory_data+count));   //读取
		count++;
		if(count > n)
		{
			FLASH_STATUS = FLASH_READ_NO;	
			return FLASH_STATUS;
		}
	}
	//FLASH_ErasePage(0x8002000);
	FLASH_STATUS = FLASH_READ_OK;
	return FLASH_STATUS;
}

void this_is_a_secret(void)
{
	int i  = 0;
	
	FLASH_READ(secret_out, 16);
	for(i = 0; i < 16; i++)
	{
	//	printf("\r\n secret_in[%d]  = %d , secret_out[%d]  = %d \r\n",i, secret_in[i],i, secret_out[i]);  
	
		if(secret_in[i] != (secret_out[i]&0xff))
		{
			
					while(1)
					{
						Delay_ms(1000);
//							printf("\r\n joke \r\n");  
					
					
					}
		}

	}


}

 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//STM32 FLASH 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
#define STM32_FLASH_BASE 0x08000000
#define STM32_FLASH_WREN 1
#define STM32_FLASH_SIZE 512
//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}
#if STM32_FLASH_WREN	//如果使能了写   
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//地址增加2.
	}  
} 
//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节


void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //扇区地址
	u16 secoff;	   //扇区内偏移地址(16位字计算)
	u16 secremain; //扇区内剩余地址(16位字计算)	   
 	u16 i;    
	u32 offaddr;   //去掉0X08000000后的地址
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
	FLASH_Unlock();						//解锁
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小   
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区  
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		   	pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain;	//写地址偏移	   
		   	NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};	
	FLASH_Lock();//上锁
}
#endif

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
		
	//	printf("\rpBuffer[%d] %d\r",i, pBuffer[i]); 
	}
}




/*********************************************END OF FILE**********************/







