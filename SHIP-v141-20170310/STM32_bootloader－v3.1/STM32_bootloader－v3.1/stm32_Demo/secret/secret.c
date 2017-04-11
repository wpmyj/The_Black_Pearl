#include "secret.h"
#include "usart1.h"


// ����sd������������İ�ȫ���⣬��ֹ���ֵ�������
// ͨ��stm32���豸id�����ơ�
// ͨ��д�뵽ĳ��flashλ�ù̶������ݣ�bootloader��д�룬Ȼ����app�н��ж�ȡ������ʧ�ܣ��򲻽�������������д��
// hex���غ���Ҫ���м��ܴ�����ֹ���������ȡ��


static u8 ChipUniqueID[12];
u16 CRCID;


const u16 wCRCTalbeAbs[] =
{
0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,
};
static u16 CRC16_2(u8* pchMsg, u16 wDataLen)
{
        u16 wCRC = 0xFFFF;
        u16 i;
        u8 chChar;
        for (i = 0; i < wDataLen; i++)
        {
                chChar = *pchMsg++;
                wCRC = wCRCTalbeAbs[(chChar ^ wCRC) & 15] ^ (wCRC >> 4);
                wCRC = wCRCTalbeAbs[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
        }
        return wCRC;
}



int Get_ChipID(void)
{
    u32 temp0,temp1,temp2;
	u8 i = 0;
		u8 temp[12];   //���оƬID����ʱ����
temp0 = *(__IO u32*)(0x1FFFF7E8);    //��ƷΨһ��ݱ�ʶ�Ĵ�����96λ��
    temp1 = *(__IO u32*)(0x1FFFF7EC);
temp2 = *(__IO u32*)(0x1FFFF7F0);
	
	// �������ڴ�����С�˴洢��ʽ
                                  
//ID���ַ�� 0x1FFFF7E8   0x1FFFF7EC  0x1FFFF7F0 ��ֻ��Ҫ��ȡ�����ַ�е����ݾͿ����ˡ�

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

		for(i = 0;i < 12;i++) //���ݵĸ���
		{
			printf("0x%02x--", temp[i]);
			//	USART_SendData(USART2, temp[i++]); //���÷��ͺ���
				//           uiData++; //�������ݵĵ�ַ��1���л�����һ��Ҫ�������ݵĵ�ַ��
				while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//���ͻ�������״̬��־λ��ֻ�е�������Ϊ��ʱ���ŷ�����һ�����ݡ�
		}
		
		printf("\r\n");	
		
		CRCID = CRC16_2(temp, 12); 
		printf("CRCID = %d\r\n", CRCID);
		//48878 0116 �𻵽�ˮ����оƬ
		
		// 2017�� 2�� 10��  37898
		
		if((CRCID ==  36777) || (CRCID ==  42927) || (CRCID ==  12207) || (CRCID ==  48878) || (CRCID ==  37898)|| \
			(CRCID ==  0x2acb)|| (CRCID ==  0x2690)|| (CRCID ==  0xd6e5)|| (CRCID ==  0x25af)|| (CRCID ==  0xb6ac) \
		|| (CRCID ==  0xae52) 		|| (CRCID ==  0xb192) //    
		
		)
			return 1;
		else
			while(1);
		
		return 0;
}



/*
stm32F4��

u32 mcuID[3];
mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
printf (" %X %X %X \n",mcuID[0],mcuID[1] ,mcuID[2] );

STM32F1:
SerialID[0] = *(unsigned int*)(0x1FFFF7E8);
    SerialID[1] = *(unsigned int*)(0x1FFFF7EC);
    SerialID[2] = *(unsigned int*)(0x1FFFF7F0);

*/
  

#define  FLASH_START_ADDR 0x801f000

FLASH_FLAG FLASH_STATUS;


static u32 count = 0;
/**---------------------------------------------------------------------------------
  * @brief   FLASHд��ز����ٲ���
  * @param  ����������ʼ��ַ
  * @retval ��������ִ�����
  --------------------------------------------------------------------------------*/

unsigned char FLASH_WRITE(unsigned short int *memory_data, int n)
{	
	//static int t;
	count = 0;
	/*Enables or disables the Internal High Speed oscillator (HSI).*/
	RCC_HSICmd(ENABLE);	
	/*��FLASH�ɲ���������*/
	FLASH_Unlock();
	/*��flash������־λȫ��*/
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(FLASH_START_ADDR);
	//t = 15; // 16 -1
	//sizeof((unsigned short int *)memory_data);
//	 t = sizeof((const char *)memory_data);  // 5
	while(count <= n)
	{
		/*flash Ϊһ���ֽڴ洢��16λ���ݱ����ַ��2*/
		FLASH_ProgramHalfWord((FLASH_START_ADDR +count*2),*(memory_data+count)); 
		count++;
		
	//	printf("\r FLASH_WRITE count %d \r",(count));
		if(count > n)
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
  * @brief   FLASH��������
  * @param  memory_data�������ŵ�ַ��n �������
  * @retval ��������ִ�����
  --------------------------------------------------------------------------------*/
unsigned char FLASH_READ(unsigned short int * memory_data,unsigned short int n)
{	
	count = 0;
	while(count < n)
	{
	  *(memory_data+count) = *(u16 *)(FLASH_START_ADDR + count*2);
	//	printf("\r %d \r",*(memory_data+count));   //��ȡ
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

/*********************************************END OF FILE**********************/







