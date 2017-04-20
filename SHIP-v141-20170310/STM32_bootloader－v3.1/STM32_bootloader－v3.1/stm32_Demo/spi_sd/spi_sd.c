/******************** (C) COPYRIGHT 2011 ����Ƕ��ʽ���������� ********************
 * �ļ���  ��spi.c
 * ����    ��SD SPI�ӿ�Ӧ�ú�����
 *
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ� ------------------------------------
 *           |                              |
 *           |PA6-SPI1-MISO��SD-SO          |
 *           |PA7-SPI1-MOSI��SD-SI          |
 *           |PA5-SPI1-SCK ��SD-SCK         |
 *           |PA4-SPI1-NSS ��SD-CS          |
 *           |                              |
 *            ------------------------------------
**********************************************************************************/
#include "spi_sd.h"

/*
 * ��������SPI1_Init
 * ����  ��ENC28J60 SPI �ӿڳ�ʼ��
 * ����  ����
 * ���  ����
 * ����  ����
 */
void SPI_SD_Init(void)
{

    SPI_InitTypeDef  SPI_InitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	 //PORTAʱ��ʹ��

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); //PA15Ĭ��jtag
    GPIO_PinRemapConfig(GPIO_Remap_SPI1,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PA2.3.4 ���� 	n_3|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_15);//PA2.3.4����



    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

    SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����


}








//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ
//SPI_BaudRatePrescaler_8   8��Ƶ
//SPI_BaudRatePrescaler_16  16��Ƶ
//SPI_BaudRatePrescaler_256 256��Ƶ

void SPI1_SetSpeed(u8 SpeedSet)
{
    SPI2->CR1&=0XFFC7;
    SPI2->CR1|=SpeedSet;
    SPI_Cmd(SPI1,ENABLE);
}
/*
 * ��������SPI1_ReadWrite
 * ����  ��SPI1��дһ�ֽ�����
 * ����  ��
 * ���  ��
 * ����  ��
 */
unsigned char	SPI1_ReadWrite(u8 writedat)
{


    u8 retry=0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
    {
        retry++;
        if(retry>200)return 0;
    }
    SPI_I2S_SendData(SPI1, writedat); //ͨ������SPIx����һ������
    retry=0;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
    {
        retry++;
        if(retry>200)return 0;
    }
    return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����

}

