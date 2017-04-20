
#include "bsp_adc.h"
#include "bsp_usart.h"

#define ADC1_DR_Address    ((u32)0x40012400+0x4c)

__IO uint16_t ADC_ConvertedValue[4];



extern u8 flag_adc_mask ;


#define ADC_SAMPLE_NUM  200      //һ��������ٵ�
static __IO u32 COUNT = ADC_SAMPLE_NUM;
__IO uint32_t ADC_SampleValue[4] = {0,0,0,0};
// ���ڱ���ת�������ĵ�ѹֵ
float ADC_ConvertedValueLocal[4] = {2.5,0,0,0 };
//__IO u16 ADC_ConvertedValueLocal;

/**
  * @brief  ʹ��ADC1��DMA1��ʱ�ӣ���ʼ��PC.0
  * @param  ��
  * @retval ��
  */
static void ADC1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable DMA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOB,  ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC1,����ʱ������������

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC2,����ʱ������������

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC3,����ʱ������������

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);				// PB0,����ʱ������������


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB,GPIO_Pin_1); // ʹ�ܶˣ��͵�ƽ �Եز�����ѹ

}

/**
  * @brief  ����ADC1�Ĺ���ģʽΪMDAģʽ
  * @param  ��
  * @retval ��
  */
static void ADC1_Mode_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    /* DMA channel1 configuration */
    DMA_DeInit(DMA1_Channel1);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 			//ADC��ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//�ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�����ַ�̶�
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//�ڴ��ַ�̶�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//����
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;										//ѭ������
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable DMA channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    /* ADC1 configuration */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//����ADCģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 				//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 4;	 								//Ҫת����ͨ����Ŀ1
    ADC_Init(ADC1, &ADC_InitStructure);

    /*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    /*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8,  4, ADC_SampleTime_55Cycles5);

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /*��λУ׼�Ĵ��� */
    ADC_ResetCalibration(ADC1);
    /*�ȴ�У׼�Ĵ�����λ��� */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* ADCУ׼ */
    ADC_StartCalibration(ADC1);
    /* �ȴ�У׼���*/
    while(ADC_GetCalibrationStatus(ADC1));

    /* ����û�в����ⲿ����������ʹ���������ADCת�� */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * @brief  ADC1��ʼ��
  * @param  ��
  * @retval ��
  */
void ADC1_Init(void)
{
    ADC1_GPIO_Config();
    ADC1_Mode_Config();
}



void ADC1_GPIO_STOP_MODE(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOB,  ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,����ʱ������������

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);				// PC0,����ʱ������������


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_1);

}


void 	ADC_Get_Data(void)
{



    if(flag_adc_mask > 0)
    {
        if(flag_adc_mask++ == 20)
            flag_adc_mask = 0;

    } else
    {

        if (COUNT != 0)
        {
            COUNT--;
            ADC_SampleValue[0] += ADC_ConvertedValue[0];
            ADC_SampleValue[1] += ADC_ConvertedValue[1];
            ADC_SampleValue[2] += ADC_ConvertedValue[2];
            ADC_SampleValue[3] += ADC_ConvertedValue[3];

        }
        else
        {
            COUNT = ADC_SAMPLE_NUM;
            ADC_ConvertedValueLocal[0] =((float) ADC_SampleValue[0])/ADC_SAMPLE_NUM/4096*3.3; // ��ȡת����ADֵ
            ADC_ConvertedValueLocal[1] =((float) ADC_SampleValue[1])/ADC_SAMPLE_NUM/4096*3.3; // ��ȡת����ADֵ
            ADC_ConvertedValueLocal[2] =((float) ADC_SampleValue[2])/ADC_SAMPLE_NUM/4096*3.3; // ��ȡת����ADֵ
            ADC_ConvertedValueLocal[3] =((float) ADC_SampleValue[3])/ADC_SAMPLE_NUM/4096*3.3; // ��ȡת����ADֵ

            //			printf("\r\n ADC1_10 = %f V\r\n ",ADC_ConvertedValueLocal[0]);
            //			printf("\r\n ADC1_11 = %f V \r\n",ADC_ConvertedValueLocal[1]);
            //			printf("\r\n ADC1_12 = %f V \r\n",ADC_ConvertedValueLocal[2]);
            //			printf(" \r\nADC1_13 = %f V \r\n",ADC_ConvertedValueLocal[3]);


            ADC_SampleValue[0] = 0;
            ADC_SampleValue[1] = 0;
            ADC_SampleValue[2] = 0;
            ADC_SampleValue[3] = 0;
        }
    }

}


/*********************************************END OF FILE**********************/
