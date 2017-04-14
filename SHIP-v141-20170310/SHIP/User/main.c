/**
  ******************************************************************************
  * @file    main.c
  * @author  lijinnan
  * @version V1.0
  * @date    2016-12-25
  * @brief    ship
  ******************************************************************************
	*/
#include "head.h"

#define WWDOG

/* Public macro --------------------------------------------------------------*/

int main(void)
{
//   int temp_number = 0;
    Ship_ALL_Init(); //��Ҫ����led�ĳ���
#ifdef WWDOG
    WWDG_Config(0X7F, 0X5F, WWDG_Prescaler_8); // 10ms
#endif
			
    while(1)
    {
        Board_Data_Refresh();
        // ƽ̨�޹أ���Ship_Board_Data�ṹ�����á�

        AS14B_Get_Buff_Before(); //���߻�ȡ�������  S14B_Get_buff();

        //�������ݣ�ƽ̨��أ���Ҫ�жϲ��Ͻ������ݺʹ���
			  Queue_Beep_On();
			
        uCMDbeep_A1(); //����������

        uControl();  //�������ú�Ŀ���ָ��

        Seek_Route_Check(); //����·����ѯ��ʱ�̲�ѯ�Ƿ񵽵�1m��Χ��

        GPS_Num_Get(); //GPS�����ݻ�ȡ��ʵ��׼ȷ���ݵĽ���

        Angle =  GY_26_Get_Angle(); //��ȡcompassģ�������

        Compass_Adjust_0ver(); // GPSУ׼����ĵȴ�ֹͣ

        HC_L9110s_Control(); 	// ���������������Ժ�����ִ��

        ADC_Get_Data();		// ��ص�ѹ����  ���� �¶Ȳ���

        Control_Led_Toggle();  //���Ƶ� ��˸��ʱ��

        if(++RefreshCnt>200) //��ʱʱ�� 1s�� ��ʱ��ִ��һ�εĲ���
        {
            RefreshCnt = 0;
            GPS_Check_in_using();
            Post_GPS_Data_Fresh(); //���߷������ݣ�ÿ��10sִ��һ�Σ�����GPS����
        }

        if(RefreshCnt/30 == 0)
				{
            Oled_Show(); // ��ʾ����ʾ��ÿ�� *ms ����һ��
				}
        Moto_Chang_Control_Roll();

#ifdef WWDOG
        Feed_Dog_When_Hungry();
#endif
        /*-------------------------------- new code  here   start --------------------------------*/
				
        /*-------------------------------- new code  here   end   --------------------------------*/
        while(!flag_4ms_set);//���ʱ�䲻��4ms�����ٴ˴�������
				flag_4ms_set = 0;
        Timer_In_Function();
        /*
        				//	sysIndictor(); //ϵͳ��ʾ�� led��˸
        				//	Go_Back_By_Trail(); //����ԭ�켣����ʵ�ֺ�����������δ��������
        */

    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////


