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
    Ship_ALL_Init(); //需要增加led的程序
	
#ifdef WWDOG
    WWDG_Config(0X7F, 0X5F, WWDG_Prescaler_8); 					// 窗口看门狗 初始化
#endif
			
	
    while(1)
    {
        Board_Data_Refresh(); 				 									// 平台无关，将Ship_Board_Data结构体填充好。

        AS14B_Get_Buff_Before(); 												// 无线获取命令及数据 接受数据，平台相关，需要中断不断接受数据和处理。	含有	S14B_Get_buff();

			  Queue_Beep_On();																// 蜂鸣器响应的先后顺序更改
			
        uCMDbeep_A1();																 	// 按键蜂鸣器

        uControl();  																		// 按键设置后的控制指令

        Seek_Route_Check();															// 返回路径查询，时刻查询是否到底1m范围内

        GPS_Num_Get(); 																	// GPS的数据获取，实现准确数据的接收

        Angle =  GY_26_Get_Angle(); 										// 获取compass模块的数据

        Compass_Adjust_0ver(); 													// GPS校准命令的等待停止

        HC_L9110s_Control(); 														// 电磁铁接收命令测试和命令执行

        ADC_Get_Data();																	// 电池电压测试  电流 温度测试

        Control_Led_Toggle();  													// 控制灯 闪烁及时间

        if(++RefreshCnt>200) 														// 计时时间 1s， 长时间执行一次的操作
        {
            RefreshCnt = 0;
            GPS_Check_in_using();												// 开机的20S内，判断GPS是否异常
            Post_GPS_Data_Fresh(); 											// 无线发送数据，每次10s执行一次，发送GPS数据
        }

        if(RefreshCnt/30 == 0)
				{
            Oled_Show(); 																// 显示屏显示，每次 *ms 更新一次
				}
				
        Moto_Chang_Control_Roll();											// 电机转动，呈现无极逐步调速状态

#ifdef WWDOG
        Feed_Dog_When_Hungry();													//喂狗操作
#endif
        /*-------------------------------- new code  here   start --------------------------------*/
				
        /*-------------------------------- new code  here   end   --------------------------------*/
        while(!flag_4ms_set);														//如果时间不足4ms，则再此处阻塞。
				flag_4ms_set = 0;
        Timer_In_Function();
        /*
        				//	sysIndictor(); //系统提示， led闪烁
        				//	Go_Back_By_Trail(); //按照原轨迹返航实现函数：：：：未定义命令
        */

    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////


