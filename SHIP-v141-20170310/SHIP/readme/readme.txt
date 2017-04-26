

/******************************************		new verion		************************************************/

version 142 
(data:2017/04/22 ~ 2017/04/26)

1st	:	调速问题，电机的mos会出问题，发烫严重。
		改变控制方式，结合需修改的原理图，测试通过 （0426）。

2nd	:	遥控器异常现象：关机后，地址丢失，还需要在再次配网。vb地址和vc有差距，最新固件没问题。

3rd	:	配对，有时候无法成功。 
		出现现象1：在慈溪，若无法配对成功，长按退出配对，但是再次进入，则成功。原因需要找到。自行测试，发现10次均成功。
		
		在更新固件后，再次配置，无法配对较为频繁。
		现象为：升级过程中，遥控器正常开机状态，进行升级，升级后立即执行配对极容易失败。且，再次重启船也不能配对成功。长按退出配对，但是再次进入，则基本都成功。
		经过对比测试发现：升级过程中，关闭遥控器，升级后打开遥控器，进入配对，打开船，也没问题。



4th ：	地磁问题,电源干扰，查看。用购买模块测试，正常，排除stm32代码的问题。锁定在stm8的hmc1022模块上。重新烧录程序，正常。出厂前，测试hmc1022的性能，具体做法查看说明书。

5th	: 	sd卡升级问题，出现现象，在stlink的3.3v下，能够完成正常。在船上偶尔不正常，初步判断为电压稳定性。
		另增加提示，若是sd卡未检测到，led2闪烁两次，若是跳转到app，则led1 led2交替闪烁一下。
		如有问题，暂先考虑硬件问题。

6th	:	导致异常，还有原因：一个版本启动程序差异，就是hd.s和md.s,进行更改了。回退版本处理。
	





void ALL_Board_Init(void)
{

    SysTick_Init();

    TIM3_Init();
    TIM4_PWM_Init();

}





    ALL_Board_Init();

// ------------------------------------------------------------------------------ 1->2



    /*

    1 	5s完成一次0上升到100的过程，每次占空比改变10（500ms一次）。5s完成一次100下降到0的过程，每次占空比改变10（500ms一次）。

    频率 10KHZ
    IN1为100，IN2改变。
    测试五分钟，暂时不发烫。基本没温度。


    2 	修改时间为500ms一个周期，50ms改变一个10个占空比。
    	测试五分钟，暂时不发烫。基本没温度。


    3 	IN1为0，IN2改变，暂时不发烫。基本没温度。

    按照理论来讲，100的占空比，也没问题


    4		高低高低频繁正反转。测试五分钟，暂时不发烫。基本没温度。




    5 修改时间为100ms一个周。 期1ms改变一个10个占空比。
    	频繁正反转。测试五分钟，暂时不发烫。基本没温度。


		6 新的板子，进行测试，反复，正反转，50分钟，1s完成一次反转过程，暂无异常。

    */


//	/*		满速度反转				*/
//
//				TIM3->CCR3= TIM3->CCR4 = 100;
//
//				TIM3->CCR1 =  100;
//				TIM4->CCR3 = 100;
//
//						TIM3->CCR2 =  0;
//				TIM4->CCR4  = 0;
//
//
////	while(1);
//
//
//	/*		满速度正转				*/
//				TIM3->CCR3= TIM3->CCR4 = 0;
//
//				TIM3->CCR1 =  0;
//				TIM4->CCR3 = 0;
//
//				TIM3->CCR2 =  100;
//				TIM4->CCR4 = 100;


    while(1)
    {



        if(flag_time2_irq % 100 == 0)
        {

            if((TIM3->CCR3 == 100)&&(TIM3->CCR4 == 100))
            {
                TIM3->CCR2 =  flag_time2_irq / 10;
                TIM4->CCR4  = flag_time2_irq / 10;
            }
            else
            {
                TIM3->CCR1 = flag_time2_irq / 10;
                TIM4->CCR3 = flag_time2_irq / 10;
            }


            if(flag_time2_irq == 1000)
            {
                flag_direction_irq = 0;

        //        flag_chang_dircetion = !flag_chang_dircetion;

                if(flag_chang_dircetion == 1)
                {
                    flag_chang_dircetion = 0;
//				GPIO_ResetBits(GPIOC, GPIO_Pin_8);
//				GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                    TIM3->CCR3= TIM3->CCR4 = 100;

                    TIM3->CCR2 =  100;
                    TIM4->CCR4  = 100;

                    TIM3->CCR1 = 100;
                    TIM4->CCR3 = 100;

                } else {

                    flag_chang_dircetion = 1;
                    TIM3->CCR3= TIM3->CCR4 =0;


                    TIM3->CCR2 =  100;
                    TIM4->CCR4  = 100;

                    TIM3->CCR1 = 100;
                    TIM4->CCR3 = 100;

//				GPIO_SetBits(GPIOC, GPIO_Pin_8);
//				GPIO_SetBits(GPIOC, GPIO_Pin_9);
									
                }
            }

            if(flag_time2_irq == 0)
            {
                flag_direction_irq = 1;
            }

        }

        Delay_ms(5); // 50  5
        if(flag_direction_irq)
            flag_time2_irq++;
        else
            flag_time2_irq--;
        //		printf("\r\n this is a usart printf demo \r\n");
        //	while(flag_time2_irq == 0);


    }
}







/******************************************		new verion		************************************************/

version 142 
(data:2017/04/22 ~ 2017/04/24)

1st	:	调速问题，电机的mos会出问题，发烫严重。改变控制方式，结合需修改的原理图，测试通过。

2nd	:	遥控器，关机后，地址丢失，还需要在再次配网。vb地址和vc有差距，最新固件没问题。

3rd	:	配对，有时候无法成功。 在慈溪，若无法配对成功，退出配对，但是再次进入，则成功。原因需要找到。自行测试，发现10次均成功。

4th ：	地磁问题,电源干扰，查看。用购买模块测试，正常，排除stm32代码的问题。锁定在stm8的hmc1022模块上。重新烧录程序，正常。

5th	: 	sd卡升级问题，出现现象，在stlink的3.3v下，能够完成正常。在船上偶尔不正常，初步判断为电压稳定性。
		另增加提示，若是sd卡未检测到，led2闪烁三次，若是跳转到app，则led1 led2交替闪烁一下。
		如有问题，暂先考虑硬件问题。
	
6th	:	导致异常，还有原因：一个版本启动程序差异，就是hd.s和md.s,进行更改了。回退版本处理。
	



资源说明：

定时器：

系统滴答定时器：

tim1	null --->修改为定时器 ok 0424
tim2  beep
TIM4 1ms定时器，即将更改pwm输出
tim3 mos驱动 pwm输出



串口通讯：

USART1	compass
usart2  gps									DMA channel 6
USART3	蓝牙接收

UART4	log信息（宏定义，stm32vc以上才可以）


adc											DMA channel 1

ADC_ConvertedValueLocal[0] 	电池电压

ADC_ConvertedValueLocal[1]	电流1 
ADC_ConvertedValueLocal[2] 	电流2

ADC_ConvertedValueLocal[3] 	温度


看门狗：

 WWDG_Config(0X7F, 0X5F, WWDG_Prescaler_8); 					// 窗口看门狗
 
 
 
 


version 141 
(data:2017/04/14)

1st	:	set less time when RECEIVE_CMD_NOTICE.

2nd	:	all style.

3rd	:	please delect ',STM32F103VCT6_MCU' in option when released.

4th	:	if there is some problem, then modify


141 0411


1 对于出现beep无缘无响起来的问题，暂时未找到相关原因。在显示屏上显示beep信息。串口打印beep的switch数据。低电量暂时不做处理。
	错误找到9.开机后，尽快按遥控的led全开键，会导致该现象。屡试不爽.
	已处理，应该不会再出现问题。
	

2 uart4 改为串口。在option中的 c/c++选项进行配置，,STM32F103VCT6_MCU，来选择是否为VC系列。去除则兼容VBT6.
		注意，一定要增加,STM32F103VCT6_MCU的说明



141 0327

修改芯片为stm32f103vbt6
由之前 256k改为了 128K

因此相关地址需要进行更改：

0- 0x7fff 占据1/4

0x7fff  - 0xfffe  1/4

剩余部分到 1ffff 剩余1／2


bootloader与app之间的读写密码地址需要更改

船体的遥控器ip需要存储地址修改

跳转地址需要整改，0x8000，也即是0x8008000，大小为0x18000。

stm32f103vbt6 定时器只有4路，串口只有三路。

tim5改为tim4
uart4改为uart1
去除串口的功能，以后只能debug了。


141 0308

确认电压检测问题，主要是二极管导通电压以及mos导通带来的电压减少。实际上的测量值还是比较对应的。
修改初始化默认数值为168，16.8V。

检查温度检测传感器，测量温度是否正常。检测结果为正常。实际测试为腔体温度。


141 0305

更改gps异常后，依然可以控制。

在gps异常后，关闭一次led和moto及其他，且能够正常控制。led toggle。


在轨迹导航中，会出现滴滴滴的声音。需要找到问题进行处理。暂时未找到原因，也未能复现。

船体运行状态，需要可以正常接收导航结束。 遥控器已经做了修改。
	对应遥控器接收到导航结束后，不应该发送停止命令。


//

140版本

-1 更改有存在360度时候，原地旋转的问题。 
	经过测试考虑，比例调节比较方便些。取消90度旋转，进行左右转操作进行尝试。
	优化判断，360度附近的两种方式。
	
-2 更改在导航过程中，出现rf丢失信号，应该停止后，返航到起点，而放弃到达设定导航点的路径。ok
-3 更改旋转角度为90度。 // 考虑去除功能。已经去除。ok
4 更改左右旋转方向的角度依据。当左右角度差值相差多的时候，优先旋转角度小的方向。//雷同与1 1的更改完毕，4相应也更改好了。。原本已经改好的程序。
	考虑在两个弧度角内部计算后，再考虑更改为左右原地旋转。

5 增加电流检测的相关问题。//已经更改 暂时放置在电压位置，为两电流数值*1000.

6 compass模块偶尔返回数据为0. //待考究

7 gps无法准确和高德地图保持一致。再次进行测试。

8 电机在p调节下，反应速度稍微存在顿挫感。需要进行考虑更改相关参数。或者是更改角度范围，应该比5再小一些？
	暂时不做修改。




接下来- 3月1号

需要增加一个相关的操作系统，如freertos。



139版本 2017 0222

增加蓝牙相关代码
更改gps问题，将数据总ddmm.mmmm更改为dddd.dddd格式。

138版本 2017 0219

1 尝试增加本机配置gps模块的功能
	已经修改结束，测试通过。
说明：
	只针对原始的模块，插上即可用。原始模块未进行相关配置，波特率应该为9600。（兼容6M。M7测试可用，未深入测试。m8暂不可用。安装只是用6m）
2 修正前左右切换到前进方向，无法关闭转向灯的问题。	

137版本 2017 0218

1 遥控返航的时候，若是进入手动控制模式，需要先行退出自动导航模式，等待遥控结束，然后再进入自动模块。
2 返航的时候，如果角度数值大于30度，则原地转弯。解决转弯半径过大的问题。
3 前进过程尽量走直线，通过结构修改，暂时不做深入更改。若是后期更改需要增加相关compass实时校准。稍微麻烦。
4 oled显示根据客户需求进行更改。// #define DEBUG_GPS 切换调试模式和客户定义模式
5 低电量声音更改，更改为一声。
6 客户需求“遥控器方向开关转弯体验不佳（要测摇杆感应范围）”：先行测试，再考虑。
		已经进行相关修改，需要实际测试。20170220已经测试通过。
7 每个beep类型，要完成一个周期，不能出先间断交替。



137版本 2017 0216

改进新的pcb
应客户需求，增加尾灯的闪烁。


136版本 2017 0116

修改原地转弯，指示灯不亮的问题。
增加未打开屏蔽的看萌狗。





136版本 2017 0112

1 无线断开的时候，返航，再次进入丢失gps，会持续滴滴滴滴响。后发现为control的beep。即为setBeepBlood = 2，需要去除相关位置，已解决。
2 根据客户需求，更改左右状态为原地打转。并生成不同的速度文件。

			if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 3:00
							Moto_Direction_Turn( MOTO_RIGHT,  100, 0); 
			if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 3:00
							Moto_Direction_Turn( MOTO_RIGHT,  0, 100);



3 根据客户反馈，测试led灯，在运动中的情况。左右正确。实际还是需要增加左侧右侧灯的闪烁，实际上侧灯为连一起的，暂不修改。


135版本 2017 0103

1 蜂鸣器控制时候，在新命令到来时候，将beep屏蔽设置为0.
2 返航过程中，角度转移在30度以内，不再进行led的闪烁。
3 修改了开机distant距离异常的问题。
4 增加一个标志，提示导航结束。与遥控段进行协调处理，规避对应导航过程中停止的问题。
5 adc检测不对应，出现异常。已经修改对应电阻为200k。检测能更加清晰。
6 运行过程中，进行compass，gps的判断。规避一些异常状态的问题。
7  ====>  接收函数，会出现解码错误，或者接收错误。导致无法控制的现象? ===》?




134版本 1231

1 修改moto。减速到0的判断，是需要+数到100的。判断错误。已经修正。
2 递进速度，可选，修改对应宏定义。



134版本 1230


1. 自动校准，修改bug。else if(cmd == 0).
2. 开电机的状态下，改变电机状态情况下，暂时不采集adc电压值。100ms。
3. 瞬间前后左右的运动，会导致数据丢失。需要解决。
			 if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MIN)&&(AS14B_receive_buf[6]  >= 0) )
4. 测试gps检测出现异常原因：
		a 接错了rx tx
		b 波特率设置错误
		c gps设置对应模式错误，gps方式，不是gps及北斗及其他。
		d 模块损坏
		e 上电异常
		f 其他
5 电机转动，以平滑方式测试，发现严重发热。5ms改变10个pwm位，整个过程需要50ms。最高到反向最高持续需要100ms。

6 修改新版本协议。已增加 1230。


133版本 1226

1 重构代码
2 修改了滴滴滴，和滴滴交叉的问题。
3 修复开机后，无法准确判断compass状态的问题。 >=0x30
4 反馈的重启复位问题，暂时未查看到。暂时搁置，等待再次反馈。
5 修改相关协议代码。
6 增加gps钓点，
7 新增加协议04，后干扰配对数据，暂时修改回去。小概率事件，也需要去规避。
8 出现heatbeats++导致增加到3的情况，后来改动改好。抽时间再检测。
9 GPS数据发送过去，解码会失败，可以交给app进行判断和处理了。
10 增加对丢星，丢gps，丢rf三种情况的处理。check_star_and_beep中进行提醒。还需要测试

起点及各个钓点的位置，需要进行判断和测试。


重构相关：
 代码合并，复用。如UART类。
 函数合并，整理。对于类似的函数进行整改传参方式。
 结构体对外进行输出。主要为船体信息及其他，作为整体和其他c文件交互
 相关容易修改的部分，采用宏定义的方式，方便修改和移植。
 函数命名采用驼峰命名法。
 能使用枚举的地方，尽量不适用宏定义。
 beep的封装，可以改动为优先级方式，或者先后次序方式。
 去除相关不再需要的代码。5883 l298n cc1101





1206  132版本

1 根据接口需求，增加了	LEDC_ALL(1);	LEDC_ALL(0);	实现遥控控制led全亮，全灭。
2 将oled代码注释打开。并尝试修改代码，使刷新均分到每个周期中。具体需要测试，是否超过看门狗时间。oled开机故障显示添加相关自定义代码，完成。
3 增加版本显示，目前以 132作为版本号。
4 led根据船的运动进行相关的控制，代码应该已经做好。如有问题，可以适当更改。
5 发送gps数据的代码以封装。需要简单修改。Post_GPS_Data(); // 本版本暂时不做进一步完善。
6 beep声音的更改，已查阅。适当更改，已确定。以代码执行为准。


1106  131


改了很多程序。
主要是修改无线，及相关协议。

仍需要修改，低功耗唤醒后正常工作。

下个版本需要做到：

无线数据重发机制。
原轨迹返航的程序封装。


1016  124 

根据新版本进行修改

1 增加获取遥控地址
	AS14B_Get_ADDR();  // 1s时间用于获取地址
	
	

2 增加反馈接收到命令及心跳的响应：

void Respond_Back(void)
具体参数还需要填充

3 更改相关的指令的含义，具体还需要细化和确认



#define FEED_BIN_D  6
#define FEED_BIN_C	5
#define FEED_BIN_B  4
#define FEED_BIN_A  3
#define FISH_HOOK 2 
#define JOY_STICK 1 //何意？ 

#define START_GPS 7
#define CH1 8
#define CH2 9
#define CH3 10
#define CH4 11
#define GO_SET 12  //   返航
#define GAMPASS 13
#define HEAD_TAIL_LIGHT 14
#define WITH_LIGHT 15
#define ALL_LIGHT 16


// 待续 待确定
#define AUTO_GO 17
//#define GO_SET 18
#define GO_BACK_BY_TAIL 18 //圆轨迹返航

#define GO_BACK 21

4 返回到钓鱼设置点的流程需要具体细化和更改。


0906 122
再次整理资料
删除不再需要的文档，增加相关的c,h文件。





0905
整理格式

0904
121 
修复出现温度数值差的问题。


0903 
 120 增加本地的压栈操作。尝试修改自动 增加linkstack.c


 
  增加GPS实时位置发送代码。 10s发送一次。

  增加加密的程序处理。secret.c
  
  
  
  
  
void Go_Back_By_Trail(void) // 注意单位的转换
{
	double distant_by_trail;
	static double tail_dimensionality = 0, tail_longitude = 0;
	
	
		if(flag_go_back_by_trail == 1) //遥控器收到数据之后，改变标志位
		{
	//	 (new_dimensionality, new_longitude, old_dimensionality, old_longitude);
		
			if(distant_test < 3)
			{	
				pop_linkstack(lsp,&old_dimensionality,&old_longitude);
				if((old_dimensionality == -1) && (old_longitude == -1))
				{		
						flag_go_back_by_trail = 0;
						tail_dimensionality = sure_dimensionality;
						tail_longitude = sure_longitude;
				}
			}
			
		}
		else
		{
			if(RefreshCnt>50) // 判断是否200ms，进行一次判断是否到达间隔距离
			{
				distant_by_trail  = get_distance(tail_dimensionality,tail_longitude ,sure_dimensionality,sure_longitude);
				if(distant_by_trail > 3)
				{
					if((sure_dimensionality != 0) && (sure_longitude != 0))
				    push_linkstack(lsp,&sure_dimensionality,&sure_longitude);
						tail_dimensionality = sure_dimensionality;
						tail_longitude = sure_longitude;
				}
			}	
		}			
}


  
  
     typedef  union result
     {
         double d;
         unsigned char data[8];
     }union_GPS_data;
 
   union_GPS_data r1,r2;


     r1.d = 234.56778;
	  printf("byte0= %d byte1= %d byte2= %d byte3= %d r1=%f r2=%f \n",r1.data[0],r1.data[1],r1.data[2],r1.data[3],r1.d,r2.d);
     r2.data[0]=r1.data[0];
     r2.data[1]=r1.data[1];
     r2.data[2]=r1.data[2];
     r2.data[3]=r1.data[3];
	 r2.data[4]=r1.data[4];
     r2.data[5]=r1.data[5];
     r2.data[6]=r1.data[6];
     r2.data[7]=r1.data[7];
    printf("byte0= %d byte1= %d byte2= %d byte3= %d r1=%f r2=%f \n",r1.data[0],r1.data[1],r1.data[2],r1.data[3],r1.d,r2.d);
  
  		GPS_dimensionality.d =  sure_dimensionality ;
				GPS_longituder.d  = sure_longitude ;
				
				USART_SendData(USART3, 0x00); // addr
				USART_SendData(USART3, 0x03); //type
				
				for(i = 0; i < 8; i ++)
				{
				USART_SendData(USART3, GPS_dimensionality.data[i]);
				}
					for(i = 0; i < 8; i ++)
				{
				USART_SendData(USART3, GPS_longituder.data[i]);
				}
			
				
					USART_SendData(USART3, 0x0a);
  
 
0809
 119 版本


0809
 118 版本



显示温度

发现led和之前的led存在配置上的干扰。暂时屏蔽.

#define LL_LED1_OFF		//	digitalHi(GPIOB,GPIO_Pin_1)
#define LL_LED1_ON		//	digitalLo(GPIOB,GPIO_Pin_1)


快速排序的升降序错了，耽误了一会。


0808
 117 版本
 
 将米字型的代码修改。
 测试显示正常，能够正常工作。
 
 
 仿照别人船的现象写的代码：
 
#define BAR_CENTER_POSITION_MAX 55  //0x32   max 102  min 0
#define BAR_CENTER_POSITION_MIN 45  //0x34   max 102 min 0

					if(	(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX) &&
						(AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN) &&
						(AS14B_receive_buf[7]  <= BAR_CENTER_POSITION_MAX) &&
						(AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MIN))//螺旋不动
					{
						Moto_Direction_Turn( MOTO_STOP, 0, 0);
					}
					else if( AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MAX)
					{
						Moto_Direction_Turn( MOTO_GO, (AS14B_receive_buf[6] -50)*2, (AS14B_receive_buf[6]-50)*2);
					}
					else if( AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MAX)
					{
						Moto_Direction_Turn( MOTO_ROLL_LIGHT,  (AS14B_receive_buf[7]-50)*2,(AS14B_receive_buf[7]-50)*2);
					}
					else if( AS14B_receive_buf[6] <= BAR_CENTER_POSITION_MIN)
					{
						Moto_Direction_Turn( MOTO_BACK, (50 - AS14B_receive_buf[6])*2, (50 - AS14B_receive_buf[6])*2);
					}
					else if( AS14B_receive_buf[7] <= BAR_CENTER_POSITION_MIN)
					{
							Moto_Direction_Turn(MOTO_ROLL_RIGHT,  (50 - AS14B_receive_buf[7])*2, (50 - AS14B_receive_buf[7]) * 2);
					}


0808
 116 版本
 心跳丢失之后，需要自动能够返航。
 增加其他灯及料仓，鱼钩的指令控制。


0807
115版本

若是未设置起点，则以开机第一次寻到的位置作为一个电压数值。需要再测试。

0807 









114版本



若是未设置起点和终点，则无法向终点及起点方向前进。

如果未设置起点或者终点，抛回错误，并不动。

无法接收心跳则自动返航。


else{ //一直未接收到数据。
		if(flag_heart_beart -- < -5000)
		{
						uCmd = USHIPGO;
							setBeepBlood = BCONTROL;	
							key3Cmd = 1;	
//			if(flag_heart_beart < -250)
//			printf("no heart flag_heart_beart = %d --------\r\n",flag_heart_beart);
//					Moto_Direction_Turn( MOTO_STOP, 0, 0);
		}

修复导航下无法停止的状态
							if(AS14B_receive_buf[8] == 1)
				{
						Moto_Direction_Turn( MOTO_STOP, 0, 0);
								key3Cmd = 0;
//						printf("stop---------\r\n");
				}

修改为新的遥控控制方式

根据别人的船的运行状态，修改本机的运行模式，四个档位，速度可调。

以下为删除的位置（还是存在bug的！）
					if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX) &&((AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MIN)))//螺旋不动
					{
					
					}
						else if((AS14B_receive_buf[6]  >= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  >= BAR_Y_CENTER_POSITION))) //第一象限
						{
														printf("第一象限 x  = %d, y=%d\r\n", AS14B_receive_buf[6] - 50 ,AS14B_receive_buf[7]-50);
							as14b_post_angle = (int) (atan2(AS14B_receive_buf[6]-50 , AS14B_receive_buf[7] -50) * 180/3.1416);
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						as14b_post_length = 100;
							if(as14b_post_length > 100)
								as14b_post_length = 100;
							printf("第一象限 MOTO_LIGHT  pwm1  = %d, pwm2  = %d, as14b_post_angle =%d\r\n", as14b_post_length*1, abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))), as14b_post_angle);
								Moto_Direction_Turn(MOTO_LIGHT, as14b_post_length*1, abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))));
						}
						else 	if((AS14B_receive_buf[6]  >= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  <= BAR_Y_CENTER_POSITION))) //第四象限
						{
								printf("第四象限 x  = %d, y=%d\r\n", AS14B_receive_buf[6] - 50 ,AS14B_receive_buf[7]);
					
									as14b_post_angle = (int) (atan2(AS14B_receive_buf[6] - 50 , AS14B_receive_buf[7] ) * 180/3.1416);
							
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						
							as14b_post_length = 100;
							if(as14b_post_length > 100)
								as14b_post_length = 100;
								Moto_Direction_Turn(MOTO_LIGHT,  abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))), as14b_post_length*1);
	
								printf("第四象限 MOTO_LIGHT  pwm1  = %d, pwm2  = %d, as14b_post_angle =%d\r\n", abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))),  as14b_post_length*1, as14b_post_angle);
						}
						else 	if((AS14B_receive_buf[6]  <= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  >= BAR_Y_CENTER_POSITION))) //第二象限
						{
						
							as14b_post_angle = (int) (atan2(AS14B_receive_buf[6] , AS14B_receive_buf[7]) * 180/3.1416);
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						
							if(as14b_post_length > 100)
								as14b_post_length = 100;
								Moto_Direction_Turn(MOTO_BACK, as14b_post_length*1, abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))));
							
							printf("第二象限 MOTO_BACK  pwm1  = %d, pwm2  = %d\r\n", as14b_post_length*1, abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))));
							
						}
						else 	if((AS14B_receive_buf[6]  <= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  <= BAR_Y_CENTER_POSITION))) //第三象限
						{
							as14b_post_angle = (int) (atan2(AS14B_receive_buf[6] , AS14B_receive_buf[7]) * 180/3.1416);
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						
							if(as14b_post_length > 100)
								as14b_post_length = 100;
								Moto_Direction_Turn(MOTO_BACK,  abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))), as14b_post_length*1 );
									printf("第三象限 MOTO_BACK  pwm1  = %d, pwm2  = %d\r\n", as14b_post_length*1, abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))));
							
						}
					


08 06 

114版本
将OLED显示的经纬度数据去除。
将电池电压检测显示。ADC1_10 * 11 

板子产生的热量显示出来。


GPS的EEPROM需要焊接，然后进一步测试。数据掉电时间长了好像还是会丢失数据的信息。

修改控制上的按键无法准确停止。指令判断不准确，缺少两个50的状态判断。






08 06 

113版本

电机控制需要修改为实时控制，不能存在延时。经过测试，发现此次的mos管完全可以实现实时改变占空比及方向，而且不会出现发烫的现象。
已经将其代码精简，修改为实时更改。

08 06 

112版本
将无线控制修改为摇杆新方式控制。考虑分成四个象限，根据角度数值及大小数值改变占空比。主要采用勾股定理。




08 05
111版本

增加led的控制，采用宏定义方式。
增加mos驱动，采用400ms的闭合，防止电磁铁发烫。测试ok。


08 05
110版本

增加adc采集。需要进一步做单位转换处理及oled显示。



08 04
109版本

更改为新的PCB。

OLED作出引脚上的修改。

gpio_MSCK PC0 ---> 
gpio_MOSI PC1 ---> PC13 
gpio_DC   PD3 --->
gpio_RST  PD5 --->
gpio_CS   PD6 --->

	GPIOC->CRL&=0XFFFFFFF0; //PC0,1 OUT
	GPIOC->CRL|=0X00000003;	  
	GPIOC->ODR|=1<<0;
	GPIOC->CRH&=0XFF0FFFFF; //PC0,1 OUT
	GPIOC->CRH|=0X00300000;	  
 	GPIOC->ODR|=1<<13;
	
	#define OLED_SCLK PCout(0)
	#define OLED_SDIN PCout(13)

显示屏正常显示


2016 -7 31
测试通过，可以瞬间停止。但是，需要同时加速到某速度，某方向。还需要优化。
增加gps自校准程序。校准开始旋转，结束停止旋转。
终点处的拐弯，需要根据偏执角度，进行相关的左转或者右转
考虑增加gps修改为1Hz，并修改为队列方式求冒泡。修改了5组数据冒泡的保存元数据。测试GPS 6，7 均没问题。8 今日定星有些问题。
compass的定位固定，需要先测试分布是否均匀，360度。然后，走一段距离定位方向。需要写文档。

108版本，
---> 在今日107版本增加电机控制的平滑控制，瞬间停止。
	case MOTO_STOP:
		TIM3->CCR3=100;
		TIM3->CCR4=100;
//		Moto_Right_Control(Moto_T, 0);
//		Moto_Left_Control(Moto_T, 0);


---> 增加， gps自校准程序
		
		else if(rf_payload[6] == 6) //返航
							{
								if((flag_calibrate_compass++%2) == 0)
								{
									USART_SendData(UART4,0X00);
									USART_SendData(UART4,0XC0); // 校准
								}
								else
								{
									USART_SendData(UART4,0X00);
									USART_SendData(UART4,0XC1); //停止校准
								}
//							uCmd = USHIPBACK;
//							setBeepBlood = BSTART;	 //报警，需要修改状态位
							key3Cmd = 2;
							}


--->  终点处的拐弯，需要根据偏执角度，进行相关的左转或者右转。需要测试（10:28）

	if(angle_normal < angle_seek)
			{
				radian1 = abs(angle_normal + 360 - angle_seek);
				radian2 = abs(angle_seek - angle_normal);
				if(radian1 > 360)
					radian1 = radian1 - 360;
			}
			else
			{
				radian1 = abs(angle_normal  - angle_seek);
				radian2 = abs(angle_seek + 360 - angle_normal);
				if(radian2 > 360)
					radian2 = radian2 - 360;
			}
		
				pwm_offset = ((radian1  - radian2)/2) * pwm_p;
				
昨日遗留测试：
考虑增加gps修改为1Hz，并修改为队列方式求冒泡。queue。暂未锁定成功。需要测试。
经检验，代码无误。
		

2016 -7 30

返航过程中，将原地转改为弧度转，并测试现象修改。
Moto_Direction_Turn(MOTO_RIGHT, 45, 65);

考虑增加PID控制转动。pid需要处理，整定参数。目前P为2 D未增加。现象目前可以。

考虑增加gps修改为1Hz，并修改为队列方式求冒泡。queue。暂未锁定成功。需要测试。

2016 -7 24

遥控控制过程中，将原地转改为弧度转


2016 -7 23

修改beep到v——char引脚，

需要将beep引脚修改，及控制部分的代码。

相关控制pwm输出可以更改为0输出。



2016 07 22
昨日今日，出现异常无法读取gps数据
猜，可能是程序哪修改错了，查无。
猜，可能GPS数据出现异常，查无。猜测参数配置有问题，暂无。
猜测，板子可能焊接有问题，简单考虑觉得应该不是。后换成昨天以为换了开发板，已经好了。

5v供电的位置，在蜂鸣器输出的时候，会出现异常gps丢星。

换模块后，发现依然会出现丢星，但是现象好很多。可能是电池电压不足，此时出现的电压是10.几伏。



2016 6 4

修改项：

将差别角度，修改为3度
	if(((abs(angle_normal  - (Angle - 90)) < 3) ||((abs(abs(angle_normal - (Angle - 90) ) - 360) < 3))))
	
修复当在270度-300度左右范围内时候的无法正反转的状态
	
		if((Angle - 90) < 0)
		angle_2_oled = Angle + 360;
		OLED_ShowNum(92,36,(u32)(angle_2_oled - 90),6,12);


			angle_seek = Angle + 360;当角度小于0的时候，需要加上一个360度。


修复定位返航的计算方法，我个人以为，角度计算应该以目前位置到返航点计算为准。距离的位置，也应该如此。







电机被拉低电压



电机接上电源后电压迅速被拉低了是什么原因?

电机运行后电源电压迅速被拉低了可能原因有：
电源容量过小造成电压降过大。
电源线过小造成电压降过大。
电机功率与电源线、电源容量不匹配造成。
电机接线错误造成。
电机相与相、相与地或匝间有短路现象。
电机负载过重等等造成。


1、电机短路，2、电源功率不够，3、供电电路接触不良。

最大的可能你的12V电源出现问题

主要看pcb板。pcb焊锡接地会造成电压拉低。可以用一个可调节电源加在电机上，可判断是否是电源问题还是电机问题。逐步排查。

电机的起动电流是运行电流的N倍


电动机在启动时，电流可以达到额定电流的3-7倍，大型的电机电流就更大了，
而且线路本身是有电阻的，这么大的电流通过线路，就会在线路上产生一个电压降，所以电机上的电压就被拉低了。因此大型的电机要采用降压启动。

检查下你的电源，如果电路没问题，多半是因为电源的带负载能力不够，导致电源电压被拉下来了。
你可以考虑换一个输出功率大的电源试试。

最近搞电机，也遇到你这问题了，后来解决了，是电源电流不够。不光要看电压，还要有看电源最大输出电流够不够



测试数据：
第一组：
new_dimensionality = 31.01614210;
new_longitude = 121.35121919;
old_dimensionality = 31.01612999;
old_longitude = 121.35142949;
2：
new_dimensionality = 31.01622150;
new_longitude = 121.35143770;
old_dimensionality = 31.01601209;
old_longitude = 121.35151149;
角度误差3度，实际343.117780！

3：
new_dimensionality =31.01601279 ;
new_longitude = 121.35152169;
old_dimensionality = 31.01559400;
old_longitude = 121.35153790;
实际：358.090649，1度误差
4：
new_dimensionality = 31.01607500;
new_longitude = 121.3531360;
old_dimensionality = 31.01652540;
old_longitude = 121.3530769;
实际：173.552256，1度误差




