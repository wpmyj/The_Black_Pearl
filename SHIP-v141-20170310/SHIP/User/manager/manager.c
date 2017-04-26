
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "gps_config.h"
#include <string.h>
#include "bsp_led.h"
#include <math.h>
#include "bsp_pwm_output.h"
#include "bsp_SysTick.h"
#include "nmea/nmea.h"
#include "TIM5.h"
#include <string.h>
#include "bsp_key.h"
#include "oled.h"
#include "moto.h"
#include "L9110S.h"
#include "bsp_adc.h"
#include "temperature.h"
#include "linkstack.h"
#include "manager.h"
#include "stop_mode.h"
#include "secret.h"

//#define STM32F103VCT6_MCU 1 //使用软件配置，保证各个文件都能共享。

/*      ===================================               宏定义          ===================================              */

#define DEBUG_MODE  // 使用printf输出



#define LOW_VOLTAGE 13.6 // should 13.6

/*      ===================================              全局变量        ===================================                 */
u8 flag_rf_send_error = 0;
u8 flag_cancel_seek = 0;

int i = 0, j = 0;
double distant_normal = 0, distant_test = 10, distant_temp = 0;
double new_longitude = 0, new_dimensionality = 0, old_longitude = 0, old_dimensionality = 0;
double longitude_CH[5], dimensionality_CH[5];
double start_longitude_temp = 0, start_dimensionality_temp = 0;
double angle_normal = 0, angle_test = 0, angle_normal1;
double sure_dimensionality = 0, sure_longitude = 0;


u16 flag_cannot_new_parse;

nmeaINFO info;          //GPS解码后得到的信息
nmeaPARSER parser;      //解码时使用的数据结构
uint8_t new_parse=0;    //是否有新的解码数据标志
nmeaTIME beiJingTime;    //北京时间

unsigned char rf_payload[10];
unsigned char rf_payload_temp = 0;


double tail_dimensionality = 0, tail_longitude = 0;

int flag_go_back_by_trail = 0;
int moto_control_led_mode = MOTO_CONTROL_LED_STAND_BY;
int flag_RIGHT_led_toggle_time = 500, flag_LEFT_led_toggle_time = 500,flag_WIDTH_led_toggle_time = 500, flag_ALL_led_toggle_time = 200;//ms
volatile int flag_heart_beart = 0;


volatile 	u8 flag_enter_in_mode_wwdg  = 0;
u8 flag_rf_stand_by_state = 0;


u8 flag_WITH_LIGHT  = 0, flag_HEAD_TAIL_LIGHT = 0, flag_ALL_LIGHT = 0;
u8 flag_gps_post = 0;
static u8 key3Cmd = 0;
static u8 uCmd =UWORK,uCmd_pre = UWORK;
u8 flag_Seek_Route_Check_Ing = 0;
//-----------------------------------------------------------------------------------//
// 地址         类型  电量  gps  同步码  运行状态   结束
uint8_t respond[10] = {0x00, 0x00, 0x01, 0xff, 0x01, 0x00, 0x00,0x0d,0x0a,0xff};

linkstack_t *lsp;


/*    ===================================                 版本号          	  ===================================            */
u32 VERSION = 141;


int flag_compass_adjust = -1;

/*    ===================================                 ADC 类数据         ===================================           */
extern __IO uint16_t ADC_ConvertedValue[4];
extern __IO uint32_t ADC_SampleValue[4];
extern float ADC_ConvertedValueLocal[4] ;

/*    =================================== 								 船体状态数据      ===================================         	 */

Ship_Board_Data_t Ship_Board_Data;
uint16_t flag_storage = 0x1f; //料仓及鱼钩状态


union_GPS_data GPS_dimensionality,GPS_longituder;

u32 flag_set_beep_free = 0;

u8 GPS_is_device_up = 10;

/*    ===================================                外部引用标志位      ===================================             */

extern uint8_t rfrxFlag;
extern char angle_receive_buf[8];
extern char AS14B_receive_buf[128];
extern  Fishhook_Storage_t Fishhook_Storage_Control;

extern double Angle;
extern u8 RefreshCnt;
extern u8 setBeepBlood;    //bsp_pwm_output.c for beep control.
extern u8 flag_4ms_set;


extern u8  UART_CHECK_CNT,RXNUM,UART3_RX_RESUME_FLAG;
extern u8 UART3_BUF[120]; 	// 接收中断数据的临时buf
extern u8 UART3_CODE[120]; // 存搬移数据的buf


char DATA_X,DATA_Y;


u8 flag_need_change_directions_all_time = 0;

//=========================================================================================================================//

//  ship  all INIT -----------------------000--------


//#define DEBUG_GPS

void 	Ship_ALL_Init(void)
{


//	Enter_Stop_Status_GPIO_Init_All_Same(); //23uA
    SysTick_Init();

//	Delay_us(100000);
//	Enter_Stop_Status();

#ifdef DEBUG_MODE


	#ifdef STM32F103VCT6_MCU
		USART4_Config();
	
		USART4_printf(UART4, "hello  uart4 \r\n");
//     USART3_printf( USART3, "\r\n %s \r\n", j );(USART1,"HELLO  version [%d]\r\n",VERSION);
//  	 USART3_printf( USART3, "\r\n %s \r\n", j );(USART1,"HELLO  version 1 [%f]version 2 [%f]\r\n",(double)(VERSION+0.1415926),(double)(VERSION+0.1415926));
//
	#endif
	
    OLED_Init();
    OLED_ShowString(0,0,"debug mode",12);
    OLED_Refresh_Gram_New();
#endif //debug	


    /* Enable Clock Security System(CSS): this will generate an NMI exception
    when HSE clock fails */
    RCC_ClockSecuritySystemCmd(ENABLE);

    this_is_a_secret();  // 1208 暂时性屏蔽

    LED_GPIO_Config();  //GPIO CONFIG!!  指示灯 自身用
		sysInitIndictor();  // led闪烁一下提示

    TIM2_PWM_Init();

    uKey_Init();

//    TIM5_Configuration(); 				 //tm32f103vbt6 没有tim5
//    TIM5_NVIC_Configuration(); 	  	//tm32f103vbt6 没有tim5
	
//    TIM4_Configuration();  //tm32f103vbt6 没有tim5，只有1 2 3 4 			// tim4作为pwm输出 tim1作为定时器 0424
//    TIM4_NVIC_Configuration(); //tm32f103vbt6 没有tim5，只有1 2 3 4  // tim4作为pwm输出 tim1作为定时器 0424
		
		
		//5ms 定时器 tim1
		TIM1_Configuration(); 
		

    /* 初始化GPS模块使用的接口（UART2） */
    GPS_Config();		 //缺少器件检测


    /* 设置用于输出调试信息的函数 */
    nmea_property()->trace_func = &trace;
    nmea_property()->error_func = &error;

    /* 初始化GPS数据结构 */
    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);

    setBeepBlood = BCONTROL;

    //USART3
    Bt_Receive_Init();
//    printf("\r\n  Bt_Receive_Init ok!!\r\n");
    //新的pcb 接受compass修改为uart4
    GY_26_Receive_Init();

    Mos_Moto_Init(); 	//修改 0425
		

//		MOTO_TEST_CHANGE(); //while 1

    LEDS_ULN2003_GPIO_Config();
    LEDC_ALL(1);
    LEDC_OTHER(1);
//    printf("\r\n LEDS_ULN2003_GPIO_Config ok!!\r\n");
    Delay_ms(100);
    RF_CHECK(); // 检测rf是否正常
//    printf("\r\n RF_CHECK ok!!\r\n");
 
		Moto_Direction_Moto_Change_One_Time();


    AS14B_Get_ADDR();  // 100ms时间用于获取地址 包含配置信道等


//	Delay_ms(500);
    LEDC_ALL(0);

    HC_MOS_GPIO_Config(); //鱼钩 料仓

    ADC1_Init(); //  电压检测 温度检测  电流检测

    Check_our_Board_and_warnning(); // 检测compass gps 是否正常

    lsp = creat_linkstack();

    OLED_Clear();



}



void 	Ship_stop_enter_normal_Init(void)
{

    SysTick_Init();


    USART1_Config();
//    printf("\r\n Ship_stop_enter_normal_Init ENTER!!\r\n");

    SetSysClockTo72();

    RCC_ClockSecuritySystemCmd(ENABLE);

    this_is_a_secret();

    LED_GPIO_Config();  //GPIO CONFIG!!  指示灯 自身用


    TIM2_PWM_Init();

    uKey_Init();

//    TIM5_Configuration();
//    TIM5_NVIC_Configuration();

    /* 初始化GPS模块使用的接口（UART2） */
    GPS_Config();		 //缺少器件检测

    /* 设置用于输出调试信息的函数 */
    nmea_property()->trace_func = &trace;
    nmea_property()->error_func = &error;

    /* 初始化GPS数据结构 */
    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);

    //USART3
    Bt_Receive_Init();

    //新的pcb 接受compass修改为uart4
    GY_26_Receive_Init();

    Mos_Moto_Init();


    LEDS_ULN2003_GPIO_Config();
    LEDC_ALL(1);
    LEDC_OTHER(1);
    //	Moto_Direction_Turn( MOTO_GO,  100, 100);


    LEDC_ALL(0);
    //	Moto_Direction_Turn( MOTO_STOP,  0, 0);
    HC_MOS_GPIO_Config(); //鱼钩 料仓

    ADC1_Init(); //  电压检测 温度检测  电流检测



    WWDG_Config(0X7F, 0X5F, WWDG_Prescaler_1); // 5ms
    Feed_Dog_When_Hungry();
//    printf("\r\n Ship_stop_enter_normal_Init END!!\r\n");
    Delay_ms(50);


}



// functions


double getAngle(double lat1, double lng1, double lat2, double lng2)
{

    double Rc=6378137;
    double Rj=6356725;
    double m1_RadLo,m1_RadLa,m2_RadLo,m2_RadLa;
    double m1_Longitude,m1_Latitude,m2_Longitude,m2_Latitude;
    double Ec;
    double Ed;
    double dx,dy,angle;
    //			double dLo = 1,dLa = 1;


    m1_Longitude = lng1/100.0; // 0515
    m1_Latitude =  lat1/100.0;
    m2_Longitude = lng2/100.0;
    m2_Latitude =  lat2/100.0;

    m1_RadLo=m1_Longitude*PI/180.0;
    m1_RadLa=m1_Latitude*PI/180.0;
    Ec=Rj+(Rc-Rj)*(90.0 - m1_Latitude)/90.0;
    Ed=Ec*cos(m1_RadLa);

    m2_RadLo=m2_Longitude*PI/180.0;
    m2_RadLa=m2_Latitude*PI/180.0;

    dx=(m2_RadLo-m1_RadLo)*Ed;
    dy=(m2_RadLa-m1_RadLa)*Ec;

    angle=atan2(dx,dy) * 180.0 / PI+ 180.0;   //abs的参数必须是整数

    return angle;
}


double radian(double d)
{
    return ((d * PI) / 180.0);   //
}


double get_distance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1,radLat2, a, b, dst;


//	printf("\r\nget_distance lat1= %lf  lat1= %lf\t",lat1, lng1);
//		printf("get_distance lat2= %lf  lng2= %lf \r\n",lat2, lng2);
    lat1 = lat1/100;
    lng1 = lng1/100;
    lat2 = lat2/100;
    lng2 = lng2/100;

    radLat1= radian(lat1);
    radLat2 = radian(lat2);

    a = radLat1 - radLat2;
    b = radian(lng1) - radian(lng2);

    dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2) )));

    dst = dst * EARTH_RADIUS;

//    dst = round(dst * 10000) / 10000;  //返回四舍五入的数值，舍弃没了，坑
//		printf("\r\n dst3 = %f  km  return %d \r\n", dst, (int)(dst * 10000)); // km
    return (dst * 10000);
}






void set_start_local(void)
{
    longitude_CH[0] = sure_longitude;
    dimensionality_CH[0] = sure_dimensionality;
//    USART1_printf(USART1,"ch 0 = [%f][%f]\r\n",longitude_CH[0],dimensionality_CH[0]);

}

void set_end_local(void)
{
    old_longitude = sure_longitude;
    old_dimensionality = sure_dimensionality;
}



void Seek_Route_Check(void)
{
    static int flag_fun_once_right = 0;
    double angle_seek = 0;
    int  pwm_offset;
    int pwm_p = 3;
    int radian1 = 0, radian2 = 0;
    static double flag_distant_test_seek_route_check ;


    if(flag_need_change_directions_all_time >= 1)
    {
        if(flag_need_change_directions_all_time++ >100)
            flag_need_change_directions_all_time = 0;
        return ;
    }


    if(	flag_cancel_seek ==  1)
    {
        respond[6] &= 0xf8; // 低三位应该设置为0后，再操作
        respond[6] |= 0x02;

        angle_seek = Angle;


//	printf("angle_normal = %f angle_seek = %f\r\n",angle_normal,angle_seek );

        if(abs(angle_normal  - angle_seek) < 30)
            flag_Seek_Route_Check_Ing = 1;
        else
            flag_Seek_Route_Check_Ing = 0;


        if((abs(angle_normal  - angle_seek) < 5) || (abs(angle_normal  + 360 - angle_seek) < 5) || (abs(angle_normal  - 360 - angle_seek) < 5)) // 角度差 范围数值小于5 则直线行驶
        {
            if(flag_fun_once_right++ == 0)
                Moto_Direction_Turn( MOTO_GO, 100, 100);

            if(flag_fun_once_right > 10)
                flag_fun_once_right = 10;
            //	setBeepBlood = BTRUE;

            P_LED2_ON;
            P_LED3_ON;
            P_LED1_OFF;
            P_LED4_OFF;
        }
//	else	if(abs(angle_normal  - angle_seek) > 90) // 如果角度差值大于 60 ，  应该进行原地旋转
//	{
//
//			P_LED2_OFF;
//			P_LED3_OFF;
//
//
//			if(angle_normal < angle_seek)
//			{
//				radian1 = abs(angle_normal + 360 - angle_seek);
//				radian2 = abs(angle_seek - angle_normal);
//				if(radian1 > 360)
//					radian1 = radian1 - 360;
//			}
//			else
//			{
//				radian1 = abs(angle_normal  - angle_seek);
//				radian2 = abs(angle_seek + 360 - angle_normal);
//				if(radian2 > 360)
//					radian2 = radian2 - 360;
//			}
//
//			pwm_offset = ((radian1  - radian2)/2) * pwm_p;
//
//		if(pwm_offset > 35)
//			pwm_offset = 35;
//		if(pwm_offset < -35)
//			pwm_offset = -35;
//
//
//		Moto_Direction_Turn(MOTO_ROLL_RIGHT, 60 - pwm_offset , 60 + pwm_offset);

//		//	setBeepBlood = BERROR;
//			flag_fun_once_right = 0;
//		 //todo
//
//	}
        else
        {
            P_LED2_OFF;
            P_LED3_OFF;

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


//			pwm_offset = (angle_normal  - angle_seek) * pwm_p;
            if(pwm_offset > 35)
                pwm_offset = 35;
            if(pwm_offset < -35)
                pwm_offset = -35;



            //	printf("pwm_offset %d\r\n",pwm_offset);

            if(abs(radian1 -radian2) < 180)
            {

                if(radian1 > radian2)
                    Moto_Direction_Turn(MOTO_ROLL_RIGHT, 60, 60 );
                else
                    Moto_Direction_Turn(MOTO_ROLL_LIGHT, 60, 60 );
            }
            else
                Moto_Direction_Turn(MOTO_RIGHT, 60 - pwm_offset, 60 + pwm_offset);

            //	setBeepBlood = BERROR;
            flag_fun_once_right = 0;
        }


        if(flag_distant_test_seek_route_check != distant_test)
        {
            if(distant_test < 10)
            {
                flag_fun_once_right = 0;

//					printf("10\r\n");
                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                flag_cancel_seek = 0;

                respond[6] &= 0xf8;  //
                respond[6] |= 0x03;  //  导航结束
                //	printf("respond[6] |= 0x03  \r\n");
                setBeepBlood = BEEPOFF;
                P_LED2_OFF;
                P_LED3_OFF;
                P_LED1_OFF;
                P_LED4_OFF;
            }

            flag_distant_test_seek_route_check = distant_test;
        }
    }
    else
    {

        flag_Seek_Route_Check_Ing = 0;
        distant_test = 10; // ---ADD  0225

    }


}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
u8 uControl(void)
{


    if(uCmd != uCmd_pre)
    {
        uCmd_pre = uCmd;
    }

    switch (uCmd_pre)
    {
    case USTARTSET:  //设置起点
        set_start_local();
        //在此处，增加修改空闲的状态。
        break;
    case UENDSET:     //设置终点
        set_end_local();
        break;
    case USHIPGO:   //向起点出发
        //make_sure_local_to_new();
        flag_cancel_seek = 1;
        //		printf("\r\n ->>> GO! \r\n");
        break;
    case USHIPBACK:  //向
        flag_cancel_seek = 1;
        //make_sure_local_to_old();
        //		printf("\r\n ->>> GOBACK! \r\n");
        break;
    case USHIPCANCEL:
        //	flag_cancel_seek = 0;
        //	printf("\r\n ->>> CANCEL! \r\n");
        break;
    case UWORK:
        break;
    default:
        break;
    }
    uCmd = UWORK;

    return 0;
}


void  GPS_Num_Get(void) //2222222222222222222222222222222222222222222222222
{
    static double dimensionality_number[5];
    static double longitude_number[5];
    static int longitude_dimensionality_temp = 0;
    static double dimensionality_number_temp[5];
    static double longitude_number_temp[5];
    static u8 i = 0, j = 0;

    u8 dimensionality_i,dimensionality_j;
    u8 longitude_i,longitude_j;
    double temp;
    int temp_lng,temp_dim;
    u8 queue_flag = 0;
    u8 gave_number_flag = 0;
    u8 flag_data_get_all_ok = 0;

    if(GPS_HalfTransferEnd)     /* 接收到GPS_RBUFF_SIZE一半的数据 */
    {
        /* 进行nmea格式解码 */
        nmea_parse(&parser, (const char*)&gps_rbuff[0], HALF_GPS_RBUFF_SIZE, &info);
        GPS_HalfTransferEnd = 0;   //清空标志位
        new_parse = 1;             //设置解码消息标志
//					printf("parser->buffer: %s\r\n", parser.buffer);
//				printf("gps_rbuff: %s\r\n",gps_rbuff);
//					printf("\r\n ->>>1  info.lon = %lf  info.lat = %lf \r\n", info.lon, info.lat);
//
//					info.lon =(((int)info.lon)/100) *100 + (((int)info.lon)%100*1.0/60.0)*100 + ((info.lon -(int)info.lon)/3600.00)*10000;
//					info.lat =(((int)info.lat)/100) *100 + (((int)info.lat)%100*1.0/60.0)*100 + ((info.lat -(int)info.lat)/3600.00)*10000;
//
//					printf("\r\n trans  ->>>1  info.lon = %lf  info.lat = %lf \r\n", info.lon, info.lat);

    }
    else if(GPS_TransferEnd)    /* 接收到另一半数据 */
    {

        nmea_parse(&parser, (const char*)&gps_rbuff[HALF_GPS_RBUFF_SIZE], HALF_GPS_RBUFF_SIZE, &info);
        // 有没有可能此处解码未完全
        GPS_TransferEnd = 0;
        new_parse =1;

        //		printf("\r\n ->>>2  info.lon = %lf  info.lat = %lf \r\n", info.lon, info.lat);

        temp_lng = 	(((int)info.lon)/100) *100;
        temp_dim =	(((int)info.lat)/100) *100;
        info.lon =	temp_lng + ((info.lon - temp_lng) * 1.0/60.0) * 100;
        info.lat =	temp_dim + ((info.lat - temp_dim) * 1.0/60.0) * 100;

        //		printf("\r\n trans  ->>>2  info.lon = %lf  info.lat = %lf \r\n", info.lon, info.lat);
        //		printf("\r\n---------------------------------------\r\n");
        flag_data_get_all_ok = 1;
    }


    if((flag_data_get_all_ok == 1)&&(new_parse == 1))                //有新的解码消息    //需要确认在没有搜到星的状态下，是不存在此处的,存在，但是只是解码
    {
        /* 对解码后的时间进行转换，转换成北京时间 */


        GMTconvert(&info.utc,&beiJingTime,8,1);
        new_parse = 0;
        //		printf("\r\n info.satinfo.inuse = %d \r\n", info.satinfo.inuse);

        //////////////////////////////////////////////////////////////////////////////////////////////0618
        //		printf("\r\n new_parse = 1  info.lon = %f  info.lat = %f \r\n", info.lon, info.lat);
        if((info.lon != 0)&&(info.lat != 0))
        {

            if((j != 5) || (i != 5))
            {
                dimensionality_number_temp[i++] = info.lat;
                longitude_number_temp[j++] = info.lon;
            }
            else
            {
                for(queue_flag = 4; queue_flag > 0;  queue_flag--)
                {
                    dimensionality_number_temp[queue_flag] =	dimensionality_number_temp[queue_flag - 1];
                    longitude_number_temp[queue_flag] = longitude_number_temp[queue_flag - 1];
                }
                dimensionality_number_temp[0] = info.lat;
                longitude_number_temp[0] = info.lon;
            }

            if((j == 5) &&(i == 5))
            {
                for(gave_number_flag = 0; gave_number_flag < 5; gave_number_flag ++)
                {
                    longitude_number[gave_number_flag] = longitude_number_temp[gave_number_flag];
                    dimensionality_number[gave_number_flag] =dimensionality_number_temp[gave_number_flag];

                }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //冒泡处理
                for (dimensionality_i = 0; dimensionality_i < 4; dimensionality_i++)
                {
                    for (dimensionality_j = 0; dimensionality_j < 4 - dimensionality_i; dimensionality_j++)
                    {
                        if (dimensionality_number[dimensionality_j] < dimensionality_number[dimensionality_j + 1])
                        {
                            temp = dimensionality_number[dimensionality_j];
                            dimensionality_number[dimensionality_j] = dimensionality_number[dimensionality_j + 1];
                            dimensionality_number[dimensionality_j + 1] = temp;
                        }
                    }
                }
                for (longitude_i = 0; longitude_i < 4; longitude_i++)
                {
                    for (longitude_j = 0; longitude_j < 4 - longitude_i ; longitude_j++)
                    {
                        if (longitude_number[longitude_j] < longitude_number[longitude_j + 1])
                        {
                            temp = longitude_number[longitude_j];
                            longitude_number[longitude_j] = longitude_number[longitude_j + 1];
                            longitude_number[longitude_j + 1] = temp;
                        }
                    }
                }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                sure_dimensionality = (dimensionality_number[2]+dimensionality_number[3] +dimensionality_number[1]  )/3;
                sure_longitude = (longitude_number[2] + longitude_number[3] + longitude_number[1]  )/3;

//							printf("\r\nsure_dimensionality==== %lf\r\n",sure_dimensionality);
//							printf("\r\nsure_longitude==== %lf\r\n",sure_longitude);



                if((sure_longitude != 0)&&(sure_dimensionality != 0))
                {
                    longitude_dimensionality_temp++;

                    if(longitude_dimensionality_temp == 10)
                    {
                        start_longitude_temp = sure_longitude;
                        start_dimensionality_temp = sure_dimensionality;

//										printf("\r\nstart_longitude_temp==== %lf\r\n",start_longitude_temp);
//										printf("\r\nstart_dimensionality_temp==== %lf\r\n",start_dimensionality_temp);
                    }
                    if(longitude_dimensionality_temp>10)
                    {
                        longitude_dimensionality_temp =11;
                    }
                }

            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////0618

//				sure_dimensionality = info.lat;
//				sure_longitude = info.lon;

        if(key3Cmd == 1)  //USHIPGO
        {
            if((sure_dimensionality != 0)||(new_dimensionality != 0))
            {
                distant_test  = get_distance(new_dimensionality,new_longitude,sure_dimensionality,sure_longitude);
                angle_normal = getAngle(new_dimensionality, new_longitude, sure_dimensionality,sure_longitude);
            }
            //			printf("\r\n ->>>4 GO now Gdistance %f! \r\n",distant_test);
        }
        else 	if(key3Cmd == 2)  //USHIPBACK
        {
            if((sure_dimensionality != 0)||(old_dimensionality != 0))
            {
                distant_test  = get_distance(sure_dimensionality,sure_longitude,old_dimensionality,old_longitude);
                angle_normal = getAngle(old_dimensionality, old_longitude, sure_dimensionality,sure_longitude);
            }
            //			printf("\r\n ->>>4 BACK now Bdistance %f! \r\n",distant_test);
        }
        else
        {
            distant_test = 10;
        }

        flag_cannot_new_parse = 0;

        //			printf("\r\n ->>>3  Angle = %f \r\n", Angle);
        //newGPSDataFlag = 4
    } else
    {

        if(flag_cannot_new_parse++ > 1000)
        {
            flag_cannot_new_parse = 1000;

        }

    }
}


int   GPS_Check_ON(void) //2222222222222222222222222222222222222222222222222
{

    if(GPS_HalfTransferEnd)     /* 接收到GPS_RBUFF_SIZE一半的数据 */
    {
        /* 进行nmea格式解码 */
        nmea_parse(&parser, (const char*)&gps_rbuff[0], HALF_GPS_RBUFF_SIZE, &info);
        GPS_HalfTransferEnd = 0;   //清空标志位
        new_parse = 1;             //设置解码消息标志

//				printf("gps_rbuff: %s\r\n",gps_rbuff);
//				printf("\r\n ->>>1  info.lon = %f  info.lat = %f \r\n", info.lon, info.lat);
    }
    else if(GPS_TransferEnd)    /* 接收到另一半数据 */
    {
        nmea_parse(&parser, (const char*)&gps_rbuff[HALF_GPS_RBUFF_SIZE], HALF_GPS_RBUFF_SIZE, &info);
        // 有没有可能此处解码未完全
        GPS_TransferEnd = 0;
        new_parse =1;

        //			printf("\r\n ->>>2  info.lon = %f  info.lat = %f \r\n", info.lon, info.lat);
    }
    if(new_parse)                //有新的解码消息    //需要确认在没有搜到星的状态下，是不存在此处的,存在，但是只是解码
    {
        new_parse = 0;
    }
    //			printf("parser->buffer: %s\r\n", parser.buffer);

    if(strstr((const char*)parser.buffer, "GPGSA")!= NULL)
        return 1;
    else
        return 0;

}




void  GPS_Check_in_using(void) //2222222222222222222222222222222222222222222222222
{
    //调用后的10s内给出反馈，以后不再进入本函数。


    if(GPS_is_device_up < 20)
    {
        if(strstr((const char*)parser.buffer, "GPGSA")!= NULL)
            GPS_is_device_up ++;
        else
            GPS_is_device_up --;
    }
    if(GPS_is_device_up == 1)
    {
        setBeepBlood = BEEPOFF;
        flag_enter_in_mode_wwdg = 1;
        TIM2_GPIO_Deinit();
			
				LEDC_OTHER(0);
				LEDC_ALL(0);
			
			
			//电机停止转动
			
			
			
//				GPIO_ResetBits(GPIOD,GPIO_Pin_15);
//				GPIO_ResetBits(GPIOD,GPIO_Pin_14);
//				GPIO_ResetBits(GPIOC,GPIO_Pin_6);
//				GPIO_ResetBits(GPIOC,GPIO_Pin_7);
//			
//				TIM3->CCR3 = 100;	
//				TIM3->CCR4 = 100;
			
					NEW_MOTO_ALL_STOP_RIGHT_NOW();
			
        Oled_Show_GPS_Error();
    }
	  if(GPS_is_device_up == 0)
		{
			  Oled_Show_GPS_Error();
			GPS_is_device_up = 50;
		
		}

}

#ifdef DEBUG_GPS

void Oled_Show(void)
{
    static int flag_refresh_page = 0;

    if(flag_refresh_page == 0)
    {
        OLED_ShowNum(0,0,info.satinfo.inuse,2,12);
        OLED_ShowString(12,0,"/",12);
        OLED_ShowNum(16,0,info.satinfo.inview,2,12);

        flag_refresh_page = 1;

    }
    else if(flag_refresh_page == 1)
    {
        OLED_ShowNum(72,0,beiJingTime.hour,2,12);
        OLED_ShowString(84,0,":",12);
        OLED_ShowNum(90,0,beiJingTime.min,2,12);

        //	OLED_ShowString(0,24, "HOT.",12);
        //	OLED_ShowNum(24,24,(u32)Get_Temperature(),6,12);

        flag_refresh_page = 2;

    }

    else if(flag_refresh_page == 2)
    {
        OLED_ShowString(0,12, "BAT.",12);
        OLED_ShowNum(30,12,Ship_Board_Data.ship_voltage ,6,12);
        OLED_ShowString(0,24, "VER.",12);  //增加oled版本号
        flag_refresh_page = 3;

    }
    else if(flag_refresh_page == 3)
    {
        OLED_ShowNum(24,24,(u32)VERSION,6,12);
        OLED_ShowString(0,36,"fact",12);
        OLED_ShowNum(24,36,(u32)distant_normal,6,12);
        flag_refresh_page = 4;
    }

    else if(flag_refresh_page == 4)
    {
        OLED_ShowString(60,36,"cps",12);
        OLED_ShowNum(92,36,(u32)Angle,6,12);
        //		printf("Angle = %d\r\n",(u32)Angle);
        OLED_ShowString(0,48,"test",12);
        flag_refresh_page = 5;
    }


    else if(flag_refresh_page == 5)
    {
        OLED_ShowNum(24,48,(u32)distant_test,6,12);
        OLED_ShowString(60,48,"ang",12);
        OLED_ShowNum(92,48,(u32)(angle_normal),6,12);
        flag_refresh_page = 0;
    }

    OLED_Refresh_Gram();

}

#else

void Oled_Show(void)
{
    static int flag_refresh_page = 0;
	uint32_t BAT_ADC;

	if(GPS_is_device_up == 50)
		return;
	
    if(flag_refresh_page == 0)
    {
        OLED_ShowString(0,0,"GPS",12);
        OLED_ShowNum(16,0,info.satinfo.inuse,2,12);


        flag_refresh_page = 1;
    }
    else if(flag_refresh_page == 1)
    {
        OLED_ShowString(28,0,"/",12);
        OLED_ShowNum(32,0,info.satinfo.inview,2,12);
        OLED_ShowString(60,0,"BAT",12);
        //	OLED_ShowString(64,0,":",12);

        flag_refresh_page = 2;

    }
    else if(flag_refresh_page == 2)
    {
			
        OLED_ShowNum(90,0,(Ship_Board_Data.ship_voltage)/10,2,12);
        OLED_ShowString(102,0,".",12);
			
				OLED_ShowNum(106,0,(Ship_Board_Data.ship_voltage)%10,2,12);

        flag_refresh_page = 3;

    }
    else if(flag_refresh_page == 3)
    {
        OLED_ShowString(0,12, "Lng ",12); //精度 第二行
        OLED_ShowNum(30,12,(u32) sure_longitude,6,12);
//			OLED_ShowString(0,12, "X",12); //精度 第二行
//			OLED_ShowNum(30,12,(u32) DATA_X ,6,12);

        flag_refresh_page = 4;
    }
    else if(flag_refresh_page == 4)
    {

        OLED_ShowString(0,24, "Lat ",12);  //增加oled版本号 //维度 第三行
        OLED_ShowNum(30,24,(u32)sure_dimensionality,6,12);
//			OLED_ShowString(0,24, "Y ",12);  //增加oled版本号 //维度 第三行
//			OLED_ShowNum(30,24,(u32)DATA_Y,6,12);

        flag_refresh_page = 5;

    }
    else if(flag_refresh_page == 5)
    {
			
        OLED_ShowString(0,36,"CA",12);  //第四行
        OLED_ShowNum(24,36,(u32)Angle,6,12);

        //第五行 暂时未使用
				OLED_ShowString(0,48,"B",12);  //第四行
				OLED_ShowNum(24,48,setBeepBlood,6,12); //显示蜂鸣器此时的数据
        flag_refresh_page = 0;
    }

    OLED_Refresh_Gram();


//			USART1_printf(USART1,"ship_voltage = %d\r\n", Ship_Board_Data.ship_voltage);
//			USART1_printf(USART1,"%d\t%d\r\n", Ship_Board_Data.ship_moto_current_1, Ship_Board_Data.ship_moto_current_2);

}

#endif


void check_star_and_beep(void)
{
    static u8 first_lost_star = 0;
    static u8 first_lost_compass = 0;

    //需要加以判断，当不进入中断，是否会refresh angle的角度数据哪？
    if((angle_receive_buf[0] == 0) &&(angle_receive_buf[1] == 0) &&(angle_receive_buf[2] == 0) )
    {
        if(flag_compass_adjust <= 0)
        {
            if(first_lost_compass++ == 100)
            {
                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                //				printf("first_lost_compass ++  == 100\r\n");
            }

            if(first_lost_compass > 200)
                first_lost_compass = 200;
        }

    }
    else
        first_lost_compass = 0;

    //	angle_receive_buf[0] =angle_receive_buf[1] =angle_receive_buf[2] = 0;

    if((info.satinfo.inuse <= 0) || (flag_cannot_new_parse >=500))
    {
        setBeepBlood = SEEK_STAR_NO;
        if(first_lost_star++ == 0)
        {
            //			printf("first_lost_star ++  == 0\r\n");
            Moto_Direction_Turn(MOTO_STOP, 0, 0);
        }

        if(first_lost_star > 10) first_lost_star = 10;

    }
    else
    {
        first_lost_star = 0;
        if(setBeepBlood == SEEK_STAR_NO)
            setBeepBlood = BEEPOFF;
    }


    if(	flag_cancel_seek ==  1)
    {
        if((first_lost_compass >= 100) || (first_lost_star >= 10))
        {
            //		printf("\r\nfirst_lost_compass=%d first_lost_star=%d\r\n",first_lost_compass,first_lost_star);

            flag_cancel_seek = 0;
            Moto_Direction_Turn(MOTO_STOP, 0, 0);


        }
    }
}

void Queue_Beep_On(void)
{
    double  distant_lost_star = 0;
    static u8 last_beep_cmd = 0;
    static u32 flag_set_flag_cancel_seek = 0;

    if((last_beep_cmd == RECEIVE_CMD_NOTICE)&&(setBeepBlood == BEEPOFF))
    {
        if(flag_set_beep_free ++ < 50)
            return;
    }
    if((flag_set_beep_free ++ < 50)&&(setBeepBlood == RECEIVE_CMD_NOTICE)) //		depend on {		if(beepCnt==40) setBeepBlood == BEEPOFF  }
    {
        last_beep_cmd = setBeepBlood;
        return ;
    }

    flag_set_beep_free = 0;
    //gps异常 从有gps到无gps的阶段，是需要立即停止
    check_star_and_beep();
    // 连接

    if(flag_rf_send_error == 5)
    {
        Moto_Direction_Turn( MOTO_STOP, 0, 0);
        //			printf("flag_rf_send_error \r\n");


        setBeepBlood = LOST_CONTROL_BERROR;

        flag_cancel_seek = 0;
    }


    if(flag_rf_send_error >= 6)  //进入休眠  //应为250
    {

        if(flag_rf_send_error == 30)
        {
//						flag_enter_in_mode_wwdg = 1;
//						AS14B_RESET();
//						flag_enter_in_mode_wwdg = 0;
            flag_rf_send_error = 20; //应为260
        }

        setBeepBlood = LOST_CONTROL_BERROR;

        if(info.satinfo.inuse > 0)
        {
            flag_set_flag_cancel_seek++;
        }
        if((flag_cancel_seek == 0) &&(flag_set_flag_cancel_seek >= 1000))
        {
            flag_set_flag_cancel_seek = 1000;
            if((longitude_CH[0] != 0) ||(dimensionality_CH[0] != 0))
            {
//													printf("use AS14B_receive_buf  0\r\n");
                new_longitude =longitude_CH[0];
                new_dimensionality = dimensionality_CH[0];

            }
            else
            {
                //	printf("no set start use auto set \r\n");
                new_longitude = start_longitude_temp;
                new_dimensionality = start_dimensionality_temp;
            }
//													printf("sure_longitude = %f\r\n",sure_longitude);
//													printf("sure_dimensionality = %f\r\n",sure_dimensionality);
//													printf("new_longitude = %f\r\n",new_longitude);
//													printf("new_dimensionality = %f\r\n",new_dimensionality);

            if((new_longitude != 0) ||(new_dimensionality != 0))
            {
                //	get_distance(new_dimensionality,new_longitude ,sure_dimensionality,sure_longitude);
                distant_lost_star = get_distance(new_dimensionality,new_longitude,sure_dimensionality,sure_longitude);
                //				printf("distant_lost_star = %d\r\n",(int)distant_lost_star);
                if(distant_lost_star > 50)
                {

                    flag_cancel_seek = 1;
                    uCmd = USHIPGO;
                    //		setBeepBlood = BCONTROL;
                    //			printf("setBeepBlood = BCONTROL;\r\n");
                    key3Cmd = 1;

                }

            }

        }

    }
    else if(flag_rf_send_error > 5)
    {
        setBeepBlood = LOST_CONTROL_BERROR;
    } else if(flag_rf_send_error < 5)
    {
        flag_set_flag_cancel_seek = 0;
        if(setBeepBlood == LOST_CONTROL_BERROR)
            setBeepBlood = BEEPOFF;
//						printf("flag_heart_beart= %d !!\r\n", flag_heart_beart);
//								printf("setBeepBlood = BEEPOFF;\r\n");
    }

//		if(flag_heart_beart != 0)
//		printf("flag_heart_beart = %d\r\n",flag_heart_beart);
    // 电量

    if((ADC_ConvertedValueLocal[0]* 6.0) < (LOW_VOLTAGE * 1.0))
    {
        //		printf("\r\n ADC_ConvertedValueLocal[0] =%f---\r\n",ADC_ConvertedValueLocal[0]* 6.0);
        setBeepBlood = BEEP_ALL_ON_LOW_POWER;
    }

		
		#ifdef STM32F103VCT6_MCU
//					USART_SendData(UART4, setBeepBlood + 0xf0);
//			while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
		#endif
		
    last_beep_cmd = setBeepBlood;

}


#define  MOTO_ROLL_PWM 80

#define  MOTO_P 2

int AS14B_Get_buff(void)
{
//	u8 flag_rf_link_fail = 0;
//    double distant_get_as14b;
    static int  AS14B_receive_temp = -1;
//    static char AS14B_receive_buf_x, AS14B_receive_buf_y;
    static int flag_control_300ms_timeout = 0;



    if(flag_control_300ms_timeout ++ == 200)
    {
        Moto_Direction_Turn( MOTO_STOP, 0, 0);
        //	printf("\r\n  flag_control_300ms_timeout = %d\r\n",flag_control_300ms_timeout);

    }
    if(flag_control_300ms_timeout >  300)
    {
        flag_control_300ms_timeout = 250;
    }

//			printf("---------------------------------------\r\n");
//		for(i=0;i<(strlen(AS14B_receive_buf));i++)
//		printf("%02x  ",AS14B_receive_buf[i]);
//		printf("\r\n");




    if( (AS14B_receive_temp != AS14B_receive_buf[5])  //收到通讯数据
            &&(AS14B_receive_buf[0] == respond[0])
            &&(AS14B_receive_buf[1] == respond[1])
      )  // 同步码	验证   确保是新的数据  ========  此处需要再次更正 不能保证差值为1  很纳闷======================
    {
        // printf("receive-%d-- receive_buf[5] -%d-\r\n", AS14B_receive_temp, AS14B_receive_buf[5] );
        AS14B_receive_temp = 		AS14B_receive_buf[5]; //同步码


        if(AS14B_receive_buf[2] == 0x00)//遥控方向
        {
            //					printf("receive_buf[6] -%d- receive_buf[7] -%d- receive_buf[8] -%d-\r\n",  AS14B_receive_buf[6] , AS14B_receive_buf[7] , AS14B_receive_buf[8] );
            LED2_TOGGLE;
            if(AS14B_receive_buf[8] == 1)
            {

                Moto_Direction_Turn(MOTO_STOP, 0, 0);

                key3Cmd = 0;
                flag_cancel_seek = 0;
                setBeepBlood = BEEPOFF;
                //			printf("stop---------\r\n");
            }
            else if(AS14B_receive_buf[8] == 0)
            {

                flag_need_change_directions_all_time  = 1;
                respond[6] &= 0xf8;
                respond[6] |= 0x01;

                flag_control_300ms_timeout = 0;


#if 1
#define DATA_X_OFFSET 51
#define DATA_Y_OFFSET 50

                DATA_X =  AS14B_receive_buf[7];
                DATA_Y =  AS14B_receive_buf[6];

                if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6] <= BAR_CENTER_POSITION_MAX )&&( AS14B_receive_buf[7] < BAR_CENTER_POSITION_MAX)&&(  AS14B_receive_buf[7] > BAR_CENTER_POSITION_MIN))
                {
                    Moto_Direction_Turn(MOTO_STOP, 0, 0);
                }
                else if( DATA_Y  > DATA_Y_OFFSET) //50
                {

                    if((DATA_X >= BAR_CENTER_POSITION_MIN )&&(DATA_X <= BAR_CENTER_POSITION_MAX ))
                    {
                        Moto_Direction_Turn( MOTO_GO,  (DATA_Y - 50)* MOTO_P, (DATA_Y - 50)* MOTO_P);
                    }
                    else	if(DATA_X  >= BAR_CENTER_POSITION_MAX) //第一象限
                    {

                        Moto_Direction_Turn( MOTO_RIGHT, (50)* MOTO_P,( 100 - DATA_X   )* MOTO_P);

                    }
                    else if(DATA_X <= BAR_CENTER_POSITION_MIN) //第二象限
                    {

                        Moto_Direction_Turn( MOTO_RIGHT, ( DATA_X )* MOTO_P, (50)* MOTO_P);

                    }

                }
                else if( DATA_Y  >= BAR_CENTER_POSITION_MIN) //40
                {

                    if(DATA_X >= BAR_CENTER_POSITION_MAX )
                    {
                        Moto_Direction_Turn( MOTO_ROLL_LIGHT,  (DATA_X - 50)* MOTO_P, (DATA_X - 50)* MOTO_P);
                    } else	if(DATA_X <= BAR_CENTER_POSITION_MIN )
                    {
                        Moto_Direction_Turn(MOTO_ROLL_RIGHT,  ( 50  - DATA_X)* MOTO_P, (50 - DATA_X)* MOTO_P);
                    }

                }
                else if( DATA_Y  < BAR_CENTER_POSITION_MIN) //40
                {

                    if((DATA_X >= BAR_CENTER_POSITION_MIN )&&(DATA_X <= BAR_CENTER_POSITION_MAX ))
                    {
                        Moto_Direction_Turn( MOTO_BACK,  (50 - DATA_Y  )* MOTO_P, (50 - DATA_Y)* MOTO_P);
                    }
                    else	if(DATA_X  >= BAR_CENTER_POSITION_MAX) //第四象限
                    {

                        Moto_Direction_Turn( MOTO_BACK, (50)* MOTO_P,( 100 - DATA_X   )* MOTO_P);

                    }
                    else if(DATA_X <= BAR_CENTER_POSITION_MIN) //第三象限
                    {

                        Moto_Direction_Turn( MOTO_BACK, ( DATA_X )* MOTO_P, (50)* MOTO_P);

                    }

                }



#else


                if( AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MAX)
                {

                    if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 3:00
                    {

                        Moto_Direction_Turn( MOTO_ROLL_LIGHT,  MOTO_ROLL_PWM, MOTO_ROLL_PWM);

                    }
                    else if(AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MAX )  // 1:30
                        Moto_Direction_Turn( MOTO_RIGHT,  100, 50);
                    else if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MIN ))  // 4:30
                        Moto_Direction_Turn( MOTO_BACK,  100, 50);
                }
                else if( (AS14B_receive_buf[7]  < BAR_CENTER_POSITION_MAX)&&( AS14B_receive_buf[7] > BAR_CENTER_POSITION_MIN))
                {
                    if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 中心
                    {
                        //			printf("\r\n MOTO_STOP \r\n");
                        Moto_Direction_Turn(MOTO_STOP, 0, 0);

                    }
                    else if(AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MAX )  // 0:00
                        Moto_Direction_Turn( MOTO_GO,  100, 100);
                    else if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MIN) )  // 6:00
                        Moto_Direction_Turn( MOTO_BACK,  100, 100);
                }
                else if( AS14B_receive_buf[7] <= BAR_CENTER_POSITION_MIN)
                {
//						printf("\r\n AS14B_receive_buf[6] == %d \r\n",AS14B_receive_buf[6]);
//						printf("\r\n AS14B_receive_buf[7] == %d \r\n",AS14B_receive_buf[7]);
                    // 最小数值后，要 > 0 的操作
                    if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 3:00
                        Moto_Direction_Turn( MOTO_ROLL_RIGHT,  MOTO_ROLL_PWM, MOTO_ROLL_PWM);
                    else if(AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MAX )  // 1:30
                        Moto_Direction_Turn( MOTO_RIGHT,  50, 100);
                    else if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MIN) )  // 4:30
                        Moto_Direction_Turn( MOTO_BACK,  50, 100);
                }


#endif




//            //push进入，当控制的时候，增加一个判断的数据点。
//					if((AS14B_receive_buf_x != AS14B_receive_buf[6]) ||(AS14B_receive_buf_y != AS14B_receive_buf[7]))
//					{
//						if((sure_dimensionality != 0)&&(sure_longitude != 0))
//						{
//								distant_get_as14b = get_distance(tail_dimensionality,tail_longitude ,sure_dimensionality,sure_longitude); //当前数值和tail 0 0比较，一定大于50，起始点肯定压
//								if(distant_get_as14b >  20)
//								{
//		//								push_linkstack(lsp,&sure_dimensionality,&sure_longitude);
//										tail_dimensionality = sure_dimensionality;
//										tail_longitude = sure_longitude;
//								}
//						}
//						AS14B_receive_buf_x =  AS14B_receive_buf[6];
//						AS14B_receive_buf_y =  AS14B_receive_buf[7];
//
//					}

            }


            if(AS14B_receive_buf[8] !=  0)
            {
                //	printf("[8]=%d\r\n",AS14B_receive_buf[8]);
                setBeepBlood = RECEIVE_CMD_NOTICE;
                flag_set_beep_free = 0;
            }


            //		printf("[8]=%d\r\n",AS14B_receive_buf[8]);
            switch(AS14B_receive_buf[8]) {

            case NO_CMD:

                break;

            case JOY_STICK_STOP:
                //printf("JOY_STICK_STOP----------------------------\r\n");
                Moto_Direction_Turn( MOTO_STOP, 0, 0);

                break;

            case FISH_HOOK:
                //		printf("FISH_HOOK----------------------------\r\n");

                Fishhook_Storage_Control.flag_hc5_on = 1;
                flag_storage &=0xef;
                break;

            case GAMPASS:

                flag_enter_in_mode_wwdg = 1;
                //		printf("compass----------------------------\r\n");
                Delay_ms(20); //为何迅速变成0
                USART_SendData(USART1,0XC0);  // 校准
                Delay_ms(20);
                USART_SendData(USART1,0XC0);  // 校准
                Delay_ms(20);
                USART_SendData(USART1,0XC0);  // 校准

                flag_compass_adjust  = 5000;
                Moto_Direction_Turn( MOTO_ROLL_LIGHT, 40, 40);

                flag_enter_in_mode_wwdg = 0;

                break;
            //				case AUTO_GO:
            //						Moto_Direction_Turn( MOTO_GO, 100, 100);
            //				break;

            case GO_SET:
                //		printf("GO_SET----------------------------\r\n");


                if((longitude_CH[0] != 0) ||(dimensionality_CH[0] != 0))
                {
                    new_longitude = longitude_CH[0];
                    new_dimensionality = dimensionality_CH[0];
                    flag_cancel_seek = 1;
                    uCmd = USHIPGO;
                    setBeepBlood = BCONTROL;
                    key3Cmd = 1;

//                    USART1_printf(USART1,"GO_CH[0]\r\n");
                }
                else
                    ;
                //		printf("GO_SET  ch [0] = 0\r\n");

                break;

            case 	DATA_REQUIRE:
                Post_GPS_Data_Fresh();
                break;

            case START_GPS:

                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                uCmd = USTARTSET;
                setBeepBlood = BSTART;
                empty_linksatck(lsp);
                respond[6] |= 0x80; //0b00001000;
                //	printf("START_GPS-----------------Moto_Direction_Turn-----------\r\n");
                break;
            case CH1:  //设置终点1
                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                setBeepBlood = BEND;
                longitude_CH[1] = sure_longitude;
                dimensionality_CH[1] = sure_dimensionality;

                respond[6] |= 0x40; //0b10000000;

                //	printf("CH1-----------------Moto_Direction_Turn-----------\r\n");
                break;

            case CH2:  //设置终点2
                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                setBeepBlood = BEND;
                longitude_CH[2] = sure_longitude;
                dimensionality_CH[2] = sure_dimensionality;
                respond[6] |=  0x20;// 0b01000000;

                break;
            case CH3:  //设置终点3
                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                setBeepBlood = BEND;
                longitude_CH[3] = sure_longitude;
                dimensionality_CH[3] = sure_dimensionality;
                respond[6] |= 0x10;//0b00100000;
      
                break;

            case CH4:  //设置终点4
                Moto_Direction_Turn(MOTO_STOP, 0, 0);
                setBeepBlood = BEND;
                longitude_CH[4] = sure_longitude;
                dimensionality_CH[4] = sure_dimensionality;
                respond[6] |= 0x08;//0b00010000;
     
                break;

            case GO_CH:
                //		new_longitude = sure_longitude;
                //		new_dimensionality = sure_dimensionality;

                new_longitude = longitude_CH[(AS14B_receive_buf[4])];
                new_dimensionality = dimensionality_CH[(AS14B_receive_buf[4])];

                flag_cancel_seek = 1;
                uCmd = USHIPGO;
                setBeepBlood = BCONTROL;
                key3Cmd = 1;
            
                break;
            case FEED_BIN_D:
                //				printf("FEED_BIN_D----------------------------\r\n");
                Fishhook_Storage_Control.flag_hc4_on = 1;
                flag_storage &=0xf7;
                break;
            case FEED_BIN_C:
                //			printf("FEED_BIN_C----------------------------\r\n");
                Fishhook_Storage_Control.flag_hc3_on = 1;
                flag_storage &=0xfb;
                break;
            case FEED_BIN_B:
                //		printf("FEED_BIN_B----------------------------\r\n");
                Fishhook_Storage_Control.flag_hc2_on = 1;
                flag_storage &=0xfd;
                break;
            case FEED_BIN_A:
                //		printf("FEED_BIN_A----------------------------\r\n");
                Fishhook_Storage_Control.flag_hc1_on = 1;
                flag_storage &=0xfe;

                break;
            case WITH_LIGHT:
                flag_WITH_LIGHT = !flag_WITH_LIGHT;
                flag_HEAD_TAIL_LIGHT = 0;
                flag_ALL_LIGHT = 0;
                chooseLED(WITH_LIGHT, flag_WITH_LIGHT );

                break;
            case HEAD_TAIL_LIGHT:
                flag_HEAD_TAIL_LIGHT = !flag_HEAD_TAIL_LIGHT;
                flag_WITH_LIGHT = 0;
                flag_ALL_LIGHT = 0;
                chooseLED(HEAD_TAIL_LIGHT, flag_HEAD_TAIL_LIGHT);

                break;
            case ALL_LIGHT:
                flag_ALL_LIGHT  =  !flag_ALL_LIGHT;
                flag_HEAD_TAIL_LIGHT = 0;
                flag_WITH_LIGHT = 0;
                chooseLED(ALL_LIGHT, flag_ALL_LIGHT);
                break;

            case DATA_START_POIINT_REQUIRE:
                Post_GPS_Data(dimensionality_CH[0],longitude_CH[0],0x0);
                break;
            case DATA_A_POIINT_REQUIRE:
                Post_GPS_Data(dimensionality_CH[1],longitude_CH[1],0x1);
                break;
            case DATA_B_POIINT_REQUIRE:
                Post_GPS_Data(dimensionality_CH[2],longitude_CH[2],0x2);
                break;

            case DATA_C_POIINT_REQUIRE:
                Post_GPS_Data(dimensionality_CH[3],longitude_CH[3],0x3);
                break;
            case DATA_D_POIINT_REQUIRE:
                Post_GPS_Data(dimensionality_CH[4],longitude_CH[4],0x4);
                break;

            default:
                ;

            };


        }
        else if(AS14B_receive_buf[2] == 0x02)//心跳
        {
//					printf("heart beart--------\r\n");
            LED2_TOGGLE;


        }	else	if(AS14B_receive_buf[2] == 0x04)//设置GPS信息
        {
            //		printf("AS14B_receive_buf[19]  = %d",AS14B_receive_buf[19] );
            if(AS14B_receive_buf[19] == 0x06)
            {
                for(i = 0; i < 8; i ++)
                {
                    GPS_dimensionality.data[i] = AS14B_receive_buf[3+i];
                    GPS_longituder.data[i]= AS14B_receive_buf[11+i];
                }
                //			printf("\r\nset ch [%d]\r\n",AS14B_receive_buf[20]);
                dimensionality_CH[AS14B_receive_buf[20]] = GPS_dimensionality.d;
                longitude_CH[AS14B_receive_buf[20]] = GPS_longituder.d;

                respond[6] |= (0x80 >>(AS14B_receive_buf[20] ));
//								USART1_printf(USART1,"ip_ch%d:[%f][%f]\r\n",AS14B_receive_buf[20],longitude_CH[AS14B_receive_buf[20]],dimensionality_CH[AS14B_receive_buf[20]]);
            }
        }

        flag_rf_send_error = 0;

        flag_heart_beart = 1;


        if(setBeepBlood == LOST_CONTROL_BERROR)
            setBeepBlood = BEEPOFF;
        //		memset();
//			memset(AS14B_receive_buf, 'f', sizeof(AS14B_receive_buf) );

//		Respond_Back();
    }
    else if((AS14B_receive_buf[0] == respond[0])
            &&(AS14B_receive_buf[1] == respond[1]))
    {

        if(AS14B_receive_buf[2] == 0x04)//设置GPS信息
        {
            //	printf("AS14B_receive_buf[19]  = %d",AS14B_receive_buf[19] );
            if(AS14B_receive_buf[19] == 0x06)
            {
                for(i = 0; i < 8; i ++)
                {
                    GPS_dimensionality.data[i] = AS14B_receive_buf[3+i];
                    GPS_longituder.data[i]= AS14B_receive_buf[11+i];
                }
                //		printf("\r\nset ch [%d]\r\n",AS14B_receive_buf[20]);
                dimensionality_CH[AS14B_receive_buf[20]] = GPS_dimensionality.d;
                longitude_CH[AS14B_receive_buf[20]] = GPS_longituder.d;
                respond[6] |= (0x80 >>(AS14B_receive_buf[20] ));
//                USART1_printf(USART1,"ip_ch%d:[%f][%f]\r\n",AS14B_receive_buf[20],longitude_CH[AS14B_receive_buf[20]],dimensionality_CH[AS14B_receive_buf[20]]);
            }
        }
        flag_rf_send_error = 0;

        flag_heart_beart = 1;


        if(setBeepBlood == LOST_CONTROL_BERROR)
            setBeepBlood = BEEPOFF;
    }


//	if(flag_rf_link_fail == 0) // 1225 应该删除
//		flag_heart_beart = 1;

    rfrxFlag = 0;
    return 0;
}

// 可能存在old 和new的不匹配，需要判断是否异常   09 03 li

void Go_Back_By_Trail(void) // 注意单位的转换
{
    double distant_by_trail;
    static double distant_test_temp;
    int ret;


    if(flag_go_back_by_trail == 1) //遥控器收到数据之后，改变标志位
    {
        //		printf("flag_go_back_by_trail == %d -------- pop_linkstack---1--\r\n", flag_go_back_by_trail);
        flag_go_back_by_trail = 2; //位置需要放置在此处

        ret  = pop_linkstack(lsp,&new_dimensionality,&new_longitude); // 应是为他
        //			printf("flag_go_back_by_trail == 1 -------- pop_linkstack------ret = %d---\r\n", ret);
        //		printf("\r\n pop_linkstack new_dimensionality  = %lf   new_longitude  = %lf -----ret = %d---\r\n",new_dimensionality, new_longitude, ret);

        flag_cancel_seek = 1;
        uCmd = USHIPGO;
        //	setBeepBlood = BCONTROL;
        key3Cmd = 1;

        if(ret < 0)
        {
            flag_go_back_by_trail = 0;
            tail_dimensionality = sure_dimensionality;
            tail_longitude = sure_longitude;

            Moto_Direction_Turn(MOTO_STOP, 0, 0);
            //printf("Go_Back_By_Trail\r\n");
            flag_cancel_seek = 0;
            setBeepBlood = BEEPOFF;
            uCmd = USHIPCANCEL;
            key3Cmd = 0;
        }
        //		printf("flag_go_back_by_trail == %d -------- pop_linkstack---2--\r\n", flag_go_back_by_trail);
    }
    else if(flag_go_back_by_trail == 2) //暂时是空闲状态
    {
        if(distant_test_temp != distant_test)
        {
            if(distant_test < 10)
            {
                flag_go_back_by_trail = 1;
            }
            distant_test_temp = distant_test;
        }
    }
    else
    {

        if(RefreshCnt == 50) // 判断是否200ms，进行一次判断是否到达间隔距离
        {

            if((sure_dimensionality != 0)&&(sure_longitude != 0))
            {
                distant_by_trail = get_distance(tail_dimensionality,tail_longitude,sure_dimensionality,sure_longitude);  //当前数值和tail 0 0比较，一定大于50，起始点肯定压
                if(distant_by_trail >  40)
                {
                    push_linkstack(lsp,&sure_dimensionality,&sure_longitude);
                    tail_dimensionality = sure_dimensionality;
                    tail_longitude = sure_longitude;
                }
            }
        }
    }
}



void AS14B_RESET(void)
{

    unsigned short int RESPOND_ADDR[2];

    STMFLASH_Read(0X0803ff00,RESPOND_ADDR, 2);

    respond[0] =(uint8_t) RESPOND_ADDR[0] ;
    respond[1] =(uint8_t)	RESPOND_ADDR[1] ;

    write_ch_rf((((u8)(RESPOND_ADDR[0]))&(~0x80)) -5);
    Delay_ms(15);
    write_address_rf((u8)(RESPOND_ADDR[1]),(u8)(RESPOND_ADDR[0]));
}


int AS14B_Get_ADDR(void)
{

    //	static int  AS14B_receive_temp = -1;
    int timeout = 0;

    unsigned short int RESPOND_ADDR[2];

    while(1)
    {


        if(AS14B_Get_Buff_Before() == 100)
            break;

        while(!flag_4ms_set);  //如果时间不足4ms，则再此处阻塞。
        flag_4ms_set = 0;
        Timer_In_Function();

        if(timeout++ > 50)
        {

            STMFLASH_Read(0X0801ff00,RESPOND_ADDR, 2);

            respond[0] =(uint8_t) RESPOND_ADDR[0] ;
            respond[1] =(uint8_t)	RESPOND_ADDR[1] ;

            write_ch_rf((((u8)(RESPOND_ADDR[0]))&(~0x80)) -5);
            Delay_ms(15);
            write_address_rf((u8)(RESPOND_ADDR[1]),(u8)(RESPOND_ADDR[0]));

//            printf("\r\nGet Addr FAIL !!! TIME OUT and use old addr  -%2x-%2x-\r\n",respond[0], respond[1] );
            break;
        }

    }

    return 0;
}


void Control_Led_Toggle(void )
{
    static 	int RIGHT_led_toggle_time = 0, LEFT_led_toggle_time = 0,   WIDTH_led_toggle_time = 0, ALL_led_toggle_time = 0;
    static 	int flag_RIGHT_led = 0, flag_LEFT_led = 0,   flag_WIDTH_led = 0,flag_ALL_led = 0;

	
	
    if(moto_control_led_mode & MOTO_CONTROL_LED_WIDTH)
    {
        if((WIDTH_led_toggle_time++) * 4  > flag_WIDTH_led_toggle_time)
        {
            flag_WIDTH_led = !flag_WIDTH_led;
            //	LEDC5(flag_WIDTH_led);
            LEDC2(flag_WIDTH_led);
            WIDTH_led_toggle_time = 0;
        }
    }else if(moto_control_led_mode & MOTO_CONTROL_LED_RIGHT )
    {
        if((RIGHT_led_toggle_time++) * 4  > flag_RIGHT_led_toggle_time)
        {
            flag_RIGHT_led = !flag_RIGHT_led;
            LEDC1(flag_RIGHT_led);
            LEDC2(0);
            LEDC6(flag_RIGHT_led);
            LEDC5(0);
            LEDC7(0);
            LEDC8(flag_RIGHT_led);
						LEDC9(0);

            RIGHT_led_toggle_time = 0;
        }
    }else if(moto_control_led_mode & MOTO_CONTROL_LED_LEFT)
    {
        if((LEFT_led_toggle_time++) * 4  > flag_LEFT_led_toggle_time)
        {
            flag_LEFT_led = !flag_LEFT_led;
            LEDC1(0);
            LEDC2(flag_LEFT_led);
            LEDC6(0);
            LEDC5(flag_LEFT_led);
            LEDC7(flag_LEFT_led);
            LEDC8(0);
						LEDC9(0);
            LEFT_led_toggle_time = 0;
        }
    }else if(MOTO_CONTROL_LED_STOP == moto_control_led_mode)
    {

        WIDTH_led_toggle_time = 0;
        RIGHT_led_toggle_time = 0;
        LEFT_led_toggle_time = 0;


        if(flag_WITH_LIGHT != 0)
            chooseLED(WITH_LIGHT, flag_WITH_LIGHT );
        else if(flag_HEAD_TAIL_LIGHT != 0)
            chooseLED(HEAD_TAIL_LIGHT, flag_HEAD_TAIL_LIGHT);
        else if(flag_ALL_LIGHT != 0)
            chooseLED(ALL_LIGHT, flag_ALL_LIGHT);
        else
        {
            LEDC_ALL(0);
        }

        moto_control_led_mode = MOTO_CONTROL_LED_STAND_BY;
    }else if((GPS_is_device_up == 50)&&(moto_control_led_mode == MOTO_CONTROL_LED_STAND_BY))
		{
			
			if((ALL_led_toggle_time++) * 4  > flag_ALL_led_toggle_time)
        {
            flag_ALL_led = !flag_ALL_led;
					
						LEDC_ALL(flag_ALL_led);
        
            ALL_led_toggle_time = 0;
        }
		
		
		}

}


void  Check_our_Board_and_warnning(void)
{

    int flag_AS14B_OK_times = 150,flag_COMPASS_OK_times = 150;
    int time_outs = 0;

    while(1)
    {

        //	printf("angle_receive_buf = 0x%02x, 0x%02x, 0x%02x,\r\n",angle_receive_buf[0],angle_receive_buf[1],angle_receive_buf[2] );
        if((angle_receive_buf[0] >= 0x30) &&(angle_receive_buf[1] >= 0x30) &&(angle_receive_buf[2] >= 0x30))
            flag_COMPASS_OK_times ++;
        else
            flag_COMPASS_OK_times --;


        if((flag_COMPASS_OK_times >0) &&(flag_AS14B_OK_times > 0))
            ;
        else
        {

            if(flag_COMPASS_OK_times <= 0)
                Oled_Show_GOMPASS_Error();

//            printf("flag_COMPASS_OK_times = %d\r\n",flag_COMPASS_OK_times);
//            printf("flag_AS14B_OK_times = %d\r\n",flag_AS14B_OK_times);
            while(1)
            {
                LEDC_ALL(1);
                Delay_ms(200);
                LEDC_ALL(0);
                Delay_ms(200);
            }
        }

        USART_SendData(USART1,0x31);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        while(!flag_4ms_set);  //如果时间不足4ms，则再此处阻塞。
        flag_4ms_set = 0;
        Timer_In_Function();

        if(time_outs++ > 200)
        {
            break;
        }
    }

//    printf("Check_our_Board Over and NO warnning  GOOD LUCK\r\n");
}

//      1225 ADD code


void Board_Data_Refresh(void)
{
    static u16 flag_start_check_voltage = 0;
    static u8 static_ship_status_last;

    Ship_Board_Data.ship_addr[0] = respond[0];
    Ship_Board_Data.ship_addr[1] = respond[1];

    Ship_Board_Data.ship_dimensionality = sure_dimensionality;
    Ship_Board_Data.ship_longitude = sure_longitude;
    Ship_Board_Data.ship_error_data = 0;
    Ship_Board_Data.ship_gps_seek_star = info.satinfo.inview;
    Ship_Board_Data.ship_gps_use_star = info.satinfo.inuse;
    Ship_Board_Data.ship_moto_current_1 = 0;
    Ship_Board_Data.ship_moto_current_2 = 0;
    Ship_Board_Data.ship_status_now = respond[6];

    Ship_Board_Data.ship_storage = flag_storage;
    Ship_Board_Data.ship_temperature = (u8)(Get_Temperature() + 100);

    Ship_Board_Data.ship_moto_current_1 = (u8)(ADC_ConvertedValueLocal[1] * 1000);
    Ship_Board_Data.ship_moto_current_2 = (u8)(ADC_ConvertedValueLocal[2] * 1000);


//			printf("dim = %f\r\n",Ship_Board_Data.ship_dimensionality);
//	if(Ship_Board_Data.ship_dimensionality == 0)
//		printf("error :Ship_Board_Data.ship_dimensionality == 0\r\n");
//
//				printf("lon = %f\r\n",Ship_Board_Data.ship_longitude);
//	if(Ship_Board_Data.ship_longitude == 0)
//		printf("error :Ship_Board_Data.ship_longitude == 0\r\n");

    if(flag_start_check_voltage ++ < 1000)
        Ship_Board_Data.ship_voltage = (u8)( 168);
    else
    {
        Ship_Board_Data.ship_voltage = (u8)(ADC_ConvertedValueLocal[0] * 60 + 6);
        flag_start_check_voltage = 1100;
    }


    Ship_Board_Data.ship_angle	=	Angle;
//
//		printf("angle = %d\r\n",Ship_Board_Data.ship_angle);
//	if(Ship_Board_Data.ship_angle == 0)
//		printf("error :Ship_Board_Data.ship_angle == 0\r\n");


    if(Ship_Board_Data.ship_status_now != static_ship_status_last)
    {
//			printf("Ship_Board_Data.ship_status_now = %02x\r\n",Ship_Board_Data.ship_status_now);
        static_ship_status_last = Ship_Board_Data.ship_status_now;
    }

}


void Post_GPS_Data_Fresh(void)
{
//	if(flag_rf_send_error < 2)
    Post_GPS_Data(sure_dimensionality,sure_longitude,0x5);


    //	for(i = 0; i < 5; i++)
    //		printf("ch%d;longitude =%lf, dimensionality_CH =%lf\r\n",i,longitude_CH[i],dimensionality_CH[i]);
}



u8 AS14B_Get_Buff_Before(void)  // rf_data_decode
{

    static u8 fla_get_addr = 0;
    u8 i,stepNum,buf[8],num,state;
    u8 a;
    unsigned short int RESPOND_ADDR[2];

    stepNum = 0;
    state = 0;

    if(UART3_CODE[0] > 10)
    {
//		printf("\r\n");
//		for(i=0;i<(UART3_CODE[0]+1);i++)
//		printf("%02x  ",UART3_CODE[i]);
//		printf("\r\n");

        for(i=0; i<(UART3_CODE[0]+1); i++)
        {
            if(!stepNum)
            {
checkFirst:
                if(UART3_CODE[i+1] == 0x3D)
                {
                    stepNum ++;
                }
                else if((UART3_CODE[i+1] == 0x75)&&((UART3_CODE[0]-i) >= 10) && (UART3_CODE[i+12] == 0x0A)) //rx end
                {
                    state = 0xff;

                    for(a=0; a<11; a++)
                    {
                        AS14B_receive_buf[a] = UART3_CODE[i+1+a+1];
                      //  printf("-%02x  ",AS14B_receive_buf[a] );
                    }

                   // printf("\r\n");
                    AS14B_Get_buff(); //无线获取命令及数据
                    i = i+ 11;
                    //----------------------------------------------------------------------------------------------//
                }
                else if((UART3_CODE[i+1] == 0x75)&&((UART3_CODE[0]-i) >= 23) && (UART3_CODE[i+25] == 0x0A)) //rx end
                {
                    state = 0xff;

                    for(a=0; a<24; a++)
                    {
                        AS14B_receive_buf[a] = UART3_CODE[i+1+a+1];
                 //       printf("%02x ",AS14B_receive_buf[a] );
                    }
                 //   printf("\r\n");

                    //					printf("data get ok\r\n");
                    AS14B_Get_buff(); //无线获取命令及数据
                    i = i+ 26;
                    //----------------------------------------------------------------------------------------------//
                }
                else if((UART3_CODE[i+1] == 0x75)&&((UART3_CODE[0]-i) >= 5) && (UART3_CODE[i+8] == 0x0A)) //接收地址 //gps配对数据
                {
                    state = 0xff;

                    for(a=0; a<7; a++)
                    {
                        AS14B_receive_buf[a] = UART3_CODE[i+1+a+1];
                    }
             

                    state = 100;
                    break;
                    //----------------------------------------------------------------------------------------------//
                }
            }
            else if(stepNum)
            {
                if((stepNum < 11) && (UART3_CODE[i+1] == 0x3D))
                {
                    stepNum ++;
                    num = 0;
                }
                else if((stepNum == 11)&&(num < 5))
                {
                    if((UART3_CODE[i+1] == 0x40) && (num > 1))
                    {
                        stepNum ++;
                    }
                    else
                    {
                        buf[num++] = UART3_CODE[i+1];
                    }
                }
                else if((stepNum > 11) && (UART3_CODE[i+1] == 0x40))
                {
                    stepNum ++;
                }
                else   //数据错误，则重新找帧头。
                {
                    stepNum = 0;
                    goto checkFirst;
                }

                if(stepNum == 22)        //cmd end!
                {
                    switch(buf[0])
                    {
                    case 0x00:  //数据发送成功

                        flag_heart_beart = 0;
                        state = 0x01;
                        flag_rf_send_error =0;

                        break;
                    case 0x01:  //数据发送失败
                        state = 0x02;
                        flag_rf_send_error ++;

                        break;
                    case 0x02:  //RF复位，上电完成
                        state = 0x03;


                        break;
                    case 0x03:  //连接模式下，连接丢失警告！
                        state = 0x04;
                        flag_heart_beart++;
                        break;
                    case 0x04:  //功耗模式


                        if(buf[2] == 1)
                        {
                            state = 0x05; //待机模式
                            flag_rf_send_error = 0;
                            flag_heart_beart = 1;
                            flag_rf_stand_by_state = 0;
                        }
                        else
                        {
                            state = 0x06; //进入休眠模式
                            flag_rf_stand_by_state = 1;
                        }

                        break;
                    case 0x05: //操作反馈
                        state = 0x07;
                        break;
                    case 0xfa: //读取返回值
                        state = 0x08;
                        break;
                    default:  //错误数据
                        state = 0x00;
                        break;
                    }
                    stepNum = 0;
                }
            }
        }
    }



    if((state == 100)&&(fla_get_addr == 0))
    {
        //		AS14B_Get_buff() ;//无线获取命令及数据

        fla_get_addr = 1;

        if( (AS14B_receive_buf[2] == 0x03)&&(AS14B_receive_buf[3] == 0xaa)&&(AS14B_receive_buf[4] == 0x55))
        {
            respond[0] = AS14B_receive_buf[0];
            respond[1] = AS14B_receive_buf[1];
//						printf("\r\nGet Addr -%2x-%2x-\r\n", respond[0] , respond[1] );

            RESPOND_ADDR[0] = respond[0];
            RESPOND_ADDR[1] = respond[1];


            Ship_Board_Data.ship_addr[0] = AS14B_receive_buf[0];
            Ship_Board_Data.ship_addr[1] = AS14B_receive_buf[1];

            STMFLASH_Write(0X0801ff00,RESPOND_ADDR,2);


            write_ch_rf((((u8)(RESPOND_ADDR[0]))&(~0x80)) -5);
            Delay_ms(15);
            write_address_rf((u8)(RESPOND_ADDR[1]),(u8)(RESPOND_ADDR[0]));
            // delay8ms(150);  //wait RF reset! //需要延时

            memset(AS14B_receive_buf, 0, sizeof(AS14B_receive_buf) );
        }

//        printf("\r\nGet NEW Addr -%2x-%2x-\r\n", respond[0], respond[1] );

    }
    UART3_CODE[0] = 0;



    return state;


}


void Timer_In_Function(void)
{
    u8 Res;

    if( RXNUM != 0 )
    {
        if(++UART_CHECK_CNT > 2 )
        {
            UART_CHECK_CNT = 3;
            if(!UART3_CODE[0])
            {
                UART3_RX_RESUME_FLAG = TRUE;
                for(Res=1; Res<(UART3_BUF[0]+1); Res++)
                    UART3_CODE[Res] = UART3_BUF[Res];
                UART3_CODE[0] = UART3_BUF[0];
                RXNUM = 0;
            }
        }
    }


}




