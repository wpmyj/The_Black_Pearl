
#ifndef __MANAGER_H_
#define __MANAGER_H_


#include "stm32f10x.h"
#include <string.h>
#include "bsp_wwdg.h"


typedef struct {
    u8	ship_addr[2]; //地址
    u8 ship_voltage;  // 电压
    u8 ship_temperature; // 温度
    u8 ship_moto_current_1; // moto1 电流
    u8 ship_moto_current_2;	// moto2 电流
    u8 ship_storage;   // 料仓及鱼钩状态
    u8 ship_status_now;  //目前运行状态
    u8 ship_error_data; // 	船异常状态
    double ship_dimensionality; //维度
    double ship_longitude; // 经度
    u8 	ship_gps_seek_star; //可见卫星
    u8 	ship_gps_use_star; // 搜索到卫星
    u32 	ship_angle;
} Ship_Board_Data_t;



typedef  union result
{
    double d;
    unsigned char data[8];
} union_GPS_data;



enum {
    NO_CMD = 0,
    JOY_STICK_STOP,
    FISH_HOOK,
    FEED_BIN_A,
    FEED_BIN_B,
    FEED_BIN_C,
    FEED_BIN_D,
    START_GPS,
    CH1,
    CH2,
    CH3, // 10
    CH4,
    GO_SET,
    GAMPASS,
    HEAD_TAIL_LIGHT,
    WITH_LIGHT,
    ALL_LIGHT,
    GO_CH,
    DATA_REQUIRE,
    DATA_START_POIINT_REQUIRE,
    DATA_A_POIINT_REQUIRE, //20
    DATA_B_POIINT_REQUIRE,
    DATA_C_POIINT_REQUIRE,
    DATA_D_POIINT_REQUIRE, // 23
    DATA_MAX
};



#define FALSE 0
#define TRUE 1





#define BAR_X_CENTER_POSITION 50  //0x32   max 102  min 0
#define BAR_Y_CENTER_POSITION 51  //0x34   max 102 min 0

#define BAR_CENTER_POSITION_MAX 60  //0x32   max 102  min 0
#define BAR_CENTER_POSITION_MIN 40  //0x34   max 102 min 0

#define BAR_POSITION_MIN 4
#define BAR_POSITION_MAX 96


#define MOTO_CONTROL_LED_WIDTH  0x0
#define MOTO_CONTROL_LED_RIGHT 0x2
#define MOTO_CONTROL_LED_LEFT 0x4
#define MOTO_CONTROL_LED_STOP 0
#define MOTO_CONTROL_LED_STAND_BY 0x8


#define PI 3.1415926
#define EARTH_RADIUS 6378.137


/* Private functions ---------------------------------------------------------*/
void SetSysClockTo72(void);
void sysInitIndictor(void);
void sysIndictor(void);
u8 uKeyscan(void);
void Hcm5883_init_test(void);
void Oled_Show(void);
void  GPS_Num_Get(void);
u8 uControl(void);


//=============================================================================//
void set_start_local(void);
void set_end_local(void);
void check_gps_init(void);
void make_sure_local_to_old(void);
void make_sure_local_to_new(void);
void real_time_distant(void);
void cancel_local_to(void);
void Seek_Route_Check(void);

void get_online_control_l298n(void);
int AS14B_Get_buff(void);
void AS14B_RESET(void);
void Go_Back_By_Trail(void) ;



double radian(double d);
double get_distance(double lat1, double lng1, double lat2, double lng2);

void 	ADC_Get_Data(void);
void Compass_Adjust_0ver(void);

void 	Ship_ALL_Init(void);

void Enter_Stop_Status_GPIO_Init_All_Same(void); // 23uA
void Control_Led_Toggle(void );

int AS14B_Get_ADDR(void);
void  Check_our_Board_and_warnning(void);

void Queue_Beep_On(void);

void Ship_stop_enter_normal_Init(void);

u8 write_address_rf(u8 ad1,u8 ad2);
u8 write_ch_rf(u8 ch);
u8 config_rf_persistent_connection_RX(void);


void Respond_Back(void);

void  GPS_Check_in_using(void);

void	RF_CHECK(void);

void Board_Data_Refresh(void);



u8 AS14B_Get_Buff_Before(void);
void Timer_In_Function(void);


#endif
