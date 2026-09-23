#include "sys.h"

/*
 * 通过用户按键以及复位按键来决定是否使用CAN总线发送数据
 * 默认不使用CAN总线（仅仅单击复位按键，程序不使用CAN总线）
 * 按住用户按键的同时按下复位按键则使用CAN总线
 */
/*
 * ****************************************
 * *																			*
 * *																			*
 * *	2																3		*
 * *																			*
 * *							显示屏										*
 * *																			*
 * *																			*
 * *																			*
 * *								    									*
 * *																			*
 * *																			*
 * *																			*
 * *																			*
 * *																			*
 * *																			*
 * *	1																4		*
 * *																			*
 * *																			*
 * ****************************************
 */


void oled_show(void);

u16 receive_cnt2,receive_cnt3,receive_cnt4,receive_cnt5;//计算成功接收数据帧次数
u16 receive_cnt2_old=0,receive_cnt3_old=0,receive_cnt4_old=0,receive_cnt5_old=0;//计算成功接收数据帧次数
u8 Lidar_1,Lidar_2,Lidar_3,Lidar_4;			//雷达是否接入的标志位
u8 count,sec_count;											//OELD刷新屏幕的计数值   秒值+1的计数值
u32 sec;																//秒
u8 CAN_Send_Data[8];										//CAN发送的数据
u8 oled_flag = 0;												//OLED屏幕刷新标志
u8 data_process_flag2,data_process_flag3,data_process_flag4,data_process_flag5;			//雷达获取到一帧数据的标志位
LidarPointStructDef Dataprocess2[800];	//存放雷达获取相关数据
LidarPointStructDef Dataprocess3[800];
LidarPointStructDef Dataprocess4[800];
LidarPointStructDef Dataprocess5[800];

u16 Dataprocess2_distance[800];				//将获取到的数据缓存，防止被覆盖
u16 Dataprocess3_distance[800];
u16 Dataprocess4_distance[800];
u16 Dataprocess5_distance[800];

u16 data_cnt2 = 0;										//
u16 data_cnt3 = 0;
u16 data_cnt4 = 0;
u16 data_cnt5 = 0;
int range_Left = 0;										//OLED及串口输出的雷达范围
int range_Right = 2;
int angle_Left,angle_Right;						//雷达的角度范围，规定在某个范围内就该用串口发送出去，OLED屏幕也要显示
u8 USART2_Rx_Buf[2820];								//串口接收数据缓冲区
u8 USART3_Rx_Buf[2820];
u8 USART4_Rx_Buf[2820];
u8 USART5_Rx_Buf[2820];
u8 USE_CAN_FLAG = 0;									//CAN使用标志位
u16 Lidar_Distance[4];								//保存串口需要打印的雷达距离
int main(void)
{	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组，即优先级分级个数
		delay_init();//延时初始化
		JTAG_Set(JTAG_SWD_DISABLE); /*关闭JTAG接口*/   
		JTAG_Set(SWD_ENABLE);    /*打开SWD接口 可以利用主板的SWD接口调试*/       
    LED_Init();
		KEY_Init();
		OLED_Init();
		if(KEY == 1)											//如果用户按键是按下的状态
		{
			CAN1_Mode_Init(1,3,2,6,0);      //=====CAN初始化
			USE_CAN_FLAG = 1;
		}
		uart_init(460800);
		uart2_init(230400);//串口接收
		uart3_init(230400);//串口接收
		uart4_init(230400);//串口接收
		uart5_init(230400);//串口接收
    time5_init(); //定时器中断，处理数据
		while(1)
		{	
			if(oled_flag)							//降低OLED屏幕刷新频率
			{
				oled_show();						//OLED显示函数
				oled_flag = 0;
			}
			Lidar1_Data_Deal();				//对串口接收到的数据进行处理（包括帧数据的梳理，CAN的发送）
			Lidar2_Data_Deal();
			Lidar3_Data_Deal();
			Lidar4_Data_Deal();		
		}
}



