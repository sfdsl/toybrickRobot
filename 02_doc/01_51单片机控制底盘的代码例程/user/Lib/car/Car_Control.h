#ifndef __Car_Control_H
#define __Car_Control_H

//---包含头文件---//
#include "common.h"
#include "uart.h"

void car_init(void) ;				//小车初始化程序
void Car_test(void) ;				//测试小车，顺序为：前后右左
void car_FollowLine(void);	//小车循迹

/*直流电机管脚配置*/
sbit MOTOR_A_EN  = P1^0;
sbit MOTOR_A_CON = P1^1;
sbit MOTOR_B_EN  = P1^2;
sbit MOTOR_B_CON = P1^3;

/*循迹端口*/
#define  xunji_level_singleH  // 高电平为检测到信号
#define  xunji_port  P0

extern unsigned char moto_run_type;

#define car_run_enable_at_cmd   "AT+MT=ON,ON,ON,ON\r\n"                                                                                                             

#define car_stop_at_cmd   "AT+MT_STOP\r\n"
#define car_go_b_at_cmd   "AT+MT_SPWM=80,80,80,80\r\n"
#define car_go_f_at_cmd   "AT+MT_SPWM=-80,-80,-80,-80\r\n"
#define car_go_r_at_cmd   "AT+MT_SPWM=-50,50,-50,50\r\n"
#define car_go_l_at_cmd   "AT+MT_SPWM=50,-50,50,-50\r\n"

#define car_go_bslow_at_cmd   "AT+MT_SPWM=40,40,40,40\r\n"
#define car_go_fslow_at_cmd   "AT+MT_SPWM=-40,-40,-40,-40\r\n"

#define car_go_adjr_at_cmd   "AT+MT_SPWM=0,50,0,50\r\n"
#define car_go_adjl_at_cmd   "AT+MT_SPWM=50,0,50,0\r\n"

#define car_go_shift_l_at_cmd   "AT+MT_SPWM=40,-40,-40,40\r\n"  // 
#define car_go_shift_r_at_cmd   "AT+MT_SPWM=-40,40,40,-40\r\n"  // 

/*声明函数*/
#define MOTOR_GO_EN   		uart4_send_array(car_run_enable_at_cmd, sizeof(car_run_enable_at_cmd));  // 使能电机
#define MOTOR_GO_F   		if (moto_run_type != 1){moto_run_type = 1; uart4_send_array(car_go_f_at_cmd, sizeof(car_go_f_at_cmd));}//车体前进	                            
#define MOTOR_GO_BACK	    if (moto_run_type != 2){moto_run_type = 2; uart4_send_array(car_go_b_at_cmd, sizeof(car_go_b_at_cmd));}//车体后退
#define MOTOR_GO_R	     	if (moto_run_type != 3){moto_run_type = 3; uart4_send_array(car_go_r_at_cmd, sizeof(car_go_r_at_cmd));}//车体右转
#define MOTOR_GO_L	    	if (moto_run_type != 4){moto_run_type = 4; uart4_send_array(car_go_l_at_cmd, sizeof(car_go_l_at_cmd));}//车体左转

#define MOTOR_GO_F_SLOW   		if (moto_run_type != 9){moto_run_type = 9; uart4_send_array(car_go_fslow_at_cmd, sizeof(car_go_fslow_at_cmd));}//车体前进  (低速)
#define MOTOR_GO_BACK_SLOW	    if (moto_run_type != 10){moto_run_type = 10; uart4_send_array(car_go_bslow_at_cmd, sizeof(car_go_bslow_at_cmd));}//车体后退  (低速)

#define MOTOR_GO_adjR	     	if (moto_run_type != 7){moto_run_type = 7; uart4_send_array(car_go_adjr_at_cmd, sizeof(car_go_adjr_at_cmd));}//车体右转  (主要用于直行校正 转弯前导)
#define MOTOR_GO_adjL	    	if (moto_run_type != 8){moto_run_type = 8; uart4_send_array(car_go_adjl_at_cmd, sizeof(car_go_adjl_at_cmd));}//车体左转  (主要用于直行校正 转弯前导)

#define MOTOR_GO_STOP	    if (moto_run_type != 0){moto_run_type = 0; uart4_send_array(car_stop_at_cmd, sizeof(car_stop_at_cmd));}	//车体停止

#define MOTOR_GO_SHIFT_R	     	if (moto_run_type != 5){moto_run_type = 5; uart4_send_array(car_go_shift_r_at_cmd, sizeof(car_go_shift_r_at_cmd));}//车体向右平移
#define MOTOR_GO_SHIFT_L	    	if (moto_run_type != 6){moto_run_type = 6; uart4_send_array(car_go_shift_l_at_cmd, sizeof(car_go_shift_l_at_cmd));}//车体向左平移

// extern unsigned int T_branch;//第几个路口

struct trafficLightStructTypedef  // opevMV摄像头结构体
{
    unsigned char tick;    // 用于长时间未检测到红绿灯时使用
    unsigned char light;   // 红绿灯颜色  0长时间未检测到红绿灯  'r'红灯  'g'绿灯  'y'黄灯
};
extern struct trafficLightStructTypedef trafficLight_opevmv;

struct carWorkingStructTypedef  // 特殊任务结构体
{
    unsigned char type;    // 特殊任务类型 1-左转  2-右转  3-左侧移  4-右侧移  5-红绿灯-红灯  6-路障向左移
    unsigned char step;    // 特殊任务当前所执行步骤编号
    unsigned short delay;  // 当前步骤执行时间  单位ms
};
extern struct carWorkingStructTypedef carWorking;

extern unsigned short distance;  // 超声波测距

#endif 