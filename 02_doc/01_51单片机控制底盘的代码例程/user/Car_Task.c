/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	car_task.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：2018-08-14
*Description：任务
*History：
****************************************************************************/

/*引入头文件*/
#include <STC8.H>
#include "car_task.h"
#include "Car_Control.h"
#include "common.h"
#include "IR.h"
#include "color.h"
#include "disp.h"

struct trafficLightStructTypedef trafficLight_opevmv = {0, 0};
struct carWorkingStructTypedef carWorking = {0, 0, 0};   // 

unsigned short distance = 2000;  // 超声波测距

extern signed char temperature;  // 读取温度值

void Car_R()//小车右转弯
{
	/*IO_BUZZ = 0;
    MOTOR_GO_adjR
    delay_ms(100);
	MOTOR_GO_R;
	delay_ms(500);
    #ifdef xunji_level_singleH  // 有效信号为高电平
    while ((xunji_port & 0x03) == 0x00);
    #else
    while ((~xunji_port & 0x03) == 0x00);
    #endif
    delay_ms(10);
    //while ((xunji_port) == 0x00);+
    //delay_ms(10);
	IO_BUZZ = 1;*/
    
    if (carWorking.type == 2)  // 1-左转  2-右转  3-左侧移  4-右侧移  5-红绿灯-红灯
    {
        if (carWorking.step == 1)
        {
            carWorking.step = 2;
            carWorking.delay = 100;
            MOTOR_GO_adjR
        }
        else if (carWorking.step == 2)
        {
            carWorking.step = 3;
            carWorking.delay = 500;
            MOTOR_GO_R;
        }
        else if (carWorking.step == 3)
        {
            #ifdef xunji_level_singleH  // 有效信号为高电平
            if ((xunji_port & 0x0F) != 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #else
            if ((~xunji_port & 0x0F) == 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #endif
        }
        else //if (carWorking.step == 4)
        {
            carWorking.step = 0;
            carWorking.type = 0;
            
            IO_BUZZ = 1;
        }
    }
    else
    {
        carWorking.type = 2;
        IO_BUZZ = 0;
        carWorking.step = 1;
        carWorking.delay = 500;
        //MOTOR_GO_F_SLOW
    }
}
void Car_L()//小车左转弯
{
	/*IO_BUZZ = 0;
    MOTOR_GO_adjL
    delay_ms(100);
	MOTOR_GO_L;
	delay_ms(500);
    #ifdef xunji_level_singleH  // 有效信号为高电平
    while ((xunji_port & 0xC0) == 0x00);
    #else
    while ((~xunji_port & 0xC0) == 0x00);
    #endif
    delay_ms(10);
    //while ((xunji_port) == 0x00);
    //delay_ms(10);
	IO_BUZZ = 1;*/
    
    if (carWorking.type == 1)  // 1-左转  2-右转  3-左侧移  4-右侧移  5-红绿灯-红灯
    {
        if (carWorking.step == 1)
        {
            carWorking.step = 2;
            carWorking.delay = 100;
            MOTOR_GO_adjL
        }
        else if (carWorking.step == 2)
        {
            carWorking.step = 3;
            carWorking.delay = 500;
            MOTOR_GO_L;
        }
        else if (carWorking.step == 3)
        {
            #ifdef xunji_level_singleH  // 有效信号为高电平
            if ((xunji_port & 0xF0) != 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #else
            if ((~xunji_port & 0xF0) == 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #endif
        }
        else //if (carWorking.step == 4)
        {
            carWorking.step = 0;
            carWorking.type = 0;
            
            IO_BUZZ = 1;
        }
    }
    else
    {
        carWorking.type = 1;
        IO_BUZZ = 0;
        carWorking.step = 1;
        carWorking.delay = 500;
        //MOTOR_GO_F_SLOW
    }
}
void Car_shift_L()//小车向左平移
{
    /*MOTOR_GO_STOP;
    delay_ms(100);
    IO_BUZZ = 0;
	MOTOR_GO_SHIFT_L;
	delay_ms(3000);
    #ifdef xunji_level_singleH  // 有效信号为高电平
    while ((xunji_port & 0xF0)==0x00);
    delay_ms(10);
    while ((xunji_port & 0xF0)==0x00);
    #else
    while (~xunji_port==0x00);
    delay_ms(10);
    while (~xunji_port==0x00);
    #endif
    delay_ms(10);
	IO_BUZZ = 1;*/
    
    if (carWorking.type == 3)  // 1-左转  2-右转  3-左侧移  4-右侧移  5-红绿灯-红灯
    {
        if (carWorking.step == 1)
        {
            carWorking.step = 2;
            carWorking.delay = 3000;
            
            IO_BUZZ = 0;
            MOTOR_GO_SHIFT_L;
        }
        else if (carWorking.step == 2)
        {
            #ifdef xunji_level_singleH  // 有效信号为高电平
            if ((xunji_port & 0xF0) != 0x00)
            {
                carWorking.step = 3;
                carWorking.delay = 5;
            }
            #else
            if (~xunji_port != 0x00)
            {
                carWorking.step = 3;
                carWorking.delay = 5;
            }
            #endif
        }
        else if (carWorking.step == 3)
        {
            #ifdef xunji_level_singleH  // 有效信号为高电平
            if ((xunji_port & 0x70) != 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #else
            if (~xunji_port != 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #endif
        }
        else //if (carWorking.step == 4)
        {
            carWorking.step = 0;
            carWorking.type = 0;
            
            IO_BUZZ = 1;
        }
    }
    else
    {
        carWorking.type = 3;
        carWorking.step = 1;
        carWorking.delay = 100;
        MOTOR_GO_STOP;
        //MOTOR_GO_F_SLOW
    }
}
void Car_shift_R()//小车向右平移
{
    /*MOTOR_GO_STOP;
    delay_ms(100);
    IO_BUZZ = 0;
	MOTOR_GO_SHIFT_R;
	delay_ms(3000);
    #ifdef xunji_level_singleH  // 有效信号为高电平
    while ((xunji_port & 0x0F)==0x00);
    delay_ms(10);
    while ((xunji_port & 0x0F)==0x00);
    #else
    while (~xunji_port==0x00);
    delay_ms(10);
    while (~xunji_port==0x00);
    #endif
    delay_ms(10);
	IO_BUZZ = 1;*/
    
    if (carWorking.type == 4)  // 1-左转  2-右转  3-左侧移  4-右侧移  5-红绿灯-红灯
    {
        if (carWorking.step == 1)
        {
            carWorking.step = 2;
            carWorking.delay = 3000;
            
            IO_BUZZ = 0;
            MOTOR_GO_SHIFT_R;
        }
        else if (carWorking.step == 2)
        {
            #ifdef xunji_level_singleH  // 有效信号为高电平
            if ((xunji_port & 0x0F) != 0x00)
            {
                carWorking.step = 3;
                carWorking.delay = 5;
            }
            #else
            if (~xunji_port != 0x00)
            {
                carWorking.step = 3;
                carWorking.delay = 5;
            }
            #endif
        }
        else if (carWorking.step == 3)
        {
            #ifdef xunji_level_singleH  // 有效信号为高电平
            if ((xunji_port & 0x0E) != 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #else
            if (~xunji_port != 0x00)
            {
                carWorking.step = 4;
                carWorking.delay = 5;
            }
            #endif
        }
        else //if (carWorking.step == 4)
        {
            carWorking.step = 0;
            carWorking.type = 0;
            
            IO_BUZZ = 1;
        }
    }
    else
    {
        carWorking.type = 4;
        carWorking.step = 1;
        carWorking.delay = 100;
        MOTOR_GO_STOP;
        //MOTOR_GO_F_SLOW
    }
}
/**
 * @brief  路口任务
 * @note   到达指定路口，执行相应的任务
 * @param  _branch: 当前所在路口的路口数
 * @retval 返回路口数，如果有平移，会补偿一个路口
 */
unsigned char All_Branch_Task(unsigned char _branch)//每经过一个路口都会调用此程序
{
    static unsigned char offset = 0;  // 路障平移补偿路口数
	switch(_branch)//所有路口
	{
        case 2:
			Car_L();
			break;
        case 6:
			Car_L();
			break;
        case 9:
			Car_L();
			break;
        case 11:
            // 打开闸道
            uart3_tx_buf[0] = 0x03;
            uart3_tx_buf[1] = 0x01;
            uart3_tx_buf[2] = 0x0A;
            uart3_tx_buf[3] = 0x5C;
            uart3_tx_buf[4] = 0x01;
            uart3_tx_buf[5] = 0x01;
            uart3_tx_buf[6] = 0x02;
            uart3_tx_buf[7] = 0x01;
            uart3_tx_buf[8] = 0x01;
            uart3_tx_buf[9] = 0x62;
            uart3_send_array(uart3_tx_buf, 10);
            uart3_restart_rx();
            delay_ms(200);
            MOTOR_GO_STOP;
            delay_ms(1800);
        
			Car_L();
			break;
        case 12:
            // 关闭闸道
            uart3_tx_buf[8] = 0x00;
            uart3_tx_buf[9] = 0x61;
            uart3_send_array(uart3_tx_buf, 10);
            uart3_restart_rx();
        
            if (distance < 900) // 有路障
            {
                offset++;    // 路障平移路口数补偿
                _branch++;   // 路障平移路口数补偿
                
                // 进入等待平移状态
                carWorking.type = 6;
                carWorking.delay = 100;
            }
			break;
        case 13:
            Car_L();
			break;
        case 14:
            if (offset == 0)
                Car_R();
			break;
        case 15:
			Car_R();
			break;
        case 17:
            // 准备开始检测红绿灯
            Color_Init();//开启颜色检测
			Car_R();
			break;
        case 19:
            // 旋转LED
            MOTOR_GO_STOP;						//停车
            // 打开旋转屏
            uart3_tx_buf[0] = 0x04;
            uart3_tx_buf[8] = 0x01;
            uart3_tx_buf[9] = 0x62;
            uart3_send_array(uart3_tx_buf, 10);
            uart3_restart_rx();
            delay_ms(2000);						//等待开启完成
            write_ir(temperature);              // 向旋转屏发送温度信息
            delay_ms(300);
            MOTOR_GO_F_SLOW
			delay_ms(300);
			Car_L();
			break;
        case 20:
			Car_L();
			break;
        default:
            //delay_ms(300);
            break;
	}
    return _branch;
}
/**
 * @brief  全局任务
 * @note   1.执行路段任务  2.执行特殊任务
 * @param  _branch: 最近一次经过路口的路口数
 * @retval 
 */
void Loop_Task(unsigned char _branch)//这里会一直执行
{
    if (carWorking.type == 0)
    {   // 未执行特殊任务时，进行正常全局任务
        if(_branch >= 17 && _branch < 19)//如果是在第2个路到第3个路口则开启颜色识别
        {   // 注意红绿灯识别需使用 摄像头识别  或  颜色传感器识别
            unsigned char del_err = 0;
            {   // openMV摄像头识别红绿灯
                if (trafficLight_opevmv.light == 'r')
                {
                    MOTOR_GO_STOP;//停车
                    IO_BUZZ = 0;
                    
                    carWorking.type = 5;
                    carWorking.step = 1;
                    carWorking.delay = 500;
                }
                else if (trafficLight_opevmv.light == 'g')
                {
                    //IO_BUZZ = 1;
                }
            }
            if((R_data > G_data+25)&&(R_data>B_data+25)) //红灯
            {   // 颜色传感器识别红绿灯   需要根据实际情况更改比较值
                MOTOR_GO_STOP;//停车
                IO_BUZZ = 0;
                while(1)
                {
                    if((R_data > G_data+25)&&(R_data>B_data+25)) //红灯
                        del_err = 0;
                    else
                        del_err++;
                    if(del_err>200)
                        break;
                }
                IO_BUZZ = 1;
            }
        }
        else if (_branch >= 20)
        {
            MOTOR_GO_STOP;						//停车
            // 关闭旋转屏
            uart3_tx_buf[8] = 0x00;
            uart3_tx_buf[9] = 0x61;
            uart3_send_array(uart3_tx_buf, 10);
            uart3_restart_rx();
            
            while(1);
        }
    }
    else if (carWorking.delay == 0)
    {   // 正在执行特殊任务时，等待当前步骤执行完，再执行下一步骤或退出特殊任务
        switch (carWorking.type)  // 如果有其它特殊任务，可在此添加
        {   // 1-左转  2-右转  3-左侧移  4-右侧移  5-红绿灯-红灯
            case 1:
                {
                    Car_L();
                    break;
                }
            case 2:
                {
                    Car_R();
                    break;
                }
            case 3:
                {
                    Car_shift_L();
                    break;
                }
            case 4:
                {
                    Car_shift_R();
                    break;
                }
            case 5:
                {   // 连续3次未检测到红灯，则退出(摄像头识别专用)
                    if (trafficLight_opevmv.light == 'r'){
                        MOTOR_GO_STOP;//停车
                        carWorking.step = 1;
                        carWorking.delay = 500;
                    }
                    else{ //if (trafficLight_opevmv.light == 'g')
                        if (carWorking.step == 2){
                            carWorking.type = 0;
                            IO_BUZZ = 1;
                        }
                        else{
                            carWorking.step = 2;
                            carWorking.delay = 100;
                        }
                    }
                    break;
                }
            case 6:
                {   // 12-13路口距离路障400mm处开始平移，同时特殊任务自动改为 左侧移
                    if (distance < 400)
                    {
                        MOTOR_GO_F_SLOW
                        delay_ms(100);
                        Car_shift_L();
                    }
                    break;
                }
            case 7:
                {
                    break;
                }
            default:
                {
                    carWorking.type = 0;
                    break;
                }
        }
    }
}

