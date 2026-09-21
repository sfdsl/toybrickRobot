/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	main.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：2018-08-14
*Description：在第一个十字路口打开闸道
*History：

timer0:
timer1:
timer2: uart2/4
timer3: uart3
timer4: 1ms 时钟滴答

****************************************************************************/

/*引入头文件*/
#include <STC8.H>
#include "car_control.h"
#include "pwm_pca.h"
#include "delay.h"
#include "ir.h"
#include "hc-sr04.h"

#include "uart.h"
#include "timer.h"
#include "disp.h"
#include "user_string.h"

#include "intrins.h"
#include "color.h"

#define KEY4   P54

signed char temperature = 0;  // 温度
signed char humidity = 0;     // 湿度
signed long encode_count_chA = 0;  // 通道A编码器计数值
signed long encode_count_chB = 0;  // 通道B编码器计数值

unsigned char at_com_type = 0;  // AT指令读数据状态

extern unsigned short R_data,G_data,B_data;
/****************************************************************************
* 名    称：main
* 功    能：程序入口
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/ 
void main()
{
    unsigned char t_10s = 10;
	unsigned int dat=0;
    
    unsigned char pre_tick = 0;
    
    xdata unsigned char u4_txBuf[20];
    
    
    P1M0 = 0xC0;
    P1M1 = 0x0;
    // P54口设为输入口
    P5M0 &= ~0x10;
    P5M1 |= 0x10;

    
	uart4_init();
    uart3_init();
    uart2_init();

	EA = 1;						/*  开总中断  */
	car_init();				/* 小车初始化 */
    
	HC_SR04_Init();		/*超声波初始化*/
    
    delay_ms(100);
    
    MOTOR_GO_EN;
    
    delay_ms(10);
    
    // disp_number(temperature);
    disp_number(distance);  // 数码管显示距离
    
    uart4_tx_idle = 10;
    uart4_send_array("AT+EON\r\n", sizeof("AT+EON\r\n"));  // 控制指令不返回OK
    
    delay_ms(10);
    uart4_tx_idle = 10;
    
    Timer4Init();
    tick = 0;
    
    //Color_Init();//开启颜色检测

    // 使能串口2/3接收状态
    uart2_restart_rx();
    uart3_restart_rx();
    
	while(1)
	{
        if (pre_tick != tick)
        {   // 每1ms刷新一次数码管 
            pre_tick = tick;
            disp_isr_call();
            
            if (carWorking.delay != 0)  // 特殊任务计时
                carWorking.delay--;
            
            if (trafficLight_opevmv.tick != 0)  // 摄像头计时
                trafficLight_opevmv.tick--;
            else
                trafficLight_opevmv.light = 0;
        }
        if (tick >= 100)
        {
            tick = 0;

            /*//disp_number(G_data);
            u4_txBuf[0] = 'E';
            u4_txBuf[1] = 'S';
            u4_txBuf[2] = 'P';
            u4_txBuf[3] = 1;
            u4_txBuf[4] = 9;
            u4_txBuf[5] = 6;
            u4_txBuf[6] = R_data >> 8;
            u4_txBuf[7] = R_data;
            u4_txBuf[8] = G_data >> 8;
            u4_txBuf[9] = G_data;
            u4_txBuf[10] = B_data >> 8;
            u4_txBuf[11] = B_data;
            u4_txBuf[12] = 0xF8 + u4_txBuf[6] + u4_txBuf[7] + u4_txBuf[8] + u4_txBuf[9] + u4_txBuf[10] + u4_txBuf[11];
            u4_txBuf[13] = 'E';
            u4_txBuf[14] = 'N';
            u4_txBuf[15] = 'D';
            uart4_send_array(u4_txBuf, 16);*/
            /*if (R_data >= G_data){
                disp_number(R_data - G_data);
            }
            else{
                disp_number(G_data - R_data);
                disp_number_1(10);
            }*/
            
            dat = HC_SR04_Read();//读取超声波距离  每100ms测量一次
            if(dat != -1)//距离采集成功
            {
                distance = dat;
                disp_number(distance);
            }
            t_10s--;
            if (t_10s == 0)
            {
                t_10s = 100;
                uart4_send_array("AT+TEMPERATURE?\r\n", 17);
                // 重新开始接收数据
                uart4_restart_rx();
                at_com_type = 1;  // AT指令读数据类型  1温度  2湿度  3编码器计数值  4速度  5电池电量
            }
        }
        if (uart4_rx_idle == 3)
        {   // 驱动板接收数据解析
            unsigned char rx_len = 100 - uart4_buf.r_free;
            if (rx_len > 6)
            {
                // if (uart4_rx_buf[0] == '+')
                {
                    unsigned char st;
                    switch (at_com_type)
                    {
                        case 1:  // 温度
                            {
                                st = user_array_seek(uart4_rx_buf, rx_len, "+TEMPERATURE:", 13);
                                if (st != 0xFF)
                                {
                                    signed long buf;
                                    if (user_str_DECnumStr2num(uart4_rx_buf + st + 15, &buf) != 0)
                                    {
                                        temperature = buf;
                                        // disp_number(temperature);
                                    }
                                }
                                break;
                            }
                        case 2:  // 湿度
                            {
                                st = user_array_seek(uart4_rx_buf, rx_len, "+HUMIDITY:", 10);
                                if (st != 0xFF)
                                {
                                    signed long buf;
                                    if (user_str_DECnumStr2num(uart4_rx_buf + st + 12, &buf) != 0)
                                    {
                                        humidity = buf;
                                        //disp_number(humidity);
                                    }
                                }
                                break;
                            }
                        case 3:  // 编码器
                            {
                                st = user_array_seek(uart4_rx_buf, rx_len, "+MTENCODER:", 11);
                                if (st != 0xFF)
                                {
                                    signed long buf;
                                    register unsigned char len = user_str_DECnumStr2num(uart4_rx_buf + st + 13, &buf);
                                    if (len != 0)
                                    {   // <channel A>
                                        encode_count_chA = buf;
                                        st = len + st + 1;
                                        len = user_str_DECnumStr2num(uart4_rx_buf + st + 13, &buf);
                                        if (len != 0)
                                        {   // <channel B>
                                            encode_count_chB = buf;
                                        }
                                    }
                                }
                                break;
                            }
                        case 4:  // 速度
                            {   // 需用户自行完善
                                st = user_array_seek(uart4_rx_buf, rx_len, "+MTRPM:", 6);
                                if (st != 0xFF)
                                {   // <channel A>,<channel B>  转/分钟
                                    signed long buf;
                                    register unsigned char len = user_str_DECnumStr2num(uart4_rx_buf + st + 8, &buf);
                                    if (len != 0)
                                    {   // <channel A>
                                    }
                                }
                                break;
                            }
                        case 5:  // 电池电量
                            {   // 需用户自行完善
                                st = user_array_seek(uart4_rx_buf, rx_len, "+BATTERY:", 8);
                                if (st != 0xFF)
                                {   // <电压>,<电流>,<剩余电量>,<节数>,<已消耗电量>
                                    signed long buf;
                                    register unsigned char len = user_str_DECnumStr2num(uart4_rx_buf + st + 10, &buf);
                                    if (len != 0)
                                    {   // <电压>
                                    }
                                }
                                break;
                            }
                        case 6:  // 报警状态
                            {   // 需用户自行编写
                                break;
                            }
                    }
                    at_com_type = 0;
                }
            }
            if (rx_len != 0)
            {   // 重新开始接收数据
                uart4_restart_rx();
            }
        }
        else if (uart4_rx_idle >= 50)
        {
            uart4_rx_idle = 5;
            if (at_com_type != 0)
            {
                at_com_type = 0;
            }
        }
        
		if((distance<300) && //距离小于300mm    如果有路障重新规划路线，则建议将此部分屏蔽掉
            (carWorking.type == 0)) // 非特殊任务时，才执行
		{
			MOTOR_GO_STOP;//停车
		}
		else
		{
			car_FollowLine();//循迹
		}
        
        if (uart3_rx_idle == 5)
        {   // lora通信测试
            unsigned char len = 100 - uart3_buf.r_free;
            if ((len >= 6) && (uart3_rx_buf[4] + 6 <= len))
            {   // 红绿灯命令解析，用户可根据实际情况进行修改
                if (uart3_rx_buf[5] == 'r')
                    disp_number_1(1);
                else if (uart3_rx_buf[5] == 'g')
                    disp_number_1(0);
                else if (uart3_rx_buf[5] == 'y')
                    disp_number_1(2);
                disp_number_2(0);
                disp_number_3(uart3_rx_buf[7] / 10);
                disp_number_4(uart3_rx_buf[7] % 10);
                
                uart3_restart_rx();
            }
        }
        
        if (uart2_rx_idle == 5)
        {   // 摄像头通信测试
            unsigned char len = 100 - uart2_buf.r_free;
            if (len > 17)
            {   // 摄像头检测到红绿灯时，发送以下字符串
                // "traffic light is red"
                // "traffic light is green"
                // "traffic light is yellow"
                if (user_str_cmp(uart2_rx_buf, "traffic light is ") >= 17)
                {   // 
                    if (uart2_rx_buf[17] == 'r')
                    {   // "red"
                        trafficLight_opevmv.light = 'r';
                        //disp_number_1(1);
                    }
                    else if (uart2_rx_buf[17] == 'g')
                    {   // "green"
                        trafficLight_opevmv.light = 'g';
                        //disp_number_1(3);
                    }
                    else if (uart2_rx_buf[17] == 'y')
                    {   // "yellow"
                        trafficLight_opevmv.light = 'y';
                        //disp_number_1(2);
                    }
                    trafficLight_opevmv.tick = 250;
                }
                uart2_restart_rx();
            }
            else
                uart2_restart_rx();
        }
	}
}


