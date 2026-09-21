/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	uart.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：
*Description：串口初始化程序，选择定时器 2 作为波特率发射器
*History：
****************************************************************************/
#include "uart.h"
#include <STC8.H>

/*波特兰定义*/
//#define OSCLK 11059200		//定义时钟频率
#define OSCLK 24000000		//定义时钟频率
#define BPS		115200			//定义波特率
/*扩展宏定义*/
#define PRINTF //定义后可以使用printf--开启后需要1.2KROM 和21字节的RAM
#define ISP		 //定义后可以免冷启动

/****************************************************************************
* 名    称：uart1_init()
* 功    能：串口初始化
* 入口参数：无
* 出口参数：无
* 说    明：串口1初始化位“96N81”的格式，选择定时器 2 作为波特率发射器
* 调用方法：无
****************************************************************************/
void uart1_init()
{
	#define BPS_H (65536-OSCLK/BPS/4)>>8
	#define BPS_L (65536-OSCLK/BPS/4)
	SCON = 0x50;
	/*设置波特率*/
//	T2L = BPS_L;		//65536-11059200/115200/4=0FFE8H
//	T2H = BPS_H;
//	AUXR = 0x15;	//启动定时器
    AUXR |= 0x01;	//
	ES = 1; 		  //使能串口中断
#ifdef PRINTF
	TI = 1;
#endif
}

/****************************************************************************
* 名    称：uart1_send_data()
* 功    能：串口1发送一个字节的数据
* 入口参数：_data 需要发送的数据
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
void uart1_send_data(char _data)
{
#ifdef PRINTF
	printf("%c",_data);
#else
	SBUF = _data;    //发送测试数据
	while(!TI);
		TI = 0;
#endif
}

/****************************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
* 调用方法：REMOVEUNUSED
****************************************************************************/
void uart1_interrupt(void) interrupt 4/* using 1*/
{
	if (TI)//发送中断
	{
	#ifndef PRINTF
		TI = 0;   //清中断标志
	#endif
	}
	if (RI)//接收中断
	{
		RI = 0;             //清中断标志
	#ifdef ISP
		IAP_CONTR = 0x60;		//复位到ISP
	#endif
	}
}



bit uart4_tx_busy = 0;
bit uart3_tx_busy = 0;
bit uart2_tx_busy = 0;
xdata unsigned char uart4_rx_buf[100];
xdata unsigned char uart3_rx_buf[100];
xdata unsigned char uart2_rx_buf[100];

xdata unsigned char uart3_tx_buf[100];

data  unsigned char uart4_rx_idle = 0;
data  unsigned char uart3_rx_idle = 0;
data  unsigned char uart2_rx_idle = 0;
data volatile unsigned char uart4_tx_idle = 100;
struct 
{
    unsigned char w_left;
    unsigned char r_free;
    // unsigned char code * wbuf_ptr;
    unsigned char * wbuf_ptr;
    unsigned char xdata * rbuf_ptr;
} uart4_buf;

struct 
{
    unsigned char w_left;
    unsigned char r_free;
    unsigned char xdata * wbuf_ptr;
    unsigned char xdata * rbuf_ptr;
} uart2_buf, uart3_buf;

void timer2forUart_init(void)
{   // timer2初始化(晶振时钟24MHz)  串口2/3/4波特率为115200
    AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
    T2L = 0xCC;		//设定定时初值
    T2H = 0xFF;		//设定定时初值
    AUXR |= 0x10;		//启动定时器2
}
/****************************************************************************
* 名    称：uart2_init()
* 功    能：串口初始化
* 入口参数：无
* 出口参数：无
* 说    明：串口1初始化位“115200N81”的格式，选择定时器 2 作为波特率发射器
* 调用方法：无
****************************************************************************/
void uart2_init()
{   // 115200bps@24.000MHz
    //P_SW2 |= 0x01;   	//RXD2_2/P4.0, TXD2_2/P4.2
    //P_SW2 &= 0xFE;   	//RXD2/P1.0, TXD2/P1.1
    
    timer2forUart_init();  // uart3/4中调用一次即可
    
    S2CON = 0x50;		//8位数据,可变波特率
    AUXR |= 0x04;		//
    
	IE2 |= 0x01;
    
    uart2_buf.w_left = 0;
    uart2_buf.r_free = 100;
    uart2_buf.rbuf_ptr = &uart2_rx_buf[0];
}
/****************************************************************************
* 名    称：uart3_init()
* 功    能：串口初始化
* 入口参数：无
* 出口参数：无
* 说    明：串口1初始化位“9600N81”的格式，选择定时器 2 作为波特率发射器
* 调用方法：无
****************************************************************************/
void uart3_init()
{   // 9600bps@24.000MHz
    P_SW2 |= 0x02;   	//RXD3_2/P5.0, TXD3_2/P5.1
    // P_SW2 &= ~0x02;   	//RXD3/P0.0, TXD3/P0.1
    
    S3CON = 0x10;		//8位数据,可变波特率
    S3CON |= 0x40;		//串口3选择定时器3为波特率发生器
    T4T3M |= 0x02;		//定时器3时钟为Fosc,即1T
    T3L = 0x8F;		//设定定时初值
    T3H = 0xFD;		//设定定时初值
    T4T3M |= 0x08;		//启动定时器3
    
    IE2 |= 0x08;
    
    uart3_buf.w_left = 0;
    uart3_buf.r_free = 100;
    uart3_buf.rbuf_ptr = &uart3_rx_buf[0];
}
/****************************************************************************
* 名    称：uart4_init()
* 功    能：串口初始化
* 入口参数：无
* 出口参数：无
* 说    明：串口1初始化位“115200N81”的格式，选择定时器 2 作为波特率发射器
* 调用方法：无
****************************************************************************/
void uart4_init()
{   // 115200bps@24.000MHz
    P_SW2 |= 0x04;   	//RXD4_2/P5.2, TXD4_2/P5.3
    // P_SW2 &= ~0x04;   	//RXD4/P0.2, TXD4/P0.3
    
    timer2forUart_init();
    
    S4CON = 0x10;		//8位数据,可变波特率
    // S4CON |= 0x40;		//串口4选择定时器4为波特率发生器
    // T4T3M |= 0x20;		//定时器4时钟为Fosc,即1T
    // T4L = 0xCC;		//设定定时初值
    // T4H = 0xFF;		//设定定时初值
    // T4T3M |= 0x80;		//启动定时器4
    S4CON &= 0xBF;		//串口4选择定时器2为波特率发生器
    
	IE2 |= 0x10;
    
    uart4_buf.w_left = 0;
    uart4_buf.r_free = 100;
    uart4_buf.rbuf_ptr = &uart4_rx_buf[0];
}

/****************************************************************************
* 名    称：uart4_send_data()
* 功    能：串口1发送一个字节的数据
* 入口参数：_data 需要发送的数据
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
void uart4_send_data(char _data)
{
    while (uart4_tx_busy);
    uart4_tx_busy = 1;
	S4BUF = _data;    //发送测试数据
}

void uart4_send_array(unsigned char *_array, unsigned char len)
{
    if (len > 0)
    {
        while (uart4_tx_busy);
        uart4_buf.wbuf_ptr = _array + 1;
        uart4_buf.w_left = len - 1;
        while (uart4_tx_idle < 2);
        //uart4_tx_idle = 0;
        //if (!uart4_tx_busy)
        {
            uart4_tx_busy = 1;
            S4BUF = *_array;    //发送数据
        }
    }
}

void uart3_send_array(unsigned char *_array, unsigned char len)
{
    if (len > 0)
    {
        while (uart3_tx_busy);
        uart3_buf.wbuf_ptr = _array + 1;
        uart3_buf.w_left = len - 1;
        //if (!uart3_tx_busy)
        {
            uart3_tx_busy = 1;
            S3BUF = *_array;    //发送数据
        }
    }
}

void uart2_send_array(unsigned char *_array, unsigned char len)
{
    if (len > 0)
    {
        while (uart2_tx_busy);
        uart2_buf.wbuf_ptr = _array + 1;
        uart2_buf.w_left = len - 1;
        //if (!uart2_tx_busy)
        {
            uart2_tx_busy = 1;
            S2BUF = *_array;    //发送数据
        }
    }
}

void uart4_restart_rx(void)
{   // 重新开始接收
    uart4_rx_idle = 0;
    IE2 &= ~0x10;
    uart4_buf.r_free = 100;
    uart4_buf.rbuf_ptr = uart4_rx_buf;
    IE2 |= 0x10;
}

void uart3_restart_rx(void)
{   // 重新开始接收
    uart3_rx_idle = 0;
    IE2 &= ~0x08;
    uart3_buf.r_free = 100;
    uart3_buf.rbuf_ptr = uart3_rx_buf;
    IE2 |= 0x08;
}
void uart2_restart_rx(void)
{   // 重新开始接收
    uart2_rx_idle = 0;
    IE2 &= ~0x01;
    uart2_buf.r_free = 100;
    uart2_buf.rbuf_ptr = uart2_rx_buf;
    IE2 |= 0x01;
}
/****************************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
void uart4_interrupt(void) interrupt 18 using 1
{
    EA = 0;
    if (S4CON & 0x02)// 发送中断
    {
        S4CON &= ~0x02;
        if (uart4_buf.w_left != 0)
        {
            uart4_buf.w_left--;
            S4BUF = *uart4_buf.wbuf_ptr++;    // 发送数据
        }
        else
            uart4_tx_busy = 0;
        
        uart4_tx_idle = 0;
    }
    if (S4CON & 0x01)// 接收中断
    {
        S4CON &= ~0x01;
        if (uart4_buf.r_free == 0)
        {
            uart4_buf.r_free = 100;
            uart4_buf.rbuf_ptr = uart4_rx_buf;
        }
        uart4_buf.r_free--;
        *uart4_buf.rbuf_ptr++ = S4BUF;
        
        uart4_rx_idle = 0;
    }
    EA = 1;
}

void Uart3Isr() interrupt 17
{
    if (S3CON & 0x02)
    {   // 发送中断
        S3CON &= ~0x02;
        if (uart3_buf.w_left != 0)
        {
            uart3_buf.w_left--;
            S3BUF = *uart3_buf.wbuf_ptr++;    // 发送数据
        }
        else
            uart3_tx_busy = 0;
    }
    if (S3CON & 0x01)
    {   // 接收中断
        S3CON &= ~0x01;
        if (uart3_buf.r_free == 0)
        {
            uart3_buf.r_free = 100;
            uart3_buf.rbuf_ptr = uart3_rx_buf;
        }
        uart3_buf.r_free--;
        *uart3_buf.rbuf_ptr++ = S3BUF;
        
        uart3_rx_idle = 0;
    }
}

void Uart2Isr() interrupt 8
{
    if (S2CON & 0x02)
    {
        S2CON &= ~0x02;
        if (uart2_buf.w_left != 0)
        {
            uart2_buf.w_left--;
            S2BUF = *uart2_buf.wbuf_ptr++;    // 发送数据
        }
        else
            uart2_tx_busy = 0;
    }
    if (S2CON & 0x01)
    {
        S2CON &= ~0x01;
        if (uart2_buf.r_free == 0)
        {
            uart2_buf.r_free = 100;
            uart2_buf.rbuf_ptr = uart2_rx_buf;
        }
        uart2_buf.r_free--;
        *uart2_buf.rbuf_ptr++ = S2BUF;
        
        uart2_rx_idle = 0;
    }
}
/*********************************FILE END**********************************/
