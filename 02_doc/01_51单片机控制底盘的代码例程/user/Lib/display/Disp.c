/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	main.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：
*Description：数码管动态显示驱动程序，需要用到定时器3，用到了595，所以需要包含HC595.h驱动头文件
*History：
****************************************************************************/

/*引入头文件*/
#include "disp.h"
#include <STC8.H>
#include "hc595.h"

/*全局变量申明*/

/*数码管段码*/
unsigned char table[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};

/*动态显存*/
unsigned char disp_Buff[4] = {0};

/*引脚定义*/
sbit DS1 = P4^0;
sbit DS2 = P4^1;
sbit DS3 = P4^2;
sbit DS4 = P4^3;

/****************************************************************************
* 名    称：disp_bit
* 功    能：显示一个数
* 入口参数：_dat 显示数据
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
void disp_number(unsigned int _dat)
{
	disp_Buff[0] = table[_dat %10000/1000];
	disp_Buff[1] = table[_dat %1000/100];
	disp_Buff[2] = table[_dat %100/10];
	disp_Buff[3] = table[_dat %10];
}

/**
 * @brief  显示单位数据
 * @note   
 * @param  _dat: 显示列表中对应的下标
 * @retval None
 */
void disp_number_1(unsigned char _dat)
{
	disp_Buff[0] = table[_dat];
}
void disp_number_2(unsigned char _dat)
{
	disp_Buff[1] = table[_dat];
}
void disp_number_3(unsigned char _dat)
{
	disp_Buff[2] = table[_dat];
}
void disp_number_4(unsigned char _dat)
{
	disp_Buff[3] = table[_dat];
}
/****************************************************************************
* 名    称：disp_bit
* 功    能：显示一位
* 入口参数：_wei 位码 _dat 段码
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
static void disp_bit(unsigned char _wei ,unsigned char _dat)
{
    DS1 = 1;
    DS2 = 1;
    DS3 = 1;
    DS4 = 1;
    HC595_Send_A(_dat);
	switch(_wei)
	{
		case 0:
			DS1 = 0;
			break;
		case 1:
			DS2 = 0;
			break;
		case 2:
			DS3 = 0;
			break;
		case 3:
			DS4 = 0;
			break;
	}
}

/****************************************************************************
* 名    称：
* 功    能：中断调用函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无需用户调用
****************************************************************************/
void disp_isr_call()
{
	static unsigned char wei=0;
	disp_bit(wei,disp_Buff[wei]);
    wei++;
    wei &= 0x03;
}





