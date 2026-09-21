/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	main.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：
*Description：红外发射编码程序
*History：
****************************************************************************/

/*引入头文件*/
#include "IR.h"
#include <STC8.H>
#include <intrins.h>
#include "dht11.h"
/*红外发射控制定义*/
sbit control_send=P1^5;

#ifndef IR_DELAY_USER		//里面是红外需要的精确延时,晶振为24M
static void Delay560us()		//@24.000MHz
{
	unsigned char i, j;

	_nop_();
	_nop_();
	i = 18;
	j = 113;
	do
	{
		while (--j);
	} while (--i);
}
static void Delay1693us()		//@24.000MHz
{
	unsigned char i, j;

	_nop_();
	i = 53;
	j = 194;
	do
	{
		while (--j);
	} while (--i);
}
static void Delay9ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();
	_nop_();
	i = 2;
	j = 25;
	k = 128;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}
static void Delay4500us()		//@24.000MHz
{
	unsigned char i, j;

	_nop_();
	_nop_();
	i = 141;
	j = 63;
	do
	{
		while (--j);
	} while (--i);
}
#endif

/****************************************************************************
* 名    称：SendIRdata_BYTE
* 功    能：通过红外发送一个字节数据
* 入口参数：irdata：数据
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
static void SendIRdata_BYTE(unsigned char irdata)
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		//先发送0.56ms的38KHZ红外波（即编码中0.56ms的高电平）
		control_send=0;
		Delay560us();
		
		//停止发送红外信号（即编码中的低电平）
		if(irdata&1)//判断最低位为1还是0。低位先发送！！
		{
			control_send=1;  //1为宽电平，1.693ms
			Delay1693us();
		}
		else 
		{
			control_send=1;  //0为窄电平，0.56ms
			Delay560us();
		}
		irdata=irdata>>1;     
	}
}

/****************************************************************************
* 名    称：SendIRdata
* 功    能：红外发送一个数据
* 入口参数：g_iraddr1：地址
*						p_irdata： 发送的数据
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
static void SendIRdata(unsigned char g_iraddr1,unsigned char p_irdata)
{
  //发送9ms的起始码，高电平有38KHZ载波
	control_send=0; 
  Delay9ms();
	
  //发送4.5ms的结果码，低电平无38KHZ载波
  control_send=1; 
	Delay4500us();
	
	//发送地址与地址反码
	SendIRdata_BYTE(g_iraddr1);
	SendIRdata_BYTE(~g_iraddr1);
	
  //发送8位数据与数据反码
	SendIRdata_BYTE(p_irdata);
	SendIRdata_BYTE(~p_irdata);
   
  control_send=0;
	Delay560us();
	control_send=1;
} 

/****************************************************************************
* 名    称：Multiple_sendIRdata
* 功    能：
* 入口参数：count  ：    发送的次数
*						ir_adress:   红外数据接收/发送的地址
*						dat：        发送的数据
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
static void Multiple_sendIRdata(unsigned char count,unsigned char ir_adress,unsigned char dat)
{
	unsigned char i;
	for(i=0;i<count;i++)
	{
		EA = 0;  //禁用中断
		SendIRdata(ir_adress,dat);//发送数据
		EA = 1;	 //恢复中断
		Delay9ms();
	}
} 
/****************************************************************************
* 名    称：
* 功    能：唯一API ，用户在此函数中修改需要发送的数据
* 入口参数：
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
void write_ir(unsigned char dat)
{
	Multiple_sendIRdata(50,0xf2,dat);//传输温度
}
 
