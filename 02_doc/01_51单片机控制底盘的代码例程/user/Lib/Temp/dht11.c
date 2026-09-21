/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	main.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：
*Description：下载时晶振选择24M频率，传感器DATA引脚接P3.2
*History：
****************************************************************************/

/*引入头文件*/
#include "dht11.h"
#include <STC8.H>
#include <intrins.h>
/*定义IO口*/
sbit DHT11_OUT = P1^4;

/*全局变量*/
static unsigned int OverTime_flag; //超时检测

/****************************************************************************
* 名    称：
* 功    能：精确延时函数
* 入口参数：
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
static void Delay_10us()		//@24.000MHz
{
	unsigned char i;
	i = 78;
	while (--i);
}

/****************************************************************************
* 名    称：
* 功    能：精确延时函数
* 入口参数：
* 出口参数：
* 说    明：
* 调用方法：
****************************************************************************/
static void Delay18ms()		//@24.000MHz
{
	unsigned char i, j, k;
	_nop_();
	_nop_();
	i = 3;
	j = 50;
	k = 4;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}

/****************************************************************************
* 名    称：Read_Byte
* 功    能：读一个字节
* 入口参数：无
* 出口参数：读到的数据
* 说    明：从温湿度传感器读一个字节
* 调用方法：无
****************************************************************************/
static unsigned char Read_Byte(void)
{
	unsigned char i,U8temp;
	unsigned char U8comdata;  
	for(i=0;i<8;i++)	   
	{
		OverTime_flag=500;	
		while((!DHT11_OUT)&&OverTime_flag--);
		Delay_10us();
		Delay_10us();
		Delay_10us();
		U8temp=0;
		if(DHT11_OUT)U8temp=1;		   //如果高电平U8temp=1  低电平U8temp=0
		OverTime_flag=500;
		while((DHT11_OUT)&&OverTime_flag--);		  
		if(OverTime_flag==0)break;	           //超时则跳出for循环		 
		U8comdata<<=1;
		U8comdata|=U8temp;        
	}
	return(U8comdata);
}

/****************************************************************************
* 名    称：
* 功    能：温湿度读取子程序
* 入口参数：
* 出口参数：高8位为温度，低8位为湿度
* 说    明：
* 调用方法：
****************************************************************************/
unsigned int DHT11_Read(void)
{
	unsigned char str[6]={"      "};
	unsigned char U8T_data_H_temp,U8T_data_L_temp;
	unsigned char U8RH_data_H_temp,U8RH_data_L_temp;
	unsigned char U8checkdata_temp;
	unsigned char U8temp1;
	unsigned int value_DHT11;

	DHT11_OUT=0;					//主机拉低18ms 
	Delay18ms();
	DHT11_OUT=1;					//释放总线

	Delay_10us();
	Delay_10us();				  //总线由上拉电阻拉高 主机延时20us
	Delay_10us();
	Delay_10us();
	DHT11_OUT=1;					//主机设为输入 判断从机响应信号 
 	  
	if(!DHT11_OUT)		  	//判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行
	{
		OverTime_flag=500;
		while((!DHT11_OUT)&&OverTime_flag--);     //判断从机发出 80us 的低电平响应信号是否结束
		OverTime_flag=500;
		while((DHT11_OUT)&&OverTime_flag--);	  	 //判断从机发出 80us 的高电平，如发出则进入数据接收状态

		U8RH_data_H_temp=Read_Byte();	   //数据接收状态
		U8RH_data_L_temp=Read_Byte();
		U8T_data_H_temp=Read_Byte();
		U8T_data_L_temp=Read_Byte();
		U8checkdata_temp=Read_Byte();
		DHT11_OUT=1;

		U8temp1=(U8T_data_H_temp+U8T_data_L_temp+U8RH_data_H_temp+U8RH_data_L_temp);//数据校验 
		if(U8temp1==U8checkdata_temp)
		{
			str[0]=U8RH_data_H_temp;
			str[1]=U8RH_data_L_temp;
			str[2]=U8T_data_H_temp;
			str[3]=U8T_data_L_temp;
			str[4]=U8checkdata_temp;
			value_DHT11=U8RH_data_H_temp*256+U8T_data_H_temp;
			return(value_DHT11);
		}
		else return(0);										
	}
	else return(0);
}
	
