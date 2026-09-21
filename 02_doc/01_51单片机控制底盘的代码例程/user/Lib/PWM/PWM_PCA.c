/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	PWM_PCA.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：
*Description：
*History：
****************************************************************************/

/*引入头文件*/
#include  "PWM_PCA.h"
#include <STC8.H>

#define PWM_F  1000 //定义速度位数 0-32768（最大15位）越大频率越低

void PWM_PCA_init()
{
	P_SW2 |= 0x80;   //PWM寄存器在XDATA区域，需要访问权限
	
	PWMCKS = 0x00;  // PWM 时钟为系统时钟
	PWMC = PWM_F;   //设置 PWM 周期为 100H 个 PWM 时钟
	
	PWM0T1= 0; 			//在计数值地方输出低电平
	PWM0T2= 0; 	  	//在计数值地方输出高电平
	
	PWM2T1= 0; 			//在计数值地方输出低电平
	PWM2T2 = 0;   	//在计数值地方输出高电平
	
	PWM0CR= 0x88;   //使能 PWM0_2 输出 P1.0
	PWM2CR= 0x88;   //使能 PWM2_2 输出 P1.1
	
	P_SW2 &= 0x7F;		//关闭访问权限
	
	PWMCR = 0x80;   //启动 PWM 模块
}

void RUN_PWM(void)
{
	if(PWMCR != 0x80) //如果没有启动
		PWMCR = 0x80;   //启动 PWM 模块
}

void STOP_PWM(void)
{
	if(PWMCR != 0x00) //如果没有停止
		PWMCR = 0x00;   //停止 PWM 模块
}

void PWM_Set(unsigned int PWM0_DATA,unsigned int PWM1_DATA)
{
	P_SW2 |= 0x80;   		//PWM寄存器在XDATA区域，需要访问权限
	PWM0T1= PWM0_DATA; 	//在计数值地方输出高电平
	PWM2T1= PWM1_DATA;  //在计数值地方输出高电平
	P_SW2 &= 0x7F;				//关闭访问权限
}