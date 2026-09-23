#include "key.h"
/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-01-04

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-01-04

All rights reserved
***********************************************/


static u8 key_press = 0;
/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);              //使能 PORTB 时钟 
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;                       //上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;                           // PB14 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
} 
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	  if(KEY==0)  Forever_count++;   //长按标志位未置1
     else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2;//双击执行的指令
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1;//单击执行的指令
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
	static u8 flag_key=1;//按键按松开标志
	
	if(flag_key&&KEY==1)
	{
		flag_key=0;
		key_press = 1;
//		return 1;	// 按键按下
	}
	else if(0==KEY)			flag_key=1;
	if(key_press==1&&KEY==0)	
	{
		key_press=0;
		return 1;
	}
	return 0;//无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
		static u16 Long_Press_count,Long_Press;
	    if(Long_Press==0&&KEY==1)  Long_Press_count++;   //长按标志位未置1
        else                       Long_Press_count=0; 
		if(Long_Press_count>200)		
		{
			Long_Press=1;
			Long_Press_count=0;
			key_press = 0;
			return 1;
		}			
			 if(Long_Press==1)     //长按标志位置1
			{
				  Long_Press=0;
			}
			return 0;
}

/**************************************************************************
函数功能：雷达测距范围增减 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click(); 
	if(tmp==1)
	{
		range_Left++;
		range_Right++;
	}
	
	tmp2=Long_Press();          
  if(tmp2==1)
	{
		range_Left--;
		range_Right--;
	}
	
	if(range_Left>=360)			range_Left = 0;
	else if(range_Left<0)		range_Left = 359;
	if(range_Right>=360)			range_Right = 0;
	else if(range_Right<0)		range_Right = 359;
	
	//雷达是顺时针旋转，一帧数据是6度，范围固定2度，所以要在左范围往前4度就开始检测，才能保证目标的2度在检测范围内
		angle_Left = range_Left - 4;
		if(angle_Left < 0)					angle_Left+=360;			//如果小于4减完就会变成负数，需要处理
		angle_Right = range_Right;
}

