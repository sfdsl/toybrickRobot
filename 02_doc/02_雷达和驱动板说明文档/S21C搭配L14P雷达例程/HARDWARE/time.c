#include "time.h"

/**************************************************************************
Function: TIM5 initialization
Input   : none
Output  : none
函数功能：定时器5初始化
入口参数：无
返回  值：无
**************************************************************************/
void time5_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 49;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7199;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode =TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM5,TIM_FLAG_Break);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5,ENABLE);
}

/**************************************************************************
Function: TIM5 interrupt function
Input   : none
Output  : none
函数功能：定时器5中断处理函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM5_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM5,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		count++;
		sec_count++;
		LED_Flash(50);		
		if(count==40)								//200ms执行一次
		{
			count=0;
			oled_flag=1;
		}
		if(sec_count==200)					//一秒执行一次
		{
			sec_count = 0;
			//检测雷达是否有接入
			//如果没接上则数据成功接收数不会增加，在下面将成功接收数复制给另外一个值之后两个值会相同
			if(receive_cnt2_old == receive_cnt2)	Lidar_1 = 0,OLED_ShowString(70,00,"XXXXX");		
			else																	Lidar_1 = 1;
			if(receive_cnt3_old == receive_cnt3)	Lidar_2 = 0,OLED_ShowString(70,30,"XXXXX");
			else																	Lidar_2 = 1;
			if(receive_cnt4_old == receive_cnt4)	Lidar_3 = 0,OLED_ShowString(70,20,"XXXXX");
			else																	Lidar_3 = 1;
			if(receive_cnt5_old == receive_cnt5)	Lidar_4 = 0,OLED_ShowString(70,10,"XXXXX");
			else																	Lidar_4 = 1;
			sec++;
			receive_cnt2_old = receive_cnt2;		//保存雷达成功接收数
			receive_cnt3_old = receive_cnt3;
			receive_cnt4_old = receive_cnt4;
			receive_cnt5_old = receive_cnt5;
		}
		//按键 实现角度范围值的左移右移
		Key();
	}
}
