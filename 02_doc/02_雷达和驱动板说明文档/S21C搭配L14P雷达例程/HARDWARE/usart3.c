/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "usart3.h"
#include <string.h>
/**************************************************************************
Function: Usart3 initialization
Input   : bound:Baud rate
Output  : none
函数功能：串口3初始化
入口参数：bound:波特率
返回  值：无
**************************************************************************/

LiDARFrameTypeDef Pack_Data3;//雷达接收的数据储存在这个变量之中
extern LidarPointStructDef Dataprocess3[800];//雷达转动一圈数据储存


void uart3_init(u32 bound)
{  	 
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能UGPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	//USART3_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART3_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure);     //初始化串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口3 

}

/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void Lidar4_Data_Deal(void)
{
	static u8 state3 = 0;//状态位	
	static u8 crc3 = 0;//校验和
	static u8 cnt3 = 0;//用于一帧12个点的计数
	static u16 count3 = 0;
	u8 temp_data;
	temp_data = USART3_Rx_Buf[count3];
	USART3_Rx_Buf[count3] = 0;
	count3++;
	if(count3 == 2820)			count3 = 0;
	if (state3 > 5)
	{
		if(state3 < 42)
		{
			if(state3%3 == 0)//一帧数据中的序号为6,9.....39的数据，距离值低8位
			{
				Pack_Data3.point[cnt3].distance = (u16)temp_data;
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
			}
			else if(state3%3 == 1)//一帧数据中的序号为7,10.....40的数据，距离值高8位
			{
				Pack_Data3.point[cnt3].distance = ((u16)temp_data<<8)+Pack_Data3.point[cnt3].distance;
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
			}
			else//一帧数据中的序号为8,11.....41的数据，置信度
			{
				Pack_Data3.point[cnt3].confidence = temp_data;
				cnt3++;	
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
			}
		}
		else 
		{
			switch(state3)
			{
				case 42:
					Pack_Data3.end_angle = (u16)temp_data;//结束角度低8位
					state3++;
					crc3 = CrcTable[(crc3^temp_data) & 0xff];
					break;
				case 43:
					Pack_Data3.end_angle = ((u16)temp_data<<8)+Pack_Data3.end_angle;//结束角度高8位
					state3++;
					crc3 = CrcTable[(crc3^temp_data) & 0xff];
					break;
				case 44:
					Pack_Data3.timestamp = (u16)temp_data;//时间戳低8位
					state3++;
					crc3 = CrcTable[(crc3^temp_data) & 0xff];
					break;
				case 45:
					Pack_Data3.timestamp = ((u16)temp_data<<8)+Pack_Data3.timestamp;//时间戳高8位
					state3++;
					crc3 = CrcTable[(crc3^temp_data) & 0xff];
					break;
				case 46:
					Pack_Data3.crc8 = temp_data;//雷达传来的校验和
					if(Pack_Data3.crc8 == crc3)//校验正确
					{
							data_process3();//接收到一帧且校验正确可以进行数据处理
						receive_cnt3++;//输出接收到正确数据的次数
						data_process_flag3=1; //标志位
					}
					else
						memset(&Pack_Data3,0,sizeof(Pack_Data3));//清零
					crc3 = 0;
					state3 = 0;
					cnt3 = 0;//复位
				default: break;
			}
		}
	}
	else 
	{
		switch(state3)
		{
			case 0:
				if(temp_data == HEADER)//头固定
				{
					Pack_Data3.header = temp_data;
					state3++;
					crc3 = CrcTable[(crc3^temp_data) & 0xff];//开始进行校验
				} else state3 = 0,crc3 = 0;
				break;
			case 1:
				if(temp_data == LENGTH)//测量的点数，目前固定
				{
					Pack_Data3.ver_len = temp_data;
					state3++;
					crc3 = CrcTable[(crc3^temp_data) & 0xff];
				} else state3 = 0,crc3 = 0;
				break;
			case 2:
				Pack_Data3.speed = (u16)temp_data;//雷达的转速低8位，单位度每秒
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
				break;
			case 3:
				Pack_Data3.speed = ((u16)temp_data<<8)+Pack_Data3.speed;//雷达的转速高8位
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
				break;
			case 4:
				Pack_Data3.start_angle = (u16)temp_data;//开始角度低8位，放大了100倍
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
				break;
			case 5:
				Pack_Data3.start_angle = ((u16)temp_data<<8)+Pack_Data3.start_angle;
				state3++;
				crc3 = CrcTable[(crc3^temp_data) & 0xff];
				break;
			default: break;

		}
	}
	
}
	
/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口接收数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
void USART3_IRQHandler(void)
{
	static int i = 1;
	u8 data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	  
		data=USART_ReceiveData(USART3); 
		if(USART3_Rx_Buf[0]!=HEADER && data == HEADER)				USART3_Rx_Buf[0] = data;			//保证数组中保存的数据的帧完整性
		if(USART3_Rx_Buf[0] == HEADER)
		{
			USART3_Rx_Buf[i] = data;				//将雷达发送的数据写进缓冲区
			i++;
			if(i>=2820)	i = 0;							//一圈数据总共有2820个字节（47个字节/帧*60帧）

		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
}

/**************************************************************************
函数功能：CAN发送一圈数据
入口参数：无
返回  值：无
**************************************************************************/
void uart3_can_trastion(void)
{
	//每次加4，总共执行180次，完成一圈数据的发送
	for(int a=0;a<720;a++)
	{
		CAN_Send_Data[0] = Dataprocess3_distance[a]>>8;
		CAN_Send_Data[1] = Dataprocess3_distance[a];
		CAN_Send_Data[2] = Dataprocess3_distance[a+1]>>8;
		CAN_Send_Data[3] = Dataprocess3_distance[a+1];
		CAN_Send_Data[4] = Dataprocess3_distance[a+2]>>8;
		CAN_Send_Data[5] = Dataprocess3_distance[a+2];
		CAN_Send_Data[6] = Dataprocess3_distance[a+3]>>8;
		CAN_Send_Data[7] = Dataprocess3_distance[a+3];
		CAN1_Send_Num(0x601,CAN_Send_Data);
		a+=3;
	}
}


void data_process3(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
		int m,n,i;
	float start_angle = Pack_Data3.start_angle/100.0;//计算12个点的开始角度
	float end_angle = Pack_Data3.end_angle/100.0;//计算12个点的结束角度
	float area_angle[12]={0};
	
	if(start_angle>end_angle)//结束角度和开始角度被0度分割的情况
	end_angle +=360;

  data_process_flag3=0; //标志位清零
	for(m=0;m<12;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/12*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}

	for(n=0;n<12;n++)//将数据传输到Dataprocess中，防止被覆写
	{
		Dataprocess3[data_cnt3+n].angle = area_angle[n];
	  Dataprocess3[data_cnt3+n].distance = Pack_Data3.point[n].distance;  //一帧数据为12个点
		Dataprocess3_distance[data_cnt3+n] = Dataprocess3[data_cnt3+n].distance;
	}
	//因为是顺时针旋转，一阵数据大概6度，想要的数据范围是2度，所以在想要读取范围值的左值之前的4个度数就得开始判断数据是否有在想要的范围内
	if((Dataprocess3[data_cnt3].angle>=angle_Left) || (Dataprocess3[data_cnt3].angle<=angle_Right))
	{
		for(i=0;i<12;i++)
		{
			if((Dataprocess3[data_cnt3+i].angle>range_Left) && (Dataprocess3[data_cnt3+i].angle<range_Right))			//已经在范围内
			{
				Lidar_Distance[3] = Dataprocess3[data_cnt3+i].distance;					//将满足条件的数据保存起来
			}
		}
	}
	data_cnt3 +=12;
	if(data_cnt3>=720) //雷达转一圈大概有720个点（大概数值，每圈的点数都不固定，一帧大约为6度，一圈大约是60帧数据=12*60=720）
	{
		if(USE_CAN_FLAG)		uart3_can_trastion();				//使能CAN则通过CAN总线发送数据
		data_cnt3 = 0;
	}
}






