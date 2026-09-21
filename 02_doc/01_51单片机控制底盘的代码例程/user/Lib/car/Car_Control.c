/****************************************************************************
*Copyright：--武汉莱斯特电子科技有限公司--
*File name：	Car_Control.c
*Author：xx
*ID：YLX-01
*Version：1.0.0
*Date：2018-08-14
*Description：
*History：
****************************************************************************/

/*引入头文件*/
#include "Car_Control.h"
#include "delay.h"
#include "car_task.h"

unsigned char moto_run_type = 0;

/*引脚定义见Car_Control.h文件*/

//unsigned char T_branch=0;		/*T或者十字路口*/
data volatile unsigned char All_branch=0;	/*所有路口*/

/****************************************************************************
* 名    称：car_init
* 功    能：小车初始化程序
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：while循环之前调用
****************************************************************************/ 
void car_init(void)
{
	/*设置P1.0为浮空输入端口-循迹输入端口要为浮空输入*/
	P0M0 = 0x00;
  P0M1 = 0xFF;
}

/*小车测试程序*/
void Car_test(void)
{
  	MOTOR_GO_F; 		//前
		delay_ms(1000);
		MOTOR_GO_STOP;
		delay_ms(1000);
		MOTOR_GO_BACK;	//后
		delay_ms(1000);
		MOTOR_GO_STOP;	
		delay_ms(1000);
		MOTOR_GO_L;			//左
		delay_ms(1000);
		MOTOR_GO_STOP;
		delay_ms(1000);
		MOTOR_GO_R;			//右
		delay_ms(1000);
}

/****************************************************************************
* 名    称：car_FollowLine
* 功    能：小车循迹程序
* 入口参数：无
* 出口参数：无
* 说    明：循迹+路口检测
* 调用方法：循环调用
****************************************************************************/ 
void car_FollowLine(void)
{
	static unsigned int L_Branch_Count_Error = 0;		//7
	static unsigned int R_Branch_Count_Error = 0;		//F
	static unsigned int All_Branch_Count_Error = 0;	//T
    
    #ifdef xunji_level_singleH  // 有效信号为高电平
    unsigned char xunji = xunji_port;
    #else
    unsigned char xunji = ~xunji_port;
    #endif
	if (carWorking.type == 0)
    {   // 未执行特殊任务时，进行正常循迹
		switch(~xunji)//注意这里取反
		{   // 主要直行车体姿态校正
			case 0xDF:
			case 0xBF:
			case 0x8F: 
			case 0x9F:
			case 0x3F:
			case 0x1F:
			case 0x7F:
			case 0x5F:MOTOR_GO_adjL;  break;
			
			case 0xFD:
			case 0xFB:
			case 0xF8:
			case 0xF9:
			case 0xF3:
			case 0xF1:
			case 0xFE:
			case 0xFC:MOTOR_GO_adjR;   break;
			
			case 0xE7:
			case 0xF7:
			case 0xEF:
			case 0xc3:
			case 0xc7:
			case 0xe3:
                if(All_branch >= 17 && All_branch < 19) // 红绿灯检测路段，降低速度
                    {MOTOR_GO_F_SLOW;}
                else
                    {MOTOR_GO_F;}
                break;	
			
			case 0xFF:MOTOR_GO_STOP;break;	//地图边缘
			case 0x00:break;								//黑线路口
			default:
                if(All_branch >= 17 && All_branch < 19) // 红绿灯检测路段，降低速度
                    {MOTOR_GO_F_SLOW;}
                else
                    {MOTOR_GO_F;}
			break;
		}
		/*右拐角路口*/
		if((xunji==0x0f)||(xunji == 0x1f)||(xunji == 0x3f)||(xunji == 0x7f))//右边全在黑线上（F字路口（反7））
		{
			R_Branch_Count_Error ++;   // 传感器检测冗余计数
			if(R_Branch_Count_Error > 50)/*减小误差*/
			{
				R_Branch_Count_Error = 0;		/*清除标志位*/
                
                L_Branch_Count_Error = 0;		/*清除标志位*/
                All_Branch_Count_Error = 0; /*清除标志位*/
				MOTOR_GO_F_SLOW   // 继续前进  驶出当前路口
				delay_ms(300);
				All_branch ++;					 		/*路口数加一*/
				All_branch = All_Branch_Task(All_branch);/*执行任务*/
			}
		}
		else
		{
            if (R_Branch_Count_Error != 0)
                R_Branch_Count_Error--;
		}
		/*左拐角路口*/
		if((xunji==0xf0)||(xunji==0xf8)||(xunji==0xfC)||(xunji==0xfE))//左边全在黑线上（7字路口）
		{
			L_Branch_Count_Error ++;   // 传感器检测冗余计数
			if(L_Branch_Count_Error > 50)/*减小误差*/
			{
				L_Branch_Count_Error = 0;		/*清除标志位*/
                R_Branch_Count_Error = 0;		/*清除标志位*/
                All_Branch_Count_Error = 0; /*清除标志位*/
                
				MOTOR_GO_F_SLOW   // 继续前进  驶出当前路口
				delay_ms(300);
				All_branch ++;					 		/*路口数加一*/
				All_branch = All_Branch_Task(All_branch);/*执行任务*/
			}
		}
		else
		{
            if (L_Branch_Count_Error != 0)
                L_Branch_Count_Error--;
		}
		/*十字路口或者T字路口*/
		if(xunji == 0xff)
		{
			All_Branch_Count_Error ++;   // 传感器检测冗余计数
			if(All_Branch_Count_Error > 15)/*减小误差*/
			{
				All_Branch_Count_Error = 0; /*清除标志位*/
                R_Branch_Count_Error = 0;		/*清除标志位*/
                L_Branch_Count_Error = 0;		/*清除标志位*/
                
				MOTOR_GO_F_SLOW   // 继续前进  驶出当前路口
				delay_ms(300);
				//T_branch ++;							  /*T字路口+1*/
				All_branch ++;					    /*路口数加一*/
				All_branch = All_Branch_Task(All_branch);/*执行任务*/
			}
		}
		else
		{
            if (All_Branch_Count_Error != 0)
                All_Branch_Count_Error--;
		}
		
    }
    Loop_Task(All_branch);  // 全局任务  (含特殊任务)
}

