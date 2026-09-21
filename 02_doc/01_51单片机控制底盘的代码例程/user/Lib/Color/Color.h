#ifndef __color_h__
#define __color_h__

extern unsigned short R_data,G_data,B_data;
void Color_Init();//颜色初始化
/****************************************************************************
* 函数名称: 
* 函数功能: 白平衡校准
*          	
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void whiteBalance(void);

#endif
