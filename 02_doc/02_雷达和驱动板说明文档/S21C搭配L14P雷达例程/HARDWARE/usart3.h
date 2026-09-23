/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：
修改时间：2021-04-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:
Update：2021-04-29

All rights reserved
***********************************************/
#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	
//#include "usart2.h"
//extern LidarPointStructDef Dataprocess3[800];//雷达转动一圈数据储存
//extern u16 data_cnt3;

void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void data_process3(void);
//void uart3_can_trastion(void);
void Lidar4_Data_Deal(void);
#endif

