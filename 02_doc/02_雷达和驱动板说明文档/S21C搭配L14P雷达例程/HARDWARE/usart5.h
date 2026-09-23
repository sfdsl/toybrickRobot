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
#ifndef __USRAT5_H
#define __USRAT5_H 
#include "sys.h"	  	




void uart5_init(u32 bound);
void UART5_IRQHandler(void);
void data_process5(void);
//void uart5_can_trastion(void);
void Lidar2_Data_Deal(void);
#endif

