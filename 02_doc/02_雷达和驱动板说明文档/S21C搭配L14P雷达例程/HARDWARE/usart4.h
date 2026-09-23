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
#ifndef __USRAT4_H
#define __USRAT4_H 
#include "sys.h"	  	




void uart4_init(u32 bound);
void UART4_IRQHandler(void);
//void uart4_can_trastion(void);
void data_process4(void);
void Lidar3_Data_Deal(void);
#endif

