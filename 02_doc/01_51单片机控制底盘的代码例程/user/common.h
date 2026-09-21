#ifndef _common_h__
#define _common_h__

#include <STC8.H>
#include "car_control.h"
#include "delay.h"

#define u8 unsigned char
#define u16 unsigned int
#define uchar unsigned char 
#define uint unsigned int

sbit KEY1=P3^3;
sbit KEY2=P3^4;
sbit KEY3=P3^5;
sbit KEY4=P5^4;

sbit IO_BUZZ = P5^5;

#endif
