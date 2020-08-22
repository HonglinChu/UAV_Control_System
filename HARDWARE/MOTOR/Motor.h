//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//编写者:  Commodore 
//创建日期:2016/4/6
//版本:    V1.0
//版权所有，盗版必究。
//All rights reserved									  
//////////////////////////////////////////////////////

#ifndef _MOTOR_H
#define _MOTOR_H

#include "Includes.h"
void Motor_Init(u16 arr,u16 psc);
void Motor_Refresh(u16 pwm1,u16 pwm2 ,u16 pwm3,u16 pwm4);
#endif 
