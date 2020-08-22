//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//编写者:  Commodore 
//创建日期:2016/4/6
//版本:    V1.0
//版权所有，盗版必究。
//All rights reserved									  
///////////////////////////////////////////////////////////////////////////////////

#ifndef _PID_CONTROL_H
#define _PID_CONTROL_H
#include "Includes.h"
void Fly_Prepare(void);
void PID_Init(void);
void PID_In_Control(void);
void PID_Out_Control(void);
void Calculate_Target(void);
#endif

