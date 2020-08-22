/***************************************************************************
* Copyright (c) 
* All rights reserved.
* 文件名称：
* 文件标识：无
* 摘    要：
* 当前版本：
*                                     
      _____                                        __    
     / ____\                                      / /
    / /                                          / /      _    
   / /   _____  ________  ________  _____  _____/ /____  / \_______ 
  / /   / ___ \/ __  __ \/ __  __ \/ ___ \/ ___  / ___ \/ ___/ ___ \
 / /___/ /__/ / / / / / / / / / / / /__/ / /__/ / /__/ / /  / /____/
 \____/\_____/_/ /_/ /_/_/ /_/ /_/\_____/\_____/\_____/_/   \_____\
                                                                                                                
* 完成日期:2016.6.1
****************************************************************************/
#include "TIM4.h"
#include "MPU6050.h"
#include "Attitude_Count.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Ultrasonic.h"
Angle Euler_Angle;
extern u8 Execute_Period2ms;//内环,MPU6050和RC
extern u8 Execute_Period5ms;//IMU和外环PID5ms
extern u8 Execute_Period10ms;//气压计和电子罗盘10ms
extern u8 Execute_Period50ms;//姿态50ms，超声波测高
extern u8 Execute_Period100ms;//定点GPS100ms
extern u8 Execute_Period200ms;//主要传感器数据200ms
extern u8 Execute_Period500ms;//次要传感器数据500ms
extern u8 MPU6050_Buffer[14];
void TIM1_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  //定时器TIM1更新中断
  NVIC_InitStruct.NVIC_IRQChannel=TIM1_UP_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  TIM_TimeBaseStruct.TIM_Period = arr;
  TIM_TimeBaseStruct.TIM_Prescaler = psc;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
  
  TIM_ClearFlag(TIM1,TIM_FLAG_Update);
  //TIM_ARRPreloadConfig(TIM1, ENABLE); //默认
  TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM1,ENABLE);
}
//每个周期是1ms  72/72=1MHz;计数1000，就是1ms周期
//注意高级定时器的更新中断函数名字和普通定时器不一样
void TIM1_UP_IRQHandler(void)
{
    static u16 TIM1_Cnt=0;
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{   
		TIM1_Cnt++;
		if(TIM1_Cnt%2==0)
		{
		  Execute_Period2ms=1;
		}
		if(TIM1_Cnt%5==0)
		{
		  Execute_Period5ms=1;
		}
		if(TIM1_Cnt%10==0)
		{
		  Execute_Period10ms=1;
		}
		if(TIM1_Cnt%50==0)
		{
		  Execute_Period50ms=1;
		}
		if(TIM1_Cnt%100==0)
		{
		  Execute_Period100ms=1;
		}
		if(TIM1_Cnt%200==0)
		{
		  Execute_Period200ms=1;
		}
		if(TIM1_Cnt%500==0)
		{
		  Execute_Period500ms=1;
		}
//		if(TIM1_Cnt%2==0)//每2ms进行一次
//		{
//		  MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,MPU6050_Buffer);//读取传感器的数据每2ms一次
//		  MPU_Data_Analyse(&ACC,&GYRO);//ACC和GYRO补偿
//		  Filter_Acc(&ACC);//ACC滤波  
//		  Filter_Gyro(&GYRO);//GYRO滤波滑动滤波
//		  PID_In_Control();//内环
//		}
//		if(TIM1_Cnt%5==0)//每5ms触发
//		{
//		  Calculate_Target();//计算RC期望角度 
//		  AHRS_Update(&GYRO,&ACC,&MAG,&Euler_Angle);//更新姿态角，当PITCH为90度的时候，ROLL会突然变成170多度，有严重的问题
//		  PID_Out_Control();//串级PID控制和电机更新
//		}
//		if(TIM1_Cnt%40==0)//40ms触发一次超声波测高
//		{   
//			HMC5883lRead(&MAG);//读取磁力计
//		    BMP085_Routing();//每40ms更新一次气压计
//		    Ultrasonic_Pulsing();
//		}
//		if(TIM1_Cnt%200==0)//200毫秒发送一次数据到上位机
//		{  
//		  Send_Data_Flag1=1;
//		}
//		if(TIM1_Cnt%500==0)//
//		{
//		   QUAD_Armed(2);//循环5次，也就是保持2——3S就变换状态
//		}
		TIM1_Cnt=TIM1_Cnt%1000;
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
     }
}


