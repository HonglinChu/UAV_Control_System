/**************************************************************************************************************
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
 \____/\_____/_/ /_/ /_/_/ /_/ / /\_____/\_____/\_____/_/   \_____\
                                                                                                                
* 完成日期:2016.6.1
**************************************************************************************************************/
#include "Includes.h"
#include "MPU6050.h"
#include "Attitude_Count.h"
#include "HMC5883L.h"
#include "Ultrasonic.h"
u8 Execute_Period2ms=0;//内环,MPU6050和RC
u8 Execute_Period5ms=0;//IMU和外环PID5ms
u8 Execute_Period10ms=0;//气压计和电子罗盘10ms
u8 Execute_Period50ms=0;//姿态50ms，超声波测高
u8 Execute_Period100ms=0;//定点GPS100ms
u8 Execute_Period200ms=0;//主要传感器数据200ms
u8 Execute_Period500ms=0;//次要传感器数据500ms
extern u8 MPU6050_Buffer[14];
int main(void)
{
	u8 i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();               //SYSTICK时钟频率的选择   
	Usart_Init(115200);         //串口初始化，通过USART1发送数据到host——computer 波特率设置
  FLASH_Unlock();	          //Flash解锁
	EE_Init();                  //EEPROM初始化
	LED_Init();                 //LED 初始化
	DMA1_Init();                //DMA 初始化,开启USART的DMA发送
  MPU_Init();                 //MPU6050初始化包括了模拟IIC总线初始化
	HMC5883L_Init();            //HMC5883初始化
	BMP085_Init();              //BMP初始化
	Ultrasonic_Init();          //超声波初始化
	PWM_Capture_Init(0xFFFF,71);//1MHz.  PWM捕获初始化，TIM4
  Motor_Init(2499,71);        //2500,1MHz;2.5ms输出一次//电机初始化
	PID_Init();                 //PID参数调节初始化
	while(1)
	{	
		if(Execute_Period2ms==1)
		{
		  Execute_Period2ms=0;
		  MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,MPU6050_Buffer);//读取传感器的数据每2ms一次
		  MPU_Data_Analyse(&ACC,&GYRO);//ACC和GYRO补偿
		  Filter_Acc(&ACC);  //ACC滤波  
		  Filter_Gyro(&GYRO);//GYRO滤波滑动滤波
		  PID_In_Control();  //内环
		}
		if(Execute_Period5ms==1)
		{
		  Execute_Period5ms=0;
		  Calculate_Target();//计算RC期望角度 
		  AHRS_Update(&GYRO,&ACC,&MAG,&Euler_Angle);//更新姿态角，当PITCH为90度的时候，ROLL会突然变成170多度，有严重的问题
		  PID_Out_Control();//串级PID控制和电机更新
		}
		if(Execute_Period10ms==1)
		{
		   Execute_Period10ms=0;
		   HMC5883lRead(&MAG);  //读取磁力计
		   BMP085_Routing();    //每40ms更新一次气压计
		}
		if(Execute_Period50ms==1)//触发一次超声波捕获
		{
		  //在这里添加一个模式改变函数。
		  Execute_Period50ms=0; 
		}
		if(Execute_Period100ms==1)//超声波定高
		{
		   Execute_Period100ms=0;
		   Ultrasonic_Pulsing();
		}
		if(Execute_Period200ms==1)
		{
		   Execute_Period200ms=0;
		   Data_Send_Status();   // 欧拉角
		   Data_Send_Sensor1();  // 传感器数据
		   Data_Send_Sensor2();  // 发送高度信息 
		}
		if(Execute_Period500ms==1)
		{
		   Execute_Period500ms=0;
		   if(Send_PID_Flag==1) //PID环中的PID系数
		   {
		    Send_PID_Flag=0;    
			  Send_PID1_Parameters();
			  Send_PID2_Parameters();
			  Send_PID3_Parameters();
		   }
		   Data_Send_RCData(); //RC通道的PWM值
		   QUAD_Armed(2);//循环5次，也就是保持2——3S就变换状态
		}
   }
}
