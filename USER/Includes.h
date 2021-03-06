/***************************************************************************
* Copyright (c) 开源代码
* All rights reserved.
* 文件名称：main函数
* 文件标识：无
* 摘要说明：本程序未经许可不得用于其他用途
* 当前版本：1.0
* 作    者: Commodore
* 完成日期：2016年04月15日
****************************************************************************/

#ifndef _INCLUDES_H_
#define _INCLUDES_H_

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "math.h"
#include "delay.h"
#include "sys.h"
#include "Usart1.h"
//#ifdef   OS_GLOBALS
//#define  OS_EXT
//#define  INIT(x)
//#else
//#define  OS_EXT  extern
//#define  INIT(x) =x
//#endif
#include "Data_ReadWrite.h"
#include "Data_Trans.h"
#include "DMA_Config.h"
#include "eeprom.h"
#include "Led.h"
#include "Motor.h"
#include "IIC.h"
#include "PID_Control.h"
#include "PWM_Capture.h"
#include "RC_Config.h"
#include "TIM4.h"
#include "BMP085.h"
//#include "SPI.h"
//#include "ADNS3080.h"

#include  "Height_Control.h"
//RC通道捕获PWM的引脚对应TIM4的B6 B7  B8  B9 使用TIM8�
#define PWM_IN_PIN1  GPIO_Pin_6
#define PWM_IN_PIN2  GPIO_Pin_7
#define PWM_IN_PIN3  GPIO_Pin_8
#define PWM_IN_PIN4  GPIO_Pin_9
//Motor的PWM输出对应A0 A1 A2 A3，使用TIM2
#define MOTOR_PIN1 GPIO_Pin_0
#define MOTOR_PIN2 GPIO_Pin_1
#define MOTOR_PIN3 GPIO_Pin_2
#define MOTOR_PIN4 GPIO_Pin_3
//LED 指示灯对应PA8 DS0(R)  和PD2 DS1(G)
#define LED0      PAout(8)
#define LED1      PDout(2)
#define LED0_Pin  GPIO_Pin_8
#define LED1_Pin  GPIO_Pin_2

#define ARMED_STATE      THROTTLE > 900 && THROTTLE < 1100 && YAW < 2100 && YAW > 1900
#define UN_ARMED_STATE   THROTTLE > 900 && THROTTLE < 1100 && YAW > 900 && YAW  < 1100
#define RC_ADJUST_CMD    ROLL > 1900 && ROLL < 2100 && PITCH > 1900 && PITCH < 2100
#define SET_OFFSET_CMD   ROLL < 1100 && ROLL > 900  && PITCH < 1100 && PITCH >900
#define MAX_ANGLE  (float)25

typedef struct
{
    float P;//proporation 
	float I;//Intergal
	float D;//Differential
	float POut;//比例项输出
	float IOut;//积分项输出
	float DOut;//微分项输出
	float Out;//总输出。
}PID;
//以下都是一些外部变量的声明和结构体的声明
extern PID  PID1_ROLL;
extern PID  PID1_PITCH;
extern PID  PID1_YAW;
extern PID  PID2_ROLL;
extern PID  PID2_PITCH;
extern PID  PID2_YAW;
extern PID Height_PID1;//高度外环
extern PID Height_PID2;//高度内环
//表示姿态角
//表示传感器的输出值,是两个字节的数
typedef struct
{
    short int X;
	short int Y;
	short int Z;
}Sensor_Data;
// extern  Sensor_Data  Acc1;
// extern  Sensor_Data  Gyro1;
// extern  Sensor_Data  Acc2;
// extern  Sensor_Data  Gyro2;
 extern  Sensor_Data  MAG;
 extern  Sensor_Data  ACC_AVG;
 extern  Sensor_Data  ACC;
 extern  Sensor_Data  GYRO;
 extern  Sensor_Data  ACC_OFFSET;
 extern  Sensor_Data  GYRO_OFFSET;

typedef struct{
	      u8 MagExist;      // MAG存在
	      u8 MagIssue;      // MAG有问题
	      u8 calibratingM;  // 磁力计校准标记
         }_Flag;
extern  _Flag  MagFlag;
		 
extern  int ALT_BAR;
extern  short int ALT_CSB;
typedef struct
{
    float roll;
	float pitch;
	float yaw;
}Angle;
extern Angle Euler_Angle;

//四个电机的PWM值
 extern float Motor1;
 extern float Motor2;
 extern float Motor3;
 extern float Motor4;

//PWM捕获到的四个通道的PWM值,站两个字节
extern short int ROLL;
extern short int PITCH;
extern short int THROTTLE;
extern short int YAW;
extern short int AUX1;//对应通道六，三挡开关

extern float Roll_Target;
extern float Pitch_Target;
extern float Yaw_Target;
extern float Throttle_Target;
//RC通道的范围

//主要是RC通道最大值和最小值的校准
extern u8  RC_Calibration_Flag;
extern u16 THR_MAX;
extern u16 THR_MIN;
extern u16 YAW_MAX;
extern u16 YAW_MIN;
extern u16 ROLL_MAX;
extern u16 ROLL_MIN;
extern u16 PITCH_MAX;
extern u16 PITCH_MIN;
extern u16 Half_Roll;
extern u16 Half_Pitch;
extern u16 Half_Yaw;
extern int16_t  HMC58X3_limit[6];//保存三个轴的最大值和最小值
extern int16_t  *mag_limt;//指向数组的指针，仅仅是一个指针而已
//ACC GYRO 校准标志，获得ACC和GYRO的补偿值
extern u8  ACC_Calibration_Flag;
extern u8  GYRO_Calibration_Flag;
extern u8  BAR_Calibration_Flag;
//飞控解锁标志
extern u8 QUAD_OK;
//发送PID参数
extern u8 Send_PID_Flag;
//发送数据到上位机的标志
extern u8 Send_Data_Flag1;
extern u8 Send_Data_Flag2;
extern u8 Send_Data_Flag3;
//姿态解算的标志
extern u8 Attitude_Count_Flag;
//PID控制标志
extern u8 PID_Control_Flag;
extern u8 BMP085_Update_Flag;
extern u8 Flight_Mode;
#endif


