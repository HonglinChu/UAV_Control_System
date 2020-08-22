/***************************************************************************
* Copyright (c) ¿ªÔ´´úÂë
* All rights reserved.
* ÎÄ¼şÃû³Æ£ºmainº¯Êı
* ÎÄ¼ş±êÊ¶£ºÎŞ
* ÕªÒªËµÃ÷£º±¾³ÌĞòÎ´¾­Ğí¿É²»µÃÓÃÓÚÆäËûÓÃÍ¾
* µ±Ç°°æ±¾£º1.0
* ×÷    Õß: Commodore
* Íê³ÉÈÕÆÚ£º2016Äê04ÔÂ15ÈÕ
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
//RCÍ¨µÀ²¶»ñPWMµÄÒı½Å¶ÔÓ¦TIM4µÄB6 B7  B8  B9 Ê¹ÓÃTIM8¡
#define PWM_IN_PIN1  GPIO_Pin_6
#define PWM_IN_PIN2  GPIO_Pin_7
#define PWM_IN_PIN3  GPIO_Pin_8
#define PWM_IN_PIN4  GPIO_Pin_9
//MotorµÄPWMÊä³ö¶ÔÓ¦A0 A1 A2 A3£¬Ê¹ÓÃTIM2
#define MOTOR_PIN1 GPIO_Pin_0
#define MOTOR_PIN2 GPIO_Pin_1
#define MOTOR_PIN3 GPIO_Pin_2
#define MOTOR_PIN4 GPIO_Pin_3
//LED Ö¸Ê¾µÆ¶ÔÓ¦PA8 DS0(R)  ºÍPD2 DS1(G)
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
	float POut;//±ÈÀıÏîÊä³ö
	float IOut;//»ı·ÖÏîÊä³ö
	float DOut;//Î¢·ÖÏîÊä³ö
	float Out;//×ÜÊä³ö¡£
}PID;
//ÒÔÏÂ¶¼ÊÇÒ»Ğ©Íâ²¿±äÁ¿µÄÉùÃ÷ºÍ½á¹¹ÌåµÄÉùÃ÷
extern PID  PID1_ROLL;
extern PID  PID1_PITCH;
extern PID  PID1_YAW;
extern PID  PID2_ROLL;
extern PID  PID2_PITCH;
extern PID  PID2_YAW;
extern PID Height_PID1;//¸ß¶ÈÍâ»·
extern PID Height_PID2;//¸ß¶ÈÄÚ»·
//±íÊ¾×ËÌ¬½Ç
//±íÊ¾´«¸ĞÆ÷µÄÊä³öÖµ,ÊÇÁ½¸ö×Ö½ÚµÄÊı
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
	      u8 MagExist;      // MAG´æÔÚ
	      u8 MagIssue;      // MAGÓĞÎÊÌâ
	      u8 calibratingM;  // ´ÅÁ¦¼ÆĞ£×¼±ê¼Ç
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

//ËÄ¸öµç»úµÄPWMÖµ
 extern float Motor1;
 extern float Motor2;
 extern float Motor3;
 extern float Motor4;

//PWM²¶»ñµ½µÄËÄ¸öÍ¨µÀµÄPWMÖµ,Õ¾Á½¸ö×Ö½Ú
extern short int ROLL;
extern short int PITCH;
extern short int THROTTLE;
extern short int YAW;
extern short int AUX1;//¶ÔÓ¦Í¨µÀÁù£¬Èıµ²¿ª¹Ø

extern float Roll_Target;
extern float Pitch_Target;
extern float Yaw_Target;
extern float Throttle_Target;
//RCÍ¨µÀµÄ·¶Î§

//Ö÷ÒªÊÇRCÍ¨µÀ×î´óÖµºÍ×îĞ¡ÖµµÄĞ£×¼
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
extern int16_t  HMC58X3_limit[6];//±£´æÈı¸öÖáµÄ×î´óÖµºÍ×îĞ¡Öµ
extern int16_t  *mag_limt;//Ö¸ÏòÊı×éµÄÖ¸Õë£¬½ö½öÊÇÒ»¸öÖ¸Õë¶øÒÑ
//ACC GYRO Ğ£×¼±êÖ¾£¬»ñµÃACCºÍGYROµÄ²¹³¥Öµ
extern u8  ACC_Calibration_Flag;
extern u8  GYRO_Calibration_Flag;
extern u8  BAR_Calibration_Flag;
//·É¿Ø½âËø±êÖ¾
extern u8 QUAD_OK;
//·¢ËÍPID²ÎÊı
extern u8 Send_PID_Flag;
//·¢ËÍÊı¾İµ½ÉÏÎ»»úµÄ±êÖ¾
extern u8 Send_Data_Flag1;
extern u8 Send_Data_Flag2;
extern u8 Send_Data_Flag3;
//×ËÌ¬½âËãµÄ±êÖ¾
extern u8 Attitude_Count_Flag;
//PID¿ØÖÆ±êÖ¾
extern u8 PID_Control_Flag;
extern u8 BMP085_Update_Flag;
extern u8 Flight_Mode;
#endif


