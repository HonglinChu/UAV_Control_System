#ifndef _DATA_TRANS_H
#define _DATA_TRANS_H
#include "Includes.h"
//定义取  temp的地址。 
#define BYTE0(dwtemp)        (*(char*)(&dwtemp))
#define BYTE1(dwtemp)        (*((char *)(&dwtemp) + 1))  
#define BYTE2(dwTemp)        (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)        (*((char *)(&dwTemp) + 3))
//void NRF_Check_Event(void);
void Data_Send_Status(void);
void Data_Send_Sensor1(void);
void Data_Send_Sensor2(void);
void Data_Send_Offset(void);
void Data_Send_RCData(void);
void Data_Send_MotorPWM(void);
void Send_Check(u8 head,u8 check);

void Send_PID1_Parameters(void);
void Send_PID2_Parameters(void);
void Send_PID3_Parameters(void);
void Send_PID4_Parameters(void);

#endif 

