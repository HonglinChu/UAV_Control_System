#ifndef _DATA_READWRITE_H
#define _DATA_READWRITE_H
#include "Includes.h"

void EE_Read_MPU6050_Gyro_Offset(void);
void EE_Read_MPU6050_Acc_Offset(void);
void EE_Save_MPU6050_Gyro_Offset(void);
void EE_Save_MPU6050_Acc_Offset(void);
void EE_Save_MPU6050_PID_Para(void);
void EE_Read_MPU6050_PID_Para(void);
void EE_Save_RC_MAX_MIN(void);
void EE_Read_RC_MAX_MIN(void);
void EE_Save_MAG_OFFSET(void);
void EE_Read_MAG_OFFSET(void);
void EE_Save_Bar_Offset(void);
void EE_Read_Bar_Offset(void);

#endif

