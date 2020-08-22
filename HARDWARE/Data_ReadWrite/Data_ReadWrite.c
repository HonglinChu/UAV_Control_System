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
 \____/\_____/_/ /_/ /_/_/ /_/ / /\_____/\_____/\_____/_/   \_____\
                                                                                                                
* 完成日期:2016.6.1
****************************************************************************/

#include "Data_ReadWrite.h"

#define  EE_MPU6050_GYRO_X_OFFSET    0
#define  EE_MPU6050_GYRO_Y_OFFSET    1
#define  EE_MPU6050_GYRO_Z_OFFSET    2

#define  EE_MPU6050_ACC_X_OFFSET     3
#define  EE_MPU6050_ACC_Y_OFFSET     4
#define  EE_MPU6050_ACC_Z_OFFSET     5

#define  EE_MAG_X_Low                6//磁力计
#define  EE_MAG_Y_Low                7

#define  EE_MAG_Z_Low                8
#define  EE_MAG_X_High               9

#define  EE_MAG_Y_High               10
#define  EE_MAG_Z_High               11

#define  EE_BAR_ALT_High              12
#define  EE_BAR_ALT_Low               13

#define  EE_PID_YAW_D                14

#define  EE_THR_MAX                  15
#define  EE_THR_MIN                  16
#define  EE_YAW_MAX                  17
#define  EE_YAW_MIN                  18
#define  EE_ROL_MAX                  19
#define  EE_ROL_MIN                  20
#define  EE_PIT_MAX                  21
#define  EE_PIT_MIN                  22
extern int32_t _cm_Offset;
extern int16_t  HMC58X3_limit[6];
extern int16_t  *mag_limt;
/* The Address of Data */
u16 VirtAddVarTab[NumbOfVar]  = {0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09,
                                      0xAA0A, 0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E, 0xAA0F, 0xAA10, 0xAA11, 0xAA12, 0xAA13,
									  0xAA14, 0xAA15, 0xAA16};
																			
																																					
// 0xAA00=43520超过了代码区的代码。																			
/* Read GYRO_OFFSET */
void EE_Read_MPU6050_Gyro_Offset(void)
{
	EE_ReadVariable(VirtAddVarTab[EE_MPU6050_GYRO_X_OFFSET], &GYRO_OFFSET.X);
	EE_ReadVariable(VirtAddVarTab[EE_MPU6050_GYRO_Y_OFFSET], &GYRO_OFFSET.Y);
	EE_ReadVariable(VirtAddVarTab[EE_MPU6050_GYRO_Z_OFFSET], &GYRO_OFFSET.Z);
}
/* Read ACC_OFFSET */
void EE_Read_MPU6050_Acc_Offset(void)
{
    EE_ReadVariable(VirtAddVarTab[EE_MPU6050_ACC_X_OFFSET], &ACC_OFFSET.X);
    EE_ReadVariable(VirtAddVarTab[EE_MPU6050_ACC_Y_OFFSET], &ACC_OFFSET.Y);
    EE_ReadVariable(VirtAddVarTab[EE_MPU6050_ACC_Z_OFFSET], &ACC_OFFSET.Z);		
}


/*Read RC MAX MIN*/
void EE_Read_RC_MAX_MIN(void)
{    
    EE_ReadVariable(VirtAddVarTab[EE_THR_MIN], (short int*)(&THR_MIN));
    EE_ReadVariable(VirtAddVarTab[EE_YAW_MAX], (short int*)(&YAW_MAX));
    EE_ReadVariable(VirtAddVarTab[EE_YAW_MIN], (short int*)(&YAW_MIN));
    EE_ReadVariable(VirtAddVarTab[EE_ROL_MAX], (short int*)(&ROLL_MAX));
    EE_ReadVariable(VirtAddVarTab[EE_ROL_MIN], (short int*)(&ROLL_MIN));
    EE_ReadVariable(VirtAddVarTab[EE_PIT_MAX], (short int*)(&PITCH_MAX));
    EE_ReadVariable(VirtAddVarTab[EE_PIT_MIN], (short int*)(&PITCH_MIN));		
}

/*Save_GYRO_OFFSET */
void EE_Save_MPU6050_Gyro_Offset(void)
{
    EE_WriteVariable(VirtAddVarTab[EE_MPU6050_GYRO_X_OFFSET], GYRO_OFFSET.X);
    EE_WriteVariable(VirtAddVarTab[EE_MPU6050_GYRO_Y_OFFSET], GYRO_OFFSET.Y);
	EE_WriteVariable(VirtAddVarTab[EE_MPU6050_GYRO_Z_OFFSET], GYRO_OFFSET.Z);
}                                                                            

/* Save GYRO_OFFSET */
void EE_Save_MPU6050_Acc_Offset(void)
{
    EE_WriteVariable(VirtAddVarTab[EE_MPU6050_ACC_X_OFFSET], ACC_OFFSET.X);
	EE_WriteVariable(VirtAddVarTab[EE_MPU6050_ACC_Y_OFFSET], ACC_OFFSET.Y);
	EE_WriteVariable(VirtAddVarTab[EE_MPU6050_ACC_Z_OFFSET], ACC_OFFSET.Z);
}

/* Save RC MAX MIN */
void EE_Save_RC_MAX_MIN(void)
{
    EE_WriteVariable(VirtAddVarTab[EE_THR_MAX], THR_MAX);
	EE_WriteVariable(VirtAddVarTab[EE_THR_MIN], THR_MIN);
	EE_WriteVariable(VirtAddVarTab[EE_YAW_MAX], YAW_MAX);
	EE_WriteVariable(VirtAddVarTab[EE_YAW_MIN], YAW_MIN);
    EE_WriteVariable(VirtAddVarTab[EE_ROL_MAX], ROLL_MAX);
	EE_WriteVariable(VirtAddVarTab[EE_ROL_MIN], ROLL_MIN);
	EE_WriteVariable(VirtAddVarTab[EE_PIT_MAX], PITCH_MAX);
	EE_WriteVariable(VirtAddVarTab[EE_PIT_MIN], PITCH_MIN);		
}
void EE_Save_MAG_OFFSET(void)
{
	u8 i;
	for(i=0;i<6;i++)//数组的6-11
    EE_WriteVariable(VirtAddVarTab[6+i],HMC58X3_limit[i]);
}
void EE_Read_MAG_OFFSET(void)
{
    u8 i;
	for(i=0;i<6;i++)//数组的6-11
    EE_ReadVariable(VirtAddVarTab[6+i],HMC58X3_limit+i);
}
void EE_Save_Bar_Offset(void)
{   
    EE_WriteVariable(VirtAddVarTab[EE_BAR_ALT_High],*((short int*)(&_cm_Offset)+1));
	EE_WriteVariable(VirtAddVarTab[EE_BAR_ALT_Low], *((short int*)(&_cm_Offset)));
}
void EE_Read_Bar_Offset(void)
{
    EE_ReadVariable(VirtAddVarTab[EE_BAR_ALT_High],(short int*)(&_cm_Offset)+1);
	EE_ReadVariable(VirtAddVarTab[EE_BAR_ALT_Low],(short int*)(&_cm_Offset));
}
