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
#include "Data_Trans.h"
extern u8 TX_Buffer[50];

u8 RC_Calibration_Flag=0;
u8 Send_PID_Flag=0;

u8 Send_Data_Flag1=0;
u8 Send_Data_Flag2=0;
u8 Send_Data_Flag3=0;

//姿态解算的标志
u8 Attitude_Count_Flag=0;
extern int ALT_BAR;
extern short int ALT_CSB;
//在数据传输的过程中，数据接收使用USART中断，，数据发送使用ＤＭＡ功能
//源程序当中是u16  sum不是u8类型的吗？？
//在协议中没有提到这个
void Send_Check(u8 head, u8 check)
{    
	   u8 i;
	   u8 sum;
       TX_Buffer[0]=0xAA;
	   TX_Buffer[1]=0xAA;
	   TX_Buffer[2]=0xEF;
	   TX_Buffer[3]=0x02;//个数据
	   TX_Buffer[4]=head;
	   TX_Buffer[5]=check;
	   for(i=0;i<6;i++)
	   {
		   sum+=TX_Buffer[i];
	   }
	   TX_Buffer[6]=sum;
	   DMA_USART_Send(7);
}

//发送机体姿态角给上位机
void Data_Send_Status(void)
{  
	 u8 i;
	 u8 sum;
     vs16 temp;
	 TX_Buffer[0]=0xAA;
	 TX_Buffer[1]=0xAA;
	 TX_Buffer[2]=0x01;
	 TX_Buffer[3]=0x0C;//发送12个数据
	 temp=(int)(Euler_Angle.roll*100);
	 TX_Buffer[4]=BYTE1(temp);
	 TX_Buffer[5]=BYTE0(temp);
	 temp=(int)(-Euler_Angle.pitch*100);
	 TX_Buffer[6]=BYTE1(temp);
	 TX_Buffer[7]=BYTE0(temp);
	 temp=(int)(Euler_Angle.yaw*100);
	 TX_Buffer[8]=BYTE1(temp);
	 TX_Buffer[9]=BYTE0(temp);
	 TX_Buffer[10]=0x00;
	 TX_Buffer[11]=0x00;
	 TX_Buffer[12]=0x00;
	 TX_Buffer[13]=0x00;
	 TX_Buffer[14]=0x00;//飞行模式
	 TX_Buffer[15]=QUAD_OK;
	 for(i=0;i<16;i++)
	 {
	  sum+=TX_Buffer[i];
	 }
	 TX_Buffer[16]=sum;
	 DMA_USART_Send(17);
}
//发送传感器的数据ACC GYRO  MAG的数据到上位机
void Data_Send_Sensor1(void)
{
     u8 i=0;
	 u8 sum=0;
	 TX_Buffer[0]=0xAA;//帧头
	 TX_Buffer[1]=0xAA;//帧头
	 TX_Buffer[2]=0x02;//功能字
	 TX_Buffer[3]=0x12;//发送18个数据
	 TX_Buffer[4]=BYTE1(ACC.X);
	 TX_Buffer[5]=BYTE0(ACC.X);
	 TX_Buffer[6]=BYTE1(ACC.Y);
	 TX_Buffer[7]=BYTE0(ACC.Y);
	 TX_Buffer[8]=BYTE1(ACC.Z);
	 TX_Buffer[9]=BYTE0(ACC.Z);
	 TX_Buffer[10]=BYTE1(GYRO.X);
	 TX_Buffer[11]=BYTE0(GYRO.X);
	 TX_Buffer[12]=BYTE1(GYRO.Y);
	 TX_Buffer[13]=BYTE0(GYRO.Y);
	 TX_Buffer[14]=BYTE1(GYRO.Z);
	 TX_Buffer[15]=BYTE0(GYRO.Z);
	 TX_Buffer[16]=BYTE1(MAG.X);
	 TX_Buffer[17]=BYTE0(MAG.X);
	 TX_Buffer[18]=BYTE1(MAG.Y);
	 TX_Buffer[19]=BYTE0(MAG.Y);
	 TX_Buffer[20]=BYTE1(MAG.Z);
	 TX_Buffer[21]=BYTE0(MAG.Z);
	 for(i=0;i<22;i++)
	 {
	   sum+=TX_Buffer[i];
	 }
	 TX_Buffer[22]=sum;         //在这种情况先
	 DMA_USART_Send(23);        
}

//
void Data_Send_Sensor2(void)
{
    u8 i=0;
	u8 sum = 0;
	vs16 _temp;
	vs32 _temp2;
	TX_Buffer[0]=0xAA;
	TX_Buffer[1]=0xAA;
	TX_Buffer[2]=0x07;
	TX_Buffer[3]=6;
	_temp2 = ALT_BAR;
	TX_Buffer[4]=BYTE3(_temp2);
	TX_Buffer[5]=BYTE2(_temp2);
	TX_Buffer[6]=BYTE1(_temp2);
	TX_Buffer[7]=BYTE0(_temp2);
	_temp = ALT_CSB;
	TX_Buffer[8]=BYTE1(_temp);
	TX_Buffer[9]=BYTE0(_temp);
	
	for(i=0;i<9;i++)
	{
		sum += TX_Buffer[i];
	}
	TX_Buffer[10] = sum;
	DMA_USART_Send(11);
}
////发送补偿的数据
//void Data_Send_Offset(void)
//{
//    u8 i=0;
//	  u8 sum=0;
//	  vs16 temp;
//	  TX_Buffer[0]=0xAA;
//	  TX_Buffer[1]=0xAA;
//	  TX_Buffer[2]=0x16;
//	  TX_Buffer[3]=0x04;
//	  temp=(int)(Euler_Offset_Roll*1000);
//	  TX_Buffer[4]=BYTE1(temp);
//	  TX_Buffer[5]=BYTE0(temp);
//	  temp=(int)(Euler_Offset_Pitch*1000);
//	  TX_Buffer[6]=BYTE1(temp);
//	  TX_Buffer[7]=BYTE0(temp);
//	  for(i=0;i<8;i++)
//	 {
//	   sum+=TX_Buffer[i];
//	 }
//	 TX_Buffer[8]=sum;//在这种情况先
//	 DMA_USART_Send(9);
//}
//发送RC_Channel的数据和Motor的数据
void Data_Send_RCData(void)
{ 
	u8 i=0;
	u8 sum=0;
	TX_Buffer[0] = 0xAA;
	TX_Buffer[1] = 0xAA;
	TX_Buffer[2] = 0x03;
	TX_Buffer[3] = 0x14;
	TX_Buffer[4] = BYTE1(THROTTLE);
	TX_Buffer[5] = BYTE0(THROTTLE);
	TX_Buffer[6] = BYTE1(YAW);
	TX_Buffer[7] = BYTE0(YAW);
	TX_Buffer[8] = BYTE1(ROLL);
	TX_Buffer[9] = BYTE0(ROLL);
	TX_Buffer[10]= BYTE1(PITCH);
	TX_Buffer[11]= BYTE0(PITCH);
	TX_Buffer[12]= BYTE1(Motor1);
	TX_Buffer[13]= BYTE0(Motor1);
	TX_Buffer[14]= BYTE1(Motor2);
	TX_Buffer[15]= BYTE0(Motor2);
	TX_Buffer[16]= BYTE1(Motor3);
	TX_Buffer[17]= BYTE0(Motor3);
	TX_Buffer[18]= BYTE1(Motor4);
	TX_Buffer[19]= BYTE0(Motor4);
	TX_Buffer[20]= BYTE1(AUX1);//对应通道5
	TX_Buffer[21]= BYTE0(AUX1);
	TX_Buffer[22]=0x00;
	TX_Buffer[23]=0x00;
	for(i=0;i<24;i++)
	{
	  sum+=TX_Buffer[i];
	}
	TX_Buffer[24]=sum;
	DMA_USART_Send(25);
}
void Data_Send_MotorPWM(void)
{
	    u8 i=0;
	    u8 sum=0;
		TX_Buffer[0] = 0xAA;
		TX_Buffer[1] = 0xAA;
		TX_Buffer[2] = 0x06;
		TX_Buffer[3] = 0x10;
		TX_Buffer[4] = BYTE1(Motor1);
		TX_Buffer[5] = BYTE0(Motor1);
		TX_Buffer[6] = BYTE1(Motor2);
		TX_Buffer[7] = BYTE0(Motor2);
		TX_Buffer[8] = BYTE1(Motor3);
		TX_Buffer[9] = BYTE0(Motor3);
		TX_Buffer[10] = BYTE1(Motor4);
		TX_Buffer[11] = BYTE0(Motor4);
		TX_Buffer[12] = 0x00;
		TX_Buffer[13] = 0x00;
		TX_Buffer[14] = 0x00;
		TX_Buffer[15] = 0x00;
		TX_Buffer[16] = 0x00;
		TX_Buffer[17] = 0x00;
		TX_Buffer[18] = 0x00;
		TX_Buffer[19] = 0x00;
		for(i = 0; i<20 ; i++)
		{
		  sum += TX_Buffer[i];
		 }		
		 TX_Buffer[20] =sum;		
		 DMA_USART_Send(21);		
}
void Send_PID1_Parameters(void)
{
	u8 i;
	u8 sum;
    vs16 temp;
	TX_Buffer[0] = 0xAA;
	TX_Buffer[1] = 0xAA;
	TX_Buffer[2] = 0x11;
	TX_Buffer[3] = 0x12;	
    temp = PID1_ROLL.P * 1000;
	TX_Buffer[4] = BYTE1(temp);
	TX_Buffer[5] = BYTE0(temp);
    temp = PID1_ROLL.I * 1000;
	TX_Buffer[6] = BYTE1(temp);
	TX_Buffer[7] = BYTE0(temp);
    temp = PID1_ROLL.D * 1000;
	TX_Buffer[8] = BYTE1(temp);
	TX_Buffer[9] = BYTE0(temp);
    temp = PID1_PITCH.P * 1000;
	TX_Buffer[10] = BYTE1(temp);
	TX_Buffer[11] = BYTE0(temp);
    temp = PID1_PITCH.I * 1000;
	TX_Buffer[12] = BYTE1(temp);
	TX_Buffer[13] = BYTE0(temp);
    temp = PID1_PITCH.D * 1000;
	TX_Buffer[14] = BYTE1(temp);
	TX_Buffer[15] = BYTE0(temp);
    temp = PID1_YAW.P * 1000;
	TX_Buffer[16] = BYTE1(temp);
	TX_Buffer[17] = BYTE0(temp);
    temp = PID1_YAW.I * 1000;
	TX_Buffer[18] = BYTE1(temp);
	TX_Buffer[19] = BYTE0(temp);
    temp = PID1_YAW.D * 1000;
	TX_Buffer[20] = BYTE1(temp);
	TX_Buffer[21] = BYTE0(temp);	
    for(i=0;i<22;i++)
	{
	  sum+=TX_Buffer[i];
	}
	TX_Buffer[22]=sum;
	DMA_USART_Send(23);
}
void Send_PID2_Parameters(void)
{
	u8 i;
	u8 sum;
    vs16 temp;
	TX_Buffer[0] = 0xAA;
	TX_Buffer[1] = 0xAA;
	TX_Buffer[2] = 0x10;
	TX_Buffer[3] = 0x12;	
    temp = PID2_ROLL.P * 1000;
	TX_Buffer[4] = BYTE1(temp);
	TX_Buffer[5] = BYTE0(temp);
    temp = PID2_ROLL.I * 1000;
	TX_Buffer[6] = BYTE1(temp);
	TX_Buffer[7] = BYTE0(temp);
    temp = PID2_ROLL.D * 1000;
	TX_Buffer[8] = BYTE1(temp);
	TX_Buffer[9] = BYTE0(temp);
    temp = PID2_PITCH.P * 1000;
	TX_Buffer[10] = BYTE1(temp);
	TX_Buffer[11] = BYTE0(temp);
    temp = PID2_PITCH.I * 1000;
	TX_Buffer[12] = BYTE1(temp);
	TX_Buffer[13] = BYTE0(temp);
    temp = PID2_PITCH.D * 1000;
	TX_Buffer[14] = BYTE1(temp);
	TX_Buffer[15] = BYTE0(temp);
    temp = PID2_YAW.P * 1000;
	TX_Buffer[16] = BYTE1(temp);
	TX_Buffer[17] = BYTE0(temp);
    temp = PID2_YAW.I * 1000;
	TX_Buffer[18] = BYTE1(temp);
	TX_Buffer[19] = BYTE0(temp);
    temp = PID2_YAW.D * 1000;
	TX_Buffer[20] = BYTE1(temp);
	TX_Buffer[21] = BYTE0(temp);	
    for(i=0;i<22;i++)
	{
	  sum+=TX_Buffer[i];
	}
	TX_Buffer[22]=sum;
	DMA_USART_Send(23);
}

void Send_PID3_Parameters(void)
{
	u8 i;
	u8 sum;
    vs16 temp;
	TX_Buffer[0] = 0xAA;
	TX_Buffer[1] = 0xAA;
	TX_Buffer[2] = 0x12;
	TX_Buffer[3] = 0x12;	
    temp = Height_PID2.P * 1000;
	TX_Buffer[4] = BYTE1(temp);
	TX_Buffer[5] = BYTE0(temp);
    temp = Height_PID2.I * 1000;
	TX_Buffer[6] = BYTE1(temp);
	TX_Buffer[7] = BYTE0(temp);
    temp = Height_PID2.D * 1000;
	TX_Buffer[8] = BYTE1(temp);
	TX_Buffer[9] = BYTE0(temp);
    temp = Height_PID1.P * 1000;
	TX_Buffer[10] = BYTE1(temp);
	TX_Buffer[11] = BYTE0(temp);
    temp = Height_PID1.I * 1000;
	TX_Buffer[12] = BYTE1(temp);
	TX_Buffer[13] = BYTE0(temp);
    temp = Height_PID1.D * 1000;
	TX_Buffer[14] = BYTE1(temp);
	TX_Buffer[15] = BYTE0(temp);
    //temp = PID2_YAW.P * 1000;
	TX_Buffer[16] = 0x00;
	TX_Buffer[17] = 0x00;
   // temp = PID2_YAW.I * 1000;
	TX_Buffer[18] = 0x00;
	TX_Buffer[19] = 0x00;
   // temp = PID2_YAW.D * 1000;
	TX_Buffer[20] = 0x00;
	TX_Buffer[21] = 0x00;	
    for(i=0;i<22;i++)
	{
	  sum+=TX_Buffer[i];
	}
	TX_Buffer[22]=sum;
	DMA_USART_Send(23);
}

