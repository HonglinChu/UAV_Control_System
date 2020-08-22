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
#include "Position_Control.h"
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )


Flow Optical_Flow;    //定义一个结构体
float     Exp_Roll;
float     Exp_Pitch;
u8        Flow_Buffer[25]; //缓存
short int Coordinate_X_Offset;//进入定点模式的那一刻的位置值
short int Coordinate_Y_Offset;//进入定点模式那一刻的位置值
short int Coordinate_X = 0;   //当前位置值
short int Coordinate_Y = 0;   //当前位置值


PID PID1_POSITION_ROLL;//外
PID PID2_POSITION_ROLL;//内
PID PID1_POSITION_PITCH;
PID PID2_POSITION_PITCH;


void Optical_Flow_Read(void)
{
	
	//计算当前坐标
	Coordinate_X += Optical_Flow.flow_comp_m_x *0.1;//长度mm
    Coordinate_Y += Optical_Flow.flow_comp_m_y *0.1;	
}
//光流定点

void Position_Control(void)
{    
	float Position_Roll_Error=0,Speed_Roll_Error;
    float Position_Pitch_Error=0,Speed_Pitch_Error;
	static float Last_Position_Roll_Error=0,Last_Position_Pitch_Error=0;//上一次roll误差
	static float Position_Roll_Speed=0,Position_Pitch_Speed=0;     //当前roll速度
	static float Last_Position_Roll_Speed=0,Last_Position_Pitch_Speed=0;//上一次roll速度
	//Position_Loop
	Position_Roll_Error = (float)(Coordinate_Y_Offset-Coordinate_Y);//左右
	Position_Pitch_Error= (float)(Coordinate_X_Offset-Coordinate_X);//前后
	//Position_P
	PID1_POSITION_ROLL.POut  =PID1_POSITION_ROLL.P*Position_Roll_Error;
	PID1_POSITION_PITCH.POut =PID1_POSITION_PITCH.P*Position_Pitch_Error;
	//Position_I
	PID1_POSITION_ROLL.IOut +=PID1_POSITION_ROLL.I*Position_Roll_Error;
	PID1_POSITION_ROLL.IOut  =LIMIT(PID1_POSITION_ROLL.IOut,-2000,2000);
	PID1_POSITION_PITCH.IOut +=PID1_POSITION_PITCH.I*Position_Pitch_Error;
	PID1_POSITION_PITCH.IOut  =LIMIT(PID1_POSITION_Pitch.IOut,-2000,2000);
	//Position_D
	PID1_POSITION_ROLL.DOut  =PID1_POSITION_ROLL.D*(Position_Roll_Error-Last_Position_Roll_Error);
	PID1_POSITION_PITCH.DOut =PID1_POSITION_PITCH.D*(Position_Pitch_Error-Last_Position_Pitch_Error);
	Last_Position_Roll_Error =Position_Roll_Error;
	Last_Position_Pitch_Error=Position_Pitch_Error;
	//积分清零
	if(thr<100)
	{
	   PID1_POSITION_ROLL.IOut=0;
	   PID1_POSITION_PITCH.IOut=0;
	}
	//Position_Out
	PID1_POSITION_ROLL.Out=LIMIT((PID1_POSITION_ROLL.POut +PID1_POSITION_ROLL.IOut+PID1_POSITION_ROLL.DOut),-3000,3000);
    PID1_POSITION_PITCH.Out=LIMIT((PID1_POSITION_PITCH.POut+PID1_POSITION_PITCH.IOut +PID1_POSITION_ROLL.DOut),-3000,3000);
	
	
	//SpeeD_Loop
	Position_Roll_Speed = (float)(Optical_Flow.flow_comp_m_y)/1000.0f;//单位mm
	Position_Pitch_Speed= (float)(Optical_Flow.flow_comp_m_x)/1000.0f;
	//期望速度-当前速度，注意正负号问题。?????????????????????????
	Speed_Roll_Error    = -(PID1_POSITION_ROLL.Out -Position_Roll_Speed)*0.01f;
	Speed_Pitch_Error   = -(PID1_POSITION_PITCH.Out-Position_Pitch_Speed)*0.01f;
	//Speed_P
	PID2_POSITION_ROLL.POut =PID2_POSITION_ROLL.P *Speed_Roll_Error ;
	PID2_POSITION_PITCH.POut=PID2_POSITION_PITCH.P*Speed_Pitch_Error;
	//Speed_I
	PID2_POSITION_ROLL.IOut +=PID2_POSITION_ROLL.I*Speed_Roll_Error;
	PID2_POSITION_ROLL.IOut  =LIMIT(PID2_POSITION_ROLL.IOut,-20,20);
	PID2_POSITION_PICTH.IOut+=PID2_POSITION_PICTH.I*Speed_Picth_Error;
	PID2_POSITION_PICTH.IOut =LIMIT(PID2_POSITION_PICTH.IOut,-20,20);
	//Speed_D
	PID2_POSITION_ROLL.DOut =PID2_POSITION_ROLL.D  *(Speed_Roll_Error-Last_Speed_Roll_Error);
	PID2_POSITION_PICTH.DOut=PID2_POSITION_PICTH.D*(Speed_Pitch_Error-Last_Speed_Pitch_Error);
	//Speed_Out
	PID2_POSITION_ROLL.Out   =LIMIT((PID2_POSITION_ROLL.POut +PID2_POSITION_ROLL.IOut+PID2_POSITION_ROLL.DOut),-30,30);
	PID2_POSITION_PICTH.Out  =LIMIT(PID2_POSITION_PITCH.POut+PID2_POSITION_PICTH.IOut+PID2_POSITION_PICTH.DOut),-30,30);
	Exp_Roll=PID2_POSITION_ROLL.Out;
	Exp_Pitch=PID2_POSITION_PICTH.Out;
	Last_Position_Roll_Speed = Position_Roll_Speed;
	Last_Position_Pitch_Speed= Position_Pitch_Speed;
} 
