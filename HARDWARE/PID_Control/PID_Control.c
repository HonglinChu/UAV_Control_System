/***************************************************************************
* Copyright (c) 
* All rights reserved.
* 文件名称：
* 文件标识：无
* 摘    要：
* 当前版本：
*                                     
      ____                                         __    
     / ___\                                       / /
    / /                                          / /      _    
   / /   _____  ________  ________  _____  _____/ /____  / \_______ 
  / /   / ___ \/ __  __ \/ __  __ \/ ___ \/ ___  / ___ \/ ___/ ___ \
 / /___/ /__/ / / / / / / / / / / / /__/ / /__/ / /__/ / /  / /____/
 \____/\_____/_/ /_/ /_/_/ /_/ / /\_____/\_____/\_____/_/   \_____\
                                                                                                                
*完成日期:2016.6.1
****************************************************************************/
#include "PID_Control.h"
#include "Attitude_Count.h"
#include "MPU6050.h"
#include "Height_Control.h"
//PID 参数初始化
PID PID1_ROLL;
PID PID1_PITCH;
PID PID1_YAW;
PID PID2_ROLL;
PID PID2_PITCH;
PID PID2_YAW;
float Roll_Target;
float Pitch_Target;
float Yaw_Target;
float Throttle_Target;
void PID_Init(void)
{ 
    PID1_ROLL.P=0.008; //2.55
	  PID1_ROLL.I=0.08; //0.01
	  PID1_ROLL.D=1;    //1，1.2, 1.5，
	  PID1_PITCH.P=0.008;//2.55
	  PID1_PITCH.I=0.05;//0.01
	  PID1_PITCH.D=1;   //90
	  PID1_YAW.P=0.005; //4.3
	  PID1_YAW.I=0.05;  //0.002
	  PID1_YAW.D=0.5;   //0
	
	  PID2_ROLL.P=0.8;  //0.6
	  PID2_ROLL.I=0.1;  //0.035
	  PID2_ROLL.D=2;   //2.6
	  PID2_PITCH.P=0.8;//0.6
	  PID2_PITCH.I=0.1;//0.035
	  PID2_PITCH.D=2;//2.6
	  PID2_YAW.P=1.2;//4.5
	  PID2_YAW.I=1;//0.035
	  PID2_YAW.D=1;//1.2
	
	  Height_PID1.P=1.5;//高度外环
	  Height_PID1.I=0;
	  Height_PID1.D=2.5;
	  Height_PID2.P=0.3;//高度内环
	  Height_PID2.I=0.12;
	  Height_PID2.D=1.4; 
}

//飞行准备
void Fly_Prepare(void)
{ 
	 u8 i;
	 while(1)
	 {
	   //飞机解锁函数
	   QUAD_Armed(20);	
	   if(QUAD_OK)
	   {
		  break;//如果解锁成功，跳出循环
	   } //一个是通过上位机发送标志，另一个是通过RC发送信号
	   if(RC_ADJUST_CMD)//翻滚和俯仰角最大
	   {
			 i++;
			 if(i==40)
			 {
		        RC_Calibration_Flag=0;
			    RC_Adjust();//读取量程范围
				i=0;
			 }
		}
		 if(SET_OFFSET_CMD)//翻滚和俯仰通道最小,通过遥控进行校准,//注意在这种情况下取消了上位机发送的情况
		 { 
			  ACC_Calibration_Flag=1;//直接在定时器函数里面进行求解补偿值
			  GYRO_Calibration_Flag=1;//只需要求解一次
		 }
  	      LED0=!LED0;
		  LED1=!LED1;
		  delay_ms(50);   
    }
	  //主要是涉及到RC通道的量程和中值和ACC和GYRO的补偿值
	 EE_Read_MPU6050_Acc_Offset(); //加速度
	 EE_Read_MPU6050_Gyro_Offset();//陀螺仪
	 EE_Read_Bar_Offset();//气压计
	 EE_Read_RC_MAX_MIN();//遥控量程
	 //EE_Read_MAG_OFFSET();//磁力计
	 Half_Roll =(ROLL_MAX+ROLL_MIN)/2;
	 Half_Pitch=(PITCH_MAX+PITCH_MIN)/2;
	 Half_Yaw  =(YAW_MAX+YAW_MIN)/2;    
 }

//******************************************串级PID控制*************************************************************************//
/*******************************************低通滤波系数表********************************************************************************/
#define MAX_CTRL_ANGLE			25.0f		//遥控能达到的最大角度
#define ANGLE_TO_MAX_AS 		30.0f		//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE //外环积分幅度

#define MAX_CTRL_ASPEED 	 	300.0f				 //ROL,PIT允许的最大控制角速度
#define MAX_CTRL_YAW_SPEED 	    150.0f			     //YAW允许的最大控制角速度
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED//内环积分幅度

#define MAX_PWM		  100	//	最大PWM输出为100%油门
#define MAX_THR       80    //	油门通道最大占比80%，留20%给控制量
 
#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b)) 
/*******************************************低通滤波系数表********************************************************************************/
//计算油门权重，油门线性补偿
float   Thr_Weight=0;
float   Throttle_Value;
void Thr_Ctrl(float T)
{
    static float thr;
    static float Thr_tmp;
	//油门值 0 ~ 1000
    thr =Throttle_Target-1000;
	//低通滤波
    Thr_tmp  += 10 *3.14f *T *(thr/400.0f - Thr_tmp);
	//后边多处分离数据会用到这个值
    Thr_Weight= LIMIT(Thr_tmp,0,1); 
	//油门值在0-800之间
	if(Flight_Mode>=1)//高度控制模式
	{
	   Height_Control(T,thr);
       Throttle_Value = LIMIT((Thr_Weight *Height_PID2.Out),0,800);//实际使用值
	}
	else
	{  
	   Throttle_Value = thr;
	}
}
 ////////////////////////////////////外环PID控制////////////////////////////////////////////////////////////////
void PID_Out_Control(void)
{ 
	 static u32  Time_Now, Time_Last;
	 float   T;
     float  Roll_Angle_Err=0,Pitch_Angle_Err=0,Yaw_Angle_Err=0;
	 static float Roll_Angle_Err_Last=0,Pitch_Angle_Err_Last=0,Yaw_Angle_Err_Last=0;
	 static float Weight_Ang[3];//误差x,y,z的权重
	 //得到前后两次的时间差单位s
	 Time_Now=Micros();
	 T=((float)(Time_Now-Time_Last))/1000000.0f;
	 if(Time_Last==0)
	 {
	   Time_Last=Time_Now;
	   return;
	 }
	 //角度误差
	 Roll_Angle_Err  =Roll_Target -Euler_Angle.roll;
	 Pitch_Angle_Err =Pitch_Target-Euler_Angle.pitch;
	 Yaw_Angle_Err   =LIMIT((Yaw_Target -Euler_Angle.yaw),-45,45);
	 //外环I
	 PID1_ROLL.IOut +=PID1_ROLL.I *Roll_Angle_Err*T;
	 PID1_PITCH.IOut+=PID1_PITCH.I*Pitch_Angle_Err*T;
	 PID1_YAW.IOut  +=PID1_YAW.I  *Yaw_Angle_Err*T;
	 PID1_ROLL.IOut  =LIMIT(PID1_ROLL.IOut, -Thr_Weight*CTRL_2_INT_LIMIT,Thr_Weight*CTRL_2_INT_LIMIT);
	 PID1_PITCH.IOut =LIMIT(PID1_PITCH.IOut,-Thr_Weight*CTRL_2_INT_LIMIT,Thr_Weight*CTRL_2_INT_LIMIT);
	 PID1_YAW.IOut   =LIMIT(PID1_YAW.IOut,  -Thr_Weight*CTRL_2_INT_LIMIT,Thr_Weight*CTRL_2_INT_LIMIT);
	 //计算权重,角度误差为30的时候，期望角速度达到最大
	 Weight_Ang[0]=ABS(Roll_Angle_Err) /ANGLE_TO_MAX_AS;
	 Weight_Ang[1]=ABS(Pitch_Angle_Err)/ANGLE_TO_MAX_AS;
	 Weight_Ang[2]=ABS(Yaw_Angle_Err)  /ANGLE_TO_MAX_AS; 
	 //外环D
	 PID1_ROLL.DOut  =10*PID1_ROLL.D *(Roll_Angle_Err-Roll_Angle_Err_Last)  *(0.005f/T)*(0.65f+0.35f*Weight_Ang[0]);
	 PID1_PITCH.DOut =10*PID1_PITCH.D*(Pitch_Angle_Err-Pitch_Angle_Err_Last)*(0.005f/T)*(0.65f+0.35f*Weight_Ang[1]);
	 PID1_YAW.DOut   =10*PID1_YAW.D  *(Yaw_Angle_Err-Yaw_Angle_Err_Last)    *(0.005f/T)*(0.65f+0.35f*Weight_Ang[2]);
	 //外环P,角度误差限幅
	 Roll_Angle_Err  =LIMIT(Roll_Angle_Err, -90,90);
	 Pitch_Angle_Err =LIMIT(Pitch_Angle_Err,-90,90);
	 Yaw_Angle_Err   =LIMIT(Yaw_Angle_Err,  -90,90); 
	 //外环总输出
	 PID1_ROLL.Out   = PID1_ROLL.P *(Roll_Angle_Err + PID1_ROLL.IOut +PID1_ROLL.DOut);
	 PID1_PITCH.Out  = PID1_PITCH.P*(Pitch_Angle_Err+ PID1_PITCH.IOut+PID1_PITCH.DOut);
	 PID1_YAW.Out    = PID1_YAW.P  *(Yaw_Angle_Err  + PID1_YAW.IOut  +PID1_YAW.DOut);
	 //感觉这个积分清零还是必须要有的
	 if(Throttle_Value<100)
	 {
		 PID1_PITCH.IOut=0;
		 PID1_ROLL.IOut =0;
		 PID1_YAW.IOut  =0;
	 }
	 //保存上次的值
	 Roll_Angle_Err_Last =Roll_Angle_Err; 
	 Pitch_Angle_Err_Last=Pitch_Angle_Err;
	 Yaw_Angle_Err_Last  =Yaw_Angle_Err;
	 //保存上一次的时间单位us
	 Time_Last=Time_Now;
}

/////////////////////////////////////////内环PID控制/////////////////////////////////////////////////////////////
void PID_In_Control(void)
{
	 float T,M[4];
	 static u32   Time_Now ,Time_Last=0;
	 float        Roll_Speed_Err=0,Pitch_Speed_Err=0,Yaw_Speed_Err=0;
	 float        Gyro_X_Now,Gyro_Y_Now,Gyro_Z_Now;     //角速度   单位度每秒 
	 static float Angular_Acc[3]={1,1,1};//角加速度默认参数1,1,1
	 static float Gyro_X_Last=0,Gyro_Y_Last=0,Gyro_Z_Last=0;
	 float        Weight_Sp[3];
	 Time_Now=Micros();
	 T=((float)(Time_Now-Time_Last))/1000000.0f;//单位化成秒
	 if(Time_Last==0)
	 {
	    Time_Last=Time_Now;
		return;
	 }
	 //通过陀螺仪数据得到当前角速度   单位度每秒
	 Gyro_X_Now=Num_To_Dps(GYRO.X);
	 Gyro_Y_Now=Num_To_Dps(GYRO.Y);
	 Gyro_Z_Now=Num_To_Dps(GYRO.Z);  
	 //期望角速度限制幅度
	 PID1_ROLL.Out  =LIMIT(MAX_CTRL_ASPEED*(PID1_ROLL.Out/ANGLE_TO_MAX_AS), -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED);
	 PID1_PITCH.Out =LIMIT(MAX_CTRL_ASPEED*(PID1_PITCH.Out/ANGLE_TO_MAX_AS),-MAX_CTRL_ASPEED,MAX_CTRL_ASPEED);
	 PID1_YAW.Out   =LIMIT(MAX_CTRL_ASPEED*(PID1_YAW.Out/ANGLE_TO_MAX_AS),  -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED);
   //角速度误差	
	 Roll_Speed_Err =(PID1_ROLL.Out -Gyro_X_Now) *(300.0f/MAX_CTRL_ASPEED);	
	 Pitch_Speed_Err=(PID1_PITCH.Out-Gyro_Y_Now) *(300.0f/MAX_CTRL_ASPEED);//注意这里符号发生了正负的变化
	 Yaw_Speed_Err  =(PID1_YAW.Out  -Gyro_Z_Now) *(300.0f/MAX_CTRL_ASPEED);//正负号的问题
	 //角速度直接微分得到角加速度
	 Angular_Acc[0]=(Gyro_X_Now  -Gyro_X_Last) *(0.002f/T);
	 Angular_Acc[1]=(Gyro_Y_Now  -Gyro_Y_Last) *(0.002f/T);
	 Angular_Acc[2]=(Gyro_Z_Now  -Gyro_Z_Last) *(0.002f/T);
	 //角速度误差权重  
	 Weight_Sp[0]=ABS( Roll_Speed_Err)/MAX_CTRL_ASPEED;
	 Weight_Sp[1]=ABS( Pitch_Speed_Err)/MAX_CTRL_ASPEED;
	 Weight_Sp[2]=ABS( Yaw_Speed_Err)/MAX_CTRL_YAW_SPEED;
	 //内环I
	 PID2_ROLL.IOut += PID2_ROLL.I *(Roll_Speed_Err-Angular_Acc[0])*T;
	 PID2_PITCH.IOut+= PID2_PITCH.I*(Pitch_Speed_Err-Angular_Acc[1])*T;
	 PID2_YAW.IOut  += PID2_YAW.I  *(Yaw_Speed_Err-Angular_Acc[2])*T;
	 //内环积分限幅1
	 PID2_ROLL.IOut  =LIMIT(PID2_ROLL.IOut, -Thr_Weight *CTRL_1_INT_LIMIT , Thr_Weight *CTRL_1_INT_LIMIT );
	 PID2_PITCH.IOut =LIMIT(PID2_PITCH.IOut,-Thr_Weight *CTRL_1_INT_LIMIT , Thr_Weight *CTRL_1_INT_LIMIT );
	 PID2_YAW.IOut   =LIMIT(PID2_YAW.IOut,  -Thr_Weight *CTRL_1_INT_LIMIT , Thr_Weight *CTRL_1_INT_LIMIT );
	 //内环D感觉这里的负号要去掉。
	 PID2_ROLL.DOut  =PID2_ROLL.D *(-10*Angular_Acc[0])*(0.002f/T);
	 PID2_PITCH.DOut =PID2_PITCH.D*(-10*Angular_Acc[1])*(0.002f/T);
	 PID2_YAW.DOut   =PID2_YAW.D  *(-10*Angular_Acc[2])*(0.002f/T);
	 //内环总输出
	 PID2_ROLL.Out  =3*(0.2*LIMIT((0.45f+0.55f*Weight_Sp[0]),0,1)*PID1_ROLL.Out+0.8*PID2_ROLL.P*(Roll_Speed_Err+PID2_ROLL.IOut+PID2_ROLL.DOut));
	 PID2_PITCH.Out =3*(0.2*LIMIT((0.45f+0.55f*Weight_Sp[1]),0,1)*PID1_PITCH.Out+0.8*PID2_PITCH.P*(Pitch_Speed_Err+PID2_PITCH.IOut+PID2_PITCH.DOut));
	 PID2_YAW.Out   =3*(0.2*LIMIT((0.45f+0.55f*Weight_Sp[2]),0,1)*PID1_YAW.Out+0.8*PID2_YAW.P*(Yaw_Speed_Err+PID2_YAW.IOut+PID2_YAW.DOut));
	 if(Throttle_Value<100)
	 {
	   PID2_ROLL.IOut=0;
	   PID2_PITCH.IOut=0;
	   PID2_YAW.IOut=0;
	 }
	 //保存这次的角速度误差
	 Gyro_X_Last= Gyro_X_Now;
	 Gyro_Y_Last= Gyro_Y_Now;
	 Gyro_Z_Last= Gyro_Z_Now;
	 Time_Last=Time_Now;//保存上一次的时间单位us
	 Thr_Ctrl(T);//对油门值进行处理
	 //PWM输出
	 if(QUAD_OK) 
	 {
		 PID2_YAW.Out=LIMIT(PID2_YAW.Out,-400,400);//限制幅度
		   
		 M[0] = LIMIT((+PID2_ROLL.Out - PID2_PITCH.Out + PID2_YAW.Out),-1000,1000);  
		 M[1] = LIMIT((-PID2_ROLL.Out - PID2_PITCH.Out - PID2_YAW.Out),-1000,1000);
		 M[2] = LIMIT((-PID2_ROLL.Out + PID2_PITCH.Out + PID2_YAW.Out),-1000,1000);
		 M[3] = LIMIT((+PID2_ROLL.Out + PID2_PITCH.Out - PID2_YAW.Out),-1000,1000);
		   
		 M[0] =(0.55f+0.45f*ABS(M[0])/1000.0f)*M[0]*Thr_Weight+Throttle_Value;
		 M[1] =(0.55f+0.45f*ABS(M[1])/1000.0f)*M[1]*Thr_Weight+Throttle_Value;
		 M[2] =(0.55f+0.45f*ABS(M[2])/1000.0f)*M[2]*Thr_Weight+Throttle_Value;
		 M[3] =(0.55f+0.45f*ABS(M[3])/1000.0f)*M[3]*Thr_Weight+Throttle_Value;
		   
		 Motor1 =LIMIT(M[0],100,1000)+1000;
		 Motor2 =LIMIT(M[1],100,1000)+1000;
		 Motor3 =LIMIT(M[2],100,1000)+1000;
		 Motor4 =LIMIT(M[3],100,1000)+1000; 
		 /* Refresh The Motor PWM */
		 Motor_Refresh(Motor1, Motor2, Motor3, Motor4);
	}
	else
	{
		Motor1=1000;
		Motor2=1000;
		Motor3=1000;
		Motor4=1000;
		Motor_Refresh(1000, 1000,1000,1000);    
	}
} 

//限制数据的范围
float Range_Limit(float Pre,float MAX,float MIN)
{
    if(Pre>MAX)return MAX;
	else if(Pre<MIN)return MIN;
	else return Pre;
}
//RC计算期望的角度函数
//#define MAX_CONTROL_THROTTLE  1600.0f
//#define CONTROL_NUM           ((MAX_CONTROL_THROTTLE-1000.0f)/((float)(THR_MAX-THR_MIN)))
void Calculate_Target(void) 
{
	  Roll_Target =50*(float)(ROLL-Half_Roll)/1000.0f; 
	  Pitch_Target=50*(float)(PITCH-Half_Pitch)/1000.0f;
	 // Throttle_Target=(float)(THROTTLE-1000)*0.8+1000.0f;
	  Throttle_Target=(float)THROTTLE;
	  if((ROLL<1520)&&(ROLL>1480))
	  {
	    Roll_Target=0;
	  }
	  if((PITCH<1520)&&(PITCH>1480))
	  {
	     Pitch_Target=0;
	  }
      //Yaw_Target=100.0f*(YAW-Half_Yaw)/(YAW_MAX-YAW_MIN);	 
	  if(QUAD_OK==0)//暂时将当前角度设定为目标角度
	  {
	     Yaw_Target=Euler_Angle.yaw;  
	  }
//	  if((Yaw_Target<2)&&(Yaw_Target>-2))//这个是角速度
//	  {
//	     Yaw_Target=0;
//	  }
}


