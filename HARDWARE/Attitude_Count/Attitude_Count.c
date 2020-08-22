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
#include "Attitude_Count.h"
#include "MPU6050.h"
#define RtA     57.324841f   //弧度到角度
#define AtR     0.0174533f   //角度到弧度
#define ACC_G   0.0011963f   //加速度到G
#define Gyro_G  0.0609756f   //角速度到度   1度 = 0.0174533 弧度
#define Gyro_Gr 0.0010642f   //角速度变弧度

Sensor_Data  Acc1;
Sensor_Data  Gyro1;
Sensor_Data  Acc2;
Sensor_Data  Gyro2;
Sensor_Data  MAG;
Sensor_Data  ACC_AVG;
Sensor_Data  ACC;
Sensor_Data  GYRO;
Sensor_Data  GYRO_OFFSET;
Sensor_Data  ACC_OFFSET;
Angle Euler_Angle;
/* 参数整定找最佳  从小到大顺序查 */
/* 显示比例后积分  最后再把微分加 */
/* 曲线震荡很频繁  比例度盘要放大 */
/* 曲线漂浮绕大弯  比例度盘往小扳 */
/* 曲线偏离回复慢  积分时间往下降 */
/* 曲线波动周期长  积分时间再加长 */
/* 曲线震荡频率快  先把微分降下来 */
/* 动差大来波动慢  微分时间应加长 */
/* 理想曲线两个波  前高后低四比一 */
/* 一看二调多分析  调节质量不会低 */
#define Kp     0.9f//抑制漂移
#define Ki     0.0008f//较少稳态误差（零偏）
#define halfT  0.0025f//采样周期怎么计算
float q0=1;
float q1=0;
float q2=0;
float q3=0;
float exInt=0;
float eyInt=0;
float ezInt=0;
float hx,hy,mag_yaw;
float vx,vy,vz;
void AHRS_Update(Sensor_Data*Gyro,Sensor_Data*Acc,Sensor_Data*Mag,Angle*euler_angle)
{
	    // float halfT,T;
	    // static u32   Last_Time=0;
	    float ex,ey,ez;
	    float norm,sin_roll,sin_pitch,cos_roll,cos_pitch;
	    float ax=Acc->X,ay=Acc->Y,az=Acc->Z;
	    float gx=Gyro->X,gy=Gyro->Y,gz=Gyro->Z;
	    // 辅助变量，以减少重复操作数
       float q0q0 = q0*q0;
       float q0q1 = q0*q1;
       float q0q2 = q0*q2;
       float q0q3 = q0*q3;
       float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;
	   //T=((float)(Micros()-Last_Time))/1000000.0f;//单位化成秒
	   //halfT=T/2.0f;
		//Printf("halfT:%.6lf",halfT);
		//弧度每秒
		gx*=Gyro_Gr;
		gy*=Gyro_Gr;
		gz*=Gyro_Gr;
		if(ax*ay*az==0)
		{
		  return;
		}
		//加速度单位化,a为传感器的重力在飞行器的分量
		norm=sqrt(ax*ax+ay*ay+az*az);
		
		ax=ax/norm;
		ay=ay/norm;
		az=az/norm;
		
		
		vx = 2*(q1q3-q0q2);
		vy = 2*(q0q1 + q2q3);
		vz = 1-2*(q1q1 + q2q2);
		
		
		//向量积
		ex = (ay*vz - az*vy);
		ey = (az*vx - ax*vz);
		ez = (ax*vy - ay*vx);
		
		//通过PI调节消除误差		
		exInt = exInt + VariableParameter(ex)*ex*Ki;
		eyInt = eyInt + VariableParameter(ey)*ey*Ki;
		ezInt = ezInt + VariableParameter(ez)*ez*Ki;
		
    gx = gx + Kp*VariableParameter(ex)*ex+exInt;
		gy = gy + Kp*VariableParameter(ey)*ey+eyInt;
		gz = gz + Kp*VariableParameter(ez)*ez+ezInt;
		
		
		
		q0 = q0 + (-q1*gx - q2*gy - q3*gz)* halfT;
		q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
		q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
		q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;
		
		//四元数单位化，只有单位化的四元数才能表示旋转矩阵
		norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
		q0 = q0 / norm;
		q1 = q1 / norm;
		q2 = q2 / norm;
		q3 = q3 / norm;
		
		
	    if((MagFlag.MagExist==1)&&(MagFlag.MagIssue==0))//磁力计存在 并且磁力传感器数值正常 则通过磁力计估算YAW
	    {   //计算欧拉角的三角函数
			 sin_roll  = sin(atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1));
			 sin_pitch = sin(asin(-2* q1q3 + 2*q0q2));
			 cos_roll  = cos(atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1));
			 cos_pitch = cos(asin(-2* q1q3 + 2*q0q2));
			 //计算hx和hy
			 hx=Mag->X*cos_pitch+Mag->Y*sin_pitch*sin_roll-Mag->Z*cos_roll*sin_pitch;
			 hy=Mag->Y*cos_roll+Mag->Z*sin_roll;
			 mag_yaw=-atan2(hy,hx)*RtA;//这里为很么是个负号	
		//	 euler_angle->yaw=mag_yaw;
        //   陀螺仪积分得到YAW
			 euler_angle->yaw+=Gyro->Z*Gyro_G*0.005f;//角度每秒
		   if((mag_yaw>90 && euler_angle->yaw<-90)||(mag_yaw<-90 && euler_angle->yaw>90)) 
			 {
				 euler_angle->yaw = -euler_angle->yaw * 0.988f + mag_yaw * 0.012f;
			 }
			 else
			 {				 
			     euler_angle->yaw = euler_angle->yaw * 0.988f + mag_yaw * 0.012f;
			 }		 
	     }
		else//磁力传感器不存在或者数值不正常
	    {		
		  euler_angle->yaw+=Gyro->Z*Gyro_G*0.005f;
		  //euler_angle->yaw=atan2(2 * (q0q1 + q2q3), -2*q2q2-2*q3q3+1)*RtA;//yaw
		}//计算pitch，计算roll，
		euler_angle->pitch = safe_asin(-2* q1q3 + 2*q0q2)*RtA;//这里有个正负号问题
	    euler_angle->roll  = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1)*RtA;
}
//******************************************************************************
// 扩展的反正弦函数：检查输入值的范围（-1~1）、若输入非数字（isnan函数判断）则返回0
//******************************************************************************
float safe_asin(float v)
{
	if (isnan(v)) return 0.0f;

	if (v >= 1.0f) return 3.1415926/2;

	if (v <= -1.0f)return -3.1415926/2;
	
	return asin(v);
}
float Num_To_Dps(short int Num)//转化成度
{
     float temp;
	 temp=(float)Num*Gyro_G;
	 return temp;
}
/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) =  b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
					
#define  IIR_ORDER     4    //使用IIR滤波器的阶数
double   b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};//系数b
double   a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f,-1.8476f,0.3708f};//系数a

===================================================================================================*/
#define  IIR_ORDER     4    //使用IIR滤波器的阶数
double   b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};//系数b
double   a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f,-1.8476f,0.3708f};//系数a
double   InPut_IIR[3][IIR_ORDER+1] = {0};

double   OutPut_IIR[3][IIR_ORDER+1] = {0};double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}
/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
#define KALMAN_Q        0.02
#define KALMAN_R        6.0000
/*           卡尔曼对三个轴加速度进行滤波处理           */
double KalmanFilter(double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
    if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
short int File_Buf[6][10];//
void Filter_Acc(Sensor_Data*Acc)
{
    // 加速度计IIR滤波  低通滤波器消除高噪声 //发现滤波效果并不是很好，，倒不如滑动平均滤波
	Acc->X = IIR_I_Filter(Acc->X, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	Acc->Y = IIR_I_Filter(Acc->Y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	Acc->Z = IIR_I_Filter(Acc->Z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
    //Acc->X=KalmanFilter(Acc->X,KALMAN_Q,KALMAN_R);//可能计算有问题暂时不用
	//Acc->Y=KalmanFilter(Acc->Y,KALMAN_Q,KALMAN_R);
	//Acc->Z=KalmanFilter(Acc->Z,KALMAN_Q,KALMAN_R);
//	static int num = 0;
//	u8 i=0;
//	float sum1=0,sum2=0,sum3=0;
//	File_Buf[3][num] =Acc->X;
//	File_Buf[4][num] =Acc->Y;
//	File_Buf[5][num] =Acc->Z;
//	
//	for(i=0;i<10;i++)
//	{
//         sum1 += File_Buf[3][i];
//		 sum2 += File_Buf[4][i];
//		 sum3 += File_Buf[5][i];
//    }
//	Acc->X= sum1 / 10;
//	Acc->Y= sum2 / 10;
//	Acc->Z= sum3 / 10;
//	num = (num + 1) % 10;
}

void Filter_Gyro(Sensor_Data*Gyro)
{
    static int num = 0;
	u8 i=0;
	float sum1=0,sum2=0,sum3=0;
	File_Buf[0][num] =Gyro->X;
	File_Buf[1][num] =Gyro->Y;
	File_Buf[2][num] =Gyro->Z;
	
	for(i=0;i<10;i++)
	{
         sum1 += File_Buf[0][i];
		 sum2 += File_Buf[1][i];
		 sum3 += File_Buf[2][i];
    }
	Gyro->X= sum1 / 10;
	Gyro->Y= sum2 / 10;
	Gyro->Z= sum3 / 10;
	num = (num + 1) % 10;
}

