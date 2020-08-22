/******************* (C) COPYRIGHT 2016  ***************************
 * 文件名   ：hmc5883.c
 * 描述     ：hmc5883配置
 * 实验平台 ：
 * 库版本   ：ST3.5.0
 *
 *  
*********************************************************************************/
#include "HMC5883L.h"
#define Gyro_G  0.0609756f   //角速度到度   1度 = 0.0174533 弧度
//当前磁场的最大值和最小值
_Flag    MagFlag;
int16_t  HMC58X3_limit[6]={0};
int16_t mag[3];
/*====================================================================================================*
**函数 : Init_HMC5883L
**功能 : 指南针初始化
**输入 : None
**輸出 : 状态
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void HMC5883L_Init(void)
{
	u8 ack; 
	ack = IIC_Read_Byte(MAG_ADDRESS,0x0A);//读取ID号
	if (!ack)
	{
	  MagFlag.MagExist=0;
	  return ;
	}
	MagFlag.MagExist=1;
	// leave test mode
	IIC_Write_Byte(MAG_ADDRESS, HMC58X3_R_CONFA, 0x78);   // 0x70Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
	IIC_Write_Byte(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   //0x20 Configuration Register B  -- 001 00000    configuration gain 1.33Ga
	IIC_Write_Byte(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
	delay_ms(100);
}
	
/*====================================================================================================*/
/*====================================================================================================*
**函数 : hmc5883lRead
**功能 : 度取地磁数据
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/

void HMC5883lRead(Sensor_Data*magData)
{
	u8 buf[6];
	static int An[3]={0,0,0};
	//读取寄存器数据
	IIC_Read_Len(MAG_ADDRESS, MAG_DATA_REGISTER,6,buf);
	//十位深度滤波
	An[0]-=An[0]/10;
	An[0]+=(int16_t)buf[0] << 8 | buf[1];//X轴
	mag[0] =An[0]/10;
	
	An[1]-=An[1]/10;
	An[1]+=(int16_t)buf[4]<< 8  | buf[5];//Y轴
	mag[1] =An[1]/10;
	
	An[2]-=An[2]/10;
	An[2]+=(int16_t)buf[2] << 8 | buf[3];//Z轴
	mag[2] =An[2]/10;
	if(MagFlag.calibratingM!=0)
	{
		MagFlag.MagIssue=0;
		Mag_Calibration(mag);//进行校准,三个轴的最大值和最小值
	}
	else 
	{
	  magData->X = mag[0] -(HMC58X3_limit[3] +HMC58X3_limit[0])/2;//(最大值+最小值)/2
      magData->Y = mag[1] -(HMC58X3_limit[4] +HMC58X3_limit[1])/2;
      magData->Z = mag[2] -(HMC58X3_limit[5] +HMC58X3_limit[2])/2;
	}
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Mag_Calibration
**功能 : 地磁校准
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Mag_Calibration(int16_t *array)
{
	u8 cy,num;
	static u8  clen_MagFlag=1; 
	static float x,y,z; 
	//校准之前先把之前数据清零
	if(clen_MagFlag)
	{
		clen_MagFlag = 0;
		x=y=z=0;
		for(cy=0;cy<6;cy++)
			HMC58X3_limit[cy]=0;
	}
	// 开始采集寻找三个轴的最大和最小值
	for(cy=0;cy<3;cy++)//0,1,2存储最小值3,4,5存储最大值
	{
		if(HMC58X3_limit[cy]>array[cy]) 
		{
			HMC58X3_limit[cy]= array[cy];//找最小
		}
		else if(HMC58X3_limit[cy+3]<array[cy]) 
		{
			HMC58X3_limit[cy+3]=array[cy];//找最大
		}
	}
	//下面就是判断进行地磁校准的动作利用加速度计判断是否垂直，利用陀螺仪判断是否转满了360度
	if(MagFlag.calibratingM == 1 && (absu16(ACC.Z) > 5000))  
	{
	    z+=GYRO.Z * Gyro_G * 0.002f;
		if(absFloat(z)>=360)  
		{
			MagFlag.calibratingM = 2;
			LED1=1;
		}
	}
	if(MagFlag.calibratingM == 2 && (absu16(ACC.X) > 5000))   
	{   
	    x += GYRO.X * Gyro_G * 0.002f;
		if(absFloat(x)>360)  
		{
			MagFlag.calibratingM = 3;
			LED1=0;
		}
	}
	if(MagFlag.calibratingM == 3 && (absu16(ACC.Y) > 5000))   
	{
	    y += GYRO.Y * Gyro_G * 0.002f;
		if(absFloat(y)>360)  
		{   
		    //三个轴的最值都偏小说明地磁有问题，停用地磁  
			for(cy=0;cy<6;cy++)	
			{   
			  //三个轴的最值都在-20和20之间
			  if(absu16(HMC58X3_limit[cy])<20)num++;
			}
			if(num>2)MagFlag.MagIssue = 1;//有问题
			LED1=1;
			clen_MagFlag = 1;
			MagFlag.calibratingM = 0;
			EE_Save_MAG_OFFSET();
		}
	}	
}
