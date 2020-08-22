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

#include "Usart1.h"
#include "stdio.h"
//定义一个接收缓冲区和一个发送缓冲区
u8 RX_Buffer[50];
u8 TX_Buffer[50];
u8 USART_DMA_Flag=0;
//USART1  初始化  TX9   RX10
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

void Usart_Init(u32 baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
	  USART_InitTypeDef   USART_InitStruct;
	  NVIC_InitTypeDef    NVIC_InitStruct;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,ENABLE);
	
	  GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	  GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	  GPIO_Init(GPIOA,&GPIO_InitStruct);
	   
	  GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
	  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA,&GPIO_InitStruct);
	  
	  NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;
	  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	  NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	  NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	  NVIC_Init(&NVIC_InitStruct);

	  USART_InitStruct.USART_BaudRate=baud;
	  USART_InitStruct.USART_WordLength=USART_WordLength_8b;
	  USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	  USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	  USART_InitStruct.USART_Parity=USART_Parity_No;
	  USART_InitStruct.USART_StopBits=USART_StopBits_1;
	  USART_Init(USART1,&USART_InitStruct);

		//串口中断使能和串口中断类型选择
	  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	  USART_Cmd(USART1,ENABLE);
}
//串口中断函数，上位机发送过来的数据帧有一定的格式
void USART1_IRQHandler(void)
{ 
    
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
	     static u8 receive;
		 static u8 state=0;//表示第几个字节
		 static u8 sum=0;
		 receive=USART1->DR;
		 if((receive==0xAA)&&(state==0))//接收到第一个字节
		 {
		      state=1;
			  RX_Buffer[0]=0xAA;
		 }
		 else if((receive==0xAF)&&(state==1))//接收到第二个字节
		 {
		      state=2;
			  RX_Buffer[1]=0xAF;
		 }
		 else if((receive<0xF1)&&(receive>0)&&(state==2))//接收到第三个字节
		 {
		      state=3;
			  RX_Buffer[2]=receive;
		 }
		 else if((receive<=50)&&(state==3))
		 {
		      state=4;
			  RX_Buffer[3]=receive;
			  sum=0;
		 }
		 else if(state==4)//开始正式接收数据
		 {  
		      RX_Buffer[4+sum]=receive;
			  if(sum==RX_Buffer[3])//接收数据完成，直接进行分析数据
			  {   
				    Analyse_Data(sum+5);
				    sum=0;
				    state=0;
			  }
			  else
			  {
			    sum++;
			  }
		 }
        else 
		{
		    state=0;
		}			
	   USART_ClearITPendingBit(USART1,USART_IT_RXNE);
  }
}
void Analyse_Data(u8 Sum)
{
	 u8 i;
	 u8 temp=0;
     for(i=0;i<Sum-1;i++)
	 {
	   temp+=RX_Buffer[i];
	 }
	 if(temp!=RX_Buffer[Sum-1])return; // 和检测
	 if((RX_Buffer[0]!=0xAA)&&(RX_Buffer[1]!=0xAF))return ;//帧头检测
	 if(RX_Buffer[2]==0x01)
	 {
	        if(RX_Buffer[4]==0x01)//，接收上位机的校准命令
			{
			  ACC_Calibration_Flag=1;
			}
			if(RX_Buffer[4]==0x02)//，接收上位机的校准命令
			{
			  GYRO_Calibration_Flag=1;
			}
			if(RX_Buffer[4]==0x03)//
			{  
			    ACC_Calibration_Flag=1;
			    GYRO_Calibration_Flag=1;
			}
			if(RX_Buffer[4]==0x04)
			{
			    LED1=0;
			    MagFlag.calibratingM=1;
			}
			if(RX_Buffer[4]==0x05)//气压计校准
			{
			    BAR_Calibration_Flag=1;
			}
	 }
	 else if(RX_Buffer[2]==0x02)
	 {
	    if(RX_Buffer[4]==0x01)//将PID发送回电脑标志
		{
			 Send_PID_Flag=1;
		}
	 }
	 else if(RX_Buffer[2]==0x10)//接收PIDhost——computer发送回来的PID的值
	 {
	     PID2_ROLL.P=(float)((short int)(RX_Buffer[4]<<8)|RX_Buffer[5])/1000;
		   PID2_ROLL.I=(float)((short int)(RX_Buffer[6]<<8)|RX_Buffer[7])/1000;
		   PID2_ROLL.D=(float)((short int)(RX_Buffer[8]<<8)|RX_Buffer[9])/1000;
		   PID2_PITCH.P=(float)((short int)(RX_Buffer[10]<<8)|RX_Buffer[11])/1000;
		   PID2_PITCH.I=(float)((short int)(RX_Buffer[12]<<8)|RX_Buffer[13])/1000;
		   PID2_PITCH.D=(float)((short int)(RX_Buffer[14]<<8)|RX_Buffer[15])/1000;
		   PID2_YAW.P=(float)((short int)(RX_Buffer[16]<<8)|RX_Buffer[17])/1000;
		   PID2_YAW.I=(float)((short int)(RX_Buffer[18]<<8)|RX_Buffer[19])/1000;//注意在这里I项是除以100
		   PID2_YAW.D=(float)((short int)(RX_Buffer[20]<<8)|RX_Buffer[21])/1000;
		   //按照协议，接收完成之后需要返回确认
		   Send_Check(RX_Buffer[2],temp);
		  // EE_Save_MPU6050_PID_Para();//保存数据到EEPROM
	 }
	 else if(RX_Buffer[2]==0x11)//接收PIDhost——computer发送回来的PID的值
	 {
	     PID1_ROLL.P=(float)((vs16)(RX_Buffer[4]<<8)|RX_Buffer[5])/1000;
		   PID1_ROLL.I=(float)((vs16)(RX_Buffer[6]<<8)|RX_Buffer[7])/1000;
		   PID1_ROLL.D=(float)((vs16)(RX_Buffer[8]<<8)|RX_Buffer[9])/1000;
		   PID1_PITCH.P=(float)((vs16)(RX_Buffer[10]<<8)|RX_Buffer[11])/1000;
		   PID1_PITCH.I=(float)((vs16)(RX_Buffer[12]<<8)|RX_Buffer[13])/1000;
		   PID1_PITCH.D=(float)((vs16)(RX_Buffer[14]<<8)|RX_Buffer[15])/1000;
		   PID1_YAW.P=(float)((vs16)(RX_Buffer[16]<<8)|RX_Buffer[17])/1000;
		   PID1_YAW.I=(float)((vs16)(RX_Buffer[18]<<8)|RX_Buffer[19])/1000;
		   PID1_YAW.D=(float)((vs16)(RX_Buffer[20]<<8)|RX_Buffer[21])/1000;
		   //按照协议，接收完成之后需要返回确认，但是上位机总是发送超时是怎么回事？？
		   Send_Check(RX_Buffer[2],temp);
		   //EE_Save_MPU6050_PID_Para();//保存数据到EEPROM
	 }
	 else if(RX_Buffer[2]==0x12)//PID3
	 {
		   Height_PID2.P=(float)((vs16)(RX_Buffer[4]<<8)|RX_Buffer[5])/1000;
		   Height_PID2.I=(float)((vs16)(RX_Buffer[6]<<8)|RX_Buffer[7])/1000;
		   Height_PID2.D=(float)((vs16)(RX_Buffer[8]<<8)|RX_Buffer[9])/1000;
		   Height_PID1.P=(float)((vs16)(RX_Buffer[10]<<8)|RX_Buffer[11])/1000;
		   Height_PID1.I=(float)((vs16)(RX_Buffer[12]<<8)|RX_Buffer[13])/1000;
		   Height_PID1.D=(float)((vs16)(RX_Buffer[14]<<8)|RX_Buffer[15])/1000;
		  // PID1_YAW.P=(float)((vs16)(RX_Buffer[16]<<8)|RX_Buffer[17])/1000;
		  // PID1_YAW.I=(float)((vs16)(RX_Buffer[18]<<8)|RX_Buffer[19])/1000;
		  // PID1_YAW.D=(float)((vs16)(RX_Buffer[20]<<8)|RX_Buffer[21])/1000; 
          Send_Check(RX_Buffer[2],temp);//只返回，不接受
	 }
//	  else if(RX_Buffer[2]==0x13)//PID4
//	 { 
//	     Send_Check(RX_Buffer[2],temp);
//	 }
//	  else if(RX_Buffer[2]==0x14)//PID5
//	 { 
//	     Send_Check(RX_Buffer[2],temp);
//	 }
//	  else if(RX_Buffer[2]==0x15)//PID6
//	 { 
//	     Send_Check(RX_Buffer[2],temp);
//	 }
}


	