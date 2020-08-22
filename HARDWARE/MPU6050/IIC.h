
#ifndef _IIC_H
#define _IIC_H
#include "sys.h"

//***************************正点原子********************
//IO方向设置
#define SDA_IN()  {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=8<<12;}
#define SDA_OUT() {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=3<<12;}

//IO操作函数	 
#define IIC_SCL    PCout(12) 		//SCL
#define IIC_SDA    PCout(11) 		//SDA	 
#define READ_SDA   PCin(11) 		//输入SDA 
//IIC所有操作函数
void IIC_Delay(void);				//MPU IIC延时函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8   IIC_Receive_Byte(unsigned char ack);//IIC读取一个字节

u8   IIC_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8   IIC_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8   IIC_Write_Byte(u8 addr,u8 reg,u8 data);
u8   IIC_Read_Byte(u8 addr,u8 reg);
u8   IIC_Wait_Ack(void); 		    //IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
#endif

