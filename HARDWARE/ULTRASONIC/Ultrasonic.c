#include "Ultrasonic.h"


void Ultrasonic_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	       
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;					    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//设为推挽输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(GPIOB, &GPIO_InitStructure);	        //初始化外设GPIO 	
  GPIO_ResetBits(GPIOB,GPIO_Pin_2);
}

void Ultrasonic_Pulsing(void)
{ 
  GPIO_SetBits(GPIOB,GPIO_Pin_2);		  //送>10US的高电平
  delay_us(20);
  GPIO_ResetBits(GPIOB,GPIO_Pin_2);	
}
//ECHO捕获引脚在PWM_Capture文件里面
