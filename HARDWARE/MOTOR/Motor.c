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
#include "Motor.h"
float Motor1;
float Motor2;
float Motor3;
float Motor4;
void Motor_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef          GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStruct;
	TIM_OCInitTypeDef         TIM_OCInitStruct;
	
	//开启RCC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=MOTOR_PIN1|MOTOR_PIN2|MOTOR_PIN3|MOTOR_PIN4;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	 
	TIM_TimeBaseStruct.TIM_Period=arr;
	TIM_TimeBaseStruct.TIM_Prescaler=psc;
	TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2,&TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);//为什么这里要使能预装在寄存器
	
	TIM_OC3Init(TIM2,&TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM2,&TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2,ENABLE);//为什么这里要使能自动重装载预装在寄存器
	TIM_SetCompare1(TIM2,1000);
	TIM_SetCompare2(TIM2,1000);
	TIM_SetCompare3(TIM2,1000);
	TIM_SetCompare4(TIM2,1000);
	TIM_Cmd(TIM2,ENABLE);
}
// 电机更新PWM值
void Motor_Refresh(u16 pwm1,u16 pwm2,u16 pwm3,u16 pwm4)
{
    TIM_SetCompare1(TIM2,pwm1);
	TIM_SetCompare2(TIM2,pwm2);
	TIM_SetCompare3(TIM2,pwm3);
	TIM_SetCompare4(TIM2,pwm4);
}
