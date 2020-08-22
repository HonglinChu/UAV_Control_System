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

#include  "Led.h"
//A8  D2指示灯
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin =LED0_Pin;						
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_SetBits(GPIOA,LED0_Pin);
 
	GPIO_InitStruct.GPIO_Pin =LED1_Pin;						
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_SetBits(GPIOD,LED1_Pin);
}

