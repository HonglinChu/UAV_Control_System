/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
volatile uint32_t sysTickUptime = 0;
extern u8 Execute_Period2ms;//内环,MPU6050和RC
extern u8 Execute_Period5ms;//IMU和外环PID5ms
extern u8 Execute_Period10ms;//气压计和电子罗盘10ms
extern u8 Execute_Period50ms;//姿态50ms，超声波测高
extern u8 Execute_Period100ms;//定点GPS100ms
extern u8 Execute_Period200ms;//主要传感器数据200ms
extern u8 Execute_Period500ms;//次要传感器数据500ms
void NMI_Handler(void)
{
	
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
	static u16 TIM1_Cnt=0;
	sysTickUptime++;
	TIM1_Cnt++;
		if(TIM1_Cnt%2==0)
		{
		  Execute_Period2ms=1;
		}
		if(TIM1_Cnt%5==0)
		{
		  Execute_Period5ms=1;
		}
		if(TIM1_Cnt%10==0)
		{
		  Execute_Period10ms=1;
		}
		if(TIM1_Cnt%50==0)
		{
		  Execute_Period50ms=1;
		}
		if(TIM1_Cnt%100==0)
		{
		  Execute_Period100ms=1;
		}
		if(TIM1_Cnt%200==0)
		{
		  Execute_Period200ms=1;
		}
		if(TIM1_Cnt%500==0)
		{
		  Execute_Period500ms=1;
		}
	   TIM1_Cnt=TIM1_Cnt%1000;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
