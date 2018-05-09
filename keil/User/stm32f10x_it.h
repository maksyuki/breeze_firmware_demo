/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_it.h
Author:      myyerrol
Version:     none
Create date: 2017.04.24
Description: Declare the Interrupt function
Others:      none
Function List:
             1. extern void HardFault_Handler(void);
             2. extern void SysTick_Handler(void);
             3. extern void TIM1_UP_IRQHandler(void);
             4. extern void TIM3_IRQHandler(void);
             5. extern void TIM4_IRQHandler(void);
             6. extern void USART1_IRQHandler(void);
             7. extern void USART2_IRQHandler(void);
History:
<author>    <date>        <desc>
myyerrol    2017.04.24    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_IT_H__
#define __STM32F10X_IT_H__

#include "stm32f10x.h"

// Current uptime for 1KHz systick timer.
#ifdef SYSTEM_INTERRUPT_VERSION
extern vu32 it_systick_uptime;
#endif
extern void HardFault_Handler(void);
extern void SysTick_Handler(void);
extern void TIM1_UP_IRQHandler(void);
extern void TIM3_IRQHandler(void);
extern void TIM4_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);

#endif
