/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_timer.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.17
Description: implement the timer set operation
Others:      none
Function List:
             1. void Timer_InitTIM1(u16 arr, u16 psc);
             2. void Timer_InitTIM4(u16 arr, u16 psc);
History:
<author>    <date>        <desc>
maksyuki    2016.12.03    Modify the module
maksyuki    2018.05.10    Modify the module
*******************************************************************************/

#include "stm32f10x_driver_nvic.h"
#include "stm32f10x_driver_timer.h"

vu16 timer_dmp = 0;

vu16 timer_loop_cnt_10hz  = 0;
vu16 timer_loop_cnt_20hz  = 0;
vu16 timer_loop_cnt_50hz  = 0;
vu16 timer_loop_cnt_100hz = 0;

bool timer_loop_flag_10hz  = false;
bool timer_loop_flag_20hz  = false;
bool timer_loop_flag_50hz  = false;
bool timer_loop_flag_100hz = false;

void Timer_InitTIM1(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    NVIC_InitTIM1();

    TIM_DeInit(TIM1);

    TIM_TimeBaseStructure.TIM_Period            = arr - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = psc - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void Timer_InitTIM4(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    NVIC_InitTIM4();

    TIM_DeInit(TIM4);

    TIM_TimeBaseStructure.TIM_Period        = arr;
    TIM_TimeBaseStructure.TIM_Prescaler     = psc - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
