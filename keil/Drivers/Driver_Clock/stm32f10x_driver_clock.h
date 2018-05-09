/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_clock.h
Author:      myyerrol
Version:     none
Create date: 2017.04.13
Description: Declare the system clock function
Others:      none
Function List:
             1. void Clock_DeInit(void);
             2. void Clock_Init(void);
             3. void SetSysClockTo64Mhz(void);
             4. s8   Clock_InitSystemClockHSI(u8 pll_multi);
             5. s8   Clock_InitSystemClockHSE(u8 pll_multi);
History:
<author>    <date>        <desc>
myyerrol    2017.04.13    Modify the module
maksyuki    2018.05.10    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_CLOCK_H__
#define __STM32F10X_DRIVER_CLOCK_H__

#include "stm32f10x.h"

extern s8 clock_system;

extern void Clock_DeInit(void);
extern void Clock_Init(void);
extern void SetSysClockTo64Mhz(void);
extern s8   Clock_InitSystemClockHSI(u8 pll_multi);
extern s8   Clock_InitSystemClockHSE(u8 pll_multi);

#endif
