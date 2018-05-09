/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_delay.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.02
Description: Implement the time delay function
Others:      none
Function List:
             1. void Delay_Init(void);
             2. void Delay_TimeMs(u16 n_ms);
             3. void Delay_TimeUs(u32 n_us);
             4. u32  Delay_GetRuntimeMs(void);
             5. u32  Delay_GetRuntimeUs(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.15    Modify the module
myyerrol    2017.04.13    Format the module
maksyuki    2018.05.10    Modify the module
*******************************************************************************/

#include "config.h"
#include "stm32f10x_it.h"
#include "stm32f10x_driver_delay.h"

#ifdef SYSTEM_INTERRUPT_VERSION
// Cycles per millisecond.
static vu32 tick_us = 0;

void Delay_Init(void)
{
    RCC_ClocksTypeDef RCC_ClocksStructure;
    RCC_GetClocksFreq(&RCC_ClocksStructure);
    tick_us = RCC_ClocksStructure.SYSCLK_Frequency / 1000000;
    tick_us *= 2; //debug!!!!!!
}

void Delay_TimeUs(u32 n_us)
{
    u32 t = Delay_GetRuntimeUs();
    while (Delay_GetRuntimeUs() - t < n_us);
}

void Delay_TimeMs(u16 n_ms)
{
    u32 t = Delay_GetRuntimeUs();
    while (Delay_GetRuntimeUs() - t < n_ms * 1000);
}

u32 Delay_GetRuntimeUs(void)
{
    register u32 milliseconds;
    register u32 cycle_count;

    do
    {
        milliseconds = it_systick_uptime;
        cycle_count  = SysTick->VAL;
    }
    while (milliseconds != it_systick_uptime);

    return (milliseconds * 1000) + (tick_us * 1000 - cycle_count) / tick_us;
}

u32 Delay_GetRuntimeMs(void)
{
    return it_systick_uptime;
}

#else
static u8 g_fac_us = 0;
static u16 g_fac_ms = 0;

void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  /* Select external clock HCLK/8 */
	g_fac_us = SystemCoreClock / 8000000;                  /* Clock time's 1/8 */
	g_fac_ms = (u16)g_fac_us * 1000;
}

void Delay_TimeUs(u32 n_us)
{
	u32 temp;
	SysTick->LOAD = n_us * g_fac_us;
	SysTick->VAL  = 0x00;                        /* Clear timer */
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /* Start countdown */

	do
	{
       temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1<<16)));  /* Wait time to arrive */

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /* Disable timer */
	SysTick->VAL  = 0x00;                        /* Clear timer */
}

void Delay_TimeMs(u16 n_ms)
{
	u32 temp;
	SysTick->LOAD = (u32)n_ms * g_fac_ms;         /* Load time, SysTick->LOAD is 24bit */
	SysTick->VAL  = 0x00;                        /* Clear timer */
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /* Start countdown */

	do
	{
		temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1<<16)));  /* Wait time to arrive */

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /* Disable timer */
	SysTick->VAL  = 0x00;                        /* Clear timer */
}
#endif
