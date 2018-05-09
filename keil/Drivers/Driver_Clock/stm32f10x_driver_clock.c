/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_clock.c
Author:      myyerrol
Version:     none
Create date: 2017.04.13
Description: Implement the system clock function
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

#include "stm32f10x_rcc.h"
#include "stm32f10x_driver_clock.h"

s8 clock_system;

// Reset all registers. Can not reset all the peripheral, otherwise the serial
// port doesn't work.
void Clock_DeInit(void)
{
    // Reset is end.
    RCC->APB1RSTR = 0x00000000;
    RCC->APB2RSTR = 0x00000000;
    // Enable sleep mode flash and SRAM clock, disable others.
    RCC->AHBENR   = 0x00000014;
    // Disable peripheral clock.
    RCC->APB2ENR  = 0x00000000;
    RCC->APB1ENR  = 0x00000000;
    // Enable internal high-speed clock HSION.
    RCC->CR      |= 0x00000001;
    // Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0], MCO[2:0].
    RCC->CFGR    &= 0xF8FF0000;
    // Reset HSEON, CSSON, PLLON.
    RCC->CR      &= 0xFEF6FFFF;
    // Reset HSEBYP.
    RCC->CR      &= 0xFFFBFFFF;
    // Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE.
    RCC->CFGR    &= 0xFF80FFFF;
    // Disable all the peripheral.
    RCC->CIR      = 0x00000000;
}

void Clock_Init(void)
{
//    Clock_InitSystemClockHSE(9);
    SetSysClockTo64Mhz();
}

// Use th HSI to set the clock!!! this fun is aslike set-clock fun in `system_stm32f10x.c`
void SetSysClockTo64Mhz(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    RCC_DeInit();

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
    /* Enable HSI */
    RCC->CR |= ((uint32_t)RCC_CR_HSION);

    /* Wait till HSI is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CR & RCC_CR_HSIRDY;
        StartUpCounter++;
    } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
    {
        HSEStatus = (uint32_t)0x01;
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
        /* Enable Prefetch Buffer */
        FLASH->ACR |= FLASH_ACR_PRFTBE;
        /* Flash 2 wait state */
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;

        /* HCLK = SYSCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

        /* PCLK2 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

        /* PCLK1 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

        /*  PLL configuration: PLLCLK = HSI/2 * 16 = 64 MHz */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | 
                                        RCC_CFGR_PLLMULL));
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2| RCC_CFGR_PLLMULL16);

        /* Enable PLL */
        RCC->CR |= RCC_CR_PLLON;
        /* Wait till PLL is ready */
        while ((RCC->CR & RCC_CR_PLLRDY) == 0){}

        /* Select PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
        /* Wait till PLL is used as system clock source */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08){}
    }
    else
    {   /* If HSE fails to start-up, the application will have wrong clock
           configuration. User can add here some code to deal with this error */
    }
}

// Use the internal HSI clock two divider (4MHz) as the PLL input.
// 2 <= pll_multi(PLL multiplier coefficient) <= 9.
s8 Clock_InitSystemClockHSI(u8 pll_multi)
{
    // Enable internal high-speed clock.
    RCC->CR   |= 1 << 0;
    // Close internal high-speed clock.
    RCC->CR   |= 0 << 16;
    RCC->CR   |= 1 << 18;
    // Wait for the internal clock to be ready.
    while (!((RCC->CR) & (1 << 1)));
    // PLL multiplier coefficient.
    RCC->CFGR |= (pll_multi - 2) << 18;
    // PLL input clock source, HSI two frequency as the PLL input source=4MHz.
    RCC->CFGR |= 0 << 16;
    // Enable PLL.
    RCC->CR   |= 1 << 24;
    // Wait for PLL stability.
    while (!((RCC->CR) & (1 << 25)));
    // System clock source configuration, PLL output as system clock.
    RCC->CFGR |= 2 << 0;

    // Return the system clock, unit is MHz.
    clock_system = 4 * pll_multi;

    return clock_system;
}

// Use the external HSE clock 8M as the PLL input.
// 2 <= pll_multi(PLL multiplier coefficient) <= 9.
s8 Clock_InitSystemClockHSE(u8 pll_multi)
{
    u8 temp = 0;

    // Reset and configure vector table.
    Clock_DeInit();
    // Enable external high-speed clock HSEON.
    RCC->CR    |= 1 << 16;
    // Wait the external clock is prepared.
    while (!(RCC->CR  >> 17));
    // APB1=DIV2; APB2=DIV1; AHB=DIV1;
    RCC->CFGR   = 0X00000400;
    // Offset two units.
    pll_multi  -= 2;
    // Set PLL 2~16.
    RCC->CFGR  |= pll_multi << 18;
    // PLLXTPRE.
    RCC->CFGR  |= 1 << 17; // important!!!!!!
    // PLLSRC ON.
    RCC->CFGR  |= 1 << 16;
    // FLASH 2 delay period.
    FLASH->ACR |= 0x32;
    // PLLON.
    RCC->CR    |= 0x01000000;
    // Wait PLL is locked.
    while (!(RCC->CR >> 25));
    // PLL is as system clock.
    RCC->CFGR  |= 0x00000002;

    // Wait the configuration is completed which PLL is as system clock.
    while (temp != 0x02)
    {
        temp  = RCC->CFGR >> 2;
        temp &= 0x03;
    }

    clock_system = (pll_multi + 2) * 8;

    return clock_system;
}
