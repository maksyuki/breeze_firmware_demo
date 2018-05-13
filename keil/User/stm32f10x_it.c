/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_it.c
Author:      myyerrol
Version:     none
Create date: 2017.04.24
Description: Implement the Interrupt function
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

#include <stdio.h>
#include "config.h"
#include "stm32f10x_it.h"
#include "stm32f10x_driver_timer.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_led.h"

#ifdef SYSTEM_INTERRUPT_VERSION
vu32 it_systick_uptime = 0;
#endif

void HardFault_Handler(void)
{
}

void SysTick_Handler(void)
{
#ifdef SYSTEM_INTERRUPT_VERSION
    it_systick_uptime++;
#endif
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        timer_dmp++;
        if (++timer_loop_cnt_100hz * 200 >= 1000)
        {
            timer_loop_cnt_100hz  = 0;
            timer_loop_flag_100hz = true;
        }
        if (++timer_loop_cnt_50hz * 100 >= 1000)
        {
            timer_loop_cnt_50hz  = 0;
            timer_loop_flag_50hz = true;
        }
        if (++timer_loop_cnt_20hz * 40 >= 1000)
        {
            timer_loop_cnt_20hz  = 0;
            timer_loop_flag_20hz = true;
        }
        if (++timer_loop_cnt_10hz * 20 >= 1000)
        {
            timer_loop_cnt_10hz  = 0;
            timer_loop_flag_10hz = true;
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

void TIM3_IRQHandler(void)
{
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        if (++timer_loop_cnt_100hz * 100 >= 1000)
        {
            timer_loop_cnt_100hz  = 0;
            timer_loop_flag_100hz = true;
        }
        if (++timer_loop_cnt_50hz * 50 >= 1000)
        {
            timer_loop_cnt_50hz  = 0;
            timer_loop_flag_50hz = true;
        }
        if (++timer_loop_cnt_20hz * 20 >= 1000)
        {
            timer_loop_cnt_20hz  = 0;
            timer_loop_flag_20hz = true;
        }
        if (++timer_loop_cnt_10hz * 10 >= 1000)
        {
            timer_loop_cnt_10hz  = 0;
            timer_loop_flag_10hz = true;
        }
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
    {
        USART_SendData(USART1, USART_ReadBuffer(&USART_RingBufferTxStructure));
        if (USART_CountBuffer(&USART_RingBufferTxStructure) == 0)
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
    else if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        //if(tmpch == 'a') LED_A_TOGGLE;
        //if(tmpch == 'b') LED_B_TOGGLE;
        //debug USART_SendData(USART1, (u8)USART_ReceiveData(USART1));
        
        if (!usart_rx_completion_flag) // need to continue to receive data, if the `usart_rx_completion_flag is true,
        {                   // it means the data haven't to be dealt with yet.
            u8 tmpch = (u8) USART_ReceiveData(USART1);
            if (tmpch == '!' || tmpch == '\"' ||
                tmpch == '#' || tmpch == '$'  || tmpch == '%')
            {
                USART_WriteBuffer(&USART_RingBufferRxStructure, tmpch);
                usart_rx_start_at_first = 1;
                if (tmpch == '!' || tmpch == '\"' ||
                    tmpch == '#' || tmpch == '$')
                {
                    usart_rx_completion_flag = 1;
                }
            }
            else
            {
                if (usart_rx_start_at_first)
                {
                    USART_WriteBuffer(&USART_RingBufferRxStructure, tmpch);

                    usart_rx_data_cnt++;
                    if (usart_rx_data_cnt == 8)
                    {
                        usart_rx_completion_flag = 1;
                        usart_rx_start_at_first  = 0;
                        usart_rx_data_cnt = 0;
                    }
                }
            }
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        
        // debug printf("\r\nI have received data!\r\n");
    }
}
