/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_usart.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.15
Description: Implement the serial port operation
Others:      none
Function List:
             1. void USART_ClearBuffer(USART_RingBuffer *ring_buffer);
             2. void USART_InitUSART(u32 baud_rate);
             3. void USART_InitUSART1(u32 baud_rate);
             4. void USART_SendBuffer(u8 *bytes, u8 length);
             5. void USART_SendByte(u8 byte);
             6. void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte);
             7. u8   USART_ReadBuffer(USART_RingBuffer *ring_buffer);
             8. u16  USART_CountBuffer(USART_RingBuffer *ring_buffer);
History:
<author>    <date>        <desc>
maksyuki    2016.12.06    Modify the module
myyerrol    2017.04.14    Format the module
*******************************************************************************/

#include <stdio.h>
#include "stm32f10x_driver_nvic.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_led.h"

/* Add the below code to support 'printf' function */
#if 1
#pragma import(__use_no_semihosting)

/* The support function is needed by standard library */
struct __FILE
{
    int handle;
};

FILE __stdout;

/* Define _sys_exit() to avoid to use semihosting */
_sys_exit(int x)
{
    x = x;
}

/* Redefine 'fputc' function */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0x40) == 0);  /* Cyclic send until complete */
    USART1->DR = (u8)ch;
    return ch;
}

#endif

u8 usart_ring_buffer_rx[USART_BUFFER_SIZE];
u8 usart_ring_buffer_tx[USART_BUFFER_SIZE];
u8 usart_rx_completion_flag = 0;
u8 usart_rx_start_at_first  = 0;

USART_RingBuffer USART_RingBufferRxStructure;
USART_RingBuffer USART_RingBufferTxStructure;

void USART_ClearBuffer(USART_RingBuffer *ring_buffer)
{
    ring_buffer->index_rd = ring_buffer->index_wt;
}

void USART_InitUSART(u32 baud_rate)
{
    //USART_InitUSART1(baud_rate);
    UART1_init(64, baud_rate);
    
    USART_RingBufferRxStructure.index_rd = 0;
    USART_RingBufferRxStructure.index_wt = 0;
    USART_RingBufferRxStructure.mask     = USART_BUFFER_SIZE - 1;
    USART_RingBufferRxStructure.buffer   = &usart_ring_buffer_rx[0];

    USART_RingBufferTxStructure.index_rd = 0;
    USART_RingBufferTxStructure.index_wt = 0;
    USART_RingBufferTxStructure.mask     = USART_BUFFER_SIZE - 1;
    USART_RingBufferTxStructure.buffer   = &usart_ring_buffer_tx[0];
}

void USART_InitUSART1(u32 baud_rate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_AFIO, ENABLE);

    // USART1_TX: GPIOA.9.
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // USART1_RX: GPIOA.10.
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_InitUSART1();

    // USART initialization settings.
    USART_InitStructure.USART_BaudRate            = baud_rate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    // Enable USART1 interrupt to read data.
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //  Clear bits.
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    // Enable USART1.
    USART_Cmd(USART1, ENABLE);

    USART_RingBufferRxStructure.index_rd = 0;
    USART_RingBufferRxStructure.index_wt = 0;
    USART_RingBufferRxStructure.mask     = USART_BUFFER_SIZE - 1;
    USART_RingBufferRxStructure.buffer   = &usart_ring_buffer_rx[0];

    USART_RingBufferTxStructure.index_rd = 0;
    USART_RingBufferTxStructure.index_wt = 0;
    USART_RingBufferTxStructure.mask     = USART_BUFFER_SIZE - 1;
    USART_RingBufferTxStructure.buffer   = &usart_ring_buffer_tx[0];
}

void USART_SendBuffer(u8 *bytes, u8 length)
{
    u8 i;

    for (i = 0; i < length; i++)
    {
        USART_WriteBuffer(&USART_RingBufferTxStructure, *bytes);
        bytes++;
    }

    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void USART_SendByte(u8 byte)
{
    // Write data to ring buffer.
    USART_WriteBuffer(&USART_RingBufferTxStructure, byte);
    // Enable USART1 interrupt to send data.
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte)
{
    // According to the mask, write data to buffer.
    ring_buffer->buffer[ring_buffer->index_wt & ring_buffer->mask] = byte;
    ring_buffer->index_wt++;
}

u8 USART_ReadBuffer(USART_RingBuffer *ring_buffer)
{
    u8 temp;
    // According to the mask, write data to buffer.
    temp = ring_buffer->buffer[ring_buffer->index_rd & ring_buffer->mask];
    ring_buffer->index_rd++;
    return temp;
}

u16 USART_CountBuffer(USART_RingBuffer *ring_buffer)
{
    // Return the length of available bytes.
    return (ring_buffer->index_wt - ring_buffer->index_rd) & ring_buffer->mask;
}




//////////////////////////////////////////////////////
void USART_PutChar(u8 ch)
{
    USART_SendData(USART1, (u8)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

u8 USART_GetChar(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return (u8)USART_ReceiveData(USART1);
}
//////////////////////////////////////////////////////



/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化UART1
输入参数：u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void UART1_init(u32 pclk2, u32 bound)
{  	 
    float temp;
    u16 mantissa;
    u16 fraction;
    temp = (float)(pclk2 * 1000000) / (bound * 16); //得到USARTDIV
    mantissa = temp;                             //得到整数部分
    fraction = (temp - mantissa) * 16; //得到小数部分
    mantissa <<= 4;
    mantissa += fraction;
    RCC->APB2ENR  |= 1<<2;       //使能PORTA口时钟
    RCC->APB2ENR  |= 1<<14;      //使能串口时钟
    GPIOA->CRH    &= 0XFFFFF00F; //IO状态设置
    GPIOA->CRH    |= 0X000008B0; //IO状态设置
    RCC->APB2RSTR |= 1<<14;      //复位串口1
    RCC->APB2RSTR &= ~(1<<14);   //停止复位
    //波特率设置
    USART1->BRR  = mantissa;     //波特率设置
    USART1->CR1 |= 0X200C;       //1位停止,无校验位
    USART1->CR1 |= 1<<8;         //PE中断使能
	USART1->CR1 |= 1<<5;         //接收缓冲区非空中断使能
    NVIC_InitUSART();
}
