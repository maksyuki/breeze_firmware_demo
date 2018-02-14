/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_usart.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.15
Description: Define the serial port operation
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

// #ifndef __STM32F10X_DRIVER_USART_H__
// #define __STM32F10X_DRIVER_USART_H__
//
// #include "stm32f10x.h"
//
// #define USART_BUFFER_SIZE 128
//
// extern u8 usart_ring_buffer_rx[USART_BUFFER_SIZE];
// extern u8 usart_ring_buffer_tx[USART_BUFFER_SIZE];
//
// typedef struct
// {
//     u8  *buffer;
//     u16  mask;
//     vu16 index_rd;
//     vu16 index_wt;
// } USART_RingBuffer;
//
// extern USART_RingBuffer USART_RingBufferRxStructure;
// extern USART_RingBuffer USART_RingBufferTxStructure;
//
// extern void USART_ClearBuffer(USART_RingBuffer *ring_buffer);
// extern void USART_InitUSART(u32 baud_rate);
// extern void USART_InitUSART1(u32 baud_rate);
// extern void USART_SendBuffer(u8 *bytes, u8 length);
// extern void USART_SendByte(u8 byte);
// extern void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte);
// extern u8   USART_ReadBuffer(USART_RingBuffer *ring_buffer);
// extern u16  USART_CountBuffer(USART_RingBuffer *ring_buffer);
//
// void USART_PutChar(u8 ch);
// u8 USART_GetChar(void);
//
// #endif








#ifndef __UART1_H_
#define __UART1_H_

#include "stm32f10x.h"


// USART Receiver buffer
#define RX_BUFFER_SIZE   32
//#define TX_BUFFER_SIZE   3



// void DEBUG_PRINTLN(unsigned char *Str);
void UART1NVIC_Configuration(void);
void UART1_init(u32 pclk2,u32 bound);
void UART1_Put_Char(unsigned char DataToSend);
u8 UART1_Get_Char(void);
void UART1_Putc_Hex(uint8_t b);
void UART1_Putw_Hex(uint16_t w);
void UART1_Putdw_Hex(uint32_t dw);
void UART1_Putw_Dec(uint32_t w);
uint8_t Uart1_Put_Int16(uint16_t DataToSend);
uint8_t Uart1_Put_Float(float DataToSend);
void UART1_Put_String(unsigned char *Str);
void UART1_ReportIMU(int16_t  yaw,int16_t pitch,int16_t roll ,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);

extern unsigned char rx_buffer[RX_BUFFER_SIZE];
#endif
