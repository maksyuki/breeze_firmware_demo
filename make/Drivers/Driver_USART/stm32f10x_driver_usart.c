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

// #include <errno.h>
// #include <sys/unistd.h>
// #include <stdio.h>
// #include "stm32f10x_driver_nvic.h"
// #include "stm32f10x_driver_usart.h"
// #include "stm32f10x_module_led.h"
//
// u8 usart_ring_buffer_rx[USART_BUFFER_SIZE];
// u8 usart_ring_buffer_tx[USART_BUFFER_SIZE];
//
// USART_RingBuffer USART_RingBufferRxStructure;
// USART_RingBuffer USART_RingBufferTxStructure;
//
// void USART_ClearBuffer(USART_RingBuffer *ring_buffer)
// {
//     ring_buffer->index_rd = ring_buffer->index_wt;
// }
//
// void USART_InitUSART(u32 baud_rate)
// {
//     USART_InitUSART1(baud_rate);
// }
//
// void USART_InitUSART1(u32 baud_rate)
// {
//     GPIO_InitTypeDef  GPIO_InitStructure;
//     USART_InitTypeDef USART_InitStructure;
//
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
//                            RCC_APB2Periph_AFIO, ENABLE);
//
//     // USART1_TX: GPIOA.9.
//     GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
//     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_Init(GPIOA, &GPIO_InitStructure);
//     // USART1_RX: GPIOA.10.
//     GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
//     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
//     GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//     NVIC_InitUSART1();
//
//     // USART initialization settings.
//     USART_InitStructure.USART_BaudRate            = baud_rate;
//     USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
//     USART_InitStructure.USART_StopBits            = USART_StopBits_1;
//     USART_InitStructure.USART_Parity              = USART_Parity_No;
//     USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//     USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
//
//     USART_Init(USART1, &USART_InitStructure);
//     // Enable USART1 interrupt to read data.
//     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//     //  Clear bits.
//     USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//     // Enable USART1.
//     USART_Cmd(USART1, ENABLE);
//
//     USART_RingBufferRxStructure.index_rd = 0;
//     USART_RingBufferRxStructure.index_wt = 0;
//     USART_RingBufferRxStructure.mask     = USART_BUFFER_SIZE - 1;
//     USART_RingBufferRxStructure.buffer   = &usart_ring_buffer_rx[0];
//
//     USART_RingBufferTxStructure.index_rd = 0;
//     USART_RingBufferTxStructure.index_wt = 0;
//     USART_RingBufferTxStructure.mask     = USART_BUFFER_SIZE - 1;
//     USART_RingBufferTxStructure.buffer   = &usart_ring_buffer_tx[0];
// }
//
// void USART_SendBuffer(u8 *bytes, u8 length)
// {
//     u8 i;
//
//     for (i = 0; i < length; i++)
//     {
//         USART_WriteBuffer(&USART_RingBufferTxStructure, *bytes);
//         bytes++;
//     }
//
//     USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
// }
//
// void USART_SendByte(u8 byte)
// {
//     // Write data to ring buffer.
//     USART_WriteBuffer(&USART_RingBufferTxStructure, byte);
//     // Enable USART1 interrupt to send data.
//     USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
// }
//
// void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte)
// {
//     // According to the mask, write data to buffer.
//     ring_buffer->buffer[ring_buffer->index_wt & ring_buffer->mask] = byte;
//     ring_buffer->index_wt++;
// }
//
// u8 USART_ReadBuffer(USART_RingBuffer *ring_buffer)
// {
//     u8 temp;
//     // According to the mask, write data to buffer.
//     temp = ring_buffer->buffer[ring_buffer->index_rd & ring_buffer->mask];
//     ring_buffer->index_rd++;
//     return temp;
// }
//
// u16 USART_CountBuffer(USART_RingBuffer *ring_buffer)
// {
//     // Return the length of available bytes.
//     return (ring_buffer->index_wt - ring_buffer->index_rd) & ring_buffer->mask;
// }
//
//
// void __io_putchar(u8 ch)
// {
//     USART_SendData(USART1, (u8)ch);
//     while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
// }
//
// u8 USART_GetChar(void)
// {
//     while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
//     return (u8)USART_ReceiveData(USART1);
// }

// int _write(int file, char *ptr, int len)
// {
//     int i;
//
//     switch (file)
//     {
//         case STDOUT_FILENO:
//         {
//             for (i = 0; i < len; i++)
//             {
//                 USART_PutChar(*ptr++ & (u16)0x01FF);
//             }
//             break;
//         }
//         case STDERR_FILENO:
//         {
//             for (i = 0; i < len; i++)
//             {
//                 USART_PutChar(*ptr++ & (u16)0x01FF);
//             }
//             break;
//         }
//         default:
//         {
//             errno = EBADF;
//             return -1;
//         }
//     }
//
//     return len;
// }

// int _write(int fd, char *pBuffer, int size)
// {
//     for(int i = 0; i < size; i++)
//     {
//         while(!(USART1->SR & USART_SR_TXE))
//         {
//         }
//         USART_SendData(USART1, pBuffer[i]);
//     }
//     return size;
// }










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

//#include <stdio.h>
//#include "stm32f10x_driver_nvic.h"
//#include "stm32f10x_driver_usart.h"
//#include "stm32f10x_module_led.h"

///* Add the below code to support 'printf' function */
//#if 1
//#pragma import(__use_no_semihosting)

///* The support function is needed by standard library */
//struct __FILE
//{
//    int handle;
//};

//FILE __stdout;

///* Define _sys_exit() to avoid to use semihosting */
//_sys_exit(int x)
//{
//    x = x;
//}

///* Redefine 'fputc' function */
//int fputc(int ch, FILE *f)
//{
//    while ((USART1->SR & 0x40) == 0);  /* Cyclic send until complete */
//    USART1->DR = (u8)ch;
//    return ch;
//}

//#endif

//u8 usart_ring_buffer_rx[USART_BUFFER_SIZE];
//u8 usart_ring_buffer_tx[USART_BUFFER_SIZE];

//USART_RingBuffer USART_RingBufferRxStructure;
//USART_RingBuffer USART_RingBufferTxStructure;

//void USART_ClearBuffer(USART_RingBuffer *ring_buffer)
//{
//    ring_buffer->index_rd = ring_buffer->index_wt;
//}

//void USART_InitUSART(u32 baud_rate)
//{
//    USART_InitUSART1(baud_rate);
//}

//void USART_InitUSART1(u32 baud_rate)
//{
//    GPIO_InitTypeDef  GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
//                           RCC_APB2Periph_AFIO, ENABLE);

//    // USART1_TX: GPIOA.9.
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    // USART1_RX: GPIOA.10.
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    NVIC_InitUSART1();

//    // USART initialization settings.
//    USART_InitStructure.USART_BaudRate            = baud_rate;
//    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
//    USART_InitStructure.USART_Parity              = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

//    USART_Init(USART1, &USART_InitStructure);
//    // Enable USART1 interrupt to read data.
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//    //  Clear bits.
//    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//    // Enable USART1.
//    USART_Cmd(USART1, ENABLE);

//    USART_RingBufferRxStructure.index_rd = 0;
//    USART_RingBufferRxStructure.index_wt = 0;
//    USART_RingBufferRxStructure.mask     = USART_BUFFER_SIZE - 1;
//    USART_RingBufferRxStructure.buffer   = &usart_ring_buffer_rx[0];

//    USART_RingBufferTxStructure.index_rd = 0;
//    USART_RingBufferTxStructure.index_wt = 0;
//    USART_RingBufferTxStructure.mask     = USART_BUFFER_SIZE - 1;
//    USART_RingBufferTxStructure.buffer   = &usart_ring_buffer_tx[0];
//}

//void USART_SendBuffer(u8 *bytes, u8 length)
//{
//    u8 i;

//    for (i = 0; i < length; i++)
//    {
//        USART_WriteBuffer(&USART_RingBufferTxStructure, *bytes);
//        bytes++;
//    }

//    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//}

//void USART_SendByte(u8 byte)
//{
//    // Write data to ring buffer.
//    USART_WriteBuffer(&USART_RingBufferTxStructure, byte);
//    // Enable USART1 interrupt to send data.
//    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//}

//void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte)
//{
//    // According to the mask, write data to buffer.
//    ring_buffer->buffer[ring_buffer->index_wt & ring_buffer->mask] = byte;
//    ring_buffer->index_wt++;
//}

//u8 USART_ReadBuffer(USART_RingBuffer *ring_buffer)
//{
//    u8 temp;
//    // According to the mask, write data to buffer.
//    temp = ring_buffer->buffer[ring_buffer->index_rd & ring_buffer->mask];
//    ring_buffer->index_rd++;
//    return temp;
//}

//u16 USART_CountBuffer(USART_RingBuffer *ring_buffer)
//{
//    // Return the length of available bytes.
//    return (ring_buffer->index_wt - ring_buffer->index_rd) & ring_buffer->mask;
//}


//void USART_PutChar(u8 ch)
//{
//    USART_SendData(USART1, (u8)ch);
//    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//}

//u8 USART_GetChar(void)
//{
//    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
//    return (u8)USART_ReceiveData(USART1);
//}












/*
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
UART1.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.串口1初始化
2.供参数打印回串口助手，安卓4.0以上版本蓝牙透传接口以及和PC上位机接口
3.提供标准输入输出printf()的底层驱动，也就是说printf可以直接调用
------------------------------------
*/
#include "stm32f10x_driver_usart.h"
#include "stdio.h"


//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 U1TxBuffer[255];
u8 U1RxBuffer[255];

//u8 U1TxPackage[TX_BUFFER_SIZE];
u8 U1TxCounter=0;
u8 U1RxCounter=0;
u8 U1count=0;
char TxPackFlag;//发送预定格式数据包标志位
char Rx_Full_Flag;//数组被串口接收的数据填满标志（接收的数据不一定是遥控控制信息数据）
typedef union {unsigned char byte[4];float num;}t_floattobyte;
t_floattobyte floattobyte;
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;
	/* Whatever you require here. If the only file you are using is */
	/* standard output using printf() for debugging, no file handling */
	/* is required. */
};
/* FILE is typedef’ d in stdio.h. */
FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x)
{
	x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
	USART1->DR = (u8) ch;
	return ch;
}
#endif




/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功　　能:		串口1中断配置
输入参数：无
输出参数：没有
*******************************************************************************/
void UART1NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure;
        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}



/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化UART1
输入参数：u32 baudrate   设置RS232串口的波特率
输出参数：没有
*******************************************************************************/
void UART1_init(u32 pclk2,u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分
  mantissa<<=4;
	mantissa+=fraction;
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟
	RCC->APB2ENR|=1<<14;  //使能串口时钟
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置
	USART1->CR1|=0X200C;  //1位停止,无校验位.
  USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能

  UART1NVIC_Configuration();//中断配置
  printf("系统时钟频率：%dMHz \r\n",pclk2);
  printf("串口1初始化波特率：%d \r\n",bound);


}


/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	U1TxBuffer[U1count++] = DataToSend;
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		u8 UART1_Get_Char(void)
*功　　能:		RS232接收一个字节  一直等待，直到UART1接收到一个字节的数据。
输入参数：		 没有
输出参数：     UART1接收到的数据
*******************************************************************************/
u8 UART1_Get_Char(void)
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART1));
}


/**************************实现函数********************************************
*函数原型:		void UART2_Put_String(unsigned char *Str)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *Str   要发送的字符串
输出参数：没有
*******************************************************************************/
void UART1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')UART1_Put_Char(0x0d);
		else if(*Str=='\n')UART1_Put_Char(0x0a);
			else UART1_Put_Char(*Str);
	//等待发送完成.
  	//while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}
/*
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')USART_SendData(USART1, 0x0d);
		else if(*Str=='\n')USART_SendData(USART1, 0x0a);
			else USART_SendData(USART1, *Str);
	//等待发送完成.
  	while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}		 */
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putc_Hex(uint8_t b)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字节数据
				先将目标字节数据高4位转成ASCCII ，发送，再将低4位转成ASCII发送
				如:0xF2 将发送 " F2 "
输入参数：
		uint8_t b   要发送的字节
输出参数：没有
*******************************************************************************/
void UART1_Putc_Hex(uint8_t b)
{
      /* 判断目标字节的高4位是否小于10 */
    if((b >> 4) < 0x0a)
        UART1_Put_Char((b >> 4) + '0'); //小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b >> 4) - 0x0a + 'A'); //大于等于10 则相应发送 A-F

    /* 判断目标字节的低4位 是否小于10*/
    if((b & 0x0f) < 0x0a)
        UART1_Put_Char((b & 0x0f) + '0');//小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b & 0x0f) - 0x0a + 'A');//大于等于10 则相应发送 A-F
   UART1_Put_Char(' '); //发送一个空格,以区分开两个字节
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Hex(uint16_t w)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字的数据.就是发送一个int
				如:0x3456 将发送 " 3456 "
输入参数：
		uint16_t w   要发送的字
输出参数：没有
*******************************************************************************/
void UART1_Putw_Hex(uint16_t w)
{
	//发送高8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w >> 8));
	//发送低8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putdw_Hex(uint32_t dw)
*功　　能:		RS232以十六进制ASCII码的方式发送32位的数据.
				如:0xF0123456 将发送 " F0123456 "
输入参数：
		uint32_t dw   要发送的32位数据值
输出参数：没有
*******************************************************************************/
void UART1_Putdw_Hex(uint32_t dw)
{
    UART1_Putw_Hex((uint16_t) (dw >> 16));
    UART1_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Dec(uint16_t w)
*功　　能:		RS232以十进制ASCII码的方式发送16位的数据.
				如:0x123 将发送它的十进制数据 " 291 "
输入参数：
		uint16_t w   要发送的16位数据值
输出参数：没有
*******************************************************************************/
void UART1_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART1_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}

uint8_t Uart1_Put_Int16(uint16_t DataToSend)
{
	uint8_t sum = 0;
	U1TxBuffer[U1TxCounter++] = BYTE1(DataToSend);
	U1TxBuffer[U1TxCounter++] = BYTE0(DataToSend);
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	return sum;
}

uint8_t Uart1_Put_Float(float DataToSend)
{
	uint8_t sum = 0;
	floattobyte.num=DataToSend;
	U1TxBuffer[U1TxCounter++] = floattobyte.byte[3];
	U1TxBuffer[U1TxCounter++] = floattobyte.byte[2];
	U1TxBuffer[U1TxCounter++] = floattobyte.byte[1];
	U1TxBuffer[U1TxCounter++] = floattobyte.byte[0];
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	sum += BYTE3(DataToSend);
	sum += BYTE2(DataToSend);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	return sum;
}
unsigned char rx_buffer[RX_BUFFER_SIZE];

//------------------------------------------------------
void USART1_IRQHandler(void)
{

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {
    USART_SendData(USART1, U1TxBuffer[U1TxCounter++]);
    USART_ClearITPendingBit(USART1, USART_IT_TXE);
    if(U1TxCounter == U1count){USART_ITConfig(USART1, USART_IT_TXE, DISABLE);}
  }

  else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
  rx_buffer[U1RxCounter++]=USART_ReceiveData(USART1);
  if(U1RxCounter==RX_BUFFER_SIZE)
  {
		U1RxCounter=0;
		Rx_Full_Flag=1;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
  }
}


// void DEBUG_PRINTLN(unsigned char *Str)
//  {
// 	  UART1_Put_String(Str);  //通过USART1 发送调试信息
//  }
void UART1_ReportIMU(int16_t  yaw,int16_t pitch,int16_t roll ,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
	unsigned int temp=0xaF+2;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+2);
	UART1_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}
