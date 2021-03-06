/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_comm_link.h
Author:      myyerrol
Version:     none
Create date: 2017.04.28
Description: Declare the communication operation function
Others:      none
Function List:
             1.  void  CommLink_AddBits8ToBuffer(u8 byte);
             2.  void  CommLink_AddBits16ToBuffer(s16 bytes);
             3.  void  CommLink_AddDataToBuffer(u8 *data, u8 length);
             4.  void  CommLink_ConvertEndian(u8 *data, u8 length);
             5.  void  CommLink_HandleCommand(void);
             6.  void  CommLink_HandleDebugDataA(void);
             7.  void  CommLink_HandleDebugDataB(void);
             8.  void  CommLink_HandleDebugDataC(void);
             9.  void  CommLink_ProcessDataFromNRF(void);
             10. void  CommLink_ReceiveDataFromNRF(void);
             11. void  CommLink_ReadPacket(u8 byte);
             12. void  CommLink_Test(void);
             13. void  CommLink_WriteBuffer(void);
             14. void  CommLink_WriteDebugData(void);
             15. void  CommLink_WritePacket(u8 command);
             16. void  CommLink_WritePID(u8 pid_type);
             17. float CommLink_CutDBScaleToLinear(float x_start, float x_end,
                                                   float deadband);
History:
<author>    <date>        <desc>
myyerrol    2017.04.28    Modify the module
maksyuki    2018.05.10    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_Module_COMM_LINK_H__
#define __STM32F10X_Module_COMM_LINK_H__

#include <math.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_algorithm_control.h"

// Multiwii Serial Protocol(MSP).
#define COMM_LINK_MSP_SET_THRO        1
#define COMM_LINK_MSP_SET_YAW         2
#define COMM_LINK_MSP_SET_PITCH       3
//#define COMM_LINK_MSP_SET_ROLL        4

#define COMM_LINK_MSP_POWER_OFF       0x24
#define COMM_LINK_MSP_SET_4CON        0x25
#define COMM_LINK_MSP_ARM_IT          0x22
#define COMM_LINK_MSP_DISARM_IT       0x23
#define COMM_LINK_MSP_ACC_CALI        0x21
#define COMM_LINK_MSP_BATTERY         0x26

//#define COMM_LINK_MSP_SETOFF          8
//#define COMM_LINK_MSP_LAND_DOWN       9
#define COMM_LINK_MSP_HOLD_ALT        10
#define COMM_LINK_MSP_STOP_HOLD_ALT   11
#define COMM_LINK_MSP_HEAD_FREE       12
#define COMM_LINK_MSP_STOP_HEAD_FREE  13
#define COMM_LINK_MSP_POS_HOLD        14
#define COMM_LINK_MSP_STOP_POS_HOLD   15
#define COMM_LINK_MSP_FLY_STATE       16


#define COMM_LINK_APP_DB_YAW          70
#define COMM_LINK_APP_DB_PR           50

#define COMM_LINK_STATE_EN_MCU        0
#define COMM_LINK_STATE_DISEN_MCU     1
#define COMM_LINK_STATE_REQ_EN_MCU    2
#define COMM_LINK_STATE_REQ_DISEN_MCU 3

#define COMM_LINK_CONVERT_ENDIAN      1

#define COMM_LINK_STATE_PC_REQ_PID    0X02
#define COMM_LINK_STATE_PC_PID_PITCH  0X10
#define COMM_LINK_STATE_PC_PID_ROLL   0X11
#define COMM_LINK_STATE_PC_PID_YAW    0X12
#define COMM_LINK_STATE_PC_PID_ALT    0X14

#define COMM_LINK_STATE_IDLE          1
#define COMM_LINK_STATE_HEADER1       2
#define COMM_LINK_STATE_HEADER2       3
#define COMM_LINK_STATE_COMMAND       4
#define COMM_LINK_STATE_LENGTH        5
#define COMM_LINK_STATE_DATA          6
#define COMM_LINK_STATE_CHECKSUM      7

#define COMM_LINK_CONSTRAIN(x, min, max) \
{                                        \
    if (x < min)                         \
    {                                    \
        x = min;                         \
    }                                    \
    if (x > max)                         \
    {                                    \
        x = max;                         \
    }                                    \
}

typedef union
{
    u8  bytes[2];
    s16 value;
} CommLink_TypeInt16;

typedef union
{
    u8  bytes[4];
    s32 value;
} CommLink_TypeInt32;

typedef struct
{
    u8                 header[2];
    u8                 cmd;
    u8                 len;
    CommLink_TypeInt16 roll;
    CommLink_TypeInt16 pitch;
    CommLink_TypeInt16 yaw;
    CommLink_TypeInt16 temp;
    CommLink_TypeInt16 speed;
    CommLink_TypeInt32 alt;
    CommLink_TypeInt32 pres;
    u8                 sum;
} CommLink_DataPacketA;

typedef struct
{
    u8 header[2];
    u8 cmd;
    u8 len;
    u8 data[30];
    u8 sum;
} CommLink_DataPacketB;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float thr;
} CommLink_Data;

extern u8    comm_link_mcu_state;
extern u16   comm_link_rc_data[4];
extern u16   comm_link_rc_data_pre[4];
extern float comm_link_rc_bat;
extern u32   comm_link_last_rc_timestamp;
extern bool  comm_link_fly_enable_flag;
extern bool  comm_link_pc_cmd_flag;

extern CommLink_Data        CommLink_DataStructure;

extern void  CommLink_ProcessDataFromUART(void);
static u8    TransUARTStringToChar(void);
static s16   TransUARTStringToNumber(void);
static float TransUARTStringToFloat(void);
extern void  CommLink_ReceiveDataFromUART(void);
extern float CommLink_CutDBScaleToLinear(float x_start, float x_end,
                                         float deadband);

#endif
