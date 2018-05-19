/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_comm_link.c
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

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_motor.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_altitude.h"

u8   comm_link_mcu_state   = COMM_LINK_STATE_DISEN_MCU;
u16  comm_link_rc_data[4]  = {1500, 1500, 1500, 1000};
u16  comm_link_rc_data_pre[4]  = {1500, 1500, 1500, 1000};
float comm_link_rc_bat = 0.0F;
u32  comm_link_last_rc_timestamp;
bool comm_link_fly_enable_flag = false;
bool comm_link_pc_cmd_flag     = false;

CommLink_Data CommLink_DataStructure = {0, 0, 0, 0};

static void CommLink_Filter(void)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        if (comm_link_rc_data_pre[i] == 1500 ||
           (comm_link_rc_data_pre[i] != 1500 && comm_link_rc_data[i] != 1500))
        {
            if (comm_link_rc_data[i] >= 1000 && comm_link_rc_data[i] <= 2000)
            {
                comm_link_rc_data_pre[i] = comm_link_rc_data[i];
            }
        }
    }

    if (comm_link_rc_data_pre[3] == 1000 ||
       (comm_link_rc_data_pre[3] != 1000 && comm_link_rc_data[3] != 1000))
    {
        if (comm_link_rc_data[3] >= 1000 && comm_link_rc_data[3] <= 2000)
        {
            comm_link_rc_data_pre[3] = comm_link_rc_data[3];
        }
    }

    for (i = 0; i < 4; i++)
    {
        comm_link_rc_data[i] = comm_link_rc_data_pre[i];
    }

//    for (i = 0; i < 4; i++)
//    {
//        if (comm_link_rc_data[i] > 2000)
//        {
//            comm_link_rc_data[i] = comm_link_rc_data_pre[i];
//        }
//        else
//        {
//            comm_link_rc_data_pre[i] = comm_link_rc_data[i];
//        }
//    }
}

static void CommLink_ClearBuffer(void)
{
    while(!usart_rx_completion_flag);
    USART_ClearBuffer(&USART_RingBufferRxStructure); //important for processing data instantly!!!
    usart_rx_completion_flag = 0;
    comm_link_rc_data_pre[0] = 1500;
    comm_link_rc_data_pre[1] = 1500;
    comm_link_rc_data_pre[2] = 1500;
    comm_link_rc_data_pre[3] = 1000;
}

void CommLink_ProcessDataFromUART(void)
{
    if (control_altitude_mode == CONTROL_STATE_LANDING)
    {
        comm_link_rc_data_pre[IMU_ROLL]   = 1500;
        comm_link_rc_data_pre[IMU_PITCH]  = 1500;
        comm_link_rc_data_pre[IMU_YAW]    = 1500;
        comm_link_rc_data_pre[IMU_THRUST] = 1000;
        comm_link_rc_data[IMU_ROLL]       = 1500;
        comm_link_rc_data[IMU_PITCH]      = 1500;
        comm_link_rc_data[IMU_YAW]        = 1500;
        comm_link_rc_data[IMU_THRUST]     = 1000;
    }

    CommLink_Filter();
    COMM_LINK_CONSTRAIN(comm_link_rc_data[IMU_ROLL], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[IMU_PITCH], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[IMU_YAW], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[IMU_THRUST], 1000, 2000);

    CommLink_DataStructure.roll  = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[IMU_ROLL] - 1500), 500, COMM_LINK_APP_DB_PR) *
        CONTROL_ANGLE_MAX;
    CommLink_DataStructure.pitch = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[IMU_PITCH] - 1500), 500, COMM_LINK_APP_DB_PR) *
        CONTROL_ANGLE_MAX;
    CommLink_DataStructure.yaw   = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[IMU_YAW] - 1500), 500, COMM_LINK_APP_DB_YAW) *
        CONTROL_YAW_RATE_MAX;
    CommLink_DataStructure.thr = comm_link_rc_data[IMU_THRUST] - 1000;

    Battery_InformationStructure.voltage_calculate = comm_link_rc_bat;

    switch (comm_link_mcu_state)
    {
        case COMM_LINK_STATE_REQ_EN_MCU:
        {
            CommLink_ClearBuffer();

            if (IMU_Check() && !Battery_InformationStructure.flag_alarm)
            {
                LED_C_ON;
                comm_link_mcu_state       = COMM_LINK_STATE_EN_MCU;
                comm_link_fly_enable_flag = true;
            }
            else
            {
                comm_link_mcu_state       = COMM_LINK_STATE_DISEN_MCU;
                comm_link_fly_enable_flag = false;
            }
            break;
        }
        case COMM_LINK_STATE_REQ_DISEN_MCU:
        {
            CommLink_ClearBuffer();
            comm_link_mcu_state          = COMM_LINK_STATE_DISEN_MCU;
            comm_link_fly_enable_flag    = false;
            control_altitude_mode        = CONTROL_STATE_MANUAL;
            control_integral_reset_flag  = true;
//            control_thrust_z_split_power = 0;
//            control_thrust_z_integral    = Control_EstimateThrustRefHover();
            control_offland_flag         = false;
            break;
        }
        default:
        {
            break;
        }
    }
}

static u8 TransUARTStringToChar(void)
{
    u8 tmpch;
    do
    {
        if (USART_CountBuffer(&USART_RingBufferRxStructure) == 0)
        {
            break;
        }

        tmpch = USART_ReadBuffer(&USART_RingBufferRxStructure);
        if (tmpch != '\r' && tmpch != '\n')
        {
            break;
        }
    } while (1);

    return tmpch;
}

union packet
{
    u16 val;
    u8 ch[2];
}tranpacket;

static s16 TransUARTStringToNumber(void)
{
    s8 i;
    for (i = 1; i >= 0; i--)
    {
        tranpacket.ch[i] = USART_ReadBuffer(&USART_RingBufferRxStructure);
    }

    if (tranpacket.ch[1] & (1 << 3))
    {
        tranpacket.ch[1] -= (1 << 3);
        tranpacket.ch[0] += 128;
    }
    return tranpacket.val;
}

static void Power_Off(void)
{
    u8 i;
    Motor_SetPWM(0, 0, 0, 0);

    for (i = 1; i <= 4; i++)
    {
        Delay_TimeMs(1000);
    }

    Motor_SetPWM(999, 999, 999, 999);
    Delay_TimeMs(1000);
    Motor_SetPWM(0, 0, 0, 0);
}

static float TransUARTStringToFloat(void)
{
    float tmpval = 0.0F;
    u8 tmpch = TransUARTStringToChar();
    tmpval += tmpch - '0';

    tmpch = TransUARTStringToChar();
    tmpch = TransUARTStringToChar();
    tmpval += 0.1F * (tmpch - '0');
    tmpch = TransUARTStringToChar();
    tmpval += 0.01F * (tmpch - '0');

    return tmpval;
}

// if the `usart_rx_completion_flag` is true, this function will be called in `main` function
void CommLink_ReceiveDataFromUART(void)
{
    u8 header = TransUARTStringToChar();
    
    switch (header)
    {
        case COMM_LINK_MSP_SET_4CON:
        {
            comm_link_rc_data[IMU_ROLL]    = TransUARTStringToNumber();
            comm_link_rc_data[IMU_PITCH]   = TransUARTStringToNumber();
            comm_link_rc_data[IMU_YAW]     = TransUARTStringToNumber();
            comm_link_rc_data[IMU_THRUST]  = TransUARTStringToNumber();
//            printf("roll: %d pitch: %d yaw: %d thrust: %d\r\n", 
//            comm_link_rc_data[IMU_ROLL], comm_link_rc_data[IMU_PITCH],
//            comm_link_rc_data[IMU_YAW],  comm_link_rc_data[IMU_THRUST]);
            break;
        }
        case COMM_LINK_MSP_BATTERY:
        {
            comm_link_rc_bat = TransUARTStringToFloat();
            break;
        }
        case COMM_LINK_MSP_POWER_OFF:
        {
            Power_Off();
            break;
        }
        case COMM_LINK_MSP_ARM_IT:
        {
            comm_link_mcu_state = COMM_LINK_STATE_REQ_EN_MCU;
            break;
        }
        case COMM_LINK_MSP_DISARM_IT:
        {
            comm_link_mcu_state = COMM_LINK_STATE_REQ_DISEN_MCU;
            break;
        }
        case COMM_LINK_MSP_ACC_CALI:
        {
            imu_cali_flag = true;
            break;
        }
    }
    comm_link_last_rc_timestamp = Delay_GetRuntimeMs();
}

// Cut deadband scale to move linear.
float CommLink_CutDBScaleToLinear(float x_start, float x_end, float deadband)
{
    if (x_start > deadband)
    {
        return (x_start - deadband) / (x_end - deadband);
    }
    else if (x_start < -deadband)
    {
        return (x_start + deadband) / (x_end - deadband);
    }
    else
    {
        return 0.0F;
    }
}
