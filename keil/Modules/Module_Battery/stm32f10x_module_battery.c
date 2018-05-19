/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_battery.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Implement the battery operation function
Others:      none
Function List:
             1. void Battery_Check(void);
             2. void Battery_Init(void);
             3. u16  Battery_GetADC(u8 ch);
             4. u16  Battery_GetADCAverage(u8 ch, u8 times);
             5. s32  Battery_GetAD(void);
             6. s32  Battery_GetTemperature(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.30    Modify the module
myyerrol    2017.04.11    Format the module
*******************************************************************************/

#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_algorithm_control.h"

Battery_Information Battery_InformationStructure;
s16 battery_check_count = 0;

void Battery_Check(void)
{
    // Calculate the real voltage of battery.
    Battery_InformationStructure.voltage_calculate = comm_link_rc_bat;

    // For thr first time reading vbat from UART
    if (fabs(Battery_InformationStructure.voltage_calculate) < 1E-4)
    {
        return ;
    }
        
    if (comm_link_fly_enable_flag)
    {
        if (Battery_InformationStructure.voltage_calculate <= (BATTERY_VOLTAGE_OVERDIS + 0.03))
        {
            Battery_InformationStructure.flag_alarm = true;
        }
        else
        {
            Battery_InformationStructure.flag_alarm = false;
        }

        if (Battery_InformationStructure.voltage_calculate <= BATTERY_VOLTAGE_OVERDIS)
        {
            Battery_InformationStructure.over_discharge_cnt++;
            if (Battery_InformationStructure.over_discharge_cnt > 8)
            {
                control_altitude_mode    = CONTROL_STATE_LANDING;
                comm_link_rc_data[0]     = 1500;
                comm_link_rc_data[1]     = 1500;
                comm_link_rc_data[2]     = 1500;
                comm_link_rc_data[3]     = 1000;
            }
        }
        else
        {
            Battery_InformationStructure.over_discharge_cnt = 0;
        }
    }
    else
    {
        if ((Battery_InformationStructure.voltage_calculate < BATTERY_VOLTAGE_ALARM) &&
            (Battery_InformationStructure.voltage_calculate > BATTERY_VOLTAGE_CHARGE))
        {
            Battery_InformationStructure.flag_alarm = true;
        }
        else
        {
            Battery_InformationStructure.flag_alarm = false;
        }
    }

//    if (Battery_InformationStructure.voltage_calculate < BATTERY_VOLTAGE_CHARGE)
//    {
//        Battery_InformationStructure.flag_charge = true;
//    }
//    else
//    {
//        Battery_InformationStructure.flag_charge = false;
//    }
}
