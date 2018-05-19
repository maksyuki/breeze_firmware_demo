/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_hmc5883l.c
Author:      maksyuki
Version:     none
Create date: 2018.05.16
Description: Implement the hmc5883l function
Others:      none
Function List:

History:
<author>    <date>        <desc>
maksyuki    2018.05.16    Modify the module
*******************************************************************************/

#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_module_hmc5883l.h"

float HMC5883_lastx;
float HMC5883_lasty;
float HMC5883_lastz;

s16  HMC5883_FIFO[3][11]; //磁力计滤波

u8 HMC5883_IS_newdata(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void HMC58X3_FIFO_Init(void)
{
    s16 temp[3];
    u8 i;
    for (i = 0; i < 50; i++)
    {
        HMC58X3_getRaw(&temp[0], &temp[1], &temp[2]);
        Delay_TimeUs(200);
    }
}

void HMC58X3_newValues(s16 x, s16 y, s16 z)
{
    u8 i, j;
    s32 sum = 0;

    for (i = 1; i < 10; i++)
    {
        HMC5883_FIFO[0][i-1] = HMC5883_FIFO[0][i];
        HMC5883_FIFO[1][i-1] = HMC5883_FIFO[1][i];
        HMC5883_FIFO[2][i-1] = HMC5883_FIFO[2][i];
    }

    HMC5883_FIFO[0][9] = x;
    HMC5883_FIFO[1][9] = y;
    HMC5883_FIFO[2][9] = z;

    for(j = 0; j < 3; j++)
    {
        sum = 0;
        for (i = 0; i < 10; i++)
        {
            sum += HMC5883_FIFO[j][i];
        }
        HMC5883_FIFO[j][10] = sum / 10;
    }
}

void HMC58X3_writeReg(u8 reg, u8 val)
{
    IIC_WriteByte(HMC58X3_ADDR, reg, val);
}

void HMC58X3_getRaw(s16 *x, s16 *y, s16 *z)
{
    u8 vbuff[6], i;
    for (i = 0; i < 6; i++)
    {
        vbuff[i] = 0;
    }

    IIC_ReadBytes(HMC58X3_ADDR, HMC58X3_R_XM, 6, vbuff);
    HMC58X3_newValues(((s16)vbuff[0] << 8) | vbuff[1],
                      ((s16)vbuff[4] << 8) | vbuff[5],
                      ((s16)vbuff[2] << 8) | vbuff[3]);

    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10];
    *z = HMC5883_FIFO[2][10];
}

void HMC58X3_getlastValues(s16 *x, s16 *y, s16 *z)
{
    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10]; 
    *z = HMC5883_FIFO[2][10]; 
}

void HMC58X3_mgetValues(float *arry)
{
    s16 xr, yr, zr;
    HMC58X3_getRaw(&xr, &yr, &zr);
    arry[0] = HMC5883_lastx = (float)(xr);
    arry[1] = HMC5883_lasty = (float)(yr);
    arry[2] = HMC5883_lastz = (float)(zr);
}

void HMC58X3_setGain(u8 gain)
{ 
    if (gain > 7) return;
    HMC58X3_writeReg(HMC58X3_R_CONFB, gain<<5);
}

static void HMC58X3_setMode(u8 mode)
{
    if (mode > 2) return;
    HMC58X3_writeReg(HMC58X3_R_MODE, mode);
    Delay_TimeUs(100);
}

static void HMC58X3_Config(void)
{
    // 8 samples averaged, 75Hz frequency, no artificial bias.
    HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); 
    HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
    HMC58X3_writeReg(HMC58X3_R_MODE,  0x00);
}

static void HMC58X3_setDOR(u8 DOR)
{
    if (DOR > 6) return;
    HMC58X3_writeReg(HMC58X3_R_CONFA, DOR<<2);
}

static void HMC58X3_getID(char id[3])
{
    id[0] = IIC_ReadByte(HMC58X3_ADDR, HMC58X3_R_IDA);
    id[1] = IIC_ReadByte(HMC58X3_ADDR, HMC58X3_R_IDB);
    id[2] = IIC_ReadByte(HMC58X3_ADDR, HMC58X3_R_IDC);
}

static u8 HMC5883_Check(void) 
{
    char ID_temp[3];
    HMC58X3_getID(&ID_temp[0]);

    if ((ID_temp[0] == 0x48) && (ID_temp[1] == 0x34) && 
        (ID_temp[2]==0x33))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}   

void HMC5883L_Init(void)
{
    HMC5883_Check();
    HMC58X3_Config();
    HMC58X3_setMode(0);
    HMC58X3_setDOR(6);  //75hz 更新率
    HMC58X3_FIFO_Init();
}