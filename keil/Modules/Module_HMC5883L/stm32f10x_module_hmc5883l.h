/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_hmc5883l.h
Author:      maksyuki
Version:     none
Create date: 2018.05.16
Description: Declare the hmc5883l function
Others:      none
Function List:

History:
<author>    <date>        <desc>
maksyuki    2018.05.16    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_HMC5883L_H__
#define __STM32F10X_MODULE_HMC5883L_H__

#include "stm32f10x.h"

#define HMC58X3_ADDR      0x3C // 7 bit address of the HMC58X3 used with the Wire library

#define HMC_POS_BIAS      1
#define HMC_NEG_BIAS      2

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA   0
#define HMC58X3_R_CONFB   1
#define HMC58X3_R_MODE    2
#define HMC58X3_R_XM      3
#define HMC58X3_R_XL      4
#define HMC58X3_R_ZM      5
#define HMC58X3_R_ZL      6
#define HMC58X3_R_YM      7
#define HMC58X3_R_YL      8
#define HMC58X3_R_STATUS  9
#define HMC58X3_R_IDA     10
#define HMC58X3_R_IDB     11
#define HMC58X3_R_IDC     12


extern void HMC5883L_Init(void);
static void HMC58X3_FIFO_Init(void);
extern void HMC58X3_newValues(s16 x, s16 y, s16 z);
extern void HMC58X3_writeReg(u8 reg, u8 val);
extern void HMC58X3_mgetValues(float *arry);
extern void HMC58X3_getRaw(s16 *x, s16 *y, s16 *z);
static void HMC58X3_getID(char id[3]);
extern void HMC58X3_getValues(s16 *x, s16 *y, s16 *z);
extern void HMC58X3_getlastValues(s16 *x, s16 *y, s16 *z);
extern void HMC58X3_setGain(u8 gain);
static void HMC58X3_setMode(u8 mode);
static void HMC58X3_Config(void);
static void HMC58X3_setDOR(u8 DOR);
static u8   HMC5883_Check(void);
extern u8   HMC5883_IS_newdata(void);

#endif
