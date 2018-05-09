/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2018, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_mpu6050_dmp.h
Author:      maksyuki
Version:     none
Create date: 2018.05.09
Description: Declare the mpu6050 function
Others:      none
Function List:
             1. u8 MPU6050_DMP_Init(void);
             2. void DMP_Routing(void);
             3. void DMP_Covert_Data(void);
             4. void DMP_getYawPitchRoll(void);
History:
<author>    <date>        <desc>
maksyuki    2018.05.10    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_MPU6050_DMP_H__
#define __STM32F10X_MODULE_MPU6050_DMP_H__

#include "stm32f10x.h"

#define ONE_G  9.80665F // Related to acc calculate

#define DMP_CALC_PRD   7       // ms
#define DMP_ACC_SCALE  8192.0F // 4g 31276/4=8192
#define DMP_GYRO_SCALE 16.4F   // 2000deg/s 31276/2000=16.4F

// MotionApps 2.0 DMP implementation.
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]


/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

struct DMP_FIFO_map
{
    s16 qw;     // DMP输出的四元数值
    s16 null0;
    s16 qx;
    s16 null1;
    s16 qy;
    s16 null2;
    s16 qz;
    s16 null3;
    s16 GYROx;  // 陀螺仪 X轴 角速度 ADC值
    s16 null4;
    s16 GYROy;  // 陀螺仪 Y轴 角速度 ADC值
    s16 null5;
    s16 GYROz;  // 陀螺仪 Z轴 角速度 ADC值
    s16 null6;
    s16 ACCx;   // 加速度计 X轴 ADC值
    s16 null7;
    s16 ACCy;   // 加速度计 Y轴 ADC值
    s16 null8;
    s16 ACCz;   // 加速度计 Z轴 ADC值
    s16 null9;
    s16 null10;

// 以下数据由 DMP_Routing 更新.
    float dmp_pitch;  // DMP算出来的俯仰角 单位(度)
    float dmp_roll;   // DMP滚转角        单位(度)
    float dmp_yaw;    // DMP航向角 由于没有磁力计参与航向角会飘  单位(度)
    float dmp_gyrox;  // 陀螺仪 X轴 角速度   单位(度每秒)
    float dmp_gyroy;  // 陀螺仪 Y轴 角速度   单位(度每秒)
    float dmp_gyroz;  // 陀螺仪 Z轴 角速度   单位(度每秒)
    float dmp_accx;	  // 加速度计 X轴   单位(g [9.8 m/S^2])
    float dmp_accy;	  // 加速度计 Y轴   单位(g [9.8 m/S^2])
    float dmp_accz;	  // 加速度计 Z轴   单位(g [9.8 m/S^2])
};

extern struct DMP_FIFO_map DMP_DATA;
extern  float q[4];

// DMP API子程序
extern u8 MPU6050_DMP_Init(void);
extern void DMP_Routing(void);	        // DMP线程，主要用于读取和处理DMP的结果[需要定时调用]
extern void DMP_Covert_Data(void);
extern void DMP_getYawPitchRoll(void);  // 读取载体的姿态角

#endif
