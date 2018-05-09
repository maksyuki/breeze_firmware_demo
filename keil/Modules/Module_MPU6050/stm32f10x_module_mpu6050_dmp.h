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
    s16 qw;     // DMP�������Ԫ��ֵ
    s16 null0;
    s16 qx;
    s16 null1;
    s16 qy;
    s16 null2;
    s16 qz;
    s16 null3;
    s16 GYROx;  // ������ X�� ���ٶ� ADCֵ
    s16 null4;
    s16 GYROy;  // ������ Y�� ���ٶ� ADCֵ
    s16 null5;
    s16 GYROz;  // ������ Z�� ���ٶ� ADCֵ
    s16 null6;
    s16 ACCx;   // ���ٶȼ� X�� ADCֵ
    s16 null7;
    s16 ACCy;   // ���ٶȼ� Y�� ADCֵ
    s16 null8;
    s16 ACCz;   // ���ٶȼ� Z�� ADCֵ
    s16 null9;
    s16 null10;

// ���������� DMP_Routing ����.
    float dmp_pitch;  // DMP������ĸ����� ��λ(��)
    float dmp_roll;   // DMP��ת��        ��λ(��)
    float dmp_yaw;    // DMP����� ����û�д����Ʋ��뺽��ǻ�Ʈ  ��λ(��)
    float dmp_gyrox;  // ������ X�� ���ٶ�   ��λ(��ÿ��)
    float dmp_gyroy;  // ������ Y�� ���ٶ�   ��λ(��ÿ��)
    float dmp_gyroz;  // ������ Z�� ���ٶ�   ��λ(��ÿ��)
    float dmp_accx;	  // ���ٶȼ� X��   ��λ(g [9.8 m/S^2])
    float dmp_accy;	  // ���ٶȼ� Y��   ��λ(g [9.8 m/S^2])
    float dmp_accz;	  // ���ٶȼ� Z��   ��λ(g [9.8 m/S^2])
};

extern struct DMP_FIFO_map DMP_DATA;
extern  float q[4];

// DMP API�ӳ���
extern u8 MPU6050_DMP_Init(void);
extern void DMP_Routing(void);	        // DMP�̣߳���Ҫ���ڶ�ȡ�ʹ���DMP�Ľ��[��Ҫ��ʱ����]
extern void DMP_Covert_Data(void);
extern void DMP_getYawPitchRoll(void);  // ��ȡ�������̬��

#endif
