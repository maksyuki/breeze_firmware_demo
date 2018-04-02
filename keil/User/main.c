#include <stdio.h>
#include "config.h"
#include "stm32f10x_it.h"

#include "stm32f10x_driver_io.h"
#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_nvic.h"
#include "stm32f10x_driver_clock.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_flash.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_driver_timer.h"
#include "stm32f10x_driver_eeprom.h"
// #include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_motor.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_module_mpu6050.h"
#include "stm32f10x_module_mpu6050_dmp.h"
// #include "stm32f10x_module_nrf24l01.h"
// #include "stm32f10x_algorithm_altitude.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_algorithm_filter.h"
#include "stm32f10x_algorithm_flight.h"

// static u16 battery_check_count = 0;

void Hardware_Init(void)
{
    Clock_Init();
    Delay_Init();
#ifdef SYSTEM_INTERRUPT_VERSION
    // SysTick open system tick timer and initialize its interrupt.
    // Interrupt overflow time is 1ms
    SysTick_Config(SystemCoreClock / 1000);
#endif
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART_InitUSART(115200);
    Timer_InitTIM1(1000, clock_system);
    Flash_Unlock();
    EEPROM_LoadParamsFromEEPROM();
    LED_Init();
    Motor_Init();
    // Battery_Init();
    IIC_Init();
#ifdef MPU6050_DMP
    MPU6050_DMP_Init();
#else
    MPU6050_Init();
#endif
    
//#ifdef NRF51822_BLUETOOTH_MODE
//    NRF51822_BLUETOOTH_Init();
//#else
//    NRF51822_RADIO_Init();
//#endif

    // Battery_Check();
    MS5611_Init();
    IMU_Init();
    MS5611_WaitBaroInitOffset();
    control_altitude_mode = CONTROL_STATE_MANUAL;
}


void MPU6050_RawDataOutput(void)
{
    u8 i;
    s16 main_acc_adc[3];
    s16 main_gyr_adc[3];
    float main_acc_raw[3];
    float main_gyr_raw[3];
    
     while (1) // Software Calc
    {
        MPU6050_ReadAcc(main_acc_adc);
        MPU6050_ReadGyr(main_gyr_adc);
        for (i = 0; i < 3; i++)
        {
            main_acc_raw[i] = (float) main_acc_adc[i] * IMU_ACC_SCALE *
                IMU_CONSTANTS_ONE_G;
            main_gyr_raw[i] = (float) main_gyr_adc[i] * IMU_GYR_SCALE *
                M_PI / 180.0F;
//            printf("acc_adc: %.8f gyro_adc: %.8f\r\n",
//            main_acc_adc[i], main_gyro_adc[i]);
//            printf("acc_raw: %.8f gyro_raw: %.8f\r\n",
//            main_acc_raw[i], main_gyr_raw[i]);
        }
        
//        printf("%.8f %.8f %.8f %.8f\r\n", Delay_GetRuntimeMs() / 1000.0F,
//                      main_gyr_raw[0], main_gyr_raw[1], main_gyr_raw[2]);
        
        printf("%.8f %.8f %.8f %.8f\r\n", Delay_GetRuntimeMs() / 1000.0F,
                      main_acc_raw[0], main_acc_raw[1], main_acc_raw[2]);
    }
}
  
static void test_pwm(void)
{
    u8 i, pwm_tmpval;
    u16 val = 0;
    
    while (1)
    {
        if (usart_rx_completion_flag)
        {
            usart_rx_completion_flag = 0;
            val = 0;
            while (USART_CountBuffer(&USART_RingBufferRxStructure))
            {
                pwm_tmpval = USART_ReadBuffer(&USART_RingBufferRxStructure);
                if (pwm_tmpval >= '0' && pwm_tmpval <= '9')
                {
                    val = val * 10 + (pwm_tmpval - '0');
                }
            }
        }
        Motor_SetPWM(val, val, val, val);
//        if (val == 100) LED_A_TOGGLE;
//        if (val == 250) LED_B_TOGGLE;
//        if (val == 300) LED_C_TOGGLE;
//        if (val == 350) LED_D_TOGGLE;
    }
}

static void test_motor(void)
{
    s16 motor_val;
    while (1)
    {
        if (usart_rx_completion_flag)
        {
            LED_A_ON;
            CommLink_ReceiveDataFromUART();
            CommLink_ProcessDataFromUART();
            motor_val = comm_link_rc_data[IMU_THRUST];
            if (motor_val >= 1000) motor_val = 999;
            Motor_SetPWM(motor_val, motor_val, motor_val, motor_val);
            usart_rx_completion_flag = 0;
        }
    }
}

static void test_motor_pwm(void)
{
    u16 pwm_val = 999;
    while (1)
    {
        if (pwm_val > 1000) pwm_val = 100;
        Motor_SetPWM(pwm_val, pwm_val, pwm_val, pwm_val);
        Delay_TimeMs(1000);
        Delay_TimeMs(1000);
        Delay_TimeMs(1000);
        pwm_val += 50;
        LED_A_TOGGLE;
    }
}


static void Power_off(void)
{
    u8 tmpch, i;
    if (usart_rx_completion_flag)
    {
        LED_A_TOGGLE;
        while (USART_CountBuffer(&USART_RingBufferRxStructure) >= 1)
        {
            LED_B_TOGGLE;
            tmpch = USART_ReadBuffer(&USART_RingBufferRxStructure);
            if (tmpch == 'P')
            {
                LED_C_TOGGLE;
                Motor_SetPWM(0, 0, 0, 0);
                for (i = 1; i <= 6; i++)
                    Delay_TimeMs(1000);
                
                Motor_SetPWM(999, 999, 999, 999);
                
                for (i = 1; i <= 2; i++)
                    Delay_TimeMs(1000);
                
                Motor_SetPWM(0, 0, 0, 0);
            }
        }
        usart_rx_completion_flag = 0;
    }
}


int main(void)
{
    //u8 is_first = 1;
    
    Hardware_Init();
    LED_SetInitialLight();
    
//    test_motor_pwm();
//    
//    test_pwm();
    
//    test_motor();
    
    Motor_SetPWM(100, 100, 100, 100);
//    Power_off();
    while (1)
    {
        if (usart_rx_completion_flag)
        {
            CommLink_ReceiveDataFromUART();
            usart_rx_completion_flag = 0;
        }
    }
//    MPU6050_RawDataOutput();

//    while(1)
//    {
//        EEPROM_ReadTableFromEEPROM();
//        
//        printf("version: %.8f\r\n", EEPROM_TableStructure.version);
//        for(i = 0; i < 3; i++) 
//        {
//            printf("%.8f\r\n", EEPROM_TableStructure.pid_roll[i]);
//            printf("%.8f\r\n", EEPROM_TableStructure.pid_pitch[i]);
//            printf("%.8f\r\n", EEPROM_TableStructure.pid_yaw[i]);

//            printf("%.8f\r\n", EEPROM_TableStructure.pid_roll_rate[i]);
//            printf("%.8f\r\n", EEPROM_TableStructure.pid_pitch_rate[i]);
//            printf("%.8f\r\n", EEPROM_TableStructure.pid_yaw_rate[i]);

//            printf("%.8f\r\n", EEPROM_TableStructure.pid_alt[i]);
//            printf("%.8f\r\n", EEPROM_TableStructure.pid_alt_vel[i]);
//            
//            printf("%.8f\r\n", EEPROM_TableStructure.offset_acc[i]);
//            printf("%.8f\r\n", EEPROM_TableStructure.offset_gyr[i]);
//            
//        }
//        
//        printf("\r\n");
//        if (is_first)
//        {
//            is_first = 0;
//            Control_PIDPitchAngle.kp = 100;
//            EEPROM_SaveParamsToEEPROM();    
//        }
//        
//    }


//    while (1)
//    {
//        if (timer_dmp >= 3)
//        {
//            timer_dmp = 0;
//            DMP_Routing();
//            printf("Pitch: %f\r\nRoll: %f\r\nYaw: %f\r\n\r\n", DMP_DATA.dmp_pitch
//            , DMP_DATA.dmp_roll, DMP_DATA.dmp_yaw);
//        }
//        
//        if (timer_loop_flag_100hz)
//        {
//            timer_loop_flag_100hz = false;
//            //printf("100hz\r\n");
//        }
//        if (timer_loop_flag_50hz)
//        {
//            timer_loop_flag_50hz = false;
//            //printf("50hz\r\n");
//        }
//        if (timer_loop_flag_20hz)
//        {
//            timer_loop_flag_20hz = false;
//            //printf("20hz\r\n");
//        }
//        if (timer_loop_flag_10hz)
//        {
//            timer_loop_flag_10hz = false;
//            //printf("10hz\r\n");
//        }
//    }

//    while(1)
//    {
//        MS5611_UpdateData();
//        printf("\r\n");
//        printf("temp: %.6f\r\n", ms5611_temperature);
//        printf("pres: %.6f\r\n", ms5611_pressure);
//        printf("alti: %.6f\r\n", ms5611_altitude);
//        printf("\r\n");
//    }
//    
//    while(1)
//    {
//        USART_PutChar('a');
//        Delay_TimeMs(1000);
//        USART_PutChar('b');
//        Delay_TimeMs(1000);
//        USART_PutChar('a');
//        Delay_TimeMs(1000);
//        USART_PutChar('b');
//        Delay_TimeMs(1000);
//    }

    
    //Motor_SetPWM(100, 100, 100, 100);
//    while(1)
//    {
//        int i;
//        for(i = 1; i <= 66; i++)
//            printf("%d\n", i);
//    }
    
    
    
//    while(1)
//    {
//        if (timer_loop_flag_100hz)
//        {
//            timer_loop_flag_100hz = false;
//            IMU_StartSO3Thread();
////          altitude_acc_update_flag = true;
//            MS5611_UpdateData();
//            if (imu_cali_flag)
//            {
//                if (IMU_Calibrate())
//                {
//                    imu_cali_flag                = false;
//                    eeprom_params_request_flag   = true;
//                    IMU_TableStructure.flag_cali = true;
//                }
//            }
//            Control_CallPIDAngleRate();
//            Control_SetMotorPWM();
//        }
//        
//        // Receive data from NRF24L01.
////       NRF24L01_IRQHandler();
//        if (timer_loop_flag_50hz)
//        {
//            timer_loop_flag_50hz = false;
////          CommLink_ProcessDataFromNRF();
              if (usart_rx_completion_flag)
              {
                  CommLink_ReceiveDataFromUART();
                  CommLink_ProcessDataFromUART();
                  usart_rx_completion_flag = 0;
              }                  
//            Flight_SetMode();
//            
//            if (control_altitude_mode == CONTROL_STATE_LANDING)
//            {
//                Flight_StartAutoland();
//            }
//            
////           Altitude_CombineData();
////           Control_SetAltitude();
//             Control_CallPIDAngle();
////           CommLink_WriteDebugData();
//        }
//        if (timer_loop_flag_10hz)
//        {
//            timer_loop_flag_10hz = false;
////             if ((++battery_check_count) * 100 >= BATTERY_CHECK_PERIOD)
////             {
////                 battery_check_count = 0;
////                 Battery_Check();
////             }
//            if (eeprom_params_request_flag)
//            {
//                eeprom_params_request_flag = false;
//                EEPROM_SaveParamsToEEPROM();
//            }
//            Flight_HandleFailures();
//            LED_JumpStateMachine();
//        }
//     }
}
