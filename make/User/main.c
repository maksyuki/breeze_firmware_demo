#include <stdio.h>
#include "stm32f10x_it.h"
#include "stm32f10x_driver_clock.h"
#include "stm32f10x_driver_delay.h"
// #include "stm32f10x_driver_eeprom.h"
// #include "stm32f10x_driver_flash.h"
// #include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_nvic.h"
#include "stm32f10x_driver_io.h"
// #include "stm32f10x_driver_timer.h"
#include "stm32f10x_driver_usart.h"
// #include "stm32f10x_module_battery.h"
// #include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_motor.h"
// #include "stm32f10x_module_mpu6050.h"
// #include "stm32f10x_module_ms5611.h"
// #include "stm32f10x_module_nrf24l01.h"
// #include "stm32f10x_algorithm_altitude.h"
// #include "stm32f10x_algorithm_control.h"
// #include "stm32f10x_algorithm_filter.h"
// #include "stm32f10x_algorithm_flight.h"
// #include "stm32f10x_algorithm_imu.h"

// static u16 battery_check_count = 0;

void Hardware_Init(void)
{
    Clock_Init();
    Delay_Init();
    // SysTick open system tick timer and initialize its interrupt.
    // Interrupt overflow time is 1ms
    // SysTick_Config(SystemCoreClock / 1000);

    // USART_InitUSART(115200);
    UART1_init(72, 115200);
    // Timer_InitTIM4(1000, clock_system);
    // Flash_Unlock();
    // EEPROM_LoadParamsFromEEPROM();
    LED_Init();
    Motor_Init();
    // Battery_Init();
    // IIC_Init();
    // MPU6050_Init();
    // NRF24L01_Init();
    // Battery_Check();
    // MS5611_Init();
    // IMU_Init();
    // MS5611_WaitBaroInitOffset();
    // control_altitude_mode = CONTROL_STATE_MANUAL;
}


int main(void)
{
    Hardware_Init();
    LED_SetInitialLight();
    while(1) printf("hello world!!!\n");
    //Motor_SetPWM(200, 200, 200, 200);
    // u16 t;
	// u16 len;
	// u16 tt = 0;
    // while(1) {
    //     if(USART_RX_STA & 0x8000) {
    //         len = USART_RX_STA & 0x3f;
    //         printf("\r\n you send message:\r\n\r\n");
    //         for(t = 0; t < len; t++) {
    //             USART_SendData(USART1, USART_RX_BUF[t]);
    //             while(USART_GetITStatus(USART1, USART_FLAG_TC) != SET);
    //         }
    //         printf("\r\n\r\n");
    //         USART_RX_STA = 0;
    //     }
    //     else {
    //         tt++;
    //         if(tt >= 0 && tt <= 80) {
    //             printf("I am maksyuki\n");
    //         }
    //         else if(tt >= 0 && tt <= 100) {
    //             printf("I am supermaker\n");
    //         }
    //         Delay_TimeMs(10);
    //     }
    // }


    // while(1)
    // {
        // LED_A_ON;
        // LED_B_ON;
        // LED_C_ON;
        // LED_D_ON;
        // if (timer_loop_flag_100hz)
        // {
        //     timer_loop_flag_100hz    = false;
        //     IMU_StartSO3Thread();
        //     altitude_acc_update_flag = true;
        //     MS5611_UpdateData();
        //     if (imu_cali_flag)
        //     {
        //         if (IMU_Calibrate())
        //         {
        //             imu_cali_flag                = false;
        //             eeprom_params_request_flag   = true;
        //             IMU_TableStructure.flag_cali = true;
        //         }
        //     }
        //     Control_CallPIDAngleRate();
        //     Control_SetMotorPWM();
        // }
        // // Receive data from NRF24L01.
        // NRF24L01_IRQHandler();
        // if (timer_loop_flag_50hz)
        // {
        //     timer_loop_flag_50hz = false;
        //     CommLink_ProcessDataFromNRF();
        //     Flight_SetMode();
        //     if (control_altitude_mode == CONTROL_STATE_LANDING)
        //     {
        //         Flight_StartAutoland();
        //     }
        //     Altitude_CombineData();
        //     Control_SetAltitude();
        //     Control_CallPIDAngle();
        //     // CommLink_WriteDebugData();
        // }
        // if (timer_loop_flag_10hz)
        // {
        //     timer_loop_flag_10hz = false;
        //     if ((++battery_check_count) * 100 >= BATTERY_CHECK_PERIOD)
        //     {
        //         battery_check_count = 0;
        //         Battery_Check();
        //     }
        //     if (eeprom_params_request_flag)
        //     {
        //         eeprom_params_request_flag = false;
        //         EEPROM_SaveParamsToEEPROM();
        //     }
        //     Flight_HandleFailures();
        //     LED_JumpStateMachine();
        // }
    // }
}
