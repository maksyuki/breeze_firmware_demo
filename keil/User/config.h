#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifndef SYSTEM_INTERRUPT_VERSION
#define SYSTEM_INTERRUPT_VERSION
#endif


#ifndef STM32_UART_DEBUG_ENABLED
#define STM32_UART_DEBUG_ENABLED 1
#endif

#if STM32_UART_DEBUG_ENABLED
#ifndef STM32_UART_DEBUG
#define STM32_UART_DEBUG
#endif
#endif


#ifndef M_PI
#define M_PI 3.1415926F
#endif

//#ifndef MPU6050_DMP
//#define MPU6050_DMP
//#endif

#ifndef NRF51822_BLUETOOTH_MODE
#define NRF51822_BLUETOOTH_MODE
#endif

#ifndef YAW_CORRECT
#define YAW_CORRECT
#endif

#endif

