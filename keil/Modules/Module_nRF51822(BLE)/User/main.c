/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_adc.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Yuting's Breeze"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */

#define TX_POWER_LEVEL 4

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

//user code==========================================
//===================================================
float volt;
bool  volt_read_complete = false;

static void nrf_delay_ms(uint32_t volatile number_of_ms)
{
    int i;
	   while(number_of_ms != 0)
    {
        number_of_ms--;
        while(i<1000)
				{
          ++i;
        }
        i=0;
    }
}


// Power managment pins

#define PM_GPIO_SYSOFF          30
#define PM_GPIO_EN1             24
#define PM_GPIO_EN2             25
#define PM_GPIO_IN_CHG          22
#define PM_GPIO_IN_PGOOD        23

APP_TIMER_DEF(LED1_toggle_timer_id);
#define CHARGED_LED_OFF_INTERVAL 1800
#define CHARGED_LED_ON_INTERVAL  200

typedef enum
{
    charge100mA,
    charge500mA,
    chargeMax,
} BatteryChargeMode;

typedef enum
{
    battery,
    charging,
    charged,
    lowPower,
    shutDown,
} BatteryChargeStates;

static void battery_managment_init(void)
{
    // Configure PM PGOOD pin (Power good)
    NRF_GPIO->PIN_CNF[PM_GPIO_IN_PGOOD] = 0;     // 输入，不上下拉，标准驱动0、1
    // Configure PM CHG pin (Charge)
    NRF_GPIO->PIN_CNF[PM_GPIO_IN_CHG]   = 3<<2;  // 输入，上拉，标准驱动0、1
    // Configure PM EN2 pin
    NRF_GPIO->PIN_CNF[PM_GPIO_EN2]      = 0x301; // 输出，高驱动0，高驱动1，不上下拉
	// Configure PM EN1 pin
    NRF_GPIO->PIN_CNF[PM_GPIO_EN1]      = 0x301; // 输出，高驱动0，高驱动1，不上下拉
    // Configure PM SYSOFF pin
//    NRF_GPIO->PIN_CNF[PM_GPIO_SYSOFF]   = 0x301; // 输出，高驱动0，高驱动1，不上下拉
    
    NRF_GPIO->PIN_CNF[PM_GPIO_SYSOFF]   = 0x303; // 输出，不上下拉
}

static void battery_setChargeMode(BatteryChargeMode chgMode)
{
    switch (chgMode)
    {
        case charge100mA:
            nrf_gpio_pin_clear(PM_GPIO_EN1);
            nrf_gpio_pin_clear(PM_GPIO_EN2);
            break;
        case charge500mA:
            nrf_gpio_pin_set(PM_GPIO_EN1);
            nrf_gpio_pin_clear(PM_GPIO_EN2);
            break;
        case chargeMax:
            nrf_gpio_pin_clear(PM_GPIO_EN1);
            nrf_gpio_pin_set(PM_GPIO_EN2);
            break;
    }
}

//static bool chg_state_is_sended = false;

static BatteryChargeStates Battery_updateChargeState(void)
{
    uint32_t next_delay = 0;
    BatteryChargeStates m_battery_chargestate;
    bool isCharging = !nrf_gpio_pin_read(PM_GPIO_IN_CHG);
    bool isPgood    = !nrf_gpio_pin_read(PM_GPIO_IN_PGOOD);

    if (isPgood && !isCharging) //充电完成
    {
        m_battery_chargestate = charged;
        bool pin_set = nrf_gpio_pin_out_read(18) ? true: false;

        if (pin_set)
        {
            nrf_gpio_pin_clear(18);
            next_delay = CHARGED_LED_OFF_INTERVAL;
        }
        else
        {
            nrf_gpio_pin_set(18);
            next_delay = CHARGED_LED_ON_INTERVAL;
        }
        app_timer_start(LED1_toggle_timer_id, APP_TIMER_TICKS(next_delay, APP_TIMER_PRESCALER), NULL);
    }
    else if (isPgood && isCharging) //正在充电
    {
        m_battery_chargestate = charging;
//        if (!chg_state_is_sended)
//        {
//            while (app_uart_put('\'') != NRF_SUCCESS);
//            chg_state_is_sended = true;
//        }
        nrf_gpio_pin_set(18); 
    }
    else if (!isPgood && !isCharging) //没插USB充电线
    {
        m_battery_chargestate = lowPower;
        nrf_gpio_pin_clear(18); 
    }
    else
    {
        m_battery_chargestate = battery;
        nrf_gpio_pin_clear(18);
    }
    return m_battery_chargestate;
}

static void battery_poweroff(void)
{
    //nrf_gpio_pin_clear(PM_GPIO_SYSOFF);
    NRF_GPIO->OUTSET = 1 << 30;
//    nrf_delay_ms(1000);
//    nrf_gpio_pin_set(18);
    //nrf_gpio_pin_set(PM_GPIO_SYSOFF);
//    NRF_GPIO->OUTCLR = 1 << 30;
//    nrf_delay_ms(1000);
//    nrf_gpio_pin_clear(18);
}

static void battery_timer_handler(void* p_context)
{
    UNUSED_PARAMETER(p_context);
    Battery_updateChargeState();
}

#define CHARGESTATE_SAMPLE_RATE 1000 //1000ms

static const nrf_drv_timer_t m_timer_battery = NRF_DRV_TIMER_INSTANCE(1);

static void battery_timeout_handler(nrf_timer_event_t event_type, void* p_context)
{
    UNUSED_PARAMETER(p_context);
    Battery_updateChargeState();
}
    
static void timer1_init(void)
{
    ret_code_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_timer_battery, &timer_cfg, battery_timeout_handler);

    APP_ERROR_CHECK(err_code);
    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer_battery, CHARGESTATE_SAMPLE_RATE);
    nrf_drv_timer_extended_compare(&m_timer_battery, NRF_TIMER_CC_CHANNEL0,
    time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer_battery);
}




#define ADC_BUFFER_SIZE 1
#define ADC_SAMPLE_RATE 5000 // 5000ms

static nrf_adc_value_t adc_buffer[ADC_BUFFER_SIZE];

static nrf_drv_adc_channel_t m_channel_config =
NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2);

static nrf_ppi_channel_t     m_ppi_channel;
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(2);

static void adc_sampling_timeout_handler(nrf_timer_event_t event_type, void*
p_context)
{}
    
static void timer2_init(void)
{
    ret_code_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg,
    adc_sampling_timeout_handler);
    
    APP_ERROR_CHECK(err_code);
    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer, ADC_SAMPLE_RATE);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0,
    time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer);
}

static void adc_ppi_config(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
    nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0),
    nrf_drv_adc_start_task_get());
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

static void adc_event_handler(nrf_drv_adc_evt_t const* p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        for (uint32_t i = 0; i < p_event->data.done.size; i++)
        {
            volt = p_event->data.done.p_buffer[i] * 3.6F * 3 / 1024.0F;
            //printf("V:%.2f", volt);
            volt_read_complete = true;
        }
        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE));
    }
}

static void adc_config(void)
{
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
    nrf_drv_adc_channel_enable(&m_channel_config);
}

//user code==========================================
//===================================================

static void SendFloatValueToUART(float val)
{
    uint8_t tmp_int, tmp_dec;
    tmp_int = (uint8_t) val;
    while (app_uart_put('&') != NRF_SUCCESS);
    while (app_uart_put(tmp_int + '0') != NRF_SUCCESS);
    while (app_uart_put('.') != NRF_SUCCESS);

    tmp_dec = (uint8_t) ((val - tmp_int) * 100);

    while (app_uart_put(tmp_dec / 10 + '0') != NRF_SUCCESS);
    while (app_uart_put(tmp_dec % 10 + '0') != NRF_SUCCESS);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        if (p_data[i] == 'M') battery_poweroff(); //for debugging!!!!!!
        if (volt_read_complete)
        {
            SendFloatValueToUART(volt);
            volt_read_complete = false;
        }

        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
//    while (app_uart_put('\r') != NRF_SUCCESS);
//    while (app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}




//=========================user code==========================
//============================================================


static void leds_init(void)
{
    nrf_gpio_cfg_output(17);
    nrf_gpio_cfg_output(18);
}

APP_TIMER_DEF(LED0_toggle_timer_id);
#define ADVERTISING_LED_OFF_INTERVAL 1800
#define ADVERTISING_LED_ON_INTERVAL  200

/**@brief Handle events from leds timer.
 *
 * @note Timer handler does not support returning an error code.
 * Errors from bsp_led_indication() are not propagated.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */

static void LED_Indication(enum BLE_GAP_EVTS led_state);
static enum BLE_GAP_EVTS cur_led_state = BLE_GAP_EVT_DISCONNECTED;

static void leds_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    LED_Indication(cur_led_state);
//UNUSED_VARIABLE(bsp_led_indication(m_stable_state));

}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one. */
       uint32_t err_code;
       err_code = app_timer_create(&LED0_toggle_timer_id, APP_TIMER_MODE_SINGLE_SHOT, leds_timer_handler);
       APP_ERROR_CHECK(err_code);
        err_code = app_timer_create(&LED1_toggle_timer_id, APP_TIMER_MODE_SINGLE_SHOT, battery_timer_handler);
       APP_ERROR_CHECK(err_code);
}

//Use for toggling the LED0(pin 17)
static void LED_Indication(enum BLE_GAP_EVTS led_state)
{
    uint32_t next_delay = 0;
    cur_led_state = led_state;
    
    if (led_state == BLE_GAP_EVT_CONNECTED)
    {
       nrf_gpio_pin_set(17);
    }
    else if(led_state == BLE_GAP_EVT_DISCONNECTED)
    {
        bool pin_set = nrf_gpio_pin_out_read(17) ? true: false;

        if (pin_set)
        {
            nrf_gpio_pin_clear(17);
            next_delay = ADVERTISING_LED_OFF_INTERVAL;
        }
        else
        {
            nrf_gpio_pin_set(17);
            next_delay = ADVERTISING_LED_ON_INTERVAL;
        }
        app_timer_start(LED0_toggle_timer_id, APP_TIMER_TICKS(next_delay, APP_TIMER_PRESCALER), NULL);
    }
    else // For sleeping mode
    {
        nrf_gpio_pin_clear(17);
    }
}
//============================================================
//=========================user code==========================


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    LED_Indication(99); // for debugging
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            LED_Indication(BLE_GAP_EVT_DISCONNECTED);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //nrf_gpio_pin_set(18);
            //nrf_gpio_pin_clear(17);
            LED_Indication(BLE_GAP_EVT_CONNECTED);
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            //nrf_gpio_pin_set(17);
            //nrf_gpio_pin_clear(18);
            LED_Indication(BLE_GAP_EVT_DISCONNECTED);
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}



/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint8_t cnt = 0;
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
//    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();

    //buttons_leds_init(&erase_bonds);

    leds_init();
//    //test
//    while (1)
//    {
//        nrf_gpio_pin_set(17);
//        nrf_delay_ms(1500);
//        nrf_gpio_pin_clear(17);
//        nrf_delay_ms(1500);
//        nrf_gpio_pin_set(18);
//        nrf_delay_ms(1500);
//        nrf_gpio_pin_clear(18);
//        nrf_delay_ms(1500);
//    }
//
    timers_init();
    battery_managment_init();
    battery_setChargeMode(chargeMax);

    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    //printf("\r\nUART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(17);
    // Enter main loop.

    timer2_init();    // 初始化timer2
    adc_ppi_config(); // 配置PPI
    adc_config();	  // 配置ADC
    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE));

    timer1_init();
//    while (Battery_updateChargeState() != charged) ;

    while (1)
    {
        battery_poweroff();
        Battery_updateChargeState();
    }

    for (;;)
    {
        //power_manage();
    }
}
