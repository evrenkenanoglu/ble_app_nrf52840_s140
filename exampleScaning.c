/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                        /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
BLE_LBS_C_DEF(m_ble_lbs_c);                                     /**< Main structure used by the LBS client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */

//static char const m_target_periph_name[] = "Nordic_Blinky";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/


#define SCAN_LIST_REFRESH_INTERVAL 10000  // 10 sec

static uint32_t device_number;

#define FOUND_DEVICE_REFRESH_TIME APP_TIMER_TICKS(SCAN_LIST_REFRESH_INTERVAL) /**< Time after which the device list is clean and refreshed. */


#define DEVICE_NAME_MAX_SIZE 20
#define DEVICE_TO_FIND_MAX 50


typedef struct
{
    bool      is_not_empty;                                   /**< Indicates that the structure is not empty. */
    uint16_t  size;                                           /**< Size of manuf data. */
    uint8_t   addr[BLE_GAP_ADDR_LEN];                         /**< Device address. */
    char      dev_name[DEVICE_NAME_MAX_SIZE];                 /**< Device name. */
    uint8_t   manuf_buffer[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an  manuf data. */

} scanned_device_t;

void device_list_print(scanned_device_t * p_device);


scanned_device_t   m_device[DEVICE_TO_FIND_MAX];                 /**< Stores device info from scan data. */

void scan_device_info_clear(void)
{
    memset(m_device, 0, sizeof(m_device));
    device_number = 0;
}

/**@brief Function for printing the devices.
 *
 *@details Function print list of devices.
 *      
 *
 * @param[in] device      Pointer to the struct storing the scanned devices.
 */
void device_list_print(scanned_device_t * p_device)
{
    NRF_LOG_INFO("Devices found %d", device_number);

    // We could now clear the list:
    scan_device_info_clear();
    

}



scanned_device_t * scan_device_info_get(void)
{
    return m_device;
}

typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;


static void device_to_list_add(ble_gap_evt_adv_report_t const * p_adv_report)
{
    uint8_t  idx             = 0;
    uint16_t dev_name_offset = 0;
    uint16_t field_len;
    data_t   adv_data;


    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_adv_report->data.p_data;
    adv_data.data_len = p_adv_report->data.len;

    for ( idx = 0; idx < DEVICE_TO_FIND_MAX; idx++)
    {
        // If address is duplicated, then return.
        if (memcmp(p_adv_report->peer_addr.addr,
                   m_device[idx].addr,
                   sizeof(p_adv_report->peer_addr.addr)) == 0)
        {
            return;
        }
    }

    // Device is not in the list.
    for (idx = 0; idx < DEVICE_TO_FIND_MAX; idx++)
    {
        if (!m_device[idx].is_not_empty) // We find an empty entry
        {
               device_number = device_number +1;

               
                m_device[idx].is_not_empty = true; //validating the list record

                memset(m_device[idx].addr,
                0,
                sizeof(p_adv_report->peer_addr.addr));

                memcpy(m_device[idx].addr,
                p_adv_report->peer_addr.addr,
                sizeof(p_adv_report->peer_addr.addr));
                return;
            
        }
    }
}

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
//            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
//            APP_ERROR_CHECK(err_code);
//
//            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
//            APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            bsp_board_led_off(CENTRAL_SCANNING_LED);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
        break;

        default:
            //APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;

         case NRF_BLE_SCAN_EVT_NOT_FOUND:
             device_to_list_add(p_scan_evt->params.p_not_found);
             break;
        default:
          break;
    }
}



/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for handling the list_timer event,
 */
static void adv_list_timer_handle(void * p_context)
{
        
    // Print devices
    scanned_device_t * p_device_list = scan_device_info_get();
    device_list_print(p_device_list);

    //scan_device_info_clear();
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Timer for refreshing scanned devices data.
    APP_TIMER_DEF(adv_list_timer);
    err_code = app_timer_create(&adv_list_timer, APP_TIMER_MODE_REPEATED, adv_list_timer_handle);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(adv_list_timer, FOUND_DEVICE_REFRESH_TIME, NULL);
    APP_ERROR_CHECK(err_code);


}


/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

//    // Setting filters for scanning.
//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
//    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    scan_init();
    gatt_init();


    // Start execution.
    NRF_LOG_INFO("BLE scanner example started, will print the number of devices found after %d seconds",SCAN_LIST_REFRESH_INTERVAL/1000);
    scan_start();

    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}