/**
 * Copyright (c) 2018, Nordic Semiconductor ASA
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
 * @defgroup zigbee_examples_thingy_master_zed_color_light_bulb ble_thingy_master.c
 * @{
 * @ingroup zigbee_examples
 * @brief Dynamic multiprotocol example application to demonstrate control on BLE device (peripheral role) using zigbee device.
 */

#include <stdint.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"

#include "ble_thingy_master.h"

/** Usage of module
/   Define thingy_device_t table                    thingy_device_t table_name[cnt]
/   Init module using function:                     ble_thingy_master_init(table_name, cnt)
/   To start module use:                            ble_thingy_master_start()
/   To control Thingy LED use:                      ble_thingy_master_update_led(...)
**/

/* Define timer to handle scanning timeout faster */
APP_TIMER_DEF(m_ble_thingy_timer_id);

/* Structure for discovery puposes */
static ble_gattc_handle_range_t m_discovery_handler_range =
{
    .start_handle = BLE_GATT_HANDLE_START,
    .end_handle = BLE_GATT_HANDLE_END
};

/* Declare variables to contain 128-bit UUID  */
static ble_uuid128_t m_vendor_uuid  = 
{
    .uuid128 =
        {
            0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B,
            0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF
        }
};
static uint8_t m_vendor_uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;

/* Structure containing module internal variables to run device logic */
typedef struct
{
    thingy_device_t   * p_dev_table;
    uint8_t             thingy_dev_cnt;
} ble_device_ctx_t;

/* Structure containing module internal variables */
static ble_device_ctx_t m_ble_dev =
{
    .p_dev_table        = NULL,
    .thingy_dev_cnt     = 0
};

/* Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active             = 1,
    .interval           = SCAN_INTERVAL,
    .window             = SCAN_WINDOW,

    .timeout            = SCAN_DURATION,
    .scan_phys          = BLE_GAP_PHY_1MBPS,
    .filter_policy      = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

/* Buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN];

/* Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

/* Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_MIN
};

/**@brief Function for getting pointer to thingy device by conn_handler.
 * 
 * @param[IN]   conn_handler   Connection handler.
 * 
 * @returns     Pointer to matching thingy_device_t
 */
static thingy_device_t * thingy_find_by_connection_handler(uint16_t conn_handler)
{
    // Search for matching conn_handler
    for (uint8_t i = 0; i < m_ble_dev.thingy_dev_cnt; i++)
    {
        if (m_ble_dev.p_dev_table[i].conn_handle == conn_handler)
        {
            return &m_ble_dev.p_dev_table[i];
        }
    }
    
    return NULL;
}

/**@brief Handler to application timer.
 * 
 * @param[IN]   context   Context which function is called with, unused.
 * 
 * @details     Function is used to to stop scanning after amount of time defined in APP_BLE_THINGY_SCANNING_TIMEOUT 
 *              if no matching device is found.
 */
static void thingy_timer_handler(void * context)
{
    UNUSED_PARAMETER(context);
    ret_code_t err_code = sd_ble_gap_scan_stop();
    APP_ERROR_CHECK(err_code);
    
    // Change LED status to indicate about end of scanning
    bsp_board_led_on(CENTRAL_CONNECTED_LED);
    bsp_board_led_off(CENTRAL_SCANNING_LED);

    NRF_LOG_INFO("Scanning stopped due to user timeout")
}

/**@brief Function for getting pointer to not connected thingy device.
 * 
 * @returns Pointer to thingy_device_t.
 */
static thingy_device_t * thingy_acquire(void)
{
    // Search for matching conn_handler
    for (uint8_t i = 0; i < m_ble_dev.thingy_dev_cnt; i++)
    {
        if (m_ble_dev.p_dev_table[i].conn_handle == APP_BLE_CONN_HANDLER_DEFAULT)
        {
            return &m_ble_dev.p_dev_table[i];
        }
    }
    
    return NULL;
}

/**
 * @brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[IN] line_num     Line number of the failing ASSERT call.
 * @param[IN] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the advertising report BLE event.
 * 
 * @param[IN] p_adv_report Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;

    if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              APP_BLE_THINGY_DEVICE_NAME))
    {
        // Name is a match, initiate connection.
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function to handle device_connected event.
 * 
 * @param[IN] p_ble_evt    Pointer to ble event structure.
 */
static void thingy_on_connected(ble_evt_t const * p_ble_evt)
{
    // Stop timer to prevent unexpected scan_stop
    ret_code_t err_code = app_timer_stop(m_ble_thingy_timer_id);
    APP_ERROR_CHECK(err_code);
    
    // Get pointer to disconnected thingy device
    thingy_device_t * p_thingy = thingy_acquire();
    if (p_thingy != NULL)
    {
        p_thingy->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        
        NRF_LOG_INFO("Thingy connected");

        // Reset handler range discovery to default value
        m_discovery_handler_range.start_handle = BLE_GATT_HANDLE_START;
        // Start discovery of characteristics
        err_code = sd_ble_gattc_characteristics_discover(p_ble_evt->evt.gattc_evt.conn_handle, &m_discovery_handler_range);
        APP_ERROR_CHECK(err_code);

        // Update LEDs status, and check if we should be looking for more
        // peripherals to connect to.
        bsp_board_led_on(CENTRAL_CONNECTED_LED);
        bsp_board_led_off(CENTRAL_SCANNING_LED);
    }
    else
    {
         NRF_LOG_INFO("Can't connect thingy, maximum number of connections reached");
    }
}

/**@brief Function to handle device_disconnected event.
 * 
 * @param[IN] p_ble_evt   Pointer to ble event structure.
 * @param[IN] p_thingy    Pointer to thingy device.
 */
static void thingy_on_disconnected(ble_evt_t const * p_ble_evt, thingy_device_t * p_thingy)
{
    // Stop timer to restart it correctly later
    ret_code_t err_code = app_timer_stop(m_ble_thingy_timer_id);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Disconnected.");
    // Free thingy connection if disconnected
    if (p_thingy != NULL)
    {
        p_thingy->conn_handle = APP_BLE_CONN_HANDLER_DEFAULT;
    }
    // Check if any thingy connection is available
    if (thingy_acquire() != NULL)
    {
        NRF_LOG_INFO("Searching for thingy");
        ble_thingy_master_start();
        //Start timer to stop scanning after temieout defined in APP_BLE_THINGY_SCANNING_TIMEOUT if no other device is in range
        err_code = app_timer_start(m_ble_thingy_timer_id,
                                   APP_TIMER_TICKS(APP_BLE_THINGY_SCANNING_TIMEOUT),
                                   NULL);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_INFO("Can not connect more thingy");
    }
}

/**@brief Function to handle GATTC Characteristic Discovery Response event.
 * 
 * @param[IN] p_thingy    Pointer to thingy device.
 * @param[IN] p_gattc_evt Pointer to GATTC event structure.
 */
static void thingy_on_characterisics_discovery_response(thingy_device_t * p_thingy, ble_gattc_evt_t const * p_gattc_evt)
{
    ret_code_t               err_code;
    const ble_gattc_char_t * p_char_data = &p_gattc_evt->params.char_disc_rsp.chars[0];

    if (p_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
        // Look for VENDOR specific characteristic: Thingy LED
        if (p_char_data->uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
        {
            if (p_char_data->uuid.uuid == BLE_UUID_THINGY_LED_CHAR)
            {
                p_thingy->led.handle = p_char_data->handle_value;
                NRF_LOG_INFO("Found %s characteristic handle: %hd", p_thingy->led.name, p_thingy->led.handle);
            }
        }

        // Look for SIG specific characteristic: Battery level
        else if (p_char_data->uuid.type == BLE_UUID_TYPE_BLE)
        {
            if (p_char_data->uuid.uuid == BLE_UUID_BATTERY_LEVEL_CHAR)
            {
                p_thingy->battery_level.handle = p_char_data->handle_value;
                NRF_LOG_INFO("Found %s characteristic handle: %hd", p_thingy->battery_level.name, p_thingy->battery_level.handle);
            }
        }

        // Prepare for discovering remaining characteristics
        m_discovery_handler_range.start_handle = p_char_data->handle_value;
        err_code = sd_ble_gattc_characteristics_discover(p_gattc_evt->conn_handle, &m_discovery_handler_range);
        APP_ERROR_CHECK(err_code);
    }
    // If no more characteristics are found read battery level value
    else if (p_gattc_evt->gatt_status == BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND)
    {
        err_code = sd_ble_gattc_read(p_gattc_evt->conn_handle, p_thingy->battery_level.handle, 0);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function to handle GATTC read response.
 * 
 * @param[IN] p_thingy    Pointer to thingy device.
 * @param[IN] p_gattc_evt Pointer to GATTC event structure.
 */
static void thingy_on_read_response(thingy_device_t * p_thingy, ble_gattc_evt_t const * p_gattc_evt)
{
    if (p_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
        ret_code_t err_code;
        // If battery level value is received read LED characteristic value
        if (p_gattc_evt->params.read_rsp.handle == p_thingy->battery_level.handle)
        {
            NRF_LOG_INFO("Battery level: %hd", *p_gattc_evt->params.read_rsp.data);
            err_code = sd_ble_gattc_read(p_gattc_evt->conn_handle, p_thingy->led.handle, 0);
            APP_ERROR_CHECK(err_code);
        }

        if (p_gattc_evt->params.read_rsp.handle == p_thingy->led.handle)
        {
            const uint8_t * ptr             = p_gattc_evt->params.read_rsp.data;
            /* Structure containing data to write to Thingy LED characteristic to set LED to red color */
            const led_params_t led_params   =
            {
                .mode       = APP_BLE_THINGY_LED_MODE_CONSTANT,
                {
                    {
                        .r_value    = 255,
                        .g_value    = 0,
                        .b_value    = 0
                    }
                }
            };
            
            /*lint -e415 -e416 */
            NRF_LOG_INFO("LED mode: %02X value: %02X%02X%02X%02X", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4]);
            /*lint -restore */

            ble_thingy_master_update_led(p_thingy, &led_params);

            if (thingy_acquire() != NULL)
            {
                NRF_LOG_INFO("Searching for another one");
                ble_thingy_master_start();
                //Start timer to stop scanning after temieout defined in APP_BLE_THINGY_SCANNING_TIMEOUT if no other device is in range
                err_code = app_timer_start(m_ble_thingy_timer_id,
                                           APP_TIMER_TICKS(APP_BLE_THINGY_SCANNING_TIMEOUT),
                                           NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                NRF_LOG_INFO("Searching is done");
            }
        }
    }
}

/**@brief Function for handling BLE events.
 * 
 * @param[IN]   p_ble_evt   Bluetooth stack event.
 * @param[IN]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t                      err_code  = NRF_SUCCESS;
    thingy_device_t               * p_thingy  = NULL;
    ble_gap_evt_t const           * p_gap_evt = &p_ble_evt->evt.gap_evt;
    static ble_gap_phys_t const     phys      =
    {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO
    };

    p_thingy = thingy_find_by_connection_handler(p_gap_evt->conn_handle);

    switch (p_ble_evt->header.evt_id)
    {
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:

            // Check if this connection handle has assigned thingy device
            if (p_thingy == NULL)
            {
                thingy_on_connected(p_ble_evt);
            }
            else
            {
                NRF_LOG_INFO("Found thingy is already connected");
            }
            break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:

            thingy_on_disconnected(p_ble_evt, p_thingy);
            break;

        case BLE_GAP_EVT_ADV_REPORT:

            on_adv_report(&p_gap_evt->params.adv_report);
            break;

        case BLE_GAP_EVT_TIMEOUT:

            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:

            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:

            NRF_LOG_DEBUG("PHY update request.");
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:

            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:

            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:

            if (p_thingy != NULL)
            {
                thingy_on_characterisics_discovery_response(p_thingy, &p_ble_evt->evt.gattc_evt);
            }
            else
            {
                NRF_LOG_INFO("Unrecognised Thingy device");
            }
            break;

        case BLE_GATTC_EVT_READ_RSP:

            if (p_thingy != NULL)
            {
                thingy_on_read_response(p_thingy, &p_ble_evt->evt.gattc_evt);
            }
            else
            {
                NRF_LOG_INFO("Unrecognised Thingy device")
            }
            break;

        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:

            if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS)
            {
                NRF_LOG_INFO("Error while sending data");
            }
            break;

        case BLE_GATTC_EVT_WRITE_RSP:

            if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS)
            {
                NRF_LOG_INFO("Error while writing data");
            }
            break;

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

/**@brief Function to init internal thingy device structures.
 * 
 */
static void thingy_struct_init(void)
{
    if (m_ble_dev.p_dev_table != NULL)
    {
        for (uint8_t i = 0; i < m_ble_dev.thingy_dev_cnt; i++)
        {
            m_ble_dev.p_dev_table[i].battery_level.handle = 0;
            memcpy(m_ble_dev.p_dev_table[i].battery_level.name, "BATTERY", 8);
            
            m_ble_dev.p_dev_table[i].conn_handle = APP_BLE_CONN_HANDLER_DEFAULT;
            
            m_ble_dev.p_dev_table[i].led.handle = 0;
            memcpy(m_ble_dev.p_dev_table[i].led.name, "LED", 4);
        }
    }
    else
    {
        NRF_LOG_ERROR("Thingy device table does not exist or ble_thingy_master_init() not called");
    }
}

void ble_thingy_master_timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ble_thingy_timer_id, APP_TIMER_MODE_SINGLE_SHOT, thingy_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void ble_thingy_master_update_led(thingy_device_t * p_thingy, const led_params_t * p_led_params)
{
    /* Structure containing write parameters required to write data to characteristic */
    ble_gattc_write_params_t m_led_write_params;
    
    /* Check if pointer to thingy device and rgb_values are not NULL */
    if ((p_thingy == NULL) || (p_led_params == NULL))
    {
        NRF_LOG_INFO("Incorrect pointer or module is not initialized");
        return;
    }
    /* Check if thingy is connected */
    if (p_thingy->conn_handle == APP_BLE_CONN_HANDLER_DEFAULT)
    {
        NRF_LOG_INFO("Thingy not connected");
        return;
    }
    
    /* Update params to write request */
    m_led_write_params.len      = (p_led_params->mode == APP_BLE_THINGY_LED_MODE_CONSTANT) ? 4 : 5;  /* Length of data (in bytes) to write to LED characteristic depends of LED mode */
    m_led_write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    m_led_write_params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    m_led_write_params.offset   = 0;
    m_led_write_params.p_value  = (uint8_t *)p_led_params;
    m_led_write_params.handle   = p_thingy->led.handle;
    
    ret_code_t err_code = sd_ble_gattc_write(p_thingy->conn_handle,
                                            &m_led_write_params);
    APP_ERROR_CHECK(err_code);
}

ret_code_t ble_thingy_master_init(thingy_device_t * p_thingy_dev_table, uint8_t thingy_cnt)
{
    ret_code_t err_code;

    /* Assign thingy_dev_table to m_ble_dev */
    m_ble_dev.p_dev_table    = p_thingy_dev_table;
    m_ble_dev.thingy_dev_cnt = thingy_cnt;
    thingy_struct_init();
    ble_stack_init();
    err_code =  sd_ble_uuid_vs_add(&m_vendor_uuid, &m_vendor_uuid_type);

    return err_code;
}

void ble_thingy_master_start(void)
{
    ret_code_t err_code;

    UNUSED_RETURN_VALUE(sd_ble_gap_scan_stop());

    err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}


/**
 * @}
 */
