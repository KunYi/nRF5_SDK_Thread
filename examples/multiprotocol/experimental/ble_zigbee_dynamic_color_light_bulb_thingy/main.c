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
 * @defgroup zigbee_examples_thingy_master_zed_color_light_bulb main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Dynamic multiprotocol example application to demonstrate control on BLE device (peripheral role) using zigbee device.
 */

#include "zboss_api.h"
#include "zb_mem_config_min.h"
#include "zigbee_color_light.h"
#include "ble_thingy_master.h"

#include "zb_error_handler.h"
#include "app_timer.h"
#include "zb_zcl_color_control.h"
#include "nrf_pwr_mgmt.h"
#include "bsp.h"

 /* ZigBee device configuration values. */
#define HA_COLOR_LIGHT_ENDPOINT_FIRST       10                      /**< Device first endpoint, used to receive light controlling commands. */
#define HA_COLOR_LIGHT_ENDPOINT_SECOND      11                      /**< Device second endpoint, used to receive light controlling commands. */
#define ERASE_PERSISTENT_CONFIG             ZB_FALSE                /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */
#define ZB_USE_LEGACY_MODE                  0                       /**< Set to 1 to use legacy mode - to be compliant with devices older than Zigbee 3.0 i.e. Amazon Echo Plus, set to 0 to use Zigee 3.0 stack version. */
#define CHECK_VALUE_CHANGE_PERIOD           120                     /**< Period of time [ms] to check if value of cluster is changing. */
#define ZB_EP_LIST_LENGTH                   2                       /**< Number of endpoints. */
#define ZIGBEE_NETWORK_STATE_LED            BSP_BOARD_LED_2         /**< LED indicating that light switch successfully joined ZigBee network. */
#define THINGY_DEVICE_CNT                   2                       /**< Amount of thingy devices to connect to. */

/* Structure to store zigbee endpoint related variables */
typedef struct
{
    zb_bulb_dev_ctx_t         * const p_device_ctx;                 /**< Pointer to structure containing cluster attributes. */
    led_params_t                led_params;                         /**< Table to store RGB color values to control thigny LED. */
    const uint8_t               ep_id;                              /**< Endpoint ID. */
    zb_bool_t                   value_changing_flag;                /**< Variable used as flag while detecting changing value in Level Control attribute. */
    uint8_t                     prev_lvl_ctrl_value;                /**< Variable used to store previous attribute value while detecting changing value in Level Control attribute. */
    thingy_device_t           * p_thingy;                           /**< Pointer to thingy device . */
} bulb_device_ep_ctx_t;

/* Define nrf app timer to handle too quick cluster attribute value changes in zboss stack */
APP_TIMER_DEF(zb_app_timer);

/* Variable to store zigbee address */
static zb_ieee_addr_t    m_ieee_addr;

/* Declare context variable and cluster attribute list for first endpoint */
static zb_bulb_dev_ctx_t zb_dev_ctx_first;
ZB_DECLARE_COLOR_LIGHT_BULB_CLUSTER_ATTR_LIST(zb_dev_ctx_first, color_light_bulb_clusters_first);

/* Declare context variable and cluster attribute list for second endpoint */
static zb_bulb_dev_ctx_t zb_dev_ctx_second;
ZB_DECLARE_COLOR_LIGHT_BULB_CLUSTER_ATTR_LIST(zb_dev_ctx_second, color_light_bulb_clusters_second);

/* Declare two endpoints for color controllable and dimmable light bulbs */
ZB_ZCL_DECLARE_COLOR_LIGHT_EP(color_light_bulb_ep_first, HA_COLOR_LIGHT_ENDPOINT_FIRST, color_light_bulb_clusters_first);
ZB_ZCL_DECLARE_COLOR_LIGHT_EP(color_light_bulb_ep_second, HA_COLOR_LIGHT_ENDPOINT_SECOND, color_light_bulb_clusters_second);

/* Declare context for endpoints */
ZBOSS_DECLARE_DEVICE_CTX_2_EP(color_light_ctx, color_light_bulb_ep_first, color_light_bulb_ep_second);

/* Declare thingy device list with device count store in APP_BLE_THINGY_DEVICE_CNT */
static thingy_device_t m_thingy_dev[THINGY_DEVICE_CNT] = {0};

/* Table to store information about zigbee endpoints */
static bulb_device_ep_ctx_t zb_ep_dev_ctx[ZB_EP_LIST_LENGTH] =
{
    {
        .p_device_ctx = &zb_dev_ctx_first,
        .ep_id = HA_COLOR_LIGHT_ENDPOINT_FIRST,
        .value_changing_flag    = ZB_FALSE,
        .prev_lvl_ctrl_value    = 0,
        .led_params             =
            {
                .mode       = APP_BLE_THINGY_LED_MODE_CONSTANT,
                {
                    {
                        .r_value    = 0,
                        .g_value    = 0,
                        .b_value    = 0
                    }
                }
            },
        .p_thingy = &m_thingy_dev[0]
    },
    {
        .p_device_ctx = &zb_dev_ctx_second,
        .ep_id = HA_COLOR_LIGHT_ENDPOINT_SECOND,
        .value_changing_flag    = ZB_FALSE,
        .prev_lvl_ctrl_value    = 0,
        .led_params             =
            {
                .mode       = APP_BLE_THINGY_LED_MODE_CONSTANT,
                {
                    {
                        .r_value    = 0,
                        .g_value    = 0,
                        .b_value    = 0
                    }
                }
            },
        .p_thingy = &m_thingy_dev[1]
    }
};

/**@brief Function for initializing the log.
 * 
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the Power management.
 * 
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to convert hue_stauration to RGB color space.
 * 
 * @param[IN]  hue          Hue value of color.
 * @param[IN]  saturation   Saturation value of color.
 * @param[IN]  brightness   Brightness value of color.
 * @param[OUT] p_led_params Pointer to structure containing parameters to write to LED characteristic
 */
static void convert_hsb_to_rgb(uint8_t hue, uint8_t saturation, uint8_t brightness, led_params_t * p_led_params)
{
    /* Check if p_leds_params is not NULL pointer */
    if (p_led_params == NULL)
    {
        NRF_LOG_INFO("Incorrect pointer to led params");
        return;
    }
    /* C, X, m are auxiliary variables */
    float C     = 0.0;
    float X     = 0.0;
    float m     = 0.0;
    /* Convertion HSB --> RGB */
    C = (brightness / 255.0f) * (saturation / 254.0f);
    X = (hue / 254.0f) * 6.0f;
    /* Casting in var X is necessary due to implementation of floating-point modulo_2 */
    /*lint -e653 */
    X = (X - (2 * (((uint8_t) X) / 2)));
    /*lint -restore */
    X -= 1.0f;
    X = C * (1.0f - ((X > 0.0f) ? (X) : (-1.0f * X)));
    m = (brightness / 255.0f) - C;
    
    /* Hue value is stored in range (0 - 255) instead of (0 - 360) degree */
    if (hue <= 42) /* hue < 60 degree */
    {
        p_led_params->r_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((0.0f + m) * 255.0f);
    }
    else if (hue <= 84)  /* hue < 120 degree */
    {
        p_led_params->r_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((0.0f + m) * 255.0f);
    }
    else if (hue <= 127) /* hue < 180 degree */
    {
        p_led_params->r_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((X + m) * 255.0f);
    }
    else if (hue < 170)  /* hue < 240 degree */
    {
        p_led_params->r_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((C + m) * 255.0f);
    }
    else if (hue <= 212) /* hue < 300 degree */
    {
        p_led_params->r_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((C + m) * 255.0f);
    }
    else                /* hue < 360 degree */
    {
        p_led_params->r_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((X + m) * 255.0f);
    }
}

/**@brief Function for updating RGB color value.
 * 
 * @param[IN] p_ep_dev_ctx pointer to endpoint device ctx.
 */
static void zb_update_color_values(bulb_device_ep_ctx_t * p_ep_dev_ctx)
{
    if ((p_ep_dev_ctx == NULL) || (p_ep_dev_ctx->p_device_ctx == NULL))
    {
        NRF_LOG_INFO("Can not update color values - bulb_device_ep_ctx uninitialised");
        return;
    }
    convert_hsb_to_rgb(p_ep_dev_ctx->p_device_ctx->color_control_attr.set_color_info.current_hue,
                       p_ep_dev_ctx->p_device_ctx->color_control_attr.set_color_info.current_saturation,
                       p_ep_dev_ctx->p_device_ctx->level_control_attr.current_level,
                       &p_ep_dev_ctx->led_params);
}

/**@brief Function for changing the hue of the light bulb.
 * 
 * @param[IN] p_ep_dev_ctx  Pointer to endpoint device ctx.
 * @param[IN] new_hue       New value for hue.
 */
static void color_control_set_value_hue(bulb_device_ep_ctx_t * p_ep_dev_ctx, zb_uint8_t new_hue)
{
    p_ep_dev_ctx->p_device_ctx->color_control_attr.set_color_info.current_hue = new_hue;

    zb_update_color_values(p_ep_dev_ctx);
    ble_thingy_master_update_led(p_ep_dev_ctx->p_thingy, &p_ep_dev_ctx->led_params);
    
    NRF_LOG_INFO("Set color hue value: %i on endpoint: %hu", new_hue, p_ep_dev_ctx->ep_id);
}

/**@brief Function for changing the saturation of the light bulb.
 * 
 * @param[IN] p_ep_dev_ctx   pointer to endpoint device ctx.
 * @param[IN] new_saturation new value for saturation.
 */
static void color_control_set_value_saturation(bulb_device_ep_ctx_t * p_ep_dev_ctx, zb_uint8_t new_saturation)
{
    p_ep_dev_ctx->p_device_ctx->color_control_attr.set_color_info.current_saturation = new_saturation;

    zb_update_color_values(p_ep_dev_ctx);
    ble_thingy_master_update_led(p_ep_dev_ctx->p_thingy, &p_ep_dev_ctx->led_params);

    NRF_LOG_INFO("Set color saturation value: %i on endpoint: %hu", new_saturation, p_ep_dev_ctx->ep_id);
}

/**@brief Function for setting the light bulb brightness.
 * 
 * @param[IN] p_ep_dev_ctx Pointer to endpoint device ctx.
 * @param[IN] new_level    Light bulb brightness value.
 */
static void level_control_set_value(bulb_device_ep_ctx_t * p_ep_dev_ctx, zb_uint16_t new_level)
{
    p_ep_dev_ctx->p_device_ctx->level_control_attr.current_level = new_level;

    zb_update_color_values(p_ep_dev_ctx);
    ble_thingy_master_update_led(p_ep_dev_ctx->p_thingy, &p_ep_dev_ctx->led_params);

    NRF_LOG_INFO("Set level value: %i on endpoint: %hu", new_level, p_ep_dev_ctx->ep_id);

    /* According to the table 7.3 of Home Automation Profile Specification v 1.2 rev 29, chapter 7.1.3. */
    p_ep_dev_ctx->p_device_ctx->on_off_attr.on_off = (new_level ? ZB_TRUE : ZB_FALSE);
}

/**@brief Function for turning ON/OFF the light bulb.
 * 
 * @param[IN] p_ep_dev_ctx Pointer to endpoint device ctx.
 * @param[IN] on           Boolean light bulb state.
 */
static void on_off_set_value(bulb_device_ep_ctx_t * p_ep_dev_ctx, zb_bool_t on)
{
    p_ep_dev_ctx->p_device_ctx->on_off_attr.on_off = on;

    NRF_LOG_INFO("Set ON/OFF value: %i on endpoint: %hu", on, p_ep_dev_ctx->ep_id);

    if (on)
    {
        level_control_set_value(p_ep_dev_ctx, p_ep_dev_ctx->p_device_ctx->level_control_attr.current_level);
    }
    else
    {
        p_ep_dev_ctx->led_params.r_value = 0;
        p_ep_dev_ctx->led_params.g_value = 0;
        p_ep_dev_ctx->led_params.b_value = 0;
        ble_thingy_master_update_led(p_ep_dev_ctx->p_thingy, &p_ep_dev_ctx->led_params);
    }
}

/**@brief Function for handling nrf app timer.
 * 
 * @param[IN]   context   Void pointer to context function is called with.
 * 
 * @details Function is called with pointer to bulb_device_ep_ctx_t as argument.
 */
static void zb_app_timer_handler(void * context)
{
    bulb_device_ep_ctx_t  * p_bulb_device        = (bulb_device_ep_ctx_t *)context;
    zb_uint8_t            * p_bulb_attr_curr_lvl = &p_bulb_device->p_device_ctx->level_control_attr.current_level;
    
    if (p_bulb_device->prev_lvl_ctrl_value == *p_bulb_attr_curr_lvl)
    {
        p_bulb_device->value_changing_flag = ZB_FALSE;
        level_control_set_value(p_bulb_device, *p_bulb_attr_curr_lvl);
    }
    else
    {
        ret_code_t err_code;
        err_code = app_timer_start(zb_app_timer, APP_TIMER_TICKS(CHECK_VALUE_CHANGE_PERIOD), context);
        APP_ERROR_CHECK(err_code);
        p_bulb_device->prev_lvl_ctrl_value = *p_bulb_attr_curr_lvl;
    }
}

/**@brief Function to called if delayed BLE scanning is required.
 *
 * @param[IN]   param   Parameter function is called with, unused.
 */
static zb_void_t ble_thingy_master_start_cb(zb_uint8_t param)
{
    UNUSED_PARAMETER(param);
    ble_thingy_master_start();
}

/**@brief Function for initializing clusters attributes.
 * 
 * @param[IN]   p_device_ctx   Pointer to structure with device_ctx.
 * @param[IN]   ep_id          Endpoint ID.
 */
static void bulb_clusters_attr_init(zb_bulb_dev_ctx_t * p_device_ctx, zb_uint8_t ep_id)
{
    /* Basic cluster attributes data */
    p_device_ctx->basic_attr.zcl_version   = ZB_ZCL_VERSION;
    p_device_ctx->basic_attr.app_version   = BULB_INIT_BASIC_APP_VERSION;
    p_device_ctx->basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
    p_device_ctx->basic_attr.hw_version    = BULB_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.mf_name,
                          BULB_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.model_id,
                          BULB_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.date_code,
                          BULB_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

    p_device_ctx->basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.location_id,
                          BULB_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));


    p_device_ctx->basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    p_device_ctx->identify_attr.identify_time       = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
    p_device_ctx->identify_attr.commission_state    = ZB_ZCL_ATTR_IDENTIFY_COMMISSION_STATE_HA_ID_DEF_VALUE;

    /* On/Off cluster attributes data */
    p_device_ctx->on_off_attr.on_off                = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;
    p_device_ctx->on_off_attr.global_scene_ctrl     = ZB_TRUE;
    p_device_ctx->on_off_attr.on_time               = 0;
    p_device_ctx->on_off_attr.off_wait_time         = 0;

    /* Level control cluster attributes data */
    p_device_ctx->level_control_attr.current_level  = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE; // Set current level value to maximum
    p_device_ctx->level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;
    ZB_ZCL_LEVEL_CONTROL_SET_ON_OFF_VALUE(ep_id, p_device_ctx->on_off_attr.on_off);
    ZB_ZCL_LEVEL_CONTROL_SET_LEVEL_VALUE(ep_id, p_device_ctx->level_control_attr.current_level);
    
    /* Color control cluster attributes data */
    p_device_ctx->color_control_attr.set_color_info.current_hue         = ZB_ZCL_COLOR_CONTROL_HUE_RED;
    p_device_ctx->color_control_attr.set_color_info.current_saturation  = ZB_ZCL_COLOR_CONTROL_CURRENT_SATURATION_MAX_VALUE;
    /* Set to use hue & saturation */
    p_device_ctx->color_control_attr.set_color_info.color_mode          = ZB_ZCL_COLOR_CONTROL_COLOR_MODE_HUE_SATURATION;
    p_device_ctx->color_control_attr.set_color_info.color_temperature   = ZB_ZCL_COLOR_CONTROL_COLOR_TEMPERATURE_DEF_VALUE;
    p_device_ctx->color_control_attr.set_color_info.remaining_time      = ZB_ZCL_COLOR_CONTROL_REMAINING_TIME_MIN_VALUE;
    p_device_ctx->color_control_attr.set_color_info.color_capabilities  = ZB_ZCL_COLOR_CONTROL_CAPABILITIES_HUE_SATURATION;
    /* According to ZCL spec 5.2.2.2.1.12 0x00 shall be set when CurrentHue and CurrentSaturation are used. */
    p_device_ctx->color_control_attr.set_color_info.enhanced_color_mode = 0x00;
    /* According to 5.2.2.2.1.10 execute commands when device is off. */
    p_device_ctx->color_control_attr.set_color_info.color_capabilities  = ZB_ZCL_COLOR_CONTROL_OPTIONS_EXECUTE_IF_OFF;
    /* According to ZCL spec 5.2.2.2.2 0xFF shall be set when specific value is unknown. */
    p_device_ctx->color_control_attr.set_defined_primaries_info.number_primaries = 0xff;
}

/**@brief Function to get pointer to device context by zigbee endpoint ID
 * 
 * @param[IN] ep  Zigbee endpoint ID.
 * 
 * @returns Pointer to matching device context
 */
static bulb_device_ep_ctx_t * find_device_ctx_by_ep_id(zb_uint8_t ep)
{
    for (uint8_t i = 0; i < ZB_EP_LIST_LENGTH; i++)
    {
        if (ep == zb_ep_dev_ctx[i].ep_id)
        {
            return &zb_ep_dev_ctx[i];
        }
    }

    return NULL;
}

/**@brief Perform local operation - leave network.
 *
 * @param[IN]   param   Reference to ZigBee stack buffer that will be used to construct leave request.
 */
static void zb_leave_nwk(zb_uint8_t param)
{
    zb_ret_t                    zb_err_code;
    zb_buf_t                  * p_buf = ZB_BUF_FROM_REF(param);
    zb_zdo_mgmt_leave_param_t * p_req_param;

    p_req_param = ZB_GET_BUF_PARAM(p_buf, zb_zdo_mgmt_leave_param_t);
    UNUSED_RETURN_VALUE(ZB_BZERO(p_req_param, sizeof(zb_zdo_mgmt_leave_param_t)));

    /* We are going to leave */
    if (param)
    {
        /* Set dst_addr == local address for local leave */
        p_req_param->dst_addr = ZB_PIBCACHE_NETWORK_ADDRESS();
        p_req_param->rejoin   = ZB_TRUE;
        UNUSED_RETURN_VALUE(zdo_mgmt_leave_req(param, NULL));
    }
    else
    {
        zb_err_code = ZB_GET_OUT_BUF_DELAYED(zb_leave_nwk);
        ZB_ERROR_CHECK(zb_err_code);
    }
}

/**@brief Function for starting join/rejoin procedure.
 *
 * param[IN]   leave_type   Type of leave request (with or without rejoin).
 */
static zb_void_t zb_retry_join(zb_uint8_t leave_type)
{
    zb_bool_t comm_status;

    if (leave_type == ZB_NWK_LEAVE_TYPE_RESET)
    {
        comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
        ZB_COMM_STATUS_CHECK(comm_status);
    }
}

/**@brief Function for leaving current network and starting join procedure afterwards.
 *
 * @param[IN]   param   Optional reference to ZigBee stack buffer to be reused by leave and join procedure.
 */
static zb_void_t zb_leave_and_join(zb_uint8_t param)
{
    if (ZB_JOINED())
    {
        /* Leave network. Joining procedure will be initiated inisde ZigBee stack signal handler. */
        zb_leave_nwk(param);
    }
    else
    {
        /* Already left network. Start joining procedure. */
        zb_retry_join(ZB_NWK_LEAVE_TYPE_RESET);

        if (param)
        {
            ZB_FREE_BUF_BY_REF(param);
        }
    }
}

/**@brief Callback function for handling ZCL commands.
 * 
 * @param[IN]   param   Reference to ZigBee stack buffer used to pass received data.
 */
static zb_void_t zcl_device_cb(zb_uint8_t param)
{
    zb_uint16_t                       cluster_id;
    zb_uint8_t                        attr_id;
    zb_buf_t                        * p_buffer          = ZB_BUF_FROM_REF(param);
    zb_zcl_device_callback_param_t  * p_device_cb_param = ZB_GET_BUF_PARAM(p_buffer, zb_zcl_device_callback_param_t);
    // For redability pointer to specific endpoint context
    bulb_device_ep_ctx_t            * p_device_ep_ctx   = NULL;

    p_device_ep_ctx = find_device_ctx_by_ep_id(p_device_cb_param->endpoint);

    NRF_LOG_INFO("Received new ZCL callback %hd on endpoint %hu", p_device_cb_param->device_cb_id, p_device_cb_param->endpoint);

    if (p_device_ep_ctx == NULL)
    {
        return;
    }

    /* Set default response value. */
    p_device_cb_param->status = RET_OK;

    switch (p_device_cb_param->device_cb_id)
    {
        case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
            /* Set new value in cluster and then use nrf_app_timer to delay thingy led update if value is changing quickly */
            NRF_LOG_INFO("Level control setting to %d", p_device_cb_param->cb_param.level_control_set_value_param.new_value);
            p_device_ep_ctx->p_device_ctx->level_control_attr.current_level = p_device_cb_param->cb_param.level_control_set_value_param.new_value;

            if (p_device_ep_ctx->value_changing_flag != ZB_TRUE)
            {
                ret_code_t err_code;

                p_device_ep_ctx->prev_lvl_ctrl_value = p_device_ep_ctx->p_device_ctx->level_control_attr.current_level;
                err_code = app_timer_start(zb_app_timer, APP_TIMER_TICKS(CHECK_VALUE_CHANGE_PERIOD), p_device_ep_ctx);
                APP_ERROR_CHECK(err_code);

                p_device_ep_ctx->value_changing_flag = ZB_TRUE;
            }
            break;

        case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
            cluster_id = p_device_cb_param->cb_param.set_attr_value_param.cluster_id;
            attr_id    = p_device_cb_param->cb_param.set_attr_value_param.attr_id;

            if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
            {
                uint8_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data8;

                NRF_LOG_INFO("on/off attribute setting to %hd", value);
                if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
                {
                    on_off_set_value(p_device_ep_ctx, (zb_bool_t)value);
                }
            }
            else if (cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
            {
                uint16_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data16;

                NRF_LOG_INFO("level control attribute setting to %hd", value);
                if (attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID)
                {
                    level_control_set_value(p_device_ep_ctx, value);
                }
            }
            else if (cluster_id == ZB_ZCL_CLUSTER_ID_COLOR_CONTROL)
            {
                if (p_device_ep_ctx->p_device_ctx->color_control_attr.set_color_info.remaining_time <= 1)
                {
                    uint16_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data16;

                    switch (attr_id)
                    {
                        case ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID:
                            color_control_set_value_hue(p_device_ep_ctx, value);
                            break;

                        case ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID:
                            color_control_set_value_saturation(p_device_ep_ctx, value);
                            break;

                        default:
                            NRF_LOG_INFO("Unused attribute");
                            break;
                    }
                }
            }
            else
            {
                /* Other clusters can be processed here */
                NRF_LOG_INFO("Unhandled cluster attribute id: %d", cluster_id);
            }
            break;

        default:
            p_device_cb_param->status = RET_ERROR;
            NRF_LOG_INFO("Default case, returned error");
            break;
    }

    NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
}

/**@brief ZigBee stack event handler.
 * 
 * @param[IN] param   Reference to ZigBee stack buffer used to pass arguments (signal).
 */
void zboss_signal_handler(zb_uint8_t param)
{
    zb_zdo_app_signal_hdr_t       * p_sg_p          = NULL;
    zb_zdo_signal_leave_params_t  * p_leave_params  = NULL;
    zb_zdo_app_signal_type_t        sig             = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t                        status          = ZB_GET_APP_SIGNAL_STATUS(param);
    zb_ret_t                        zb_err_code     = 0;

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == RET_OK)
            {
                NRF_LOG_INFO("Joined network successfully");
                bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
                /* Start scanning on BLE */
                zb_err_code = ZB_SCHEDULE_CALLBACK(ble_thingy_master_start_cb, 0);
                ZB_ERROR_CHECK(zb_err_code);
            }
            else
            {
                NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
                zb_err_code = ZB_SCHEDULE_ALARM(zb_leave_and_join, 0, ZB_TIME_ONE_SECOND);
                ZB_ERROR_CHECK(zb_err_code);
            }
            break;

        case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            if (status != RET_OK)
            {
                NRF_LOG_WARNING("Production config is not present or invalid");
            }
            break;
            
        case ZB_ZDO_SIGNAL_LEAVE:
            if (status == RET_OK)
            {
                bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
                p_leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(p_sg_p, zb_zdo_signal_leave_params_t);
                NRF_LOG_INFO("Network left. Leave type: %d", p_leave_params->leave_type);
            }
            else
            {
                NRF_LOG_ERROR("Unable to leave network. Status: %d", status);
            }
          break;
          
        case ZB_COMMON_SIGNAL_CAN_SLEEP:
            {
                zb_sleep_now();
            }
            break;
            
        default:
            /* Unhandled signal. For more information see: zb_zdo_app_signal_type_e and zb_ret_e */
            NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
            break;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}

/**@brief Function for application main entry.
 * 
 */
int main(void)
{
    /* Declare variables to initialize ZED address */
    zb_ret_t    zb_err_code;
    ret_code_t  err_code;
    
    /* Initialise nrf app timer for thingy discovery purposes*/
    ble_thingy_master_timer_init();
    bsp_board_init(BSP_INIT_LEDS);
    log_init();
    power_management_init();

    /* Init Thingy master module */
    err_code = ble_thingy_master_init(m_thingy_dev, THINGY_DEVICE_CNT);
    APP_ERROR_CHECK(err_code);

    // Create app timer for handling fast changing cluster attribute values
    // app_timer_init() is previously called in ble_thingy_master_timer_init
    err_code = app_timer_create(&zb_app_timer, APP_TIMER_MODE_SINGLE_SHOT, zb_app_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* Read long address from FICR. */
    zb_osif_get_ieee_eui64(m_ieee_addr);

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("color_led_bulb");

    /* Set static long IEEE address. */
    zb_set_long_address(m_ieee_addr);

    /* Set device role */
    #if (ZB_USE_LEGACY_MODE == 1)
    zb_set_network_ed_role_legacy(IEEE_CHANNEL_MASK);
    #else
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    #endif

    zb_set_nvram_erase_at_start(ERASE_PERSISTENT_CONFIG);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(500));
    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_rx_on_when_idle(ZB_FALSE);

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&zb_dev_ctx_first, 0, sizeof(zb_dev_ctx_first)));
    UNUSED_RETURN_VALUE(ZB_MEMSET(&zb_dev_ctx_second, 0, sizeof(zb_dev_ctx_second)));

    /* Register color light bulb device context. */
    ZB_AF_REGISTER_DEVICE_CTX(&color_light_ctx);
    
     /* Register callback for handling ZCL commands. */
    ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

    /* Init attributes for endpoints */
    bulb_clusters_attr_init(zb_ep_dev_ctx[0].p_device_ctx, zb_ep_dev_ctx[0].ep_id);
    level_control_set_value(&zb_ep_dev_ctx[0], zb_ep_dev_ctx[0].p_device_ctx->level_control_attr.current_level);
    
    bulb_clusters_attr_init(zb_ep_dev_ctx[1].p_device_ctx, zb_ep_dev_ctx[1].ep_id);
    level_control_set_value(&zb_ep_dev_ctx[1], zb_ep_dev_ctx[1].p_device_ctx->level_control_attr.current_level);

    /* Start Zigbee Stack. */
    zb_err_code = zboss_start();
    ZB_ERROR_CHECK(zb_err_code);
    
    /* Enter main loop */
    for (;;)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}

/**
 * @}
 */
