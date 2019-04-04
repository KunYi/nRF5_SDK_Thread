/**
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
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
 * @defgroup cloud_coap_client_example_main main.c
 * @{
 * @ingroup cloud_coap_client_example_example
 * @brief Simple Cloud CoAP Client Example Application main file.
 *
 * @details This example demonstrates a CoAP client application that sends emulated
 *          temperature value to the thethings.io cloud. Example uses NAT64 on the
 *          Nordic's Thread Border Router soulution for IPv4 connectivity.
 *
 *
 */

#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"

#include "thread_coap_utils.h"
#include "thread_dns_utils.h"
#include "thread_utils.h"

#include <openthread/ip6.h>
#include <openthread/thread.h>

#define CLOUD_HOSTNAME            "coap.thethings.io"              /**< Hostname of the thethings.io cloud. */

#define CLOUD_URI_PATH            "v2/things/{THING-TOKEN}"        /**< Put your things URI here. */
#define CLOUD_THING_RESOURCE      "temp"                           /**< Thing resource name. */
#define CLOUD_COAP_CONTENT_FORMAT 50                               /**< Use application/json content format type. */

#define TEMPERATURE_INIT          22                               /**< The initial value of temperature. */
#define TEMPERATURE_MIN           15                               /**< Minimal possible temperature value. */
#define TEMPERATURE_MAX           30                               /**< Maximum possible temperature value. */

#define SCHED_QUEUE_SIZE          32                               /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE     APP_TIMER_SCHED_EVENT_DATA_SIZE  /**< Maximum app_scheduler event size. */

static thread_coap_cloud_information_t m_cloud_information =
{
    .p_cloud_hostname            = CLOUD_HOSTNAME,
    .p_cloud_uri_path            = CLOUD_URI_PATH,
    .p_cloud_thing_resource      = CLOUD_THING_RESOURCE,
    .cloud_coap_content_format   = CLOUD_COAP_CONTENT_FORMAT,
};

static uint16_t m_temperature = TEMPERATURE_INIT;

static void dns_response_handler(void         * p_context,
                                 const char   * p_hostname,
                                 otIp6Address * p_resolved_address,
                                 uint32_t       ttl,
                                 otError        error)
{
    if (error != OT_ERROR_NONE)
    {
        NRF_LOG_INFO("DNS response error %d.\r\n", error);
        return;
    }

    thread_coap_utils_peer_addr_set(p_resolved_address);
}

/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/

static void bsp_event_handler(bsp_event_t event)
{
    // If IPv6 address of the cloud is unspecified try to resolve hostname.
    if (!thread_coap_utils_peer_addr_is_set())
    {
        UNUSED_VARIABLE(thread_dns_utils_hostname_resolve(thread_ot_instance_get(),
                                                          m_cloud_information.p_cloud_hostname,
                                                          dns_response_handler,
                                                          NULL));
        return;
    }

    switch (event)
    {
        case BSP_EVENT_KEY_0:
            if (m_temperature == TEMPERATURE_MIN)
            {
                 // The minimal temperature has been already reached.
                break;
            }

             // Decrement temperature value.
            m_temperature -= 1;

            thread_coap_utils_cloud_data_update(&m_cloud_information, m_temperature);
            break;

        case BSP_EVENT_KEY_1:
            if (m_temperature == TEMPERATURE_MAX)
            {
                 // The maximum temperature has been already reached.
                break;
            }

            // Increment temperature value.
            m_temperature += 1;

            thread_coap_utils_cloud_data_update(&m_cloud_information, m_temperature);
            break;

        default:
            return; // no implementation needed
    }
}

/***************************************************************************************************
 * @section Callbacks
 **************************************************************************************************/

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        switch(otThreadGetDeviceRole(p_context))
        {
            case OT_DEVICE_ROLE_CHILD:
            case OT_DEVICE_ROLE_ROUTER:
            case OT_DEVICE_ROLE_LEADER:
                UNUSED_VARIABLE(thread_dns_utils_hostname_resolve(p_context,
                                                                  m_cloud_information.p_cloud_hostname,
                                                                  dns_response_handler,
                                                                  NULL));
                break;

            case OT_DEVICE_ROLE_DISABLED:
            case OT_DEVICE_ROLE_DETACHED:
            default:
                thread_coap_utils_peer_addr_clear();
                break;
        }
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n",
                flags,
                otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

 /**@brief Function for initializing the Thread Board Support Package
 */
static void thread_bsp_init(void)
{
    uint32_t error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(error_code);

    error_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(error_code);
}


/**@brief Function for initializing the Application Timer Module
 */
static void timer_init(void)
{
    uint32_t error_code = app_timer_init();
    APP_ERROR_CHECK(error_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Thread Stack
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .role              = RX_ON_WHEN_IDLE,
        .autocommissioning = true,
    };
    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(thread_state_changed_callback);
}


/**@brief Function for initializing the Constrained Application Protocol Module
 */
static void thread_coap_init(void)
{
    thread_coap_configuration_t thread_coap_configuration =
    {
        .coap_server_enabled               = false,
        .coap_client_enabled               = true,
        .coap_cloud_enabled                = true,
        .configurable_led_blinking_enabled = false,
    };

    thread_coap_utils_init(&thread_coap_configuration);
}


/**@brief Function for initializing scheduler module.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char * argv[])
{
    log_init();
    scheduler_init();
    timer_init();

    thread_instance_init();
    thread_coap_init();
    thread_bsp_init();

    while (true)
    {
        thread_process();
        app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }
    }
}

/**
 *@}
 **/

