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
 * @defgroup thread_secure_dfu_example_main main.c
 * @{
 * @ingroup thread_secure_dfu_example
 * @brief Thread Secure DFU Example Application main file.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "app_util.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp_thread.h"
#include "nrf_delay.h"
#include "mem_manager.h"
#include "nrf_assert.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "sdk_config.h"
#include "nrf_dfu_utils.h"
#include "coap_dfu.h"
#include "background_dfu_state.h"

#include "thread_utils.h"

#include <openthread/diag.h>
#include <openthread/cli.h>
#include <openthread/thread.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/random.h>

extern const app_timer_id_t nrf_dfu_inactivity_timeout_timer_id;
void handle_dfu_command(int argc, char *argv[]);

typedef struct
{
    otNetifAddress   addresses[10];
    bool             trigger_dfu;
} application_t;

static application_t m_app =
{
    .trigger_dfu   = false,
};

static otCliCommand m_user_commands[] =
{
    {
        .mName = "dfu",
        .mCommand = handle_dfu_command
    }
};


__WEAK bool nrf_dfu_button_enter_check(void)
{
    // Dummy function for Keil compilation. This should not be called.
    return false;
}


__WEAK void nrf_bootloader_app_start(uint32_t start_addr)
{
    (void)start_addr;
    // Dummy function for Keil compilation. This should not be called.
}


void handle_dfu_command(int argc, char *argv[])
{
    if (argc == 0)
    {
        otCliUartAppendResult(OT_ERROR_PARSE);
        return;
    }

    if (strcmp(argv[0], "diag") == 0)
    {
        struct background_dfu_diagnostic diag;
        coap_dfu_diagnostic_get(&diag);
        otCliUartOutputFormat("build_id: 0x%08x, "
                              "state: %d, "
                              "prev_state: %d, ",
                              diag.build_id,
                              diag.state,
                              diag.prev_state);
        otCliUartOutputFormat("\r\n");
        otCliUartAppendResult(OT_ERROR_NONE);
    }
}


void coap_dfu_handle_error(void)
{
    coap_dfu_reset_state();
}


static void address_print(const otIp6Address *addr)
{
    char ipstr[40];
    snprintf(ipstr, sizeof(ipstr), "%x:%x:%x:%x:%x:%x:%x:%x",
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 0)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 1)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 2)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 3)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 4)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 5)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 6)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 7)));

    NRF_LOG_INFO("%s\r\n", (uint32_t)ipstr);
}


static void addresses_print(otInstance * aInstance)
{
    for (const otNetifAddress *addr = otIp6GetUnicastAddresses(aInstance); addr; addr = addr->mNext)
    {
        address_print(&addr->mAddress);
    }
}


static void state_changed_callback(uint32_t aFlags, void *aContext)
{
    if (aFlags & OT_CHANGED_THREAD_NETDATA)
    {
        otIp6SlaacUpdate(thread_ot_instance_get(),
                         m_app.addresses,
                         sizeof(m_app.addresses) / sizeof(m_app.addresses[0]),
                         otIp6CreateRandomIid,
                         NULL);

        addresses_print(thread_ot_instance_get());
    }

    otDeviceRole role = otThreadGetDeviceRole(thread_ot_instance_get());
    NRF_LOG_INFO("New role: %d\r\n", role);

    if (aFlags & OT_CHANGED_THREAD_ROLE)
    {
        switch(role)
        {
            case OT_DEVICE_ROLE_CHILD:
            case OT_DEVICE_ROLE_ROUTER:
            case OT_DEVICE_ROLE_LEADER:
                m_app.trigger_dfu = true;
                break;

            case OT_DEVICE_ROLE_DISABLED:
            case OT_DEVICE_ROLE_DETACHED:
            default:
                break;
        }
    }
}


/**@brief Function for initializing the Thread Board Support Package.
 */
static void thread_bsp_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Thread Stack.
 */
static void thread_instance_init(void)
{
    thread_configuration_t  thread_configuration =
    {
        .role              = RX_ON_WHEN_IDLE,
        .autocommissioning = true,
    };

    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(state_changed_callback);

    otCliUartSetUserCommands(m_user_commands, sizeof(m_user_commands) / sizeof(otCliCommand));
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    log_init();

    uint32_t err_code = nrf_mem_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    thread_instance_init();

    err_code = coap_dfu_init(thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);

    thread_bsp_init();

    uint32_t now, before = otPlatAlarmMilliGetNow();

    while (true)
    {
        coap_dfu_process();

        thread_process();

        now = otPlatAlarmMilliGetNow();
        if (now - before > 1000)
        {
            coap_time_tick();
            before = now;
        }

        if (m_app.trigger_dfu)
        {
            m_app.trigger_dfu = false;
            coap_dfu_trigger(NULL);
        }

        NRF_LOG_PROCESS();
    }
}

/** @} */
