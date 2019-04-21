/**
 * Copyright (c) 2018 - 2019, Nordic Semiconductor ASA
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

#include <stdio.h>

#include "zboss_api.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "boards.h"

#ifdef BOARD_PCA10056
#define ERASE_PIN    NRF_GPIO_PIN_MAP(1,9)                                          /**< Pin 1.09. */
#define READ_RETRIES 10                                                             /**< Number of retries until the pin value stabilizes. */
#endif

/**@brief Function to set the Erase persistent storage depending on the erase pin
 */
zb_void_t zigbee_erase_persistent_storage(zb_bool_t erase)
{
#ifdef BOARD_PCA10056
    if (!erase)
    {
        nrf_gpio_cfg_input(ERASE_PIN, NRF_GPIO_PIN_PULLUP);
        
        int i;
        volatile uint32_t pin_state;

        /* Read a few times to make sure the value in the register has been updated. */
        for (i = 0; i < READ_RETRIES; i++)
        {
            pin_state = nrf_gpio_pin_read(ERASE_PIN);
        }

        if (pin_state == 0)
        {
            erase = ZB_TRUE;
            NRF_LOG_INFO("Forcing flash erasure due to pin state");
        }

        nrf_gpio_cfg_default(ERASE_PIN);
    }
#endif
    zb_set_nvram_erase_at_start(erase);
}

int to_hex_string(char * p_out, uint16_t out_size,
                  const uint8_t * p_in, uint8_t in_size,
                  bool reverse)
{
    int bytes_written = 0;
    int status;
    int i = reverse ? in_size - 1 : 0;

    for (; in_size > 0; in_size--)
    {
        status = snprintf(p_out + bytes_written,
                          out_size - bytes_written,
                          "%02x", p_in[i]);
        if (status < 0)
        {
            return status;
        }

        bytes_written += status;
        i += reverse ? -1 : 1;
    }

    return bytes_written;
}

int ieee_addr_to_str(char * p_str_buf, uint16_t buf_len, const zb_ieee_addr_t p_addr)
{
    return to_hex_string(p_str_buf, buf_len,
                         (const uint8_t *)p_addr, sizeof(zb_ieee_addr_t),
                         true);
}
