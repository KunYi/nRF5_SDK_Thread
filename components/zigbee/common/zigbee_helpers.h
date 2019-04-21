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
#ifndef ZIGBEE_HELPERS_H__
#define ZIGBEE_HELPERS_H__

#include <stdint.h>
#include <stdbool.h>

#include "zboss_api.h"

/**@brief Name of the log module related to Zigbee. */
#define ZIGBEE_NRF_LOG_MODULE_NAME  zigbee

/**@brief Function for setting the Erase persistent storage, depending on the erase pin.
 * If the erase pin (1.39 by default, defined in zigbee_helpers.c) is shortened to
 * the ground, then the persistent storage is erased. Otherwise, whether the storage
 * is erased is decided upon the input parameter 'erase'. This behaviour is only valid
 * if PCA10056 is used.
 * 
 * @param [IN] erase Whether to erase the persistent storage in case the erase pin is 
 *                   not shortened to the ground.
 */
zb_void_t zigbee_erase_persistent_storage(zb_bool_t erase);

/**@brief Function for converting 64-bit address to hex string.
 *
 * @param[out] p_str_buf    Pointer to output buffer.
 * @param[in]  buf_len      Length of the buffer pointed by p_str_buf.
 * @param[in]  p_in         Zigbee IEEE address to be converted to string.
 *
 * @return     snprintf-compatible value. Less than zero means encoding error.
 *             Non-negative value is the number of characters that would have been written
 *             if the supplied buffer had been large enough. Value greater than or equal to
 *             buf_len means that the supplied buffer was too small.
 *
 * @note    Null terminator is written if buf_len is large enough, but does not
 *          count for the return value.
 */
int ieee_addr_to_str(char * p_str_buf, uint16_t buf_len, const zb_ieee_addr_t p_in);

/**@brief Function for converting an input buffer to a hex string.
 *
 * @param[out] p_out     Pointer to the output buffer.
 * @param[in]  out_size  Size of the output buffer.
 * @param[in]  p_in      Pointer to the input buffer.
 * @param[in]  in_size   Size of the input buffer.
 * @param[in]  reverse   If true, data output happens in the reverse order.
 *
 * @return     snprintf-compatible value. Less than zero means encoding error.
 *             Non-negative value is the number of characters that would have been written
 *             if the supplied buffer had been large enough. Value greater than or equal to
 *             buf_len means that the supplied buffer was too small.
 *
 * @note    Null terminator is written if buf_len is large enough, but does not
 *          count for the return value.
 */
int to_hex_string(char * p_out, uint16_t out_size,
                  const uint8_t * p_in, uint8_t in_size,
                  bool reverse);


#endif /* ZIGBEE_HELPERS_H__ */

