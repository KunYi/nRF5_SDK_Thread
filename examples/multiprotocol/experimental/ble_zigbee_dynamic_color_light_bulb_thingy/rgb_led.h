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
/** @file
 *
 * @defgroup zigbee_examples_ble_zigbee_color_light_bulb_thingy rgb_led.h
 * @{
 * @ingroup zigbee_examples
 */

#ifndef RGB_LED_H__
#define RGB_LED_H__

#include "app_util_platform.h"

#ifdef __CC_ARM
#pragma anon_unions
#endif

/* LED modes */
typedef enum led_mode_e
{
    LED_MODE_OFF       = 0,
    LED_MODE_CONSTANT  = 1,
    LED_MODE_BREATHING = 2,
    LED_MODE_ONE_SHOT  = 3
} led_mode_t;

/* Structure for storing LED configuration */
typedef PACKED_STRUCT led_params_s
{
    led_mode_t mode;                 /**< Mode of LED behaviour. */
    __PACKED union
    {
        PACKED_STRUCT
        {
            uint8_t  r;         /**< Red color value. */
            uint8_t  g;         /**< Green color value. */
            uint8_t  b;         /**< Blue color value. */
        };
        PACKED_STRUCT
        {
            uint8_t  color;     /**< Value related to color in breathing mode, binary mask. */
            uint8_t  intensity; /**< Intensity of LED in breathing mode (0 - 100)%. */
            uint16_t delay;     /**< Amount of time determining how fast LED is breathing (50 ms - 10s). */
        };
    };
} led_params_t;

/**
 * @brief Init RGB LED module.
 */
void rgb_led_init(void);

/**
 * @brief Update LED state.
 *
 * @param[in] p_led_params a pointer to LED parameters.
 */
void rgb_led_update(led_params_t * p_led_params);

#endif

/**
 * @}
 */
