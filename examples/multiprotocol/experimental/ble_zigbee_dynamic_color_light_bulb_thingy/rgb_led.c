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
 * @defgroup zigbee_examples_ble_zigbee_color_light_bulb_thingy rgb_led.c
 * @{
 * @ingroup zigbee_examples
 */
#include <string.h>
#include "rgb_led.h"
#include "nrf_gpio.h"
#include "nrf_drv_pwm.h"

#define RGB_LED_PWM_INSTANCE    NRF_DRV_PWM_INSTANCE(0) /**< PWM instance used to drive dimmable light bulb. */
#define RGB_LED_PWM_VALUE_MAX   1024                    /**< PWM counter maximum value. */
#define RGB_LED_PWM_VALUE_MIN   35                      /**< Minimal PWM counter value, which lights up the LED. */

#define BULB_LED_R              NRF_GPIO_PIN_MAP(1,12)  /**< Pin number of red LED of the RGB tape. */
#define BULB_LED_G              NRF_GPIO_PIN_MAP(1,13)  /**< Pin number of green LED of the RGB tape. */
#define BULB_LED_B              NRF_GPIO_PIN_MAP(1,11)  /**< Pin number of blue LED of the RGB tape. */

/* Declare app PWM instance for controlling LED tape. */
static led_params_t                m_led_params;
static nrf_drv_pwm_t               m_led_pwm = RGB_LED_PWM_INSTANCE;
static nrf_pwm_values_individual_t m_led_values;

static nrf_pwm_sequence_t const m_led_seq =
{
    .values.p_individual = &m_led_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_led_values),
    .repeats             = 0,
    .end_delay           = 0
};

/* LED sequence counter increment. */
const uint16_t c_led_phase_step = 1;
/* LED sequence counter value, after which the next element is applied. */
const uint16_t c_led_phase_step_divisor = 100;
/* LED brightness sequence, played to imitate 'breathe' effect. */
const uint8_t  c_led_breathe_seq[] = {10, 20, 40, 80, 120, 160, 200, 240, 255, 200, 120, 40, 20};
/* LED correction values, in percent, relative to the current brightness level. */
const uint16_t c_led_color_cal[] = {150, 110, 100};
/* Brightness level to PWM counter matrix. */
const uint16_t LED_BRIGHTNESS[32] = {
    0, 1, 3, 6, 10, 16, 23, 32, 43, 56, 71, 88, 107,
    129, 154, 181, 210, 242, 277, 315, 356, 400, 447,
    498, 551, 608, 668, 732, 799, 869, 944, 1023};

/**@brief Function for converting single-byte brightness level to PWM counter value.
 *
 * @param[in]  brightness  Brightness leve in 0-255 range.
 * @param[in]  correction  Brightness correction value, in percent.
 *
 * @returns  PWM counter value.
 **/
static uint16_t brightness_to_pwm(uint8_t brightness, uint16_t correction)
{
    uint16_t pwm_signal;
    brightness = brightness / 8;

    pwm_signal = RGB_LED_PWM_VALUE_MIN + (LED_BRIGHTNESS[brightness] * correction) / 100;

    if (LED_BRIGHTNESS[brightness] == 0)
    {
        pwm_signal = 0;
    }
    else if (pwm_signal > RGB_LED_PWM_VALUE_MAX)
    {
        pwm_signal = RGB_LED_PWM_VALUE_MAX;
    }

    return RGB_LED_PWM_VALUE_MAX - pwm_signal;
}

/* Function for modulating LED strip. */
static void led_seq_handler(nrf_drv_pwm_evt_type_t event_type)
{
    static uint16_t   m_phase    = 0;
    uint16_t        * p_channels = (uint16_t *)&m_led_values;
    /*lint -e662 */
    uint8_t         * p_colors   = (uint8_t *)&m_led_params.r;
    /*lint -restore */

    UNUSED_VARIABLE(p_channels);

    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            switch (m_led_params.mode)
            {
                case LED_MODE_OFF:
                    p_channels[i] = brightness_to_pwm(0, c_led_color_cal[i]);
                    break;

                case LED_MODE_CONSTANT:
                    {
                        /*lint -e661 -e662 */
                        p_channels[i] = brightness_to_pwm(p_colors[i], c_led_color_cal[i]);
                        /*lint -restore */
                      }
                    break;

                case LED_MODE_BREATHING:
                case LED_MODE_ONE_SHOT:
                {
                    m_phase += c_led_phase_step;
                    uint16_t current_phase = m_phase / c_led_phase_step_divisor;

                    if (current_phase >= ARRAY_SIZE(c_led_breathe_seq))
                    {
                        current_phase = 0;
                        m_phase = 0;

                        if (m_led_params.mode == LED_MODE_ONE_SHOT)
                        {
                            m_led_params.mode = LED_MODE_OFF;
                        }
                    }

                    if (m_led_params.color & (1 << i))
                    {
                        uint16_t brightness = ((m_led_params.intensity * c_led_breathe_seq[current_phase]) / 100);
                        p_channels[i] = brightness_to_pwm(brightness, c_led_color_cal[i]);
                    }
                    else
                    {
                        p_channels[i] = brightness_to_pwm(0, c_led_color_cal[i]);
                    }
                } break;


                default:
                    break;
            }
        }
    }
}

void rgb_led_update(led_params_t * p_led_params)
{
    m_led_params = *p_led_params;
}

void rgb_led_init(void)
{
    uint32_t err_code;

    nrf_drv_pwm_config_t const led_pwm_config =
    {
        .output_pins =
        {
            BULB_LED_R,               // channel 0
            BULB_LED_G,               // channel 1
            BULB_LED_B,               // channel 2
            NRF_DRV_PWM_PIN_NOT_USED, // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = RGB_LED_PWM_VALUE_MAX,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    /* Initialize PWM in order to control dimmable RGB LED tape. */
    err_code = nrf_drv_pwm_init(&m_led_pwm, &led_pwm_config, led_seq_handler);
    APP_ERROR_CHECK(err_code);

    /* Set default color (red, constant) on the LED tape. */
    err_code = nrf_drv_pwm_simple_playback(&m_led_pwm, &m_led_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    APP_ERROR_CHECK(err_code);
}
