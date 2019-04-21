/**
 * Copyright (c) 2019, Nordic Semiconductor ASA
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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <nrfx.h>
#include <nrfx_pwm.h>
#include <hal/nrf_gpio.h>

#include "drv_ws2812.h"

#define WS2812_T1H                  (14U | 0x8000U)
#define WS2812_T0H                  (6U | 0x8000U)

#define LED_CHAIN_TOTAL_BYTE_WIDTH  (DRV_WS2812_LED_CHAIN_PIXELS_COUNT_MAX * 3U)
#define LED_CHAIN_TOTAL_BIT_WIDTH   (LED_CHAIN_TOTAL_BYTE_WIDTH * 8U)

typedef struct
{
    uint8_t g;
    uint8_t r;
    uint8_t b;
} rgb_color_t;

/**@brief Led state buffer */
static rgb_color_t m_led_matrix_buffer[DRV_WS2812_LED_CHAIN_PIXELS_COUNT_MAX];

/**@brief PWM module used by the driver */
static nrfx_pwm_t m_pwm = NRFX_PWM_INSTANCE(DRV_WS2812_PWM_INSTANCE_NO);

/**@brief Buffer used directly by PWM module to generate DOUT waveform */
static nrf_pwm_values_common_t pwm_duty_cycle_values[LED_CHAIN_TOTAL_BIT_WIDTH];

static volatile bool pwm_sequencue_finished = true;

static nrf_pwm_sequence_t pwm_sequence =
{
    .values.p_common = pwm_duty_cycle_values,
    .length          = (sizeof(pwm_duty_cycle_values) / sizeof(uint16_t)),
    .repeats         = 0,
    .end_delay       = 0
};

static void pwm_handler(nrfx_pwm_evt_type_t event_type)
{
    switch(event_type)
    {
        case NRFX_PWM_EVT_FINISHED:
            pwm_sequencue_finished = true;
            break;

        default:
            break;
    }
}

static uint32_t pwm_init(uint8_t dout_pin)
{
    nrfx_pwm_config_t pwm_config = NRFX_PWM_DEFAULT_CONFIG;

    pwm_config.output_pins[0] = NRFX_PWM_PIN_NOT_USED; 
    pwm_config.output_pins[1] = dout_pin;
    pwm_config.output_pins[2] = NRFX_PWM_PIN_NOT_USED;
    pwm_config.output_pins[3] = NRFX_PWM_PIN_NOT_USED;
    pwm_config.load_mode      = NRF_PWM_LOAD_COMMON;
    // WS2812 protocol requires a 800 kHz PWM frequency. PWM Top value = 20 and Base Clock = 16 MHz achieves this
    pwm_config.top_value      = 20;
    pwm_config.base_clock     = NRF_PWM_CLK_16MHz;
    
    return nrfx_pwm_init(&m_pwm, &pwm_config, pwm_handler);
}

static void make_rgb_color(rgb_color_t *rgb_color, uint32_t color)
{
    rgb_color->b = (uint8_t)color;
    color >>= 8;
    rgb_color->g = (uint8_t)color;
    color >>= 8;
    rgb_color->r = (uint8_t)color;
}

static void convert_rgb_to_pwm_sequence(void)
{
    uint8_t * ptr = (uint8_t *)m_led_matrix_buffer;
    size_t    pwm_duty_cycle_idx = 0;
    size_t    pixel_no;

    for (pixel_no = 0U; pixel_no < LED_CHAIN_TOTAL_BYTE_WIDTH; ++pixel_no)
    {
        uint_fast8_t bit;
        uint_fast8_t b = *(ptr++);

        /* Process bits in byte b, MSB first */
        for (bit = 0U; bit < 8U; ++bit)
        {
            uint16_t pwm = WS2812_T0H;
            if ( (b & 0x80U) != 0U)
            {
                pwm = WS2812_T1H;
            }
            pwm_duty_cycle_values[pwm_duty_cycle_idx++] = pwm;
            b <<= 1;
        }
    }
}

uint32_t drv_ws2812_init(uint8_t dout_pin)
{   
    memset(m_led_matrix_buffer, 0x00, sizeof(m_led_matrix_buffer));   
    return pwm_init(dout_pin);
}

uint32_t drv_ws2812_display(void)
{
    uint32_t result = NRF_ERROR_BUSY;

    if (pwm_sequencue_finished)
    {
        convert_rgb_to_pwm_sequence();
        result = drv_ws2812_refresh();
    }

    return result;
}

uint32_t drv_ws2812_refresh(void)
{
    uint32_t result = NRF_ERROR_BUSY;

    if (pwm_sequencue_finished)
    {
        pwm_sequencue_finished = false;
        result = nrfx_pwm_simple_playback(&m_pwm, &pwm_sequence, 1, NRFX_PWM_FLAG_STOP);
    }

    return result;
}

bool drv_ws2812_is_refreshing(void)
{
    return !pwm_sequencue_finished;
}

void drv_ws2812_set_pixel(uint32_t pixel_no, uint32_t color)
{
    if (pixel_no < DRV_WS2812_LED_CHAIN_PIXELS_COUNT_MAX)
    {
        make_rgb_color(&m_led_matrix_buffer[pixel_no], color);
    }
}

void drv_ws2812_set_pixel_all(uint32_t color)
{
    rgb_color_t rgb_color;
    make_rgb_color(&rgb_color, color);

    rgb_color_t * p_iter_rgb_color;
    for (p_iter_rgb_color = &m_led_matrix_buffer[0];
            p_iter_rgb_color < &m_led_matrix_buffer[DRV_WS2812_LED_CHAIN_PIXELS_COUNT_MAX];
            ++p_iter_rgb_color)
    {
        *p_iter_rgb_color = rgb_color;
    }
}
