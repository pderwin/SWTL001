/*!
 * @file      lr11xx_hal.c
 *
 * @brief     HAL implementation for LR11xx radio chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_hal.h"
#include "configuration.h"
#include "system.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_reset( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;

    system_gpio_set_pin_state( radio_local->reset, SYSTEM_GPIO_PIN_STATE_LOW );
    system_time_wait_ms( 1 );
    system_gpio_set_pin_state( radio_local->reset, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_time_wait_ms( 1 );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;
    uint8_t                     command[4]     = { 0 };

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_write( radio_local->spi, command, 4 );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    uint8_t  dummy_byte  = 0x00;

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    /* 1st SPI transaction */
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_write( radio_local->spi, cbuffer, cbuffer_length );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    /* 2nd SPI transaction */
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_write( radio_local->spi, &dummy_byte, 1 );
    system_spi_read_with_dummy_byte( radio_local->spi, rbuffer, rbuffer_length, LR11XX_NOP );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_write( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                      const uint8_t* cdata, const uint16_t cdata_length )
{
    radio_t* radio_local = ( radio_t* ) radio;

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_write( radio_local->spi, cbuffer, cbuffer_length );
    system_spi_write( radio_local->spi, cdata, cdata_length );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* radio, uint8_t* data, const uint16_t data_length )
{
    radio_t* radio_local = ( radio_t* ) radio;

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_read_with_dummy_byte( radio_local->spi, data, data_length, LR11XX_NOP );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
