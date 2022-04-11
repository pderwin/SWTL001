/*!
 * @file      lr1110_modem_hal.c
 *
 * @brief     HAL implementation for LR1110 radio chip with LoRa Basics Modem-E Firmware
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

#include "lr1110_modem_hal.h"
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

lr1110_modem_hal_status_t lr1110_modem_hal_write( const void* context, const uint8_t* command,
                                                  const uint16_t command_length, const uint8_t* data,
                                                  const uint16_t data_length )
{
    radio_t*                  radio_local = ( radio_t* ) context;
    uint8_t                   crc         = 0;
    lr1110_modem_hal_status_t rc;

    if( system_gpio_get_pin_state( radio_local->busy ) == SYSTEM_GPIO_PIN_STATE_HIGH )
    {
        system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
        system_spi_write( radio_local->spi, &crc, 1 );
        system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );
    }

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
    crc = lr1110_modem_compute_crc( crc, data, data_length );

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_write( radio_local->spi, command, command_length );
    system_spi_write( radio_local->spi, data, data_length );
    system_spi_write( radio_local->spi, &crc, 1 );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_HIGH );

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_read_with_dummy_byte( radio_local->spi, ( uint8_t* ) &rc, 1, 0x00 );
    system_spi_read_with_dummy_byte( radio_local->spi, ( uint8_t* ) &crc, 1, 0x00 );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR1110_MODEM_HAL_STATUS_OK;
}

lr1110_modem_hal_status_t lr1110_modem_hal_write_without_rc( const void* radio, const uint8_t* cbuffer,
                                                             const uint16_t cbuffer_length, const uint8_t* cdata,
                                                             const uint16_t cdata_length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    if( lr1110_modem_hal_wakeup( radio_local ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc    = 0;
        lr1110_modem_hal_status_t status = LR1110_MODEM_HAL_STATUS_OK;

        /* NSS low */
        system_gpio_set_pin_state( radio_local->nss, 0 );

        /* Send CMD */
        system_spi_write( radio_local->spi, cbuffer, cbuffer_length );

        /* Send Data */
        system_spi_write( radio_local->spi, cdata, cdata_length );

        /* Compute and send CRC */
        crc = lr1110_modem_compute_crc( 0xFF, cbuffer, cbuffer_length );
        crc = lr1110_modem_compute_crc( crc, cdata, cdata_length );

        system_spi_write( radio_local->spi, &crc, 1 );

        /* NSS high */
        system_gpio_set_pin_state( radio_local->nss, 1 );

        return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_bootloader_hal_write( const void* context, const uint8_t* command,
                                                       const uint16_t command_length, const uint8_t* data,
                                                       const uint16_t data_length )
{
    return LR1110_MODEM_HAL_STATUS_BAD_FRAME;
}

lr1110_modem_hal_status_t lr1110_modem_hal_read( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    radio_t*                  radio_local = ( radio_t* ) context;
    uint8_t                   crc         = 0;
    lr1110_modem_hal_status_t rc          = 0;

    if( system_gpio_get_pin_state( radio_local->busy ) == SYSTEM_GPIO_PIN_STATE_HIGH )
    {
        system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
        system_spi_write( radio_local->spi, &crc, 1 );
        system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );
    }

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_LOW );

    crc = lr1110_modem_compute_crc( 0xFF, command, command_length );

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_write( radio_local->spi, command, command_length );
    system_spi_write( radio_local->spi, &crc, 1 );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    system_gpio_wait_for_state( radio_local->busy, SYSTEM_GPIO_PIN_STATE_HIGH );

    crc = 0;

    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_LOW );
    system_spi_read_with_dummy_byte( radio_local->spi, ( uint8_t* ) &rc, 1, 0x00 );
    system_spi_read_with_dummy_byte( radio_local->spi, data, data_length, 0x00 );
    system_spi_read_with_dummy_byte( radio_local->spi, ( uint8_t* ) &crc, 1, 0x00 );
    system_gpio_set_pin_state( radio_local->nss, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR1110_MODEM_HAL_STATUS_OK;
}

lr1110_modem_hal_status_t lr1110_bootloader_hal_read( const void* context, const uint8_t* command,
                                                      const uint16_t command_length, uint8_t* data,
                                                      const uint16_t data_length )
{
    return LR1110_MODEM_HAL_STATUS_BAD_FRAME;
}

lr1110_modem_hal_status_t lr1110_modem_hal_write_read( const void* context, const uint8_t* command, uint8_t* data,
                                                       const uint16_t data_length )
{
    return LR1110_MODEM_HAL_STATUS_BAD_FRAME;
}

lr1110_modem_hal_status_t lr1110_modem_hal_reset( const void* context )
{
    radio_t* radio_local = ( radio_t* ) context;

    system_gpio_set_pin_state( radio_local->reset, SYSTEM_GPIO_PIN_STATE_LOW );
    system_time_wait_ms( 1 );
    system_gpio_set_pin_state( radio_local->reset, SYSTEM_GPIO_PIN_STATE_HIGH );

    return LR1110_MODEM_HAL_STATUS_OK;
}

void lr1110_modem_hal_enter_dfu( const void* context ) {}

lr1110_modem_hal_status_t lr1110_modem_hal_wakeup( const void* context ) { return LR1110_MODEM_HAL_STATUS_BAD_FRAME; }

lr1110_modem_hal_status_t lr1110_bootloader_hal_wakeup( const void* context )
{
    return LR1110_MODEM_HAL_STATUS_BAD_FRAME;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
