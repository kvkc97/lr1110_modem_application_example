/*!
 * @ingroup   wifi_code
 * @file      main_wifi.c
 *
 * @brief     Wi-Fi Example implementation
 *
 * @copyright
 * @parblock
 * Revised BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @endparblock
 */

/*!
 * @addtogroup wifi_code
 * LR1110 Simple Wi-Fi test application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_wifi.h"

#include "wifi_scan.h"
#include "lr1110_modem_board.h"

#include "configuration.h"
#include "mcu.h"
#include "usb_device.h"

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

/*!
 * @brief Radio hardware and global parameters
 */
extern radio_t lr1110;

/*!
 * @brief Array for data logging.
 */
char data[200];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point to WiFi scan.
 */
int main_wifi( void )
{
    lr1110_modem_event_callback_t  lr1110_modem_event_callback = { NULL };
    lr1110_modem_version_t         modem;
    wifi_settings_t                wifi_settings;
    static wifi_scan_all_results_t capture_result;

    /* Init board */
    //hal_mcu_init( );
    //hal_mcu_init_periph( );

    /* Board is initialized */
    //leds_blink( LED_ALL_MASK, 100, 2, true );

    /* Init LR1110 modem-e event */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.wifi_scan_done = lr1110_modem_wifi_scan_done;
    lr1110_modem_event_callback.reset          = lr1110_modem_reset_event;
    lr1110_modem_board_init( &lr1110, &lr1110_modem_event_callback );

    sprintf( data,"\r\n\r\n" );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
    sprintf( data,"[INFO]:###### ===== LoRa Basics Modem-E Wi-Fi demo application ==== ######\r\n\r\n" );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);

    lr1110_modem_get_version( &lr1110, &modem );

    /* Wi-Fi Parameters */
    wifi_settings.enabled       = true;
    wifi_settings.channels      = 0x3FFF;  // by default enable all channels
    wifi_settings.types         = WIFI_TYPE_SCAN;
    wifi_settings.scan_mode     = WIFI_SCAN_MODE;
    wifi_settings.nbr_retrials  = WIFI_NBR_RETRIALS;
    wifi_settings.max_results   = WIFI_MAX_RESULTS;
    wifi_settings.timeout       = WIFI_TIMEOUT_IN_MS;
    wifi_settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;

    while( 1 )
    {
        sprintf( data, "[INFO]:###### ===== Wi-FI SCAN ==== ######\r\n\r\n" );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);

        if( wifi_execute_scan( &lr1110, &wifi_settings, &capture_result ) == WIFI_SCAN_SUCCESS )
        {
            lr1110_modem_display_wifi_scan_results( &capture_result );
        }
        else
        {
            sprintf( data, "[INFO]:Wi-Fi Scan error\n\r" );
            while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
            HAL_Delay(5);
        }

        HAL_Delay( WIFI_SCAN_PERIOD_MS );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    sprintf( data, "[INFO]:###### ===== LR1110 MODEM-E RESET %lu ==== ######\r\n\r\n", reset_count );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);

    if( lr1110_modem_board_is_ready( ) == true )
    {
        /* System reset */
        hal_mcu_reset( );
    }
    else
    {
        lr1110_modem_board_set_ready( true );
    }
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
