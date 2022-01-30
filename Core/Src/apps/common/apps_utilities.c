/*!
 * @file      apps_utilities.c
 *
 * @brief     Common Application Helper function implementations
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "apps_utilities.h"
#include "lr1110_modem_lorawan.h"
#include "usb_device.h"
//#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
/*!
 * @brief Array for data logging.
 */
char data[200];

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
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void print_hex_buffer( const uint8_t* buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            sprintf( data,"\r\n" );
            while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
            HAL_Delay(5);
            newline = 0;
        }

        sprintf( data,"%02X ", buffer[i] );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    sprintf( data,"\r\n" );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
}

void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key, uint32_t pin,
                         const bool use_semtech_join_server )
{
    sprintf( data,"[INFO]:DevEui      : %02X", dev_eui[0] );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
    for( int i = 1; i < 8; i++ )
    {
        sprintf( data,"-%02X", dev_eui[i] );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    sprintf( data,"\r\n" );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
    sprintf( data,"[INFO]:AppEui      : %02X", join_eui[0] );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
    for( int i = 1; i < 8; i++ )
    {
        sprintf( data,"-%02X", join_eui[i] );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    sprintf( data,"\r\n" );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
    if( use_semtech_join_server )
    {
        sprintf( data,"[INFO]:AppKey      : Semtech join server used\r\n" );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    else
    {
        sprintf( data,"[INFO]:AppKey      : %02X", app_key[0] );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
        for( int i = 1; i < 16; i++ )
        {
            sprintf( data,"-%02X", app_key[i] );
            while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
            HAL_Delay(5);
        }
        sprintf( data,"\r\n" );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }

    sprintf( data,"[INFO]:Pin         : %08X\r\n\r\n", pin );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
}

void modem_status_to_string( lr1110_modem_status_t modem_status )
{
    sprintf( data,"[INFO]:Modem status : " );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);

    if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
    {
        sprintf( data,"CRASH " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    if( ( modem_status & LR1110_LORAWAN_MUTE ) == LR1110_LORAWAN_MUTE )
    {
        sprintf( data,"MUTE " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    if( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED )
    {
        sprintf( data,"JOINED " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    if( ( modem_status & LR1110_LORAWAN_SUSPEND ) == LR1110_LORAWAN_SUSPEND )
    {
        sprintf( data,"SUSPEND " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    if( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD )
    {
        sprintf( data,"UPLOAD " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
    {
        sprintf( data,"JOINING " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }
    if( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM )
    {
        sprintf( data,"STREAM " );
        while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
        HAL_Delay(5);
    }

    sprintf( data,"\r\n\r\n " );
    while(CDC_Transmit_FS(&data, strlen(data)) == USBD_BUSY)
    HAL_Delay(5);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
