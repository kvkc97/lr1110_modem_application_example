/*!
 * \file      configuration.h
 *
 * \brief     Configuration definition
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_hal.h"
#include "stm32wb55xx.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_spi.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define LR1110_NSS_PORT GPIOB
#define LR1110_NSS_PIN LL_GPIO_PIN_12
#define LR1110_RESET_PORT GPIOB
#define LR1110_RESET_PIN LL_GPIO_PIN_0
#define LR1110_IRQ_PORT GPIOA
#define LR1110_IRQ_PIN LL_GPIO_PIN_5
#define LR1110_BUSY_PORT GPIOB
#define LR1110_BUSY_PIN LL_GPIO_PIN_1


#define LR1110_LED_RED_D4_PORT GPIOB		//LED2
#define LR1110_LED_RED_D4_PIN LL_GPIO_PIN_2
#define LR1110_LED_GREEN_D5_PORT GPIOE		//LED1
#define LR1110_LED_GREEN_D5_PIN LL_GPIO_PIN_4
#define LR1110_LED_BLUE_D7_PORT GPIOC		//LED3
#define LR1110_LED_BLUE_D7_PIN LL_GPIO_PIN_11
#define LR1110_LED_AMBER_D6_PORT GPIOA		//LED_P
#define LR1110_LED_AMBER_D6_PIN LL_GPIO_PIN_10

#define LR1110_LNA_PORT GPIOB
#define LR1110_LNA_PIN LL_GPIO_PIN_5

/*
#define DISPLAY_NSS_PORT GPIOB
#define DISPLAY_NSS_PIN LL_GPIO_PIN_6
#define DISPLAY_DC_PORT GPIOC
#define DISPLAY_DC_PIN LL_GPIO_PIN_7

#define TOUCH_IRQ_PORT GPIOA
#define TOUCH_IRQ_PIN LL_GPIO_PIN_10

#define FLASH_NSS_PORT GPIOB
#define FLASH_NSS_PIN LL_GPIO_PIN_10
#define ACCELERATOR_IRQ_PORT GPIOA
#define ACCELERATOR_IRQ_PIN LL_GPIO_PIN_9

#define ANTENNA_SWITCH_CTRL_PORT ( GPIOC )
#define ANTENNA_SWITCH_CTRL_PIN ( LL_GPIO_PIN_8 )
#define ANTENNA_SWITCH_N_CTRL_PORT ( GPIOC )
#define ANTENNA_SWITCH_N_CTRL_PIN ( LL_GPIO_PIN_6 )

#define BUTTON_BLUE_PORT ( GPIOC )
#define BUTTON_BLUE_PIN ( LL_GPIO_PIN_13 )
*/

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief GPIO setup data structure
 */
typedef struct configuration
{
    GPIO_TypeDef* port;
    uint32_t      pin;
} gpio_t;

/*!
 * @brief GPIO IRQ data context
 */
typedef struct hal_gpio_irq_s
{
    gpio_t				irq;
    void*                context;
    void ( *callback )( void* context );
} hal_gpio_irq_t;

/*!
 * @brief Radio hardware and global parameters
 */
typedef struct
{
    SPI_TypeDef* 	spi;
    gpio_t       	nss;
    gpio_t       	reset;
    hal_gpio_irq_t  event;
    gpio_t       	busy;
    //    uint32_t       spi_id;

} radio_t;

#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
