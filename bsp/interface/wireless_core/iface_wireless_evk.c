/** @file  iface_swc_evk.c
 *  @brief This file contains the implementation of functions configuring the
 *         wireless core which calls the functions of the BSP of the EVK1.4.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_wireless.h"
#include "evk.h"

/* CONSTANTS ******************************************************************/
#define FREE_RUNNING_QUARTER_MS_TIMER_IRQ_PRIORITY 15

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_swc_hal_init(swc_hal_t *hal)
{
    hal->radio_hal[0].set_shutdown_pin   = evk_radio_set_shutdown_pin;
    hal->radio_hal[0].reset_shutdown_pin = evk_radio_reset_shutdown_pin;
    hal->radio_hal[0].set_reset_pin      = evk_radio_set_reset_pin;
    hal->radio_hal[0].reset_reset_pin    = evk_radio_reset_reset_pin;
    hal->radio_hal[0].read_irq_pin       = evk_radio_read_irq_pin;
    hal->radio_hal[0].set_cs             = evk_radio_spi_set_cs;
    hal->radio_hal[0].reset_cs           = evk_radio_spi_reset_cs;
    hal->radio_hal[0].delay_ms           = evk_timer_delay_ms;
    hal->radio_hal[0].get_tick           = evk_timer_free_running_ms_get_tick_count;
    hal->radio_hal[0].tick_frequency_hz  = 1000;

    hal->radio_hal[0].transfer_full_duplex_blocking     = evk_radio_spi_transfer_full_duplex_blocking;
    hal->radio_hal[0].transfer_full_duplex_non_blocking = evk_radio_spi_transfer_full_duplex_non_blocking;
    hal->radio_hal[0].is_spi_busy                       = evk_radio_is_spi_busy;
    hal->radio_hal[0].context_switch                    = evk_radio_context_switch;
    hal->radio_hal[0].disable_radio_irq                 = evk_radio_disable_irq_it;
    hal->radio_hal[0].enable_radio_irq                  = evk_radio_enable_irq_it;
    hal->radio_hal[0].disable_radio_dma_irq             = evk_radio_disable_dma_irq_it;
    hal->radio_hal[0].enable_radio_dma_irq              = evk_radio_enable_dma_irq_it;

    hal->context_switch = evk_radio_callback_context_switch;

    evk_timer_free_running_ms_init(FREE_RUNNING_QUARTER_MS_TIMER_IRQ_PRIORITY);
}

void iface_swc_handlers_init(void)
{
    evk_set_radio_irq_callback(swc_radio_irq_handler);
    evk_set_radio_dma_rx_callback(swc_radio_spi_receive_complete_handler);
    evk_set_pendsv_callback(swc_connection_callbacks_processing_handler);
}
