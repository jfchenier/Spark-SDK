/** @file  iface_pairing_evk.c
 *  @brief This file contains the implementation of functions used for the Wireless Core
 *         pairing module which calls the functions of the BSP of the EVK1.4.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_pairing.h"
#include "evk.h"

/* CONSTANTS ******************************************************************/
#define PAIRING_TIMER_IRQ_PRIORITY 15

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_pairing_timer_init(uint16_t period_ms)
{
    evk_timer_cfg_t timer_cfg = {
        .timer_selection = EVK_TIMER_SELECTION_TIMER20,
        .time_base = EVK_TIMER_TIME_BASE_MILLISECOND,
        .time_period = period_ms,
        .irq_priority = PAIRING_TIMER_IRQ_PRIORITY,
    };
    evk_timer_init(timer_cfg);
}

void iface_pairing_timer_set_callback(void (*callback)(void))
{
    evk_it_set_timer20_callback((irq_callback)callback);
}

void iface_pairing_timer_start(void)
{
    evk_timer_start(EVK_TIMER_SELECTION_TIMER20);
}

void iface_pairing_timer_stop(void)
{
    evk_timer_stop(EVK_TIMER_SELECTION_TIMER20);
}

void iface_delay_ms(uint32_t ms_delay)
{
    evk_timer_delay_ms(ms_delay);
}
