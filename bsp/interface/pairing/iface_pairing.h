/** @file  iface_app_pairing.h
 *  @brief This file contains the prototype of functions used for the Wireless Core
 *         pairing module functions which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_APP_PAIRING_H_
#define IFACE_APP_PAIRING_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PUBLIC FUNCTIONS ***********************************************************/

/** @brief Initialize the pairing timer with the period time in milliseconds.
 *
 * @param[in] period_ms  The period time in milliseconds.
 */
void iface_pairing_timer_init(uint16_t period_ms);

/** @brief Set the pairing timer function callback.
 *
 *  @param[in] callback  Function callback
 */
void iface_pairing_timer_set_callback(void (*callback)(void));

/** @brief Start pairing timer.
 */
void iface_pairing_timer_start(void);

/** @brief Stop pairing timer.
 */
void iface_pairing_timer_stop(void);

/** @brief Blocking delay with a 1 ms resolution.
 *
 *  @param[in] ms_delay  milliseconds delay.
 */
void iface_delay_ms(uint32_t ms_delay);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_APP_PAIRING_H_ */
