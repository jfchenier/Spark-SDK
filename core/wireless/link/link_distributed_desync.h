/** @file link_distributed_desync.h
 *  @brief Distributed de-syncronization algorithm.
 *
 *  This algorithm is used for link concurrency to drift the schedule in a slot
 *  where there is less CCA fails, thus optimizing the air time usage.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_DISTRIBUTED_DESYNC_H_
#define LINK_DISTRIBUTED_DESYNC_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define DISTRIBUTED_DESYNC_DISABLE  0
#define UNSYNC_TX_OFFSET_PLL_CYCLES 1024

/* TYPES **********************************************************************/
/** @brief TX offset module instance.
 */
typedef struct link_distributed_desync {
    /*< Current target offset value in PLL cycle. */
    uint16_t target_offset;
    /*< Maximum target offset to be applied on a timeslot in PLL cycle. */
    uint16_t max_timeslot_offset;
    /*< Wether the last transmission was successful. */
    bool is_tx_event_success;
    /*< Number of consecutive transmission failures. */
    uint8_t consecutive_tx_fail_count;
    /*< Maximum frame lost count before link is considered unsynced. */
    uint16_t frame_lost_max_count;
    /*< TX offset enable flag. */
    bool enabled;
} link_distributed_desync_t;

/* PUBLIC GLOBALS *************************************************************/
/** @brief Initialize the distributed desync module.
 *
 *  @note When the distributed desync module is disabled, every call to the
 *        link_distributed_desync_get_offset will return 0.
 *
 *  @param[in] instance               Distributed desync module instance.
 *  @param[in] max_timeslot_offset    Maximum offset to apply every timeslot in pll cycles.
 *                                    Set to DISTRIBUTED_DESYNC_DISABLE to disable the module.
 *  @param[in]  frame_lost_max_count  Maximum frame lost count before link is
 *                                    considered unsynced.
 */
static inline void link_distributed_desync_init(link_distributed_desync_t *instance,
                                                uint16_t max_timeslot_offset,
                                                uint16_t frame_lost_max_count)
{
    if (instance == NULL) {
        return;
    }

    instance->target_offset        = 0;
    instance->max_timeslot_offset  = max_timeslot_offset;
    instance->enabled              = (max_timeslot_offset != DISTRIBUTED_DESYNC_DISABLE);
    instance->is_tx_event_success  = false;
    instance->frame_lost_max_count = frame_lost_max_count;
}

/** @brief Update the distributed desync instance.
 *
 *  @param[in] instance             Distributed desync module instance.
 *  @param[in] cca_wait_time        Last CCA transmission delay in pll cycles.
 *  @param[in] is_tx_event_success  Wether the last transmission was successful.
 */
static inline void link_distributed_desync_update(link_distributed_desync_t *instance,
                                                  uint16_t cca_wait_time, bool is_tx_event_success)
{
    if ((instance == NULL) || !instance->enabled) {
        return;
    }

    instance->is_tx_event_success = is_tx_event_success;

    if (!is_tx_event_success) {
        instance->consecutive_tx_fail_count++;
    } else {
        instance->consecutive_tx_fail_count = 0;
        if (instance->target_offset == 0) {
            /* Update the target offset once the previous target was achieved. */
            instance->target_offset = cca_wait_time;
        }
    }
}

/** @brief Get the offset to apply to the current timeslot.
 *
 *  @param[in] instance  Distributed desync module instance.
 *  @return  Offset in pll cycles to be applied.
 */
static inline uint16_t link_distributed_desync_get_offset(link_distributed_desync_t *instance)
{
    uint16_t timeslot_offset;

    if ((instance == NULL) || !instance->enabled || !instance->is_tx_event_success) {
        return 0;
    }

    if ((instance->consecutive_tx_fail_count > 0) &&
        ((instance->consecutive_tx_fail_count % instance->frame_lost_max_count) == 0)) {
        /* Apply a bigger offset when unable to transmit to try to find a free air time slot. */
        instance->target_offset = 0;
        return UNSYNC_TX_OFFSET_PLL_CYCLES;
    }

    if (instance->target_offset > instance->max_timeslot_offset) {
        /* Use the maximum offset. */
        timeslot_offset = instance->max_timeslot_offset;
    } else {
        /* Use the remaining offset. */
        timeslot_offset = instance->target_offset;
    }
    /* Update the target_offset. */
    instance->target_offset -= timeslot_offset;

    return timeslot_offset;
}

#ifdef __cplusplus
}
#endif
#endif /* LINK_DISTRIBUTED_DESYNC_H_ */
