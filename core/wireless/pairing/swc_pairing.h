/** @file  swc_pairing.h
 *  @brief The pairing module is used exchange data between two devices and make them connect with each other.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_PAIRING_H_
#define SWC_PAIRING_H_

/* INCLUDES *******************************************************************/
#include "swc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#ifndef SWC_PAIRING_DEVICE_LIST_MAX_COUNT
/*! Default device list array size. */
#define SWC_PAIRING_DEVICE_LIST_MAX_COUNT 10
#endif

/* Output power configuration. */
/*! Pulse count of transmitted data frames. */
#define PAIRING_TX_DATA_PULSE_COUNT 2
/*! Pulse width of transmitted data frames. */
#define PAIRING_TX_DATA_PULSE_WIDTH 7
/*! Pulse gain of transmitted data frames. */
#define PAIRING_TX_DATA_PULSE_GAIN  0
/*! Pulse count of transmitted ACK frames. */
#define PAIRING_TX_ACK_PULSE_COUNT  2
/*! Pulse width of transmitted ACK frames. */
#define PAIRING_TX_ACK_PULSE_WIDTH  7
/*! Pulse gain of transmitted ACK frames. */
#define PAIRING_TX_ACK_PULSE_GAIN   0

/* Input power configuration. */
/*! Pulses count of received ACK frames. */
#define PAIRING_RX_ACK_PULSE_COUNT  2
/*! Pulses count of received data frames. */
#define PAIRING_RX_DATA_PULSE_COUNT 2

/* SWC queue size. */
/*! Queue size for the RX and TX fifo. */
#define PAIRING_DATA_QUEUE_SIZE 2

/* Miscellaneous configurations. */
/*! Pairing schedule modulation. */
#define PAIRING_SWC_MODULATION  SWC_MOD_2BITPPM
/*! Pairing schedule FEC level. */
#define PAIRING_SWC_FEC_LEVEL   SWC_FEC_2
/*! Pairing schedule sleep level. */
#define PAIRING_SWC_SLEEP_LEVEL SWC_SLEEP_IDLE

/* Schedule configuration. */
/** @brief Pairing schedule timing.
 */
#define PAIRING_SCHEDULE { \
    3000, 3000             \
}

/** @brief Pairing coordinator to node timeslots.
 */
#define COORD_TO_NODE_TIMESLOTS { \
    MAIN_TIMESLOT(0)              \
}

/** @brief Pairing node to coordinator timeslots.
 */
#define NODE_TO_COORD_TIMESLOTS { \
    MAIN_TIMESLOT(1)              \
}

/** @brief Pairing channels to meet FCC and ETSI regulations.
 */
#define PAIRING_CHANNEL_FREQ_FCC_ETSI { \
    171                                 \
}

/** @brief Pairing channels to meet ARIB regulations.
 */
#define PAIRING_CHANNEL_FREQ_ARIB { \
    207                             \
}

/** @brief Pairing channels sequence.
 */
#define PAIRING_CHANNEL_SEQUENCE { \
    0                              \
}

/* TYPES **********************************************************************/
/** @brief Paired device identification.
 */
typedef struct swc_paired_device {
    /*! Generated unique ID. */
    uint64_t unique_id;
    /*! Address of the node. */
    uint16_t node_address;
} swc_pairing_device_t;

/** @brief Pairing parameters.
 */
typedef struct swc_pairing {
    /*! List of paired devices. */
    swc_pairing_device_t paired_device[SWC_PAIRING_DEVICE_LIST_MAX_COUNT];
    /*! Ultra-wideband regulation used for the pairing process. */
    swc_uwb_regulation_t uwb_regulation;
    /*! Network role of the current device. */
    swc_role_t network_role;
    /*! Memory pool instance from which memory allocation is done. */
    uint8_t *memory_pool;
    /*! Memory pool size in bytes. */
    uint32_t memory_pool_size;
    /*! Coordinator's PAN ID. */
    uint16_t pan_id;
    /*! Coordinator's address. */
    uint8_t coordinator_address;
    /*! Node's assigned address. */
    uint8_t node_address;
    /*! Device's role. */
    uint8_t device_role;
} swc_pairing_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the pairing process.
 *
 *  @param[in] pairing  Pairing structure to exchange data between applications.
 *  @param[in] hal      Board specific functions.
 */
void swc_pairing_init_pairing_process(swc_pairing_t *pairing, swc_hal_t *hal);

/** @brief Function to be called by an application to run the pairing process.
 *
 *  @return True if the pairing was successful.
 *          False if the pairing is still running.
 */
bool swc_pairing_process(void);

/** @brief Deinitialize the pairing process and its Wireless Core instance.
 */
void swc_pairing_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SWC_PAIRING_H_ */
