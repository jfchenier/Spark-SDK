/** @file  swc_cfg_node.h
 *  @brief Application specific configuration constants for the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_CFG_NODE_H_
#define SWC_CFG_NODE_H_

/* CONSTANTS ******************************************************************/
#define NETWORK_ROLE        SWC_ROLE_NODE
#define PAN_ID              0xBCD
#define COORDINATOR_ADDRESS 0x01
#define NODE_ADDRESS        0x02
#define LOCAL_ADDRESS       NODE_ADDRESS
#define REMOTE_ADDRESS      COORDINATOR_ADDRESS

/* Output power configuration */
#define TX_DATA_PULSE_COUNT 2
#define TX_DATA_PULSE_WIDTH 5
#define TX_DATA_PULSE_GAIN  0
#define TX_ACK_PULSE_COUNT  3
#define TX_ACK_PULSE_WIDTH  6
#define TX_ACK_PULSE_GAIN   0

/* Input power configuration */
#define RX_ACK_PULSE_COUNT  3 /* Pulses configuration of received ACK frames */
#define RX_DATA_PULSE_COUNT 1 /* Pulses configuration of received data frames */

/* SWC queue size */
#define TX_DATA_QUEUE_SIZE 2
#define RX_DATA_QUEUE_SIZE 2

/* Misc configurations */
#define SWC_MODULATION  SWC_MOD_IOOK
#define SWC_FEC_LEVEL   SWC_FEC_2
#define SWC_SLEEP_LEVEL SWC_SLEEP_IDLE

/* Schedule configuration */
#define SCHEDULE { \
    250, 250, 250, 250, 250 \
}
#define TX_TIMESLOTS { \
    MAIN_TIMESLOT(4) \
}
#define RX_TIMESLOTS { \
    MAIN_TIMESLOT(0), MAIN_TIMESLOT(1), MAIN_TIMESLOT(2), MAIN_TIMESLOT(3) \
}

/* Channels */
#define CHANNEL_FREQ { \
    167, 174, 181, 188 \
}
#define CHANNEL_SEQUENCE { \
    0, 1, 2, 3 \
}


#endif /* SWC_CFG_NODE_H_ */
