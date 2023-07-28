/** @file wps_mac.h
 *  @brief Wireless protocol stack layer 2.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef MAC_LAYER_H_
#define MAC_LAYER_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "link_channel_hopping.h"
#include "link_distributed_desync.h"
#include "link_fallback.h"
#include "link_gain_loop.h"
#include "link_lqi.h"
#include "link_phase.h"
#include "link_protocol.h"
#include "link_random_datarate_offset.h"
#include "link_scheduler.h"
#include "link_tdma_sync.h"
#include "wps_def.h"
#include "xlayer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief Wireless protocol stack MAC Layer input signal.
 */
typedef enum wps_mac_input_signal {
    MAC_SIGNAL_RX_FRAME = 0,           /*!< MAC layer frame receive notify signal */
    MAC_SIGNAL_RX_FRAME_MISS,          /*!< MAC layer frame miss notify signal */
    MAC_SIGNAL_TX_SENT_ACK,            /*!< MAC layer ack receive notify signal */
    MAC_SIGNAL_TX_SENT_NACK,           /*!< MAC layer ack miss notify signal */
    MAC_SIGNAL_TX_NOT_SENT,            /*!< MAC layer TX not sent signal */
    MAC_SIGNAL_TX,                     /*!< MAC layer TX end notify signal */
    MAC_SIGNAL_SCHEDULE,               /*!< MAC layer scheduler notify signal */
    MAC_SIGNAL_EMPTY,                  /*!< MAC layer empty signal */
    MAC_SIGNAL_PREPARE_INTERNAL_FRAME, /*!< MAC layer prepare internal frame */
    MAC_SIGNAL_NUM                     /*!< Not a signal, represents the number of possible signals */
} wps_mac_input_signal_t;

/** @brief Wireless protocol stack MAC Layer output signal.
 */
typedef enum wps_mac_output_signal {
    MAC_SIGNAL_WPS_EMPTY = 0,        /*!< MAC layer empty output signal */
    MAC_SIGNAL_WPS_FRAME_RX_SUCCESS, /*!< MAC layer frame receive output signal */
    MAC_SIGNAL_WPS_FRAME_RX_FAIL,    /*!< MAC layer frame miss output signal */
    MAC_SIGNAL_WPS_FRAME_RX_OVERRUN, /*!< MAC layer No more space available in RX queue */
    MAC_SIGNAL_WPS_TX_SUCCESS,       /*!< MAC layer successful transmission output signal */
    MAC_SIGNAL_WPS_TX_FAIL,          /*!< MAC layer unsuccessful transmission output signal */
    MAC_SIGNAL_WPS_TX_DROP,          /*!< MAC layer dropped frame output signal */
    MAC_SIGNAL_WPS_PREPARE_DONE,     /*!< MAC layer frame prepare done signal */
    MAC_SIGNAL_SYNCING,              /*!< MAC layer Enter syncing state output signal */
} wps_mac_output_signal_t;

/** @brief Wireless protocol stack MAC Layer protocols identifiers.
 */
typedef enum wps_mac_proto_id {
    MAC_PROTO_ID_TIMESLOT_SAW = 0,      /*!< MAC layer timeslot ID and SAW protocol identifier */
    MAC_PROTO_ID_CHANNEL_INDEX,         /*!< MAC layer channel index protocol identifier */
    MAC_PROTO_ID_RDO,                   /*!< MAC layer RDO protocol identifier */
    MAC_PROTO_ID_RANGING_RESPONDER,      /*!< MAC layer ranging phase provider ID protocol identifier */
    MAC_PROTO_ID_RANGING_INITIATOR,   /*!< MAC layer ranging phase protocol identifier */
    MAC_PROTO_ID_CONNECTION_ID,         /*!< MAC layer connection ID protocol identifier */
} wps_mac_proto_id_t;

/** @brief Wireless protocol stack MAC Layer state function pointer.
 */
typedef void (*wps_mac_state_t)(void *signal_data);

/** @brief Wireless protocol stack MAC Layer output signal parameter.
 */
typedef struct wps_mac_output_signal_info {
    wps_mac_output_signal_t main_signal; /*!< Main output signal */
    wps_mac_output_signal_t auto_signal; /*!< Pending output signal */
} wps_mac_output_signal_info_t;

/** @brief Wireless protocol stack Layer 2 input signal parameters.
 */
typedef struct wps_mac_input_signal_info {
    wps_mac_input_signal_t main_signal; /*!< MAC Layer input signal */
    wps_mac_input_signal_t auto_signal; /*!< MAC Layer input signal */
} wps_mac_input_signal_info_t;

/** @brief Wireless protocol stack MAC Layer sync module init field.
 */
typedef struct wps_mac_sync_cfg {
    sleep_lvl_t sleep_level;  /*!< Desired sleep level for Sync */
    uint32_t preamble_len;    /*!< Frame preamble length */
    uint32_t syncword_len;    /*!< Frame syncword length */
    isi_mitig_t isi_mitig;    /*!< ISI mitigation level */
    uint8_t isi_mitig_pauses; /*!< ISI mitigation level corresponding pauses */
    bool tx_jitter_enabled;   /*!< TX jitter enable flag */
} wps_mac_sync_cfg_t;

/** @brief Wireless protocol stack MAC Layer main structure.
 */
typedef struct wps_mac_struct {
    wps_mac_input_signal_info_t input_signal;   /*!< Input signal instance */
    wps_mac_output_signal_info_t output_signal; /*!< Output signal instance */

    wps_mac_state_t *state_machine[MAC_SIGNAL_NUM]; /*!< State machine jump tables */
    uint8_t state_process_idx;                      /*!< State machine process index for jump table */

    timeslot_t *current_timeslot;         /*!< Current scheduler timeslot */
    bool current_ts_auto_reply;           /*!< True if current timeslot is auto reply */
    bool current_ts_auto_reply_tx;        /*!< True if current timeslot is auto reply and TX */
    scheduler_t scheduler;                /*!< Schedule instance */
    channel_hopping_t channel_hopping;    /*!< Channel hopping instance */
    uint8_t current_channel_index;        /*!< Current channel hopping index */
    bool random_channel_sequence_enabled; /*!< Random channel sequence enable flag */
    uint8_t network_id;                   /*!< Concurrent network ID */
    bool fast_sync_enabled;               /*!< Fast sync enable flag */
    bool delay_in_last_timeslot;          /*!< Delay was applied in last timeslot */
    uint16_t last_timeslot_delay;         /*!< Delay, in radio clock cycle, of the last timeslot. */

    uint16_t local_address;   /*!< Node address to handle RX/TX timeslot */
    uint16_t syncing_address; /*!< Syncing address */

    tdma_sync_t tdma_sync; /*!< Synchronization module instance */

    wps_role_t node_role; /*!< Current node role (Coordinator/Node) */

    xlayer_t empty_frame_tx; /*!< Xlayer instance when application TX queue is empty */
    xlayer_t empty_frame_rx; /*!< Xlayer instance when application RX queue is empty */

    wps_mac_input_signal_t current_input;   /*!< Currently processed input signal */
    wps_mac_output_signal_t current_output; /*!< WPS MAC output signal */
    xlayer_t *current_xlayer;               /*!< Currently processed xlayer */
    xlayer_t *main_xlayer;                  /*!< MAC Layer main xlayer node */
    xlayer_t *auto_xlayer;                  /*!< MAC Layer auto xlayer node */
    xlayer_cfg_internal_t config;           /*!< Configuration */

    circular_queue_t *callback_queue; /*!< Callback queue for stop and wait */
    link_rdo_t link_rdo;              /*!< Random Datarate Offset (RDO) instance. */
    link_distributed_desync_t link_distributed_desync; /*!< Distributed desync instance. */
    xlayer_queue_node_t *rx_node;                      /*!< RX node */
    uint8_t main_connection_id;                        /*!< Main connection ID */
    uint8_t auto_connection_id;                        /*!< Auto_reply connection ID */
    wps_connection_t *current_main_connection;         /*!< Current main connection */
    wps_connection_t *current_auto_connection;         /*!< Current auto reply connection */

    /* phases */
    wps_phase_info_t phase_data;      /*!< Phase data */
} wps_mac_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief MAC Layer initialization function.
 *
 *  @param[out] wps_mac                          MAC layer instance.
 *  @param[in]  callback_queue                   Pointer to the callback queue instance.
 *  @param[in]  channel_sequence                 Channel sequence.
 *  @param[in]  sync_cfg                         Synchronization module configuration.
 *  @param[in]  local_address                    Node local address.
 *  @param[in]  node_role                        Node role.
 *  @param[in]  random_channel_sequence_enabled  Random channel sequence enabled.
 *  @param[in]  network_id                       Network ID.
 *  @param[in]  frame_lost_max_duration          Maximum frame lost duration before link is
 *                                               considered unsynced.
 */
void wps_mac_init(wps_mac_t *wps_mac, circular_queue_t *callback_queue,
                  channel_sequence_t *channel_sequence, wps_mac_sync_cfg_t *sync_cfg,
                  uint16_t local_address, wps_role_t node_role,
                  bool random_channel_sequence_enabled, uint8_t network_id,
                  uint32_t frame_lost_max_duration);

/** @brief Reset the MAC Layer object.
 *
 *  @param wps_mac  MAC Layer instance.
 */
void wps_mac_reset(wps_mac_t *wps_mac);

/** @brief MAC state machine process.
 *
 *  WPS should initialize the input signal inside the MAC layer instance
 *  in order to provide an input signal to the MAC process. Output will be
 *  available through the output_signal member inside the wps_mac object.
 *
 *  @param[in]  wps_mac         Already initialized MAC Layer.
 */
void wps_mac_process(wps_mac_t *wps_mac);

/** @brief Enable fast sync.
 *
 *  @param wps_mac  MAC Layer instance.
 */
void wps_mac_enable_fast_sync(wps_mac_t *wps_mac);

/** @brief Disable fast sync.
 *
 *  @param wps_mac  MAC Layer instance.
 */
void wps_mac_disable_fast_sync(wps_mac_t *wps_mac);

/** @brief Interface to write the channel index to the header buffer.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @param[in] index   Channel index buffer pointer.
 */
void wps_mac_send_channel_index(void *wps_mac, uint8_t *index);

/** @brief Interface to read the channel index to the header buffer.
 *
 *  @param[in]  wps_mac MAC Layer instance.
 *  @param[out] index   Channel index buffer pointer.
 */
void wps_mac_receive_channel_index(void *wps_mac, uint8_t *index);

/** @brief Get the size of the channel index header field.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @return Header field size.
 */
uint8_t wps_mac_get_channel_index_proto_size(void *wps_mac);

/** @brief Interface to write the timeslot id and stop and wait to the header buffer.
 *
 *  @param[in] wps_mac         MAC Layer instance.
 *  @param[in] timeslot_id_saw Timeslot id and stop and wait buffer pointer.
 */
void wps_mac_send_timeslot_id_saw(void *wps_mac, uint8_t *timeslot_id_saw);

/** @brief Interface to read the timeslot id and stop and wait to the header buffer.
 *
 *  @param[in]  wps_mac         MAC Layer instance.
 *  @param[out] timeslot_id_saw Timeslot id and stop and wait buffer pointer.
 */
void wps_mac_receive_timeslot_id_saw(void *wps_mac, uint8_t *timeslot_id_saw);

/** @brief Get the size of the timeslot id and stop and wait header field.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @return Header field size.
 */
uint8_t wps_mac_get_timeslot_id_saw_proto_size(void *wps_mac);

/** @brief Interface to write the random datarate offset to the header buffer.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @param[in] rdo     Random datarate offset buffer pointer.
 */
void wps_mac_send_rdo(void *wps_mac, uint8_t *rdo);

/** @brief Interface to read the random datarate offset to the header buffer.
 *
 *  @param[in]  wps_mac MAC Layer instance.
 *  @param[out] rdo     Random datarate offset buffer pointer.
 */
void wps_mac_receive_rdo(void *wps_mac, uint8_t *rdo);

/** @brief Get the size of the random datarate offset header field.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @return Header field size.
 */
uint8_t wps_mac_get_rdo_proto_size(void *wps_mac);

/** @brief Interface to write the ranging phases to the header buffer.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @param[in] phases  Ranging phases buffer pointer.
 */
void wps_mac_send_ranging_phases(void *wps_mac, uint8_t *phases);

/** @brief Interface to read the ranging phases to the header buffer.
 *
 *  @param[in]  wps_mac MAC Layer instance.
 *  @param[out] phases  Ranging phases buffer pointer.
 */
void wps_mac_receive_ranging_phases(void *wps_mac, uint8_t *phases);

/** @brief Get the size of the ranging phases header field.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @return Header field size.
 */
uint8_t wps_mac_get_ranging_phases_proto_size(void *wps_mac);

/** @brief Interface to write the ranging phase count to the header buffer.
 *
 *  @param[in] wps_mac      MAC Layer instance.
 *  @param[in] phase_count  Ranging phases count buffer pointer.
 */
void wps_mac_send_ranging_phase_count(void *wps_mac, uint8_t *phase_count);

/** @brief Interface to read the ranging phase count to the header buffer.
 *
 *  @param[in]  wps_mac      MAC Layer instance.
 *  @param[out] phase_count  Ranging phases count buffer pointer.
 */
void wps_mac_receive_ranging_phase_count(void *wps_mac, uint8_t *phase_count);

/** @brief Get the size of the ranging phase count header field.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @return Header field size.
 */
uint8_t wps_mac_get_ranging_phase_count_proto_size(void *wps_mac);

/** @brief Interface to write the connection ID to the header buffer.
 *
 *  @param[in] wps_mac        MAC Layer instance.
 *  @param[in] connection_id  Connection ID buffer pointer.
 */
void wps_mac_send_connection_id(void *wps_mac, uint8_t *connection_id);

/** @brief Interface to read the connection ID to the header buffer.
 *
 *  @param[in]  wps_mac        MAC Layer instance.
 *  @param[out] connection_id  Connection ID buffer pointer.
 */
void wps_mac_receive_connection_id(void *wps_mac, uint8_t *connection_id);

/** @brief Get the size of the connection ID header field.
 *
 *  @param[in] wps_mac MAC Layer instance.
 *  @return Header field size.
 */
uint8_t wps_mac_get_connection_id_proto_size(void *wps_mac);

#ifdef __cplusplus
}
#endif

#endif /* MAC_LAYER_H_ */
