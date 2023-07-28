/** @file wps_mac.c
 *  @brief Wireless protocol stack MAC.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps_mac.h"
#include "wps_callback.h"
#ifdef SPARK_WPS_CFG_FILE_EXISTS
#include "spark_wps_cfg.h"
#endif

/* CONSTANTS ******************************************************************/
#define RADIO_MAX_PACKET_SIZE          255
#define SYNC_PLL_STARTUP_CYCLES        ((uint32_t)0x60)
#define SYNC_RX_SETUP_PLL_CYCLES       ((uint32_t)147)
#define HEADER_BYTE0_SEQ_NUM_MASK      BIT(7)
#define HEADER_BYTE0_TIME_SLOT_ID_MASK BITS8(6, 0)
#define MULTI_RADIO_BASE_IDX           0

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void state_link_quality(void *signal_data);
static void state_sync(void *signal_data);
static void state_post_rx(void *signal_data);
static void state_post_tx(void *signal_data);
static void state_mac_prepare_frame(void *signal_data);
static void state_setup_primary_link(void *signal_data);
static void state_setup_auto_reply_link(void *signal_data);
static void state_scheduler(void *signal_data);
static void end(void *signal_data);

static bool outcome_is_frame_received(wps_mac_t *wps_mac);
static bool outcome_is_tx_sent_ack(wps_mac_t *wps_mac);
static bool outcome_is_tx_not_sent(wps_mac_t *wps_mac);
static bool is_network_node(wps_mac_t *wps_mac);
static bool is_current_timeslot_tx(wps_mac_t *wps_mac);
static bool is_current_auto_reply_timeslot_tx(wps_mac_t *wps_mac);
static bool is_current_auto_reply_timeslot_rx(wps_mac_t *wps_mac);
static bool is_current_timeslot_auto_reply(wps_mac_t *wps_mac);
static bool is_saw_arq_enable(wps_connection_t *connection);
static void update_phases_data(wps_phase_info_t *phase_data, uint16_t rx_wait_time);
static bool is_phase_data_valid(wps_phase_info_t *phase_data);
static bool no_payload_received(xlayer_t *current_queue);
static bool scheduling_needed(wps_mac_t *wps_mac);
static void update_xlayer_sync(wps_mac_t *wps_mac, xlayer_cfg_internal_t *xlayer_cfg);
static void update_main_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer);
static void update_auto_reply_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer);
static void update_xlayer_modem_feat(wps_mac_t *wps_mac,  xlayer_cfg_internal_t *xlayer_cfg);
static void extract_header(wps_mac_t *wps_mac, xlayer_t *current_queue);
static void fill_header(wps_mac_t *wps_mac, xlayer_t *current_queue);
static void setup_auto_reply_timeslot_status(wps_mac_t *wps_mac);
static void process_scheduler(wps_mac_t *wps_mac);
static void process_rx_tx_outcome(wps_mac_t *wps_mac);
static void flush_timeout_frames_before_sending(wps_mac_t *wps_mac, wps_connection_t *connection);
static void flush_tx_frame(wps_mac_t *wps_mac, wps_connection_t *connection);
static xlayer_t *get_xlayer_for_tx(wps_mac_t *wps_mac, wps_connection_t *connection);
static xlayer_t *get_xlayer_for_rx(wps_mac_t *wps_mac, wps_connection_t *connection);
static bool send_done(wps_connection_t *connection);
static void handle_link_throttle(wps_mac_t *wps_mac, uint8_t *inc_count);
static inline wps_error_t get_status_error(link_connect_status_t *link_connect_status);
static uint8_t get_highest_priority_conn_index(wps_connection_t **connections, uint8_t connection_count);
static void find_received_timeslot_and_connection(wps_mac_t *wps_mac);
#ifndef WPS_DISABLE_LINK_STATS
static void update_wps_stats(wps_mac_t *MAC);
#endif /* WPS_DISABLE_LINK_STATS */
static void update_connect_status(wps_mac_t *wps_mac, bool synced, bool ack_enabled);

/* TYPES **********************************************************************/
wps_mac_state_t rx_frame_sm[]      = {state_sync, state_post_rx, state_link_quality, end};
wps_mac_state_t rx_frame_miss_sm[] = {state_sync, state_post_rx, state_link_quality, end};
wps_mac_state_t tx_sm[]            = {state_post_tx, end};
wps_mac_state_t schedule_sm[]      = {state_scheduler, state_setup_primary_link,
                                      state_setup_auto_reply_link, state_mac_prepare_frame, end};
wps_mac_state_t empty_sm[]         = {end};

/* PRIVATE GLOBALS ************************************************************/
uint8_t overrun_buffer[RADIO_MAX_PACKET_SIZE];

/* PUBLIC FUNCTIONS ***********************************************************/
void wps_mac_init(wps_mac_t *wps_mac, circular_queue_t *callback_queue,
                  channel_sequence_t *channel_sequence, wps_mac_sync_cfg_t *sync_cfg,
                  uint16_t local_address, wps_role_t node_role,
                  bool random_channel_sequence_enabled, uint8_t network_id,
                  uint32_t frame_lost_max_duration)
{
    wps_mac->state_machine[MAC_SIGNAL_RX_FRAME]      = rx_frame_sm;
    wps_mac->state_machine[MAC_SIGNAL_RX_FRAME_MISS] = rx_frame_miss_sm;
    wps_mac->state_machine[MAC_SIGNAL_TX_SENT_ACK]   = tx_sm;
    wps_mac->state_machine[MAC_SIGNAL_TX_SENT_NACK]  = tx_sm;
    wps_mac->state_machine[MAC_SIGNAL_TX_NOT_SENT]   = tx_sm;
    wps_mac->state_machine[MAC_SIGNAL_TX]            = tx_sm;
    wps_mac->state_machine[MAC_SIGNAL_SCHEDULE]      = schedule_sm;
    wps_mac->state_machine[MAC_SIGNAL_EMPTY]         = empty_sm;

    wps_mac->state_process_idx      = 0;
    wps_mac->current_ts_auto_reply       = false;
    wps_mac->current_ts_auto_reply_tx    = false;
    wps_mac->local_address          = local_address;
    wps_mac->node_role              = node_role;
    wps_mac->callback_queue         = callback_queue;
    wps_mac->delay_in_last_timeslot = false;
    wps_mac->last_timeslot_delay    = 0;

    wps_mac->random_channel_sequence_enabled = random_channel_sequence_enabled;
    wps_mac->network_id                      = network_id;

    /* Scheduler init */
    link_scheduler_init(&wps_mac->scheduler, wps_mac->local_address);
    link_scheduler_set_first_time_slot(&wps_mac->scheduler);
    link_scheduler_enable_tx(&wps_mac->scheduler);
    wps_mac->current_timeslot   = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
    wps_mac->main_connection_id = 0;
    wps_mac->auto_connection_id = 0;
    wps_mac->current_main_connection =
        link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                   wps_mac->main_connection_id);
    wps_mac->current_auto_connection =
        link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                   wps_mac->auto_connection_id);

    link_channel_hopping_init(&wps_mac->channel_hopping, channel_sequence, wps_mac->random_channel_sequence_enabled, wps_mac->network_id);

    /* Sync module init */
    link_tdma_sync_init(&wps_mac->tdma_sync, sync_cfg->sleep_level,
                        SYNC_RX_SETUP_PLL_CYCLES * (sync_cfg->isi_mitig_pauses + 1),
                        frame_lost_max_duration, sync_cfg->syncword_len, sync_cfg->preamble_len,
                        SYNC_PLL_STARTUP_CYCLES, sync_cfg->isi_mitig, sync_cfg->isi_mitig_pauses,
                        local_address, wps_mac->fast_sync_enabled, sync_cfg->tx_jitter_enabled);
}

void wps_mac_reset(wps_mac_t *wps_mac)
{
    /* Sync module reset */
    wps_mac->tdma_sync.frame_lost_duration = 0;
    wps_mac->tdma_sync.sync_slave_offset = 0;
    wps_mac->tdma_sync.slave_sync_state  = STATE_SYNCING;

    /* Internal state machine reset */
    wps_mac->current_input  = MAC_SIGNAL_EMPTY;
    wps_mac->current_output = MAC_SIGNAL_WPS_EMPTY;
}

void wps_mac_process(wps_mac_t *wps_mac)
{
    wps_mac->state_process_idx = 0;
    wps_mac->current_input     = wps_mac->input_signal.main_signal;
    wps_mac->current_output    = MAC_SIGNAL_WPS_EMPTY;

    if (scheduling_needed(wps_mac)) {
        process_scheduler(wps_mac);
    } else {
        process_rx_tx_outcome(wps_mac);
    }

#ifndef WPS_DISABLE_LINK_STATS
    update_wps_stats(wps_mac);
#endif /* WPS_DISABLE_LINK_STATS */
}

void wps_mac_enable_fast_sync(wps_mac_t *wps_mac)
{
    wps_mac->fast_sync_enabled = true;
}

void wps_mac_disable_fast_sync(wps_mac_t *wps_mac)
{
    wps_mac->fast_sync_enabled = false;
}

/* PRIVATE STATE FUNCTIONS ****************************************************/
/** @brief Link quality state.
 *
 * This state handle LQI and gain loop module update.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_link_quality(void *signal_data)
{
    wps_mac_t *wps_mac             = (wps_mac_t *)signal_data;
    gain_loop_t *current_gain_loop = NULL;
    lqi_t *current_lqi             = NULL;
    lqi_t *current_channel_lqi     = NULL;
    uint8_t gain_index;

    if (is_current_auto_reply_timeslot_rx(wps_mac)) {
        /* RX auto reply frame, update LQI and gain loop */
        current_lqi         = &wps_mac->current_auto_connection->lqi;
        current_channel_lqi = &wps_mac->current_auto_connection->channel_lqi[wps_mac->current_channel_index];
        current_gain_loop   = wps_mac->current_auto_connection->gain_loop[wps_mac->current_channel_index];
    } else {
        /* Timeslot is not auto reply OR is auto reply but TX */
        current_lqi         = &wps_mac->current_main_connection->lqi;
        current_channel_lqi = &wps_mac->current_main_connection->channel_lqi[wps_mac->current_channel_index];
        current_gain_loop   = wps_mac->current_main_connection->gain_loop[wps_mac->current_channel_index];
    }

    gain_index = link_gain_loop_get_gain_index(current_gain_loop);
#ifndef WPS_DISABLE_PHY_STATS
    /* Update LQI */
    link_lqi_update(current_lqi, gain_index, wps_mac->current_xlayer->frame.frame_outcome, wps_mac->config.rssi_raw,
                    wps_mac->config.rnsi_raw, wps_mac->config.phase_offset);
#ifdef WPS_ENABLE_PHY_STATS_PER_BANDS
    link_lqi_update(current_channel_lqi, gain_index, wps_mac->current_xlayer->frame.frame_outcome, wps_mac->config.rssi_raw,
                    wps_mac->config.rnsi_raw, wps_mac->config.phase_offset);
#else
    (void)current_channel_lqi;
    (void)gain_index;
#endif /* WPS_ENABLE_PHY_STATS_PER_BANDS */
#else
    (void)current_channel_lqi;
    (void)current_lqi;
    (void)gain_index;
#endif /* WPS_DISABLE_PHY_STATS */
}

/** @brief Sync state.
 *
 * This state handle sync module update when node role
 * is set to NETWORK_NODE.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_sync(void *signal_data)
{
    wps_mac_t *wps_mac = (wps_mac_t *)signal_data;

    if (wps_mac->output_signal.main_signal == MAC_SIGNAL_SYNCING) {
        wps_mac->config.rx_wait_time = 0;
    }

    if (is_network_node(wps_mac)) {
        if ((!wps_mac->current_ts_auto_reply) || (wps_mac->current_ts_auto_reply_tx)) {
            if (!link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync)) {
                link_tdma_sync_slave_find(&wps_mac->tdma_sync, wps_mac->current_xlayer->frame.frame_outcome,
                                          wps_mac->config.rx_wait_time, &wps_mac->current_main_connection->cca,
                                          wps_mac->config.rx_cca_retry_count);
            } else if (wps_mac->current_main_connection->source_address == wps_mac->syncing_address) {
                link_tdma_sync_slave_adjust(&wps_mac->tdma_sync, wps_mac->current_xlayer->frame.frame_outcome,
                                            wps_mac->config.rx_wait_time, &wps_mac->current_main_connection->cca,
                                            wps_mac->config.rx_cca_retry_count);
            }
        }
    }
}

/** @brief Update the connection status for the current main connection.
 *
 *  @param[in] wps_mac      WPS MAC instance.
 *  @param[in] synced       Device is synced.
 *  @param[in] ack_enabled  Acknowledge enabled.
 */
static void update_connect_status(wps_mac_t *wps_mac, bool synced, bool ack_enabled)
{
    if (link_update_connect_status(&wps_mac->current_main_connection->connect_status,
                                   wps_mac->current_xlayer->frame.frame_outcome, synced,
                                   ack_enabled)) {
        wps_mac->config.callback_main.callback = wps_mac->current_main_connection->evt_callback;
        wps_mac->config.callback_main.parg_callback =
            wps_mac->current_main_connection->evt_parg_callback;
        wps_callback_enqueue(wps_mac->callback_queue, &wps_mac->config.callback_main);

        connect_status_t status = wps_mac->current_main_connection->connect_status.status;

        wps_mac->current_main_connection->wps_event = (status == CONNECT_STATUS_CONNECTED) ?
                                                          WPS_EVENT_CONNECT :
                                                          WPS_EVENT_DISCONNECT;
    }
}

/** @brief Post RX state.
 *
 * This state handle header extraction and operation after
 * the reception of valid frame.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_post_rx(void *signal_data)
{
    wps_mac_t *wps_mac = (wps_mac_t *)signal_data;
    bool duplicate     = false;
    bool ack_enabled   = wps_mac->current_main_connection->ack_enable;
    bool synced        = is_network_node(wps_mac) ? link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync) : true;
    wps_connection_t *rx_connection;
    xlayer_callback_t *callback;

    if (!outcome_is_frame_received(wps_mac)) {
        update_connect_status(wps_mac, synced, ack_enabled);
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node        = NULL;
        wps_mac->current_output = MAC_SIGNAL_WPS_FRAME_RX_FAIL;
        return;
    }

    /* Extract Header, Current connection might be adjusted if timeslot ID don't match*/
    extract_header(wps_mac, wps_mac->current_xlayer);

    /* Update connection status for the current connection. */
    update_connect_status(wps_mac, synced, ack_enabled);

    /* Copy application specific info */
    wps_mac->current_xlayer->config.rssi_raw = wps_mac->config.rssi_raw;
    wps_mac->current_xlayer->config.rnsi_raw = wps_mac->config.rnsi_raw;

    duplicate = link_saw_arq_is_rx_frame_duplicate(
        &wps_mac->current_main_connection->stop_and_wait_arq);

    /* No payload received  or duclicate */
    if (no_payload_received(wps_mac->current_xlayer) || duplicate) {
        /* Frame received is internal to MAC */
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node        = NULL;
        wps_mac->current_output = MAC_SIGNAL_WPS_EMPTY;
        return;
    }

    if (is_current_auto_reply_timeslot_rx(wps_mac)) {
        rx_connection = wps_mac->current_auto_connection;
        callback      = &wps_mac->config.callback_auto;
    } else {
        rx_connection = wps_mac->current_main_connection;
        callback      = &wps_mac->config.callback_main;
        memcpy(&wps_mac->current_xlayer->config.phases_info, wps_mac->config.phases_info, sizeof(phase_info_t));
    }

#ifndef WPS_DISABLE_STATS_USED_TIMESLOTS
    lqi_t *current_wps_lqi         = &rx_connection->used_frame_lqi;
    gain_loop_t *current_gain_loop = rx_connection->gain_loop[wps_mac->current_channel_index];
    uint8_t gain_index             = link_gain_loop_get_gain_index(current_gain_loop);

    link_lqi_update(current_wps_lqi, gain_index, wps_mac->current_xlayer->frame.frame_outcome, wps_mac->config.rssi_raw,
                    wps_mac->config.rnsi_raw, wps_mac->config.phase_offset);
#endif /* WPS_DISABLE_STATS_USED_TIMESLOTS */

    /* Frame is receive but there's no place for it in connection queue */
    if (!xlayer_queue_get_free_space(&rx_connection->xlayer_queue)) {
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node        = NULL;
        callback->callback      = rx_connection->evt_callback;
        callback->parg_callback = rx_connection->evt_parg_callback;
        wps_mac->current_output = MAC_SIGNAL_WPS_FRAME_RX_OVERRUN;
        return;
    }

    /* Frame successfully received */
    wps_mac->current_output = MAC_SIGNAL_WPS_FRAME_RX_SUCCESS;
    callback->callback      = rx_connection->rx_success_callback;
    callback->parg_callback = rx_connection->rx_success_parg_callback;
}

/** @brief Post TX state.
 *
 * This state handle operation after
 * the transmission of a frame with or
 * without acknowledgment.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_post_tx(void *signal_data)
{
    wps_mac_t *wps_mac             = (wps_mac_t *)signal_data;
    frame_outcome_t xlayer_outcome = wps_mac->current_xlayer->frame.frame_outcome;
    uint32_t ack_rssi              = wps_mac->config.rssi_raw;
    uint32_t ack_rnsi              = wps_mac->config.rnsi_raw;
    uint8_t *ack_phase_offset      = wps_mac->config.phase_offset;
    gain_loop_t *current_gain_loop;
    lqi_t *current_lqi;
    lqi_t *current_channel_lqi;
    wps_connection_t *current_connection;
    uint8_t gain_index;
    bool ack_enabled = wps_mac->current_main_connection->ack_enable;
    bool synced = is_network_node(wps_mac) ? link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync) :
                                             true;

    update_connect_status(wps_mac, synced, ack_enabled);

    if (wps_mac->current_xlayer != &wps_mac->empty_frame_tx) {
        if (outcome_is_tx_sent_ack(wps_mac)) {
            current_connection                            = wps_mac->current_main_connection;
            wps_mac->current_output                       = MAC_SIGNAL_WPS_TX_SUCCESS;
            wps_mac->config.callback_main.callback      = wps_mac->current_main_connection->tx_success_callback;
            wps_mac->config.callback_main.parg_callback = wps_mac->current_main_connection->tx_success_parg_callback;
            current_gain_loop   = wps_mac->current_main_connection->gain_loop[wps_mac->current_channel_index];
            current_lqi         = &wps_mac->current_main_connection->lqi;
            current_channel_lqi = &wps_mac->current_main_connection->channel_lqi[wps_mac->current_channel_index];
            if (is_current_timeslot_tx(wps_mac)) {
                link_saw_arq_inc_seq_num(&wps_mac->current_main_connection->stop_and_wait_arq);
            }
            send_done(wps_mac->current_main_connection);
        } else {
            if (is_current_timeslot_tx(wps_mac)) {
                current_connection = wps_mac->current_main_connection;
                current_gain_loop =
                    wps_mac->current_main_connection->gain_loop[wps_mac->current_channel_index];
                current_lqi = &wps_mac->current_main_connection->lqi;
                current_channel_lqi =
                    &wps_mac->current_main_connection->channel_lqi[wps_mac->current_channel_index];
                if (!ack_enabled) {
                    wps_mac->current_output                       = MAC_SIGNAL_WPS_TX_SUCCESS;
                    wps_mac->config.callback_main.callback      = wps_mac->current_main_connection->tx_success_callback;
                    wps_mac->config.callback_main.parg_callback = wps_mac->current_main_connection->tx_success_parg_callback;
                    send_done(wps_mac->current_main_connection);
                } else {
                    wps_mac->current_output                       = MAC_SIGNAL_WPS_TX_FAIL;
                    wps_mac->config.callback_main.callback      = wps_mac->current_main_connection->tx_fail_callback;
                    wps_mac->config.callback_main.parg_callback = wps_mac->current_main_connection->tx_fail_parg_callback;
                    if (!is_saw_arq_enable(wps_mac->current_main_connection)) {
                        send_done(wps_mac->current_main_connection);
                    }
                }
            } else {
                /* SaW only for main connection */
                xlayer_outcome      = outcome_is_tx_not_sent(wps_mac) ? FRAME_WAIT : FRAME_SENT_ACK_LOST;
                current_connection  = wps_mac->current_auto_connection;
                current_gain_loop   = wps_mac->current_auto_connection->gain_loop[wps_mac->current_channel_index];
                current_lqi         = &wps_mac->current_auto_connection->lqi;
                current_channel_lqi = &wps_mac->current_auto_connection->channel_lqi[wps_mac->current_channel_index];
                if (outcome_is_tx_not_sent(wps_mac)) {
                    wps_mac->current_output                  = MAC_SIGNAL_WPS_TX_FAIL;
                    wps_mac->config.callback_auto.callback = wps_mac->current_auto_connection->tx_fail_callback;
                    wps_mac->config.callback_auto.parg_callback =
                        wps_mac->current_auto_connection->tx_fail_parg_callback;
                } else {
                    wps_mac->current_output                  = MAC_SIGNAL_WPS_TX_SUCCESS;
                    wps_mac->config.callback_auto.callback = wps_mac->current_auto_connection->tx_success_callback;
                    wps_mac->config.callback_auto.parg_callback =
                        wps_mac->current_auto_connection->tx_success_parg_callback;
                    send_done(wps_mac->current_auto_connection);
                }
            }
        }
        gain_index = link_gain_loop_get_gain_index(current_gain_loop);
#ifndef WPS_DISABLE_STATS_USED_TIMESLOTS
        link_lqi_update(&current_connection->used_frame_lqi, gain_index, xlayer_outcome, ack_rssi, ack_rnsi, ack_phase_offset);
#else
        (void)current_connection;
        (void)gain_index;
        (void)xlayer_outcome;
        (void)ack_rssi;
        (void)ack_rnsi;
        (void)ack_phase_offset;
#endif /* WPS_ENABLE_STATS_USED_TIMESLOTS */
    } else {
        wps_mac->current_output = MAC_SIGNAL_WPS_EMPTY;
        if (wps_mac->current_ts_auto_reply_tx) {
            xlayer_outcome      = FRAME_SENT_ACK_LOST;
            current_lqi         = &wps_mac->current_auto_connection->lqi;
            current_channel_lqi = &wps_mac->current_auto_connection->channel_lqi[wps_mac->current_channel_index];
            current_gain_loop   = wps_mac->current_auto_connection->gain_loop[wps_mac->current_channel_index];
        } else {
            current_lqi         = &wps_mac->current_main_connection->lqi;
            current_channel_lqi = &wps_mac->current_main_connection->channel_lqi[wps_mac->current_channel_index];
            if (outcome_is_tx_sent_ack(wps_mac)) {
                current_gain_loop = wps_mac->current_main_connection->gain_loop[wps_mac->current_channel_index];
            } else {
                current_gain_loop = wps_mac->current_main_connection->gain_loop[wps_mac->current_channel_index];
            }
        }
    }
    gain_index = link_gain_loop_get_gain_index(current_gain_loop);
#ifndef WPS_DISABLE_PHY_STATS
    link_lqi_update(current_lqi, gain_index, xlayer_outcome, ack_rssi, ack_rnsi, ack_phase_offset);
#ifdef WPS_ENABLE_PHY_STATS_PER_BANDS
    link_lqi_update(current_channel_lqi, gain_index, xlayer_outcome, ack_rssi, ack_rnsi, ack_phase_offset);
#else
    (void)current_channel_lqi;
    (void)gain_index;
    (void)xlayer_outcome;
    (void)ack_rssi;
    (void)ack_rnsi;
    (void)ack_phase_offset;
#endif /* WPS_ENABLE_PHY_STATS_PER_BANDS */
#else
    (void)current_channel_lqi;
    (void)current_lqi;
    (void)gain_index;
    (void)xlayer_outcome;
    (void)ack_rssi;
    (void)ack_rnsi;
    (void)ack_phase_offset;
#endif /* WPS_ENABLE_PHY_STATS */

    link_distributed_desync_update(&wps_mac->link_distributed_desync,
                                   wps_mac->config.cca_try_count * wps_mac->config.cca_retry_time,
                                   wps_mac->current_output == MAC_SIGNAL_WPS_TX_SUCCESS);
}

/** @brief Prepare frame state.
 *
 * This state handle preparation of MAC header
 * for frame transmition.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_mac_prepare_frame(void *signal_data)
{
    wps_mac_t *wps_mac = (wps_mac_t *)signal_data;

    /* TODO Add packet fragmentation module */
    if (is_current_timeslot_tx(wps_mac) || wps_mac->current_ts_auto_reply) {
        /* TX timeslot / TX timeslot auto reply */
        if (wps_mac->current_xlayer->frame.header_begin_it != NULL) {
            fill_header(wps_mac, wps_mac->current_xlayer);
        }
    }
}

/** @brief Prepare frame state.
 *
 * This state handle preparation of MAC header
 * for frame transmition.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_setup_primary_link(void *signal_data)
{
    wps_mac_t *wps_mac     = (wps_mac_t *)signal_data;
    uint32_t next_channel  = link_channel_hopping_get_channel(&wps_mac->channel_hopping);
    uint16_t rdo_value     = link_rdo_get_offset(&wps_mac->link_rdo);
    int32_t timeslot_delay = 0;

    link_rdo_update_offset(&wps_mac->link_rdo);

    if (is_current_timeslot_tx(wps_mac)) {
        if (!is_network_node(wps_mac)) {
            timeslot_delay += link_distributed_desync_get_offset(&wps_mac->link_distributed_desync);
        }
        for (uint8_t i = 0; i < wps_mac->current_timeslot->main_connection_count; i++) {
            if (is_saw_arq_enable(wps_mac->current_timeslot->connection_main[i])) {
                flush_timeout_frames_before_sending(wps_mac,
                                                    wps_mac->current_timeslot->connection_main[i]);
            }
            if (wps_mac->current_timeslot->connection_main[i]->tx_flush) {
                flush_tx_frame(wps_mac, wps_mac->current_timeslot->connection_main[i]);
            }
        }
        if (wps_mac->current_timeslot->main_connection_count > 1) {
            wps_mac->main_connection_id = get_highest_priority_conn_index(wps_mac->current_timeslot->connection_main,
                                                                          wps_mac->current_timeslot->main_connection_count);
            wps_mac->current_main_connection = link_scheduler_get_current_main_connection(&wps_mac->scheduler, wps_mac->main_connection_id);
        }
        wps_mac->main_xlayer = get_xlayer_for_tx(wps_mac, wps_mac->current_main_connection);
        wps_mac->auto_xlayer = NULL;

        if (wps_mac->main_xlayer == &wps_mac->empty_frame_tx && wps_mac->empty_frame_tx.frame.header_memory == NULL) {
            timeslot_delay += wps_mac->current_main_connection->empty_queue_max_delay;
        }

        if (wps_mac->delay_in_last_timeslot) {
            timeslot_delay -= wps_mac->last_timeslot_delay;
            wps_mac->delay_in_last_timeslot = false;
        }

        link_tdma_sync_update_tx(&wps_mac->tdma_sync,
                                 timeslot_delay +
                                     link_scheduler_get_sleep_time(&wps_mac->scheduler) + rdo_value,
                                 &wps_mac->current_main_connection->cca);

        if (wps_mac->main_xlayer == &wps_mac->empty_frame_tx && wps_mac->empty_frame_tx.frame.header_memory == NULL) {
            wps_mac->last_timeslot_delay = wps_mac->current_main_connection->empty_queue_max_delay;
            wps_mac->delay_in_last_timeslot = true;
        }

        /* Set current xlayer for the prepare frame */
        wps_mac->current_xlayer = wps_mac->main_xlayer;

        wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
        wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_EMPTY;

    } else {
        if (wps_mac->delay_in_last_timeslot) {
            timeslot_delay -= wps_mac->last_timeslot_delay;
            wps_mac->delay_in_last_timeslot = false;
        }

        link_tdma_sync_update_rx(&wps_mac->tdma_sync,
                                 timeslot_delay + link_scheduler_get_sleep_time(&wps_mac->scheduler) + rdo_value,
                                 &wps_mac->current_main_connection->cca);

        wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
        wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_EMPTY;
        wps_mac->main_xlayer               = get_xlayer_for_rx(wps_mac, wps_mac->current_main_connection);
        wps_mac->auto_xlayer               = NULL;

        if ((!link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync)) &&
            (wps_mac->node_role == NETWORK_NODE) &&
            wps_mac->current_main_connection->source_address == wps_mac->syncing_address) {
            if (wps_mac->fast_sync_enabled) {
                wps_mac->output_signal.main_signal = MAC_SIGNAL_SYNCING;

                next_channel = (wps_mac->channel_hopping.middle_channel_idx %
                                wps_mac->channel_hopping.channel_sequence->sequence_size);
            }
        }
    }

    if (wps_mac->current_main_connection->connect_status.status == CONNECT_STATUS_DISCONNECTED) {
        /* Consider link broken, so maximize gain to increase chances to
         * resync at high attenuation/high range
         */
        for (uint8_t i = 0; i < MAX_CHANNEL_COUNT; i++) {
            for (uint8_t j = 0; j < WPS_RADIO_COUNT; j++) {
                link_gain_loop_reset_gain_index(&wps_mac->current_main_connection->gain_loop[i][j]);
            }
        }
    }

    uint8_t payload_size   = wps_mac->main_xlayer->frame.payload_end_it - wps_mac->main_xlayer->frame.payload_begin_it;
    uint8_t fallback_index = link_fallback_get_index(&wps_mac->current_main_connection->link_fallback, payload_size);
    uint8_t cca_max_try_count;

    if (is_current_timeslot_tx(wps_mac)) {
        if ((wps_mac->current_main_connection->cca.fbk_try_count != NULL) &&
            (wps_mac->current_main_connection->link_fallback.threshold != NULL)) {
            cca_max_try_count = wps_mac->current_main_connection->cca.fbk_try_count[fallback_index];
        } else {
            cca_max_try_count = wps_mac->current_main_connection->cca.max_try_count;
        }
    } else {
        cca_max_try_count = wps_mac->current_main_connection->cca.max_try_count;
    }
    if (cca_max_try_count == 0) {
        wps_mac->config.cca_threshold = WPS_DISABLE_CCA_THRESHOLD;
    } else {
        wps_mac->config.cca_threshold = wps_mac->current_main_connection->cca.threshold;
    }

    wps_mac->config.channel =
        &wps_mac->current_main_connection->channel[fallback_index][next_channel][MULTI_RADIO_BASE_IDX];
    wps_mac->config.packet_cfg        = wps_mac->current_main_connection->packet_cfg;
    wps_mac->config.cca_retry_time    = wps_mac->current_main_connection->cca.retry_time_pll_cycles;
    wps_mac->config.cca_max_try_count = cca_max_try_count;
    wps_mac->config.cca_try_count     = 0;
    wps_mac->config.cca_fail_action   = wps_mac->current_main_connection->cca.fail_action;
    wps_mac->config.sleep_level       = wps_mac->tdma_sync.sleep_mode;
    wps_mac->config.gain_loop         = wps_mac->current_main_connection->gain_loop[wps_mac->current_channel_index];
    wps_mac->config.phases_info               = &wps_mac->phase_data.local_phases_info;
    wps_mac->config.isi_mitig                 = wps_mac->tdma_sync.isi_mitig;
    wps_mac->config.expect_ack                = wps_mac->current_main_connection->ack_enable;

    update_main_xlayer_link_parameter(wps_mac, wps_mac->main_xlayer);
    update_xlayer_sync(wps_mac, &wps_mac->config);
    update_xlayer_modem_feat(wps_mac, &wps_mac->config);
}

/** @brief Setup auto reply link state.
 *
 * This state handle preparation of xlayer for proper
 * auto reply timeslot frame reception/transmition.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_setup_auto_reply_link(void *signal_data)
{
    wps_mac_t *wps_mac    = (wps_mac_t *)signal_data;

    if (wps_mac->current_ts_auto_reply) {
        /* Prime status */
        if (wps_mac->current_ts_auto_reply_tx) {
            for (uint8_t i = 0; i < wps_mac->current_timeslot->auto_connection_count; i++) {
                if (is_saw_arq_enable(wps_mac->current_timeslot->connection_auto_reply[i])) {
                    flush_timeout_frames_before_sending(
                        wps_mac, wps_mac->current_timeslot->connection_auto_reply[i]);
                }
                if (wps_mac->current_timeslot->connection_auto_reply[i]->tx_flush) {
                    flush_tx_frame(wps_mac, wps_mac->current_timeslot->connection_auto_reply[i]);
                }
            }
            if (wps_mac->current_timeslot->auto_connection_count > 1) {
                wps_mac->auto_connection_id = get_highest_priority_conn_index(wps_mac->current_timeslot->connection_auto_reply,
                                                                              wps_mac->current_timeslot->auto_connection_count);
                wps_mac->current_auto_connection = link_scheduler_get_current_auto_connection(&wps_mac->scheduler, wps_mac->auto_connection_id);
            }
            /* TX Prime */
            /* TODO Add tx pulse/Channel cfg */
            wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
            wps_mac->auto_xlayer               = get_xlayer_for_tx(wps_mac, wps_mac->current_auto_connection);

            /* Set current xlayer for Prepare frame */
            wps_mac->current_xlayer = wps_mac->auto_xlayer;
        } else {
            /* RX Prime*/
            wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
            wps_mac->auto_xlayer               = get_xlayer_for_rx(wps_mac, wps_mac->current_auto_connection);
        }
        update_auto_reply_xlayer_link_parameter(wps_mac, wps_mac->auto_xlayer);
    }
}

/** @brief Scheduler state.
 *
 * This state get the next timeslot to
 * handle.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void state_scheduler(void *signal_data)
{
    wps_mac_t *wps_mac = (wps_mac_t *)signal_data;
    uint8_t inc_count;

    link_scheduler_reset_sleep_time(&wps_mac->scheduler);
    inc_count = link_scheduler_increment_time_slot(&wps_mac->scheduler);
    handle_link_throttle(wps_mac, &inc_count);
    link_channel_hopping_increment_sequence(&wps_mac->channel_hopping, inc_count);

    wps_mac->current_channel_index = link_channel_hopping_get_channel(&wps_mac->channel_hopping);
    wps_mac->current_timeslot      = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
    wps_mac->current_main_connection =
        link_scheduler_get_current_main_connection(&wps_mac->scheduler, 0);
    wps_mac->current_auto_connection =
        link_scheduler_get_current_auto_connection(&wps_mac->scheduler, 0);
    setup_auto_reply_timeslot_status(wps_mac);
}

/** @brief End state.
 *
 * This state is use in the jump tables to
 * know when jump table are ending.
 *
 *  @param[in] signal_data  MAC structure.
 */
static void end(void *signal_data)
{
    (void)signal_data;
}

void wps_mac_send_channel_index(void *wps_mac, uint8_t *index)
{
    wps_mac_t *mac = wps_mac;
    *index         = link_channel_hopping_get_seq_index(&mac->channel_hopping);
}

void wps_mac_receive_channel_index(void *wps_mac, uint8_t *index)
{
    wps_mac_t *mac = wps_mac;

    if (mac->node_role == NETWORK_NODE) {
        link_channel_hopping_set_seq_index(&mac->channel_hopping, *index);
    }
}

uint8_t wps_mac_get_channel_index_proto_size(void *wps_mac)
{
    wps_mac_t *mac = wps_mac;

    return sizeof(mac->channel_hopping.hop_seq_index);
}

void wps_mac_send_timeslot_id_saw(void *wps_mac, uint8_t *timeslot_id_saw)
{
    wps_mac_t *mac = wps_mac;
    uint16_t index = link_scheduler_get_next_timeslot_index(&mac->scheduler);

    *timeslot_id_saw = MASK2VAL(index, HEADER_BYTE0_TIME_SLOT_ID_MASK) |
                       MOV2MASK(link_saw_arq_get_seq_num(&mac->current_main_connection->stop_and_wait_arq),
                                HEADER_BYTE0_SEQ_NUM_MASK);
}

void wps_mac_receive_timeslot_id_saw(void *wps_mac, uint8_t *timeslot_id_saw)
{
    wps_mac_t *mac = wps_mac;
    uint8_t time_slot_id;

    if (is_network_node(mac)) {
        time_slot_id = MASK2VAL(*timeslot_id_saw, HEADER_BYTE0_TIME_SLOT_ID_MASK);
        if (time_slot_id < mac->scheduler.schedule.size) {
            if (link_scheduler_get_next_timeslot_index(&mac->scheduler) != time_slot_id) {
                link_scheduler_set_mismatch(&mac->scheduler);
            }
            link_scheduler_set_time_slot_i(&mac->scheduler, time_slot_id);
        }
    }

    /* Check if received frame is an auto sync frame */
    if (mac->current_xlayer->frame.header_begin_it + mac->current_xlayer->frame.header_memory_size !=
        mac->current_xlayer->frame.payload_end_it) {
        link_saw_arq_update_rx_seq_num(&mac->current_main_connection->stop_and_wait_arq,
                                       MASK2VAL(*timeslot_id_saw, HEADER_BYTE0_SEQ_NUM_MASK));
        if (link_saw_arq_is_rx_frame_duplicate(&mac->current_main_connection->stop_and_wait_arq)) {
            /* Frame is duplicate */
            mac->current_output = MAC_SIGNAL_WPS_EMPTY;
        }
    }
}

uint8_t wps_mac_get_timeslot_id_saw_proto_size(void *wps_mac)
{
    wps_mac_t *mac = wps_mac;

    return sizeof(mac->scheduler.current_time_slot_num);
}

void wps_mac_send_rdo(void *wps_mac, uint8_t *rdo)
{
    wps_mac_t *mac = wps_mac;

    link_rdo_send_offset(&mac->link_rdo, rdo);
}

void wps_mac_receive_rdo(void *wps_mac, uint8_t *rdo)
{
    wps_mac_t *mac = wps_mac;

    link_rdo_set_offset(&mac->link_rdo, rdo);
}

uint8_t wps_mac_get_rdo_proto_size(void *wps_mac)
{
    wps_mac_t *mac = wps_mac;

    return sizeof(mac->link_rdo.offset);
}

void wps_mac_send_ranging_phases(void *wps_mac, uint8_t *phases)
{
    wps_mac_t *mac = wps_mac;

    *phases = mac->phase_data.local_phases_count;
    phases++;
    *phases = mac->phase_data.local_phases_info.phase1;
    phases++;
    *phases = mac->phase_data.local_phases_info.phase2;
    phases++;
    *phases = mac->phase_data.local_phases_info.phase3;
    phases++;
    *phases = mac->phase_data.local_phases_info.phase4;
}

void wps_mac_receive_ranging_phases(void *wps_mac, uint8_t *phases)
{
    wps_mac_t *mac = wps_mac;
    link_phase_t *link_phase = &mac->current_auto_connection->link_phase;

    mac->phase_data.remote_phases_count = *phases;
    phases++;
    mac->phase_data.remote_phases_info.phase1 = *phases;
    phases++;
    mac->phase_data.remote_phases_info.phase2 = *phases;
    phases++;
    mac->phase_data.remote_phases_info.phase3 = *phases;
    phases++;
    mac->phase_data.remote_phases_info.phase4 = *phases;

    if (is_phase_data_valid(&mac->phase_data)) {
        if (link_phase_add_data(link_phase, mac->phase_data.last_local_phases_info, mac->phase_data.remote_phases_info)) {
            mac->config.callback_auto.callback = mac->current_auto_connection->ranging_data_ready_callback;
            mac->config.callback_auto.parg_callback = mac->current_auto_connection->ranging_data_ready_parg_callback;
            wps_callback_enqueue(mac->callback_queue, &mac->config.callback_auto);
        };
    }
    update_phases_data(&mac->phase_data, mac->config.rx_wait_time);

}

uint8_t wps_mac_get_ranging_phases_proto_size(void *wps_mac)
{
    wps_mac_t *mac = wps_mac;

    return sizeof(mac->phase_data.local_phases_count) + sizeof(mac->phase_data.local_phases_info.phase1) +
           sizeof(mac->phase_data.local_phases_info.phase2) + sizeof(mac->phase_data.local_phases_info.phase3) +
           sizeof(mac->phase_data.local_phases_info.phase4);
}

void wps_mac_send_ranging_phase_count(void *wps_mac, uint8_t *phase_count)
{
    wps_mac_t *mac = wps_mac;

    /* Transmit count */
    *phase_count = mac->phase_data.local_phases_count;
}

void wps_mac_receive_ranging_phase_count(void *wps_mac, uint8_t *phase_count)
{
    wps_mac_t *mac = wps_mac;

    mac->phase_data.local_phases_count = *phase_count;
}

uint8_t wps_mac_get_ranging_phase_count_proto_size(void *wps_mac)
{
    wps_mac_t *mac = wps_mac;

    return sizeof(mac->phase_data.local_phases_count);
}

void wps_mac_send_connection_id(void *wps_mac, uint8_t *connection_id)
{
    wps_mac_t *mac = wps_mac;

    if (mac->current_ts_auto_reply_tx) {
        *connection_id = mac->auto_connection_id;
    } else {
        *connection_id = mac->main_connection_id;
    }
}

void wps_mac_receive_connection_id(void *wps_mac, uint8_t *connection_id)
{
    wps_mac_t *mac = wps_mac;
    uint8_t connection_count;
    uint8_t *conn_id;

    if (mac->current_ts_auto_reply && !mac->current_ts_auto_reply_tx) {
        connection_count = mac->current_timeslot->auto_connection_count;
        conn_id = &mac->auto_connection_id;
    } else {
        connection_count = mac->current_timeslot->main_connection_count;
        conn_id = &mac->main_connection_id;
    }
    if (connection_count > 1) {
        if (*connection_id < connection_count) {
            *conn_id = *connection_id;
        } else {
            *conn_id = 0;
        }
    } else {
        *conn_id = 0;
    }
}

uint8_t wps_mac_get_connection_id_proto_size(void *wps_mac)
{
    wps_mac_t *mac = wps_mac;

    return sizeof(mac->main_connection_id);
}

/* PRIVATE FUNCTIONS *********************************************************/
/** @brief Output if input signal is RX_FRAME.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Input signal is RX_FRAME.
 *  @retval False  Input signal is not RX_FRAME.
 */
static bool outcome_is_frame_received(wps_mac_t *wps_mac)
{
    return (wps_mac->current_input == MAC_SIGNAL_RX_FRAME);
}

/** @brief Output if sent frame has been acknowledge.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Frame sent has been acknowledge.
 *  @retval False  Frame sent has not been acknowledge.
 */
static bool outcome_is_tx_sent_ack(wps_mac_t *wps_mac)
{
    return (wps_mac->current_input == MAC_SIGNAL_TX_SENT_ACK);
}

/** @brief Output if frame has not been sent
 *
 *  @param[in] wps_mac
 *  @return True   The frame is not sent.
 *  @return False  The frame is sent.
 */
static bool outcome_is_tx_not_sent(wps_mac_t *wps_mac)
{
    return (wps_mac->current_input == MAC_SIGNAL_TX_NOT_SENT);
}

/** @brief Output if node role is NETWORK_NODE.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Node role is NETWORK_NODE.
 *  @retval False  Node role is not NETWORK_NODE.
 */
static bool is_network_node(wps_mac_t *wps_mac)
{
    return (wps_mac->node_role == NETWORK_NODE);
}

/** @brief Output if current main connection timeslot is TX.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Main connection timeslot is TX.
 *  @retval False  Main connection timeslot is RX.
 */
static bool is_current_timeslot_tx(wps_mac_t *wps_mac)
{
    return (wps_mac->current_main_connection->source_address == wps_mac->local_address);
}

/** @brief Output if auto-reply connection timeslot is TX.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Auto-reply connection timeslot is TX.
 *  @retval False  Auto-reply connection timeslot is RX.
 */
static bool is_current_auto_reply_timeslot_tx(wps_mac_t *wps_mac)
{
    return (wps_mac->current_auto_connection->source_address == wps_mac->local_address);
}

/** @brief Output if current timeslot is auto reply RX.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval true   Current timeslot is not auto reply RX.
 *  @retval false  Current timeslot is auto reply RX.
 */
static bool is_current_auto_reply_timeslot_rx(wps_mac_t *wps_mac)
{
    return (wps_mac->current_ts_auto_reply && !wps_mac->current_ts_auto_reply_tx);
}

/** @brief Output if auto-reply connection exist in timeslot.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   There is an auto-reply connection.
 *  @retval False  There is not an auto-reply connection.
 */
static bool is_current_timeslot_auto_reply(wps_mac_t *wps_mac)
{
    return (wps_mac->current_auto_connection != NULL);
}

/** @brief Return if stop and wait is enable or not.
 *
 *  @param wps_mac  MAC structure.
 *  @retval True   Stop and wait ARQ is enable.
 *  @retval False  Stop and wait ARQ is disable.
 */
static bool is_saw_arq_enable(wps_connection_t *connection)
{
    return connection->stop_and_wait_arq.enable;
}

/** @brief Update phases data.
 *
 *  @param[in] phase_data   Phase data.
 *  @param[in] rx_wait_time Reception wait time.
 */
static void update_phases_data(wps_phase_info_t *phase_data, uint16_t rx_wait_time)
{
    phase_data->last_local_phases_info.phase1     = phase_data->local_phases_info.phase1;
    phase_data->last_local_phases_info.phase2     = phase_data->local_phases_info.phase2;
    phase_data->last_local_phases_info.phase3     = phase_data->local_phases_info.phase3;
    phase_data->last_local_phases_info.phase4     = phase_data->local_phases_info.phase4;
    phase_data->last_local_phases_info.rx_waited0 = rx_wait_time & 0x00ff;
    phase_data->last_local_phases_info.rx_waited1 = (rx_wait_time & 0x7f00) >> 8;
    phase_data->local_phases_count++;
}

/** @brief Return if current phase data are valid.
 *
 *  @param[in] phase_data   Phase data.
 *  @retval True   Current situationis valid.
 *  @retval False  Current is not valid.
 */
static bool is_phase_data_valid(wps_phase_info_t *phase_data)
{
    return (((uint8_t)(phase_data->remote_phases_count + 1)) == phase_data->local_phases_count);
}

/** @brief Update the xlayer sync module value for PHY.
 *
 *  @param[in]  wps_mac         MAC structure.
 *  @param[out] current_xlayer  Current xlayer node to update.
 */
static void update_xlayer_sync(wps_mac_t *wps_mac, xlayer_cfg_internal_t *xlayer_cfg)
{
    xlayer_cfg->power_up_delay = link_tdma_sync_get_pwr_up(&wps_mac->tdma_sync);
    xlayer_cfg->rx_timeout     = link_tdma_sync_get_timeout(&wps_mac->tdma_sync);
    xlayer_cfg->sleep_time     = link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync);
}

/** @brief Update the main connection xlayer gain loop value for PHY.
 *
 *  @param[in] wps_mac MAC structure.
 *  @param[in] xlayer  xlayer node to update.
 */
static void update_main_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer)
{
    xlayer->frame.destination_address = wps_mac->current_main_connection->destination_address;
    xlayer->frame.source_address      = wps_mac->current_main_connection->source_address;
}

/** @brief Update the main connection xlayer gain loop value for PHY.
 *
 *  @param[in] wps_mac MAC structure.
 *  @param[in] xlayer  xlayer node to update.
 */
static void update_auto_reply_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer)
{
    if (xlayer != NULL) {
        xlayer->frame.destination_address = wps_mac->current_auto_connection->destination_address;
        xlayer->frame.source_address      = wps_mac->current_auto_connection->source_address;
    }
}

/** @brief Update the main connection xlayer modem feat value for PHY.
 *
 *  @param[in] wps_mac MAC structure.
 *  @param[in] xlayer  xlayer node to update.
 */
static void update_xlayer_modem_feat(wps_mac_t *wps_mac,  xlayer_cfg_internal_t *xlayer_cfg)
{
    xlayer_cfg->fec        = wps_mac->current_main_connection->frame_cfg.fec;
    xlayer_cfg->modulation = wps_mac->current_main_connection->frame_cfg.modulation;
}

/** @brief Return the corresponding queue for TX depending on the input connection.
 *
 * For TX timeslot, Application should have enqueue a node
 * inside the queue, so the MAC only need to get the front of
 * the queue in order to get the good node for the process.
 *
 *  @param[in] connection  Queue node connection.
 *  @return Current pending node in queue.
 */
static xlayer_t *get_xlayer_for_tx(wps_mac_t *wps_mac, wps_connection_t *connection)
{
    xlayer_t *free_xlayer;
    xlayer_queue_node_t *node;
    bool unsync = ((wps_mac->tdma_sync.slave_sync_state == STATE_SYNCING) && (wps_mac->node_role == NETWORK_NODE));

    if (connection->currently_enabled) {
        node = xlayer_queue_get_node(&connection->xlayer_queue);
    } else {
        node = NULL;
    }
    if (node == NULL) {
        free_xlayer = NULL;
    } else {
        free_xlayer = &node->xlayer;
    }

    if (free_xlayer == NULL || unsync) {
        if (connection->auto_sync_enable && !unsync) {
            wps_mac->empty_frame_tx.frame.header_memory = overrun_buffer;
            wps_mac->empty_frame_tx.frame.header_end_it = overrun_buffer + connection->header_size;
        } else {
            wps_mac->empty_frame_tx.frame.header_memory = NULL;
            wps_mac->empty_frame_tx.frame.header_end_it = NULL;
        }
        wps_mac->empty_frame_tx.frame.header_begin_it  = wps_mac->empty_frame_tx.frame.header_end_it;
        wps_mac->empty_frame_tx.frame.payload_end_it   = wps_mac->empty_frame_tx.frame.header_end_it;
        wps_mac->empty_frame_tx.frame.payload_begin_it = wps_mac->empty_frame_tx.frame.header_end_it;
        free_xlayer                                    = &wps_mac->empty_frame_tx;
        wps_mac->empty_frame_tx.frame.time_stamp       = connection->get_tick();
    } else {
        free_xlayer->frame.header_begin_it = free_xlayer->frame.header_end_it;
    }

    return free_xlayer;
}

/** @brief Return the corresponding queue for RX depending on the input connection.
 *
 * For RX timeslot, MAC should get the first free slot, WPS
 * should enqueue for application .
 *
 *  @param[in] connection  Queue node connection.
 *  @return Current pending node in queue.
 */
static xlayer_t *get_xlayer_for_rx(wps_mac_t *wps_mac, wps_connection_t *connection)
{
    wps_mac->rx_node = xlayer_queue_get_free_node(connection->free_queue);

    /* if free node is not available, will return an empty frame*/
    if (wps_mac->rx_node == NULL) {
        wps_mac->empty_frame_rx.frame.header_memory       = overrun_buffer;
        wps_mac->empty_frame_rx.frame.header_end_it       = overrun_buffer;
        wps_mac->empty_frame_rx.frame.header_begin_it     = wps_mac->empty_frame_rx.frame.header_end_it;
        wps_mac->empty_frame_rx.frame.payload_end_it      = wps_mac->empty_frame_rx.frame.header_end_it;
        wps_mac->empty_frame_rx.frame.payload_begin_it    = wps_mac->empty_frame_rx.frame.header_end_it;
        wps_mac->empty_frame_rx.frame.payload_memory_size = connection->payload_size;
        wps_mac->empty_frame_rx.frame.header_memory_size  = connection->header_size;
        return &wps_mac->empty_frame_rx;
    }

    wps_mac->rx_node->xlayer.frame.payload_memory_size = connection->payload_size;
    wps_mac->rx_node->xlayer.frame.header_memory_size = connection->header_size;
    return &wps_mac->rx_node->xlayer;
}

/** @brief Extract the header fields from a received node queue.
 *
 *  @param[out] wps_mac        Frame header MAC instance.
 *  @param[in]  current_queue  xlayer header.
 *  @retval True   The received timeslot ID was different from the expected one.
 *  @retval False  The received timeslot ID was identical to the expected one.
 */
static void extract_header(wps_mac_t *wps_mac, xlayer_t *current_queue)
{
    wps_mac_t *mac = wps_mac;

    find_received_timeslot_and_connection(mac);

    /* MAC should always be the first to extract */
    current_queue->frame.header_begin_it = current_queue->frame.header_memory;
    if (current_queue->frame.header_begin_it != NULL) {
        /* First byte should always be the radio automatic response */
        current_queue->frame.header_begin_it++;

        if (wps_mac->current_ts_auto_reply && !wps_mac->current_ts_auto_reply_tx) {
            link_protocol_receive_buffer(&wps_mac->current_auto_connection->link_protocol,
                                         wps_mac->current_xlayer->frame.header_begin_it,
                                         wps_mac->current_auto_connection->header_size);
            wps_mac->current_auto_connection = link_scheduler_get_current_auto_connection(&wps_mac->scheduler, wps_mac->auto_connection_id);
            wps_mac->current_xlayer->frame.header_begin_it += wps_mac->current_auto_connection->header_size;

        } else {
            link_protocol_receive_buffer(&wps_mac->current_main_connection->link_protocol,
                                         wps_mac->current_xlayer->frame.header_begin_it,
                                         wps_mac->current_main_connection->header_size);
            wps_mac->current_main_connection = link_scheduler_get_current_main_connection(&wps_mac->scheduler, wps_mac->main_connection_id);
            wps_mac->current_xlayer->frame.header_begin_it += wps_mac->current_main_connection->header_size;
        }

        /* Assign payload pointer as if there is no other layer on top. */
        current_queue->frame.payload_begin_it = current_queue->frame.header_begin_it;
    }
}

/** @brief Fill the header fields for a TX node queue.
 *
 *  @param[in]  wps_mac        Frame header MAC instance.
 *  @param[in]  current_queue  header xlayer.
 */
static void fill_header(wps_mac_t *wps_mac, xlayer_t *current_queue)
{
    uint32_t size = 0;

    if (wps_mac->current_ts_auto_reply_tx) {
        current_queue->frame.header_begin_it -= wps_mac->current_auto_connection->header_size;
        link_protocol_send_buffer(&wps_mac->current_auto_connection->link_protocol, current_queue->frame.header_begin_it,
                                  &size);
    } else {
        current_queue->frame.header_begin_it -= wps_mac->current_main_connection->header_size;
        link_protocol_send_buffer(&wps_mac->current_main_connection->link_protocol, current_queue->frame.header_begin_it, &size);
    }
}

/** @brief Fill the header fields for a TX node queue.
 *
 *  @param[in] current_queue  Current xlayer.
 *  @retval True   No payload have been received.
 *  @retval False  Payload have been received.
 */
static bool no_payload_received(xlayer_t *current_queue)
{
    return (current_queue->frame.header_begin_it == current_queue->frame.payload_end_it);
}

/** @brief Setup MAC instance flag for timeslot auto reply status.
 *
 *  @param[in] wps_mac  MAC instance.
 */
static void setup_auto_reply_timeslot_status(wps_mac_t *wps_mac)
{
    if (is_current_timeslot_auto_reply(wps_mac)) {
        wps_mac->current_ts_auto_reply = true;
        if (is_current_auto_reply_timeslot_tx(wps_mac)) {
            wps_mac->current_ts_auto_reply_tx = true;
        } else {
            wps_mac->current_ts_auto_reply_tx = false;
        }
    } else {
        wps_mac->current_ts_auto_reply    = false;
        wps_mac->current_ts_auto_reply_tx = false;
    }
}

/** @brief Output if current input signal is MAC_SIGNAL_SCHEDULE.
 *
 *  @param[in] wps_mac  WPS MAC instance.
 *  @retval true   Scheduling needed.
 *  @retval false  No scheduling needed.
 */
static bool scheduling_needed(wps_mac_t *wps_mac)
{
    return (wps_mac->current_input == MAC_SIGNAL_SCHEDULE);
}

/** @brief Process the MAC_SIGNAL_SCHEDULE input.
 *
 *  @param[in] wps_mac  WPS MAC instance.
 */
static void process_scheduler(wps_mac_t *wps_mac)
{
    do {
        wps_mac->state_machine[wps_mac->current_input][wps_mac->state_process_idx++](wps_mac);
    } while (wps_mac->state_machine[wps_mac->current_input][wps_mac->state_process_idx] != end);
}

/** @brief Process frame outcome coming from WPS PHY layer.
 *
 *  @param[in] wps_mac  WPS MAC instance.
 */
static void process_rx_tx_outcome(wps_mac_t *wps_mac)
{
    wps_mac->current_xlayer = wps_mac->main_xlayer;
    /* Handle main xlayer processing */
    do {
        wps_mac->state_machine[wps_mac->current_input][wps_mac->state_process_idx++](wps_mac);
    } while (wps_mac->state_machine[wps_mac->current_input][wps_mac->state_process_idx] != end);

    /* Assign main output */
    wps_mac->output_signal.main_signal = wps_mac->current_output;

    /* Reset state process for state machine */
    wps_mac->state_process_idx = 0;

    wps_mac->current_output = MAC_SIGNAL_WPS_EMPTY;

    if (wps_mac->auto_xlayer != NULL) {
        /* Get WPS input xlayer and reset output */
        wps_mac->current_input  = wps_mac->input_signal.auto_signal;
        wps_mac->current_xlayer = wps_mac->auto_xlayer;
        /* Handle auto xlayer processing*/
        while (wps_mac->state_machine[wps_mac->current_input][wps_mac->state_process_idx] != end) {
            wps_mac->state_machine[wps_mac->current_input][wps_mac->state_process_idx++](wps_mac);
        }
    }

    /* Assign auto output */
    wps_mac->output_signal.auto_signal = wps_mac->current_output;
}

/** @brief Finish a transmission.
 *
 *  @param[in] connection wps_connection_t instance.
 *  @retval true   On success.
 *  @retval false  On error.
 */
static bool send_done(wps_connection_t *connection)
{
    xlayer_queue_node_t *node;

    connection->tx_flush = false;
    node = xlayer_queue_dequeue_node(&connection->xlayer_queue);
    xlayer_queue_free_node(node);
    return true;
}

/** @brief  Check and flush timeout frame before sending to PHY.
 *
 *  @param wps_mac  WPS MAC instance.
 */
static void flush_timeout_frames_before_sending(wps_mac_t *wps_mac, wps_connection_t *connection)
{
    bool timeout = false;
    xlayer_queue_node_t *xlayer_queue_node;

    do {
        xlayer_queue_node = xlayer_queue_get_node(&connection->xlayer_queue);
        if ((xlayer_queue_node != NULL) && (&xlayer_queue_node->xlayer != NULL)) {
            timeout = link_saw_arq_is_frame_timeout(&connection->stop_and_wait_arq,
                                                    xlayer_queue_node->xlayer.frame.time_stamp,
                                                    xlayer_queue_node->xlayer.frame.retry_count++,
                                                    connection->get_tick());
            if (timeout) {
                if (is_current_timeslot_auto_reply(wps_mac)) {
                    wps_mac->config.callback_auto.callback      = connection->tx_drop_callback;
                    wps_mac->config.callback_auto.parg_callback = connection->tx_drop_parg_callback;
                } else {
                    wps_mac->config.callback_main.callback      = connection->tx_drop_callback;
                    wps_mac->config.callback_main.parg_callback = connection->tx_drop_parg_callback;
                }
                wps_callback_enqueue(wps_mac->callback_queue, &wps_mac->config.callback_main);
                wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_TX_DROP;
#ifndef WPS_DISABLE_LINK_STATS
                update_wps_stats(wps_mac);
#endif /* WPS_DISABLE_LINK_STATS */
                send_done(connection);
            }
        } else {
            timeout = false;
        }
    } while (timeout);
}

/** @brief  Flush the next packet from the wps tx queue.
 *
 *  @param wps_mac  WPS MAC instance.
 */
static void flush_tx_frame(wps_mac_t *wps_mac, wps_connection_t *connection)
{
    xlayer_t *xlayer;

    xlayer = &xlayer_queue_get_node(&connection->xlayer_queue)->xlayer;

    if (xlayer != NULL) {
        if (is_current_timeslot_auto_reply(wps_mac)) {
            wps_mac->config.callback_auto.callback      = connection->tx_drop_callback;
            wps_mac->config.callback_auto.parg_callback = connection->tx_drop_parg_callback;
        } else {
            wps_mac->config.callback_main.callback      = connection->tx_drop_callback;
            wps_mac->config.callback_main.parg_callback = connection->tx_drop_parg_callback;
        }
        wps_callback_enqueue(wps_mac->callback_queue, &wps_mac->config.callback_main);
        wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_TX_DROP;
#ifndef WPS_DISABLE_LINK_STATS
        update_wps_stats(wps_mac);
#endif /* WPS_DISABLE_LINK_STATS */
        send_done(connection);
    }
}

#ifndef WPS_DISABLE_LINK_STATS
/** @brief Update WPS statistics
 *
 *  @param[in] MAC  WPS MAC instance.
 */
static void update_wps_stats(wps_mac_t *mac)
{
    xlayer_t *current_xlayer;

    current_xlayer = mac->main_xlayer;
    switch (mac->output_signal.main_signal) {
    case MAC_SIGNAL_WPS_FRAME_RX_SUCCESS:
        mac->current_main_connection->wps_stats.rx_received++;
        mac->current_main_connection->wps_stats.rx_byte_received += (current_xlayer->frame.payload_end_it -
                                                                               current_xlayer->frame.payload_begin_it);
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_OVERRUN:
        mac->current_main_connection->wps_stats.rx_overrun++;
        break;
    case MAC_SIGNAL_WPS_TX_SUCCESS:
        mac->current_main_connection->wps_stats.tx_success++;
        mac->current_main_connection->wps_stats.tx_byte_sent += (current_xlayer->frame.payload_end_it -
                                                                           current_xlayer->frame.payload_begin_it);
        if (mac->current_main_connection->cca.enable) {
            if (mac->config.cca_try_count >= mac->config.cca_max_try_count) {
                mac->current_main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->current_main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->current_main_connection->wps_stats.cca_tx_fail++;
                mac->current_main_connection->total_cca_tx_fail_count++;
            } else if (current_xlayer->frame.frame_outcome != FRAME_WAIT) {
                mac->current_main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->current_main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->current_main_connection->wps_stats.cca_pass++;
            }
        }
        break;
    case MAC_SIGNAL_WPS_TX_FAIL:

        mac->current_main_connection->wps_stats.tx_fail++;

        if (mac->current_main_connection->cca.enable) {
            if (mac->config.cca_try_count >= mac->config.cca_max_try_count) {
                mac->current_main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->current_main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->current_main_connection->wps_stats.cca_tx_fail++;
                mac->current_main_connection->total_cca_tx_fail_count++;
            } else if (current_xlayer->frame.frame_outcome != FRAME_WAIT) {
                mac->current_main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->current_main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->current_main_connection->wps_stats.cca_pass++;
            }
        }
        break;
    case MAC_SIGNAL_WPS_TX_DROP:
        mac->current_main_connection->wps_stats.tx_drop++;
        mac->current_main_connection->total_pkt_dropped++;
        break;
    case MAC_SIGNAL_WPS_EMPTY:
        /* PHY NACK signal occurred but SAW has not yet trigger, handle CCA stats only. */
        if (mac->current_main_connection->cca.enable) {
            if (mac->config.cca_try_count >= mac->config.cca_max_try_count) {
                mac->current_main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->current_main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->current_main_connection->wps_stats.cca_tx_fail++;
                mac->current_main_connection->total_cca_tx_fail_count++;
            } else if (current_xlayer->frame.frame_outcome != FRAME_WAIT) {
                mac->current_main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->current_main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->current_main_connection->wps_stats.cca_pass++;
            }
        }
        break;
    default:
        break;
    }
    current_xlayer = mac->auto_xlayer;
    switch (mac->output_signal.auto_signal) {
    case MAC_SIGNAL_WPS_FRAME_RX_SUCCESS:
        mac->current_auto_connection->wps_stats.rx_received++;
        mac->current_auto_connection->wps_stats.rx_byte_received += (current_xlayer->frame.payload_end_it -
                                                                                     current_xlayer->frame.payload_begin_it);
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_OVERRUN:
        mac->current_auto_connection->wps_stats.rx_overrun++;
        break;
    case MAC_SIGNAL_WPS_TX_SUCCESS:
        mac->current_auto_connection->wps_stats.tx_success++;
        mac->current_auto_connection->wps_stats.tx_byte_sent += (current_xlayer->frame.payload_end_it -
                                                                                 current_xlayer->frame.payload_begin_it);
        break;
    case MAC_SIGNAL_WPS_TX_FAIL:
        mac->current_auto_connection->wps_stats.tx_fail++;
        break;
    case MAC_SIGNAL_WPS_TX_DROP:
        mac->current_auto_connection->wps_stats.tx_drop++;
        mac->current_main_connection->total_pkt_dropped++;
        break;
    default:
        break;
    }
}
#endif /* WPS_DISABLE_LINK_STATS */

/** @brief Handle link throttle.
 *
 *  @param[in] wps_mac    WPS MAC instance.
 *  @param[in] inc_count  Increment count.
 */
static void handle_link_throttle(wps_mac_t *wps_mac, uint8_t *inc_count)
{
    wps_connection_t *candidate_connection;
    timeslot_t *time_slot;
    bool ts_enabled;

    do {
        time_slot = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
        for (uint8_t i = 0; i < time_slot->main_connection_count; i++) {
            candidate_connection                    = time_slot->connection_main[i];
            candidate_connection->currently_enabled = true;

            if (candidate_connection->pattern != NULL) {
                candidate_connection->pattern_count = (candidate_connection->pattern_count + 1) %
                                                      candidate_connection->pattern_total_count;

                candidate_connection->currently_enabled =
                    candidate_connection->pattern[candidate_connection->pattern_count];
            }
        }

        for (uint8_t i = 0; i < time_slot->auto_connection_count; i++) {
            candidate_connection                    = time_slot->connection_auto_reply[i];
            candidate_connection->currently_enabled = true;
        }

        ts_enabled = false;
        for (uint8_t i = 0; i < time_slot->main_connection_count; i++) {
            ts_enabled = time_slot->connection_main[i]->currently_enabled;
            if (ts_enabled == true) {
                break;
            }
        }

        if (ts_enabled == false) {
            *inc_count += link_scheduler_increment_time_slot(&wps_mac->scheduler);
        }

    } while (ts_enabled == false);
}

/** @brief Get the event associated with the current connection status.
 *
 *  @param[in] link_connect_status Link connection status module instance.
 *  @retval [WPS_EVENT_CONNECT]    if the status is connected.
 *  @retval [WPS_EVENT_DISCONNECT] if the status is disconnected.
 */
static inline wps_error_t get_status_error(link_connect_status_t *link_connect_status)
{
    return (link_connect_status->status == CONNECT_STATUS_CONNECTED) ? WPS_CONNECT_EVENT : WPS_DISCONNECT_EVENT;
}

/** @brief Get the index of the highest priority connection.
 *
 *  @param[in] connections       Connection table.
 *  @param[in] connection_count  Connection count.
 */
static uint8_t get_highest_priority_conn_index(wps_connection_t **connections, uint8_t connection_count)
{
    xlayer_t *free_xlayer;
    xlayer_queue_node_t *node;
    uint8_t min_prio  = WPS_MAX_CONN_PRIORITY + 1;
    uint8_t min_index = 0;

    for (uint8_t i = 0; i < connection_count; i++) {
        if (connections[i]->currently_enabled) {
            node = xlayer_queue_get_node(&connections[i]->xlayer_queue);
        } else {
            node = NULL;
        }
        if (node == NULL) {
            free_xlayer = NULL;
        } else {
            free_xlayer = &node->xlayer;
        }
        if (free_xlayer != NULL) {
            if (connections[i]->priority < min_prio) {
                min_prio = connections[i]->priority;
                min_index = i;
            }
            if (min_prio == 0) {
                break;
            }

        }
    }
    return min_index;
}

/** @brief Find the received time slot ID and connection ID.
 *
 *  @param[out] wps_mac  Frame header MAC instance.
 */
static void find_received_timeslot_and_connection(wps_mac_t *wps_mac)
{
    uint8_t ts_id_saw;
    uint8_t time_slot_id;
    uint8_t connection_id;
    wps_connection_t *connection;
    uint8_t connection_count;
    uint8_t *conn_id;

    if (wps_mac->current_ts_auto_reply && !wps_mac->current_ts_auto_reply_tx) {
        connection = wps_mac->current_auto_connection;
        connection_count = wps_mac->current_timeslot->auto_connection_count;
        conn_id = &wps_mac->auto_connection_id;
    } else {
        connection = wps_mac->current_main_connection;
        connection_count = wps_mac->current_timeslot->main_connection_count;
        conn_id = &wps_mac->main_connection_id;
        if (is_network_node(wps_mac)) {
            ts_id_saw = *(wps_mac->current_xlayer->frame.header_begin_it +
                            link_protocol_get_buffer_offset(&connection->link_protocol,
                                                            MAC_PROTO_ID_TIMESLOT_SAW));
            time_slot_id = MASK2VAL(ts_id_saw, HEADER_BYTE0_TIME_SLOT_ID_MASK);
            if (time_slot_id < wps_mac->scheduler.schedule.size) {
                if (link_scheduler_get_next_timeslot_index(&wps_mac->scheduler) != time_slot_id) {
                    link_scheduler_set_mismatch(&wps_mac->scheduler);
                }
                link_scheduler_set_time_slot_i(&wps_mac->scheduler, time_slot_id);
            }
        }

        if ((!link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync) && (wps_mac->node_role == NETWORK_NODE)) ||
            link_scheduler_get_mismatch(&wps_mac->scheduler)) {
            wps_mac->current_timeslot = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
            wps_mac->current_main_connection = link_scheduler_get_current_main_connection(&wps_mac->scheduler, wps_mac->main_connection_id);
            connection = wps_mac->current_main_connection;
            connection_count = wps_mac->current_timeslot->main_connection_count;
        }
    }

    if (connection_count > 1) {
        connection_id = *(wps_mac->current_xlayer->frame.header_begin_it +
                            link_protocol_get_buffer_offset(&connection->link_protocol,
                                                            MAC_PROTO_ID_CONNECTION_ID));
        if (connection_id < connection_count) {
            *conn_id = connection_id;
        } else {
            *conn_id = 0;
        }
    } else {
        *conn_id = 0;
    }
    wps_mac->current_main_connection = link_scheduler_get_current_main_connection(&wps_mac->scheduler, wps_mac->main_connection_id);
    wps_mac->current_auto_connection = link_scheduler_get_current_auto_connection(&wps_mac->scheduler, wps_mac->auto_connection_id);

}
