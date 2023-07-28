/** @file wps_process.c
 *  @brief Wireless Protocol Stack process
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps.h"

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void idle(wps_t *wps);
static void mac_pre(wps_t *wps);
static void phy_handle(wps_t *wps);
static void error(wps_t *wps);
static void yield(wps_t *wps);
static void mac_post(wps_t *wps);

/** WPS Process State Machine
 *
 *              |
 *              |
 *       +------v------+
 *       |             |
 *       |    IDLE     |
 *       |             |
 *       +------+------+         PHY_YIELD
 *              |                 +-----+
 *              |CONNECT          |     |
 *              |                 |     |
 *       +------v------+      +---v-----+---+     +-------------+
 *       |             |      |             |     |             |
 *  +---->   MAC_PRE   +------>  PHY_HANDLE +----->  MAC_POST   +----+
 *  |    |             |      |             |     |             |    |
 *  |    +------^------+      +-+----^----+-+     +-------------+    |
 *  |           |               |    |    |                          |
 *  |           |               |    |    |PHY_DONE                  |
 *  |           |               |    |    |                          |
 *  |    +------+------+        |  +-+----v------+                   |
 *  |    |             |        |  |             |                   |
 *  |    |    ERROR    <--------+  |    YIELD    |                   |
 *  |    |             |PHY_ERROR  |             |                   |
 *  |    +-------------+           +-------------+                   |
 *  |                                                                |
 *  |                                                                |
 *  |                                                                |
 *  +----------------------------------------------------------------+
 *
 */

static void enqueue_states(wps_t *wps, wps_process_state_t state);
static void end_state(wps_t *wps);

static void set_signal_mac_to_wps(wps_mac_t *mac, wps_t *wps);
static void set_signal_mac_to_phy(wps_mac_t *mac, wps_phy_t *phy);
static void set_signal_phy_to_mac(wps_phy_t *phy, wps_mac_t *mac);

static void process_pending_request(wps_t *wps);
static void process_schedule_request(wps_request_info_t *request);
static void process_write_request(wps_t *wps, wps_request_info_t *request);
static void process_read_request(wps_t *wps, wps_request_info_t *request);
static void process_disconnect_request(wps_t *wps);

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
void wps_process_init(wps_t *wps)
{
    wps->state_step    = 0;
    wps->current_state = idle;
    circular_queue_init(&wps->next_states, wps->next_state_pool, PROCESS_STATE_Q_SIZE, sizeof(wps_phy_state_t **));
}

void wps_process_callback(wps_t *wps)
{
    wps_callback_inst_t *callback;

    while (circular_queue_is_empty(&wps->l7.callback_queue) == false) {
        callback = circular_queue_front(&wps->l7.callback_queue);
        if (callback != NULL && callback->func != NULL) {
            callback->func(callback->parg);
        }
        circular_queue_dequeue(&wps->l7.callback_queue);
    }
}

/* PRIVATE FUNCTION DEFINITIONS ***********************************************/
/** @brief State : IDLE - State machine waiting for external command.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void idle(wps_t *wps)
{
    if (wps->signal == WPS_CONNECT) {
        wps->signal = WPS_NONE;
        wps->process_signal = PROCESS_SIGNAL_EXECUTE;
        enqueue_states(wps, mac_pre);
    }
    end_state(wps);
}

/** @brief State : MAC pre - Prepare PHY transfer in the MAC layer.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void mac_pre(wps_t *wps)
{
    wps->mac.input_signal.main_signal = MAC_SIGNAL_SCHEDULE;
    wps_mac_process(&wps->mac);
    set_signal_mac_to_phy(&wps->mac, wps->phy);
    wps_phy_set_main_xlayer(wps->phy, wps->mac.main_xlayer, &wps->mac.config);
    wps_phy_set_auto_xlayer(wps->phy, wps->mac.auto_xlayer);
    wps_phy_prepare_frame(wps->phy);

    wps->process_signal = PROCESS_SIGNAL_EXECUTE;
    enqueue_states(wps, phy_handle);
    end_state(wps);
}

/** @brief State : PHY handle - Handle PHY signals during SPI transfers with the radio and frame outcome reception.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void phy_handle(wps_t *wps)
{
    switch (wps_phy_get_main_signal(wps->phy)) {
    case PHY_SIGNAL_YIELD:
        wps->process_signal = PROCESS_SIGNAL_YIELD;
        break;
    case PHY_SIGNAL_CONFIG_COMPLETE:
        process_pending_request(wps);
        wps->process_signal = PROCESS_SIGNAL_YIELD;
        break;
    case PHY_SIGNAL_PREPARE_DONE:
        wps->process_signal = PROCESS_SIGNAL_EXECUTE;
        enqueue_states(wps, yield);
        end_state(wps);
        break;
#if (RADIO_QSPI_ENABLED == 1)
    case PHY_SIGNAL_BLOCKING_CONFIG_DONE:
        process_pending_request(wps);
        wps->process_signal = PROCESS_SIGNAL_EXECUTE;
        enqueue_states(wps, yield);
        end_state(wps);
        break;
#endif
    case PHY_SIGNAL_FRAME_SENT_ACK:
    case PHY_SIGNAL_FRAME_SENT_NACK:
    case PHY_SIGNAL_FRAME_RECEIVED:
    case PHY_SIGNAL_FRAME_MISSED:
    case PHY_SIGNAL_FRAME_NOT_SENT:
        wps->process_signal = PROCESS_SIGNAL_EXECUTE;
        set_signal_phy_to_mac(wps->phy, &wps->mac);
        wps_phy_end_process(wps->phy);
        enqueue_states(wps, mac_post);
        end_state(wps);
        break;
    case PHY_SIGNAL_ERROR:
        wps->process_signal = PROCESS_SIGNAL_EXECUTE;
        enqueue_states(wps, error);
        end_state(wps);
        break;
    default:
        break;
    }
}

/** @brief State : PHY error - Dectect erroneous signal from the phy and restart the state machine.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void error(wps_t *wps)
{
    wps->mac.input_signal.main_signal = MAC_SIGNAL_SCHEDULE;
    wps_mac_reset(&wps->mac);
    wps_phy_disconnect(wps->phy);
    wps_phy_connect(wps->phy);
    wps->mac.current_timeslot->connection_main[wps->mac.main_connection_id]->wps_error = WPS_PHY_CRITICAL_ERROR;
    wps->mac.config.callback_main.callback                = wps->mac.current_timeslot->connection_main[wps->mac.main_connection_id]->evt_callback;
    wps->mac.config.callback_main.parg_callback           = wps->mac.current_timeslot->connection_main[wps->mac.main_connection_id]->evt_parg_callback;
    wps_callback_enqueue(&wps->l7.callback_queue, &wps->mac.config.callback_main);

    wps->process_signal = PROCESS_SIGNAL_EXECUTE;
    enqueue_states(wps, mac_pre);
    end_state(wps);
}

/** @brief State : APP yield - Routine to be executed when the WPS is done with the configuration of a frame transfer.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void yield(wps_t *wps)
{
    wps->callback_context_switch();

    wps->process_signal = PROCESS_SIGNAL_YIELD;
    enqueue_states(wps, phy_handle);
    end_state(wps);
}

/** @brief State : MAC post - Process frame outcome in the MAC and enqueue proper application callbacks.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void mac_post(wps_t *wps)
{
    wps_mac_process(&wps->mac);
    set_signal_mac_to_wps(&wps->mac, wps);

    wps->process_signal = PROCESS_SIGNAL_EXECUTE;
    enqueue_states(wps, mac_pre);
    end_state(wps);
}

/** @brief Enqueue a new state to the state machine.
 *
 *   @param[in] wps    WPS instance struct.
 *   @param[in] state  New state to enqueue.
 */
static void enqueue_states(wps_t *wps, wps_process_state_t state)
{
    wps_process_state_t *enqueue_states;

    enqueue_states  = (wps_process_state_t *)circular_queue_get_free_slot_raw(&wps->next_states);
    *enqueue_states = state;
    circular_queue_enqueue_raw(&wps->next_states);
}

/** @brief State : End of a state machine sequence.
 *
 *   @param[in] wps  WPS instance struct.
 */
static void end_state(wps_t *wps)
{
    wps_process_state_t *dequeue_state;

    wps->state_step    = 0;
    dequeue_state      = (wps_process_state_t *)circular_queue_front_raw(&wps->next_states);
    wps->current_state = *dequeue_state;
    circular_queue_dequeue_raw(&wps->next_states);
}

/** @brief Set signal between phy and mac.
 *
 *  @param[in] phy PHY layer instance.
 *  @param[in] mac MAC layer instance.
 */
static void set_signal_phy_to_mac(wps_phy_t *phy, wps_mac_t *mac)
{
    switch (wps_phy_get_main_signal(phy)) {
    case PHY_SIGNAL_FRAME_RECEIVED:
        mac->input_signal.main_signal = MAC_SIGNAL_RX_FRAME;
        break;
    case PHY_SIGNAL_FRAME_MISSED:
        mac->input_signal.main_signal = MAC_SIGNAL_RX_FRAME_MISS;
        break;
    case PHY_SIGNAL_FRAME_SENT_ACK:
        mac->input_signal.main_signal = MAC_SIGNAL_TX_SENT_ACK;
        break;
    case PHY_SIGNAL_FRAME_SENT_NACK:
        mac->input_signal.main_signal = MAC_SIGNAL_TX_SENT_NACK;
        break;
    case PHY_SIGNAL_NONE:
        mac->input_signal.main_signal = MAC_SIGNAL_EMPTY;
        break;
    default:
        break;
    }

    switch (wps_phy_get_auto_signal(phy)) {
    case PHY_SIGNAL_FRAME_RECEIVED:
        mac->input_signal.auto_signal = MAC_SIGNAL_RX_FRAME;
        break;
    case PHY_SIGNAL_FRAME_MISSED:
        mac->input_signal.auto_signal = MAC_SIGNAL_RX_FRAME_MISS;
        break;
    case PHY_SIGNAL_FRAME_SENT_ACK:
        mac->input_signal.auto_signal = MAC_SIGNAL_TX_SENT_ACK;
        break;
    case PHY_SIGNAL_FRAME_SENT_NACK:
        mac->input_signal.auto_signal = MAC_SIGNAL_TX_SENT_NACK;
        break;
    case PHY_SIGNAL_FRAME_NOT_SENT:
        mac->input_signal.auto_signal = MAC_SIGNAL_TX_NOT_SENT;
        break;
    case PHY_SIGNAL_NONE:
        mac->input_signal.auto_signal = MAC_SIGNAL_EMPTY;
        break;
    default:
        break;
    }
}

/** @brief Set signal between mac and WPS.
 *
 *  @param[in] mac MAC layer instance.
 *  @param[in] wps WPS layer instance.
 */
static void set_signal_mac_to_wps(wps_mac_t *mac, wps_t *wps)
{
    switch (mac->output_signal.main_signal) {
    case MAC_SIGNAL_WPS_EMPTY:
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_SUCCESS:
        xlayer_queue_enqueue_node(mac->current_timeslot->connection_main[wps->mac.main_connection_id]->rx_queue, mac->rx_node);
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_main);
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_FAIL:
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_OVERRUN:
        mac->current_timeslot->connection_main[wps->mac.main_connection_id]->wps_error = WPS_RX_OVERRUN_ERROR;
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_main);
        break;
    case MAC_SIGNAL_WPS_TX_SUCCESS:
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_main);
        break;
    case MAC_SIGNAL_WPS_TX_FAIL:
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_main);
        break;
    default:
        break;
    }

    switch (mac->output_signal.auto_signal) {
    case MAC_SIGNAL_WPS_EMPTY:
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_SUCCESS:
        xlayer_queue_enqueue_node(mac->current_timeslot->connection_auto_reply[wps->mac.auto_connection_id]->rx_queue, mac->rx_node);
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_auto);
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_FAIL:
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_OVERRUN:
        mac->current_timeslot->connection_auto_reply[wps->mac.auto_connection_id]->wps_error = WPS_RX_OVERRUN_ERROR;
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_auto);
        break;
    case MAC_SIGNAL_WPS_TX_SUCCESS:
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_auto);
        break;
    case MAC_SIGNAL_WPS_TX_FAIL:
        wps_callback_enqueue(&wps->l7.callback_queue, &mac->config.callback_auto);
        break;
    default:
        break;
    }
}

/** @brief Set signal between mac and phy.
 *
 *  @param[in] mac MAC layer instance.
 *  @param[in] phy PHY layer instance.
 */
static void set_signal_mac_to_phy(wps_mac_t *mac, wps_phy_t *phy)
{
    switch (mac->output_signal.main_signal) {
    case MAC_SIGNAL_SYNCING:
        wps_phy_set_input_signal(phy, PHY_SIGNAL_SYNCING);
        break;
    default:
        wps_phy_set_input_signal(phy, PHY_SIGNAL_PREPARE_RADIO);
        break;
    }
}

/** @brief Process application pending request.
 *
 *  @param[in] request  WPS request info structure.
 */
static void process_pending_request(wps_t *wps)
{
    wps_request_info_t *request;

    request = circular_queue_front(&wps->l7.request_queue);
    if (request != NULL) {
        switch (request->type) {
        case REQUEST_MAC_CHANGE_SCHEDULE_RATIO: {
            process_schedule_request(request);
            break;
        }
        case REQUEST_PHY_WRITE_REG: {
            if (WPS_RADIO_COUNT == 1) {
                process_write_request(wps, request);
            }
            break;
        }
        case REQUEST_PHY_READ_REG: {
            if (WPS_RADIO_COUNT == 1) {
                process_read_request(wps, request);
            }
            break;
        }
        case REQUEST_PHY_DISCONNECT:
            process_disconnect_request(wps);
            break;
        default:
            break;
        }
        circular_queue_dequeue(&wps->l7.request_queue);
    }
}

/** @brief Process MAC schedule change.
 *
 *  @note This allow the user to modify the active timeslot
 *        in the schedule of a given connection.
 *
 *  @note This process the request of type REQUEST_MAC_CHANGE_SCHEDULE_RATIO.
 *        Config structure should be of type wps_schedule_ratio_cfg_t.
 *
 *  @param[in] request  WPS request info structure.
 */
static void process_schedule_request(wps_request_info_t *request)
{
    wps_schedule_ratio_cfg_t *request_cfg = (wps_schedule_ratio_cfg_t *)request->config;
    bool *pattern                         = circular_queue_front(request_cfg->pattern_queue);

    if (pattern != NULL) {
        request_cfg->target_conn->active_ratio        = request_cfg->active_ratio;
        request_cfg->target_conn->pattern_total_count = request_cfg->pattern_total_count;
        request_cfg->target_conn->pattern_count       = request_cfg->pattern_current_count;
        memcpy(request_cfg->target_conn->pattern, pattern, request_cfg->pattern_total_count);
        circular_queue_dequeue(request_cfg->pattern_queue);
    }
}

/** @brief Process a write register request from application
 *
 *  @param[in] wps     WPS instance.
 *  @param[in] request WPS request info structure.
 */
static void process_write_request(wps_t *wps, wps_request_info_t *request)
{
    wps_write_request_info_t *write_request = (wps_write_request_info_t *)request->config;

    wps_phy_write_register(wps->phy, write_request->target_register, write_request->data);

    circular_queue_dequeue(wps->write_request_queue);
}

/** @brief Process a read register request from application
 *
 *  @param[in] wps     WPS instance.
 *  @param[in] request WPS request info structure.
 */
static void process_read_request(wps_t *wps, wps_request_info_t *request)
{
    wps_read_request_info_t *read_request = (wps_read_request_info_t *)request->config;

    wps_phy_read_register(wps->phy, read_request->target_register, read_request->rx_buffer, read_request->xfer_cmplt);

    circular_queue_dequeue(wps->read_request_queue);
}

/** @brief Process disconnection request.
 *
 *  @param[in] wps  WPS instance.
 */
static void process_disconnect_request(wps_t *wps)
{
    wps_phy_disconnect(wps->phy);

    /* Free MAC RX node in case a frame was received after the disconnect request */
    xlayer_queue_free_node(wps->mac.rx_node);

    wps->signal = WPS_DISCONNECT;
}
