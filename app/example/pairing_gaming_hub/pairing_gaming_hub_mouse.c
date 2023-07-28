/** @file  pairing_gaming_hub_mouse.c
 *  @brief The mouse device is part of the gaming hub to demonstrate a point to multipoint pairing application.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_audio.h"
#include "iface_pairing.h"
#include "iface_pairing_gaming_hub.h"
#include "iface_wireless.h"
#include "sac_api.h"
#include "swc_api.h"
#include "swc_cfg_mouse.h"
#include "swc_pairing.h"
#include "swc_utils.h"

/* CONSTANTS ******************************************************************/
#define SWC_MEM_POOL_SIZE          8000
#define MOUSE_PAYLOAD_SIZE         1
#define PAIRING_DEVICE_ROLE        2

#define PAIRING_TIMEOUT_IN_SECONDS 10

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t hal;
static swc_node_t *node;
static swc_connection_t *rx_from_beacon_conn;
static swc_connection_t *tx_to_dongle_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_from_beacon_timeslots[] = RX_FROM_BEACON_TIMESLOTS;
static int32_t tx_to_dongle_timeslots[] = TX_TO_DONGLE_TIMESLOTS;

/* ** Application Specific ** */
static bool is_device_paired;
static swc_pairing_t app_pairing;
static uint16_t timeout_counter;
static bool is_pairing_timeout;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_pairing_t *app_pairing, swc_error_t *err);

static void send_mouse_pressed_state(void);
static void send_mouse_released_state(void);
static void enter_pairing_mode(void);
static void unpair_device(void);

static void pairing_timer_cb(void);
static void start_pairing_timer(void);
static void stop_and_reset_pairing_timer(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    swc_error_t swc_err;

    iface_board_init();

    iface_swc_hal_init(&hal);
    iface_swc_handlers_init();

    /* Give the default addresses when entering pairing */
    app_pairing.coordinator_address = COORDINATOR_ADDRESS;
    app_pairing.node_address = NODE_ADDRESS;
    app_pairing.pan_id = PAN_ID;
    app_swc_core_init(&app_pairing, &swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    swc_connect(&swc_err);

    while (1) {
        iface_state_machine_delay(1);
        if (!is_device_paired) {
            /* Since the device is unpaired the send_mouse_pressed_state should do nothing */
            iface_button_handling_osr(NULL, enter_pairing_mode);
        } else {
            iface_button_handling_osr(send_mouse_pressed_state, unpair_device);
            iface_button_handling_osf(send_mouse_released_state, NULL);
        }
    }
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[out] err  Wireless Core error code.
 */
static void app_swc_core_init(swc_pairing_t *app_pairing, swc_error_t *err)
{
    uint16_t local_address;
    uint16_t remote_address;

    local_address = app_pairing->node_address;
    remote_address = app_pairing->coordinator_address;

    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = SWC_ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = SWC_ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = app_pairing->pan_id,
        .coordinator_address = app_pairing->coordinator_address,
        .local_address = local_address,
        .sleep_level = SWC_SLEEP_LEVEL
    };
    node = swc_node_init(node_cfg, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_radio_cfg_t radio_cfg = {
        .irq_polarity = SWC_IRQ_ACTIVE_HIGH,
        .spi_mode = SWC_SPI_STANDARD,
    };
    swc_node_add_radio(node, radio_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    /* ** Mouse receiving Beacon from Dongle connection ** */
    swc_connection_cfg_t rx_from_beacon_conn_cfg = {
        .name = "RX from Beacon Connection",
        .source_address = remote_address,
        .destination_address = local_address,
        .max_payload_size = 0,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_BEACON_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_from_beacon_timeslots,
        .timeslot_count = SWC_ARRAY_SIZE(rx_from_beacon_timeslots),
        .ack_enabled = false,
        .arq_enabled = false,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    rx_from_beacon_conn = swc_connection_init(node, rx_from_beacon_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_beacon_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        rx_from_beacon_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_from_beacon_conn, node, rx_from_beacon_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** Mouse sending to Dongle connection ** */
    swc_connection_cfg_t tx_to_dongle_conn_cfg = {
        .name = "Mouse to Dongle connection",
        .source_address = local_address,
        .destination_address = remote_address,
        .max_payload_size = MOUSE_PAYLOAD_SIZE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_to_dongle_timeslots,
        .timeslot_count = SWC_ARRAY_SIZE(tx_to_dongle_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .rdo_enabled = false
    };
    tx_to_dongle_conn = swc_connection_init(node, tx_to_dongle_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_to_dongle_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        tx_to_dongle_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_to_dongle_conn, node, tx_to_dongle_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_setup(node, err);
}

/** @brief Send the mouse pressed state to the Dongle.
 */
static void send_mouse_pressed_state(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    uint8_t *mouse_press_status = NULL;

    swc_connection_get_payload_buffer(tx_to_dongle_conn, &mouse_press_status, &swc_err);
    if (mouse_press_status != NULL) {
        *mouse_press_status = 0x01;
        swc_connection_send(tx_to_dongle_conn, mouse_press_status, MOUSE_PAYLOAD_SIZE, &swc_err);
    }
    iface_mouse_button_latched_status();
}



/** @brief Send the state of the button on a button release event.
 */
static void send_mouse_released_state(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    uint8_t *mouse_press_status = NULL;

    swc_connection_get_payload_buffer(tx_to_dongle_conn, &mouse_press_status, &swc_err);
    if (mouse_press_status != NULL) {
        *mouse_press_status = 0x00;
        swc_connection_send(tx_to_dongle_conn, mouse_press_status, MOUSE_PAYLOAD_SIZE, &swc_err);
    }
    iface_mouse_button_unlatched_status();
}

/** @brief Enter in Pairing Mode using the Pairing Module.
 */
static void enter_pairing_mode(void)
{
    swc_error_t swc_err;

    iface_notify_enter_pairing();

    /* Give the information to the Pairing Application */
    app_pairing.network_role = NETWORK_ROLE;
    app_pairing.device_role  = PAIRING_DEVICE_ROLE;
    app_pairing.memory_pool = swc_memory_pool;
    app_pairing.memory_pool_size = SWC_MEM_POOL_SIZE;
    app_pairing.uwb_regulation = SWC_REGULATION_FCC;

    swc_pairing_init_pairing_process(&app_pairing, &hal);

    is_pairing_timeout = false;
    iface_pairing_timer_set_callback(pairing_timer_cb);
    start_pairing_timer();

    do {
        is_device_paired = swc_pairing_process();
    } while ((!is_device_paired) && (!is_pairing_timeout));

    stop_and_reset_pairing_timer();
    swc_pairing_deinit();

    if (is_device_paired) {
        /* Reconfigure the Coordinator and Node addresses */
        app_swc_core_init(&app_pairing, &swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }
        swc_connect(&swc_err);

        /* Indicate that the pairing process was successful */
        iface_notify_pairing_successful();
    } else {
        memset(&app_pairing, 0, sizeof(swc_pairing_t));
        app_pairing.coordinator_address = COORDINATOR_ADDRESS;
        app_pairing.node_address = NODE_ADDRESS;
        app_pairing.pan_id = PAN_ID;
        app_swc_core_init(&app_pairing, &swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }
        swc_connect(&swc_err);

        /* Indicate that the pairing process was unsuccessful */
        iface_notify_not_paired();
    }
}

/** @brief Unpair the device, this will put its connection addresses back to default value.
 */
static void unpair_device(void)
{
    swc_error_t swc_err;

    is_device_paired = false;

    /* Give the default addresses */
    swc_disconnect(&swc_err);
    memset(&app_pairing, 0, sizeof(swc_pairing_t));
    app_pairing.coordinator_address = COORDINATOR_ADDRESS;
    app_pairing.node_address = NODE_ADDRESS;
    app_pairing.pan_id = PAN_ID;
    app_swc_core_init(&app_pairing, &swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }
    swc_connect(&swc_err);

    /* Indicate that the device is unpaired */
    iface_notify_not_paired();
}

/** @brief Callback for the Pairing timer.
 */
static void pairing_timer_cb(void)
{
    timeout_counter++;

    if (timeout_counter >= (PAIRING_TIMEOUT_IN_SECONDS)) {
        stop_and_reset_pairing_timer();
        is_pairing_timeout = true;
    }
}

/** @brief Initialize and start the Pairing timer.
 */
static void start_pairing_timer(void)
{
    iface_pairing_timer_init(1000); /* Trigger callback each second */

    iface_pairing_timer_start();
}

/** @brief Stop Pairing timer and reload the timeout counter.
 */
static void stop_and_reset_pairing_timer(void)
{
    iface_pairing_timer_stop();
    timeout_counter = 0;
}
