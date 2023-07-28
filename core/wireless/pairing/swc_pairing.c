/** @file  swc_pairing.c
 *  @brief The pairing module is used exchange data between two devices and make them connect with each other.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "swc_pairing.h"
#include "swc_utils.h"

/* CONSTANTS ******************************************************************/
#define GENERATE_SERIALIZED_LEN              4
#define GENERATE_SERIALIZED_CRC_POLY         0x1021
#define GENERATE_SERIALIZED_CRC_CCITT_RELOAD 0xFFFFFFFF

/* Pairing constants. */
#define PAIRING_COORD_ADDRESS     0xF1
#define PAIRING_NODE_ADDRESS      0xF2
#define PAIRING_PAN_ID            0xBCD
#define PAIRING_CODE              0xCAFE
#define PAIRING_WAIT_DELAY_MS     100

/* Bytes position common to all pairing messages. */
#define PAIRING_BYTE_CODE_0  0
#define PAIRING_BYTE_CODE_1  1
#define PAIRING_BYTE_COMMAND 2

/* Bytes position for the pairing request message. */
#define PAIRING_REQUEST_BYTE_PAN_ID_MSB 3
#define PAIRING_REQUEST_BYTE_PAN_ID_LSB 4
#define PAIRING_REQUEST_BYTE_COORD_ID   5
#define PAIRING_REQUEST_BYTE_NODE_ID    6

/* Bytes position for the pairing response message. */
#define PAIRING_RESPONSE_BYTE_DEVICE_ROLE  3
#define PAIRING_RESPONSE_BYTE_UNIQUE_ID_0  4
#define PAIRING_RESPONSE_BYTE_UNIQUE_ID_1  5
#define PAIRING_RESPONSE_BYTE_UNIQUE_ID_2  6
#define PAIRING_RESPONSE_BYTE_UNIQUE_ID_3  7
#define PAIRING_RESPONSE_BYTE_UNIQUE_ID_4  8

/* TYPES **********************************************************************/
/** @brief Pairing commands.
 */
typedef enum pairing_command {
    PAIRING_COMMAND_NONE,
    PAIRING_COMMAND_SENT_REQUEST,
    PAIRING_COMMAND_SENT_RESPONSE,
    PAIRING_COMMAND_SENT_CONFIRMATION,
} pairing_command_t;

/** @brief Pairing states.
 */
typedef enum pairing_state {
    /*! Pairing state to enter and initialize the pairing process. */
    PAIRING_STATE_ENTER_PAIRING,
    /*! Pairing state used by the coordinator to prepare a pairing request. */
    PAIRING_STATE_PREPARE_PAIRING_REQUEST,
    /*! Pairing state used by the coordinator to send the prepared pairing request. */
    PAIRING_STATE_SEND_PAIRING_REQUEST,
    /*! Pairing state used by the node to wait for a pairing request from the coordinator. */
    PAIRING_STATE_WAIT_FOR_PAIRING_REQUEST,
    /*! Pairing state used by the node to prepare a pairing response. */
    PAIRING_STATE_PREPARE_PAIRING_RESPONSE,
    /*! Pairing state used by the node to send the prepared pairing response. */
    PAIRING_STATE_SEND_PAIRING_RESPONSE,
    /*! Pairing state used by the coordinator to wait for a pairing response from the node. */
    PAIRING_STATE_WAIT_FOR_PAIRING_RESPONSE,
    /*! Pairing state used by the coordinator to prepare a pairing confirmed message. */
    PAIRING_STATE_PREPARE_PAIRING_CONFIRMED,
    /*! Pairing state used by the coordinator to send the prepared pairing confirmed message. */
    PAIRING_STATE_SEND_PAIRING_CONFIRMED,
    /*! Pairing state used by the node to wait for a pairing confirmed message from the coordinator. */
    PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED,
    /*! Pairing state used by the coordinator to wait for the acknowledge of the node for the pairing confirmed message. */
    PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED_ACK,
    /*! Pairing state to close and deinitialize the pairing. */
    PAIRING_STATE_PAIRING_COMPLETE,
    /*! Pairing state to stop the pairing process. */
    PAIRING_STATE_EXIT_PAIRING,
} pairing_state_t;

/** @brief Pairing request message broadcasted by the coordinator.
 */
typedef struct pairing_request_message {
    /*! Pairing code most significant byte. */
    uint8_t pairing_code_msb;
    /*! Pairing code least significant byte. */
    uint8_t pairing_code_lsb;
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! PAN ID sent to the node (most significant byte). */
    uint8_t pan_id_msb;
    /*! PAN ID sent to the node (least significant byte). */
    uint8_t pan_id_lsb;
    /*! Coordinator ID inside the PAN sent to the node. */
    uint8_t coordinator_id;
    /*! Node ID available to be used by the node. */
    uint8_t node_id;
} pairing_request_message_t;

/** @brief Pairing response message sent when the node receives a pairing request.
 */
typedef struct pairing_response_message {
    /*! Pairing code most significant byte. */
    uint8_t pairing_code_msb;
    /*! Pairing code least significant byte. */
    uint8_t pairing_code_lsb;
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! The pairing device's application specific role. */
    uint8_t device_role;
    /*! Unique ID 0 generated by the device's radio (most significant byte). */
    uint8_t unique_id_0;
    /*! Unique ID 1 generated by the device's radio. */
    uint8_t unique_id_1;
    /*! Unique ID 2 generated by the device's radio. */
    uint8_t unique_id_2;
    /*! Unique ID 3 generated by the device's radio. */
    uint8_t unique_id_3;
    /*! Unique ID 4 generated by the device's radio (least significant byte). */
    uint8_t unique_id_4;
} pairing_response_message_t;

/** @brief Pairing confirmed message sent from the coordinator to the node when successfully paired.
 */
typedef struct pairing_confirmed_message {
    /*! Pairing code most significant byte. */
    uint8_t pairing_code_msb;
    /*! Pairing code least significant byte. */
    uint8_t pairing_code_lsb;
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
} pairing_confirmed_message_t;

/** @brief Pairing state machine callback function.
 */
typedef void (*const pairing_state_machine_callback_t)(void);

/** @brief Pairing state machine callback function link.
 *
 *  Link a pairing state with a callback function for the pairing state machine.
 */
typedef struct state_machine_function_table {
    /*! Pairing state. */
    uint8_t state;
    /*! Pairing state callback function. */
    pairing_state_machine_callback_t fp_state_machine_callback;
} state_machine_function_link_t;

/* PRIVATE GLOBALS ************************************************************/
/* Wireless Core. */
static swc_node_t *node;
static swc_connection_t *coord_to_node_conn;
static swc_connection_t *node_to_coord_conn;
static swc_hal_t *swc_hal;

static uint32_t timeslot_us[] = PAIRING_SCHEDULE;
static uint32_t channel_sequence[] = PAIRING_CHANNEL_SEQUENCE;
static uint32_t *channel_frequency;
static uint32_t channel_frequency_fcc_etsi[] = PAIRING_CHANNEL_FREQ_FCC_ETSI;
static uint32_t channel_frequency_arib[] = PAIRING_CHANNEL_FREQ_ARIB;
static int32_t coord_to_node_timeslots[] = COORD_TO_NODE_TIMESLOTS;
static int32_t node_to_coord_timeslots[] = NODE_TO_COORD_TIMESLOTS;

/* Pairing specific. */
static pairing_state_t current_pairing_state;
static swc_pairing_t *app_pairing;
static swc_pairing_t local_pairing;

static bool is_pairing_success;

static pairing_request_message_t pairing_request_message;
static pairing_response_message_t pairing_response_message;
static pairing_confirmed_message_t pairing_confirmed_message;

static uint8_t received_payload[9];
static pairing_command_t received_pairing_command;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void swc_pairing_core_init(swc_error_t *err);
static void reconfigure_swc_addresses(void);
static void conn_rx_success_callback(void *conn);
static void conn_tx_success_callback(void *conn);

static void enter_pairing(void);
static void prepare_pairing_request(void);
static void send_pairing_request(void);
static void wait_for_pairing_request(void);
static void prepare_pairing_response(void);
static void send_pairing_response(void);
static void wait_for_pairing_response(void);
static void prepare_pairing_confirmed(void);
static void wait_for_pairing_confirmed(void);
static void wait_for_pairing_confirmed_ack(void);
static void send_pairing_confirmed(void);
static void pairing_complete(void);
static void pairing_execute_state_function(uint8_t state);

static uint64_t get_radio_chip_id(void);
static uint32_t generate_serialized_address(void);
static uint8_t get_available_node_id(uint16_t coordinator_address);
static void add_node_to_paired_devices_list(uint8_t index, uint16_t address, uint64_t unique_id);
static void apply_new_state(pairing_state_t pairing_state);

static state_machine_function_link_t pairing_state_machine_function_table[] = {
    {(uint8_t)PAIRING_STATE_ENTER_PAIRING, enter_pairing},
    {(uint8_t)PAIRING_STATE_PREPARE_PAIRING_REQUEST, prepare_pairing_request},
    {(uint8_t)PAIRING_STATE_SEND_PAIRING_REQUEST, send_pairing_request},
    {(uint8_t)PAIRING_STATE_WAIT_FOR_PAIRING_REQUEST, wait_for_pairing_request},
    {(uint8_t)PAIRING_STATE_PREPARE_PAIRING_RESPONSE, prepare_pairing_response},
    {(uint8_t)PAIRING_STATE_SEND_PAIRING_RESPONSE, send_pairing_response},
    {(uint8_t)PAIRING_STATE_WAIT_FOR_PAIRING_RESPONSE, wait_for_pairing_response},
    {(uint8_t)PAIRING_STATE_PREPARE_PAIRING_CONFIRMED, prepare_pairing_confirmed},
    {(uint8_t)PAIRING_STATE_SEND_PAIRING_CONFIRMED, send_pairing_confirmed},
    {(uint8_t)PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED, wait_for_pairing_confirmed},
    {(uint8_t)PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED_ACK, wait_for_pairing_confirmed_ack},
    {(uint8_t)PAIRING_STATE_PAIRING_COMPLETE, pairing_complete},
};

/* PUBLIC FUNCTIONS ***********************************************************/
void swc_pairing_init_pairing_process(swc_pairing_t *pairing, swc_hal_t *hal)
{
    /* Set pairing channels with the desired region. */
    switch (pairing->uwb_regulation) {
    case SWC_REGULATION_FCC:
    case SWC_REGULATION_ETSI:
        channel_frequency = channel_frequency_fcc_etsi;
        break;
    case SWC_REGULATION_ARIB:
        channel_frequency = channel_frequency_arib;
        break;
    default:
        break;
    }

    /* Get the pairing handle from the application. */
    app_pairing = pairing;
    if (app_pairing->device_role == SWC_ROLE_COORDINATOR) {
        app_pairing->coordinator_address = 0x0;  /* Uninitialized coordinator address. */
        app_pairing->node_address        = 0x0F; /* Does not match node address on purpose. */
    } else {
        app_pairing->coordinator_address = 0x0F; /* Does not match coordinator address on purpose. */
        app_pairing->node_address        = 0x00; /* Uninitialized node address. */
    }

    /* Create a local pairing instance. */
    local_pairing = *app_pairing;
    local_pairing.pan_id = 0;

    /* Store SWC HAL pointer. */
    swc_hal = hal;

    apply_new_state(PAIRING_STATE_ENTER_PAIRING);

    /* Reconfigure the Wireless Core with the pairing app. */
    reconfigure_swc_addresses();

    received_pairing_command = PAIRING_COMMAND_NONE;

    is_pairing_success = false;
}

bool swc_pairing_process(void)
{
    if (current_pairing_state != PAIRING_STATE_EXIT_PAIRING) {
        pairing_execute_state_function(current_pairing_state);
    }

    return is_pairing_success;
}

void swc_pairing_deinit(void)
{
    swc_error_t err;

    memset(&local_pairing, 0, sizeof(swc_pairing_t));

    /* Free the memory before returning to the application. */
    swc_disconnect(&err);
    swc_free_memory();

    is_pairing_success = false;

    apply_new_state(PAIRING_STATE_EXIT_PAIRING);
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[in]  hal  Board specific functions.
 *  @param[out] err  Wireless Core error code.
 */
static void swc_pairing_core_init(swc_error_t *err)
{
    uint16_t local_address;

    swc_cfg_t core_cfg = {
        .timeslot_sequence               = timeslot_us,
        .timeslot_sequence_length        = SWC_ARRAY_SIZE(timeslot_us),
        .channel_sequence                = channel_sequence,
        .channel_sequence_length         = SWC_ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled               = false,
        .random_channel_sequence_enabled = false,
        .memory_pool                     = app_pairing->memory_pool,
        .memory_pool_size                = app_pairing->memory_pool_size
    };
    swc_init(core_cfg, swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    /* Update the addresses depending on the role. */
    if (local_pairing.network_role == SWC_ROLE_COORDINATOR) {
        local_address = PAIRING_COORD_ADDRESS;
    } else {
        local_address = PAIRING_NODE_ADDRESS;
    }

    swc_node_cfg_t node_cfg = {
        .role                = local_pairing.network_role,
        .pan_id              = 0xBCD,
        .coordinator_address = PAIRING_COORD_ADDRESS,
        .local_address       = local_address,
        .sleep_level         = PAIRING_SWC_SLEEP_LEVEL
    };
    node = swc_node_init(node_cfg, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_radio_cfg_t radio_cfg = {
        .irq_polarity = SWC_IRQ_ACTIVE_HIGH,
        .spi_mode     = SWC_SPI_FAST,
#ifdef SR1100
        .outimped     = SWC_OUTIMPED_2,
#endif
        .no_reset = false,
        .no_calibration = false
    };
    swc_node_add_radio(node, radio_cfg, swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }
#if (WPS_RADIO_COUNT == 2)
    swc_node_add_radio(node, radio_cfg, swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }
#endif

    /* Coordinator to node connection. */
    swc_connection_cfg_t coord_to_node_conn_cfg = {
        .name                       = "Coord to Node Connection",
        .source_address             = PAIRING_COORD_ADDRESS,
        .destination_address        = PAIRING_NODE_ADDRESS,
        .max_payload_size           = 9,
        .queue_size                 = PAIRING_DATA_QUEUE_SIZE,
        .modulation                 = PAIRING_SWC_MODULATION,
        .fec                        = PAIRING_SWC_FEC_LEVEL,
        .timeslot_id                = coord_to_node_timeslots,
        .timeslot_count             = SWC_ARRAY_SIZE(coord_to_node_timeslots),
        .ack_enabled                = true,
        .arq_enabled                = true,
        .arq_settings.try_count     = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled          = true,
#if defined(SR1000)
        .cca_enabled                = true,
#elif defined(SR1100)
        .cca_enabled                = false,
#endif
        .cca_settings.fail_action   = SWC_CCA_ABORT_TX,
        .cca_settings.threshold     = 25,
        .cca_settings.retry_time    = 962,
        .cca_settings.try_count     = 10,
        .throttling_enabled         = false,
        .rdo_enabled                = false,
        .fallback_enabled           = false,
    };
    coord_to_node_conn = swc_connection_init(node, coord_to_node_conn_cfg, swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    /* Node to coordinator connection. */
    swc_connection_cfg_t node_to_coord_conn_cfg = {
        .name                       = "Node to Coord Connection",
        .source_address             = PAIRING_NODE_ADDRESS,
        .destination_address        = PAIRING_COORD_ADDRESS,
        .max_payload_size           = 9,
        .queue_size                 = PAIRING_DATA_QUEUE_SIZE,
        .modulation                 = PAIRING_SWC_MODULATION,
        .fec                        = PAIRING_SWC_FEC_LEVEL,
        .timeslot_id                = node_to_coord_timeslots,
        .timeslot_count             = SWC_ARRAY_SIZE(node_to_coord_timeslots),
        .ack_enabled                = true,
        .arq_enabled                = true,
        .arq_settings.try_count     = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled          = true,
#if defined(SR1000)
        .cca_enabled                = true,
#elif defined(SR1100)
        .cca_enabled                = false,
#endif
        .cca_settings.fail_action   = SWC_CCA_ABORT_TX,
        .cca_settings.threshold     = 25,
        .cca_settings.retry_time    = 962,
        .cca_settings.try_count     = 10,
        .throttling_enabled         = false,
        .rdo_enabled                = false,
        .fallback_enabled           = false,
    };
    node_to_coord_conn = swc_connection_init(node, node_to_coord_conn_cfg, swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_channel_cfg = {
        .tx_pulse_count = PAIRING_TX_DATA_PULSE_COUNT,
        .tx_pulse_width = PAIRING_TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = PAIRING_TX_DATA_PULSE_GAIN,
        .rx_pulse_count = PAIRING_RX_ACK_PULSE_COUNT
    };

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = PAIRING_TX_ACK_PULSE_COUNT,
        .tx_pulse_width = PAIRING_TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = PAIRING_TX_ACK_PULSE_GAIN,
        .rx_pulse_count = PAIRING_RX_DATA_PULSE_COUNT
    };

    /* Add the channels to the connections and set the connections callbacks. */
    if (local_pairing.network_role == SWC_ROLE_COORDINATOR) {
        for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
            tx_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(coord_to_node_conn, node, tx_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }

            rx_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(node_to_coord_conn, node, rx_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }
        }
        swc_connection_set_rx_success_callback(node_to_coord_conn, conn_rx_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
        swc_connection_set_tx_success_callback(coord_to_node_conn, conn_tx_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    } else {
        for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
            tx_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(node_to_coord_conn, node, tx_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }

            rx_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(coord_to_node_conn, node, rx_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }
        }
        swc_connection_set_rx_success_callback(coord_to_node_conn, conn_rx_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
        swc_connection_set_tx_success_callback(node_to_coord_conn, conn_tx_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    swc_setup(node, err);
}

/** @brief Reconfigure SWC addresses for pairing mode.
 *
 *  @param[in] hal  Board specific functions.
 */
static void reconfigure_swc_addresses(void)
{
    swc_error_t err;

    swc_disconnect(&err);
    swc_free_memory();

    swc_pairing_core_init(&err);
    if (err != SWC_ERR_NONE) {
        while (1);
    }

    swc_connect(&err);
    if (err != SWC_ERR_NONE) {
        while (1);
    }
}

/** @brief Callback function when a previously sent frame has been ACK'd.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_success_callback(void *conn)
{
    (void)conn;

    /* Used by the pairing procedure to ensure that the message was sent successfully. */
    if (current_pairing_state == PAIRING_STATE_SEND_PAIRING_RESPONSE) {
        apply_new_state(PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED);
    } else if (current_pairing_state == PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED_ACK) {
        apply_new_state(PAIRING_STATE_PAIRING_COMPLETE);
    }
}

/** @brief Callback function when a frame has been successfully received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_success_callback(void *conn)
{
    swc_error_t err;
    uint8_t *payload = NULL;
    uint8_t payload_size = 0;

    payload_size = swc_connection_receive(conn, &payload, &err);

    /* The receiving frame validates that it's a pairing message and extracts the pairing command and payload. */
    if ((payload[PAIRING_BYTE_CODE_0] == EXTRACT_BYTE(PAIRING_CODE, 1)) &&
        (payload[PAIRING_BYTE_CODE_1] == EXTRACT_BYTE(PAIRING_CODE, 0))) {

        received_pairing_command = payload[PAIRING_BYTE_COMMAND];
        memcpy(received_payload, payload, payload_size);
    }

    /* Notify the SWC that the new payload has been read. */
    swc_connection_receive_complete(conn, &err);
}

/** @brief This step directs the next step depending on the SWC role.
 */
static void enter_pairing(void)
{
    uint32_t generated_address;

    if (local_pairing.network_role == SWC_ROLE_COORDINATOR) {

        /* Generate a PAN ID if not configured. */
        if (local_pairing.pan_id == 0) {
            generated_address = generate_serialized_address();
            local_pairing.pan_id = generated_address >> 8;
            local_pairing.coordinator_address = (uint8_t)generated_address;
        }

        /* Add the coordinator to the pairing list. */
        add_node_to_paired_devices_list(0, local_pairing.coordinator_address, get_radio_chip_id());

        apply_new_state(PAIRING_STATE_PREPARE_PAIRING_REQUEST);
    } else {
        apply_new_state(PAIRING_STATE_WAIT_FOR_PAIRING_REQUEST);
    }
}

/** @brief Used by the coordinator to prepare a pairing request message.
 */
static void prepare_pairing_request(void)
{
    uint16_t pan_id;
    uint8_t coordinator_id;
    uint8_t available_node_id;

    pan_id = local_pairing.pan_id;
    coordinator_id = local_pairing.coordinator_address;
    available_node_id = get_available_node_id(local_pairing.coordinator_address);

    /* Store the available node address. */
    local_pairing.node_address = available_node_id;

    /* Prepare the coordinator pairing request message. */
    pairing_request_message.pairing_code_msb = EXTRACT_BYTE(PAIRING_CODE, 1);
    pairing_request_message.pairing_code_lsb = EXTRACT_BYTE(PAIRING_CODE, 0);
    pairing_request_message.pairing_command  = (uint8_t)PAIRING_COMMAND_SENT_REQUEST;
    pairing_request_message.pan_id_msb       = EXTRACT_BYTE(pan_id, 1);
    pairing_request_message.pan_id_lsb       = EXTRACT_BYTE(pan_id, 0);
    pairing_request_message.coordinator_id   = coordinator_id;
    pairing_request_message.node_id          = available_node_id;

    apply_new_state(PAIRING_STATE_SEND_PAIRING_REQUEST);
}

/** @brief Used by the coordinator to send a pairing request to the node.
 *
 *  Since the Broadcast message can't use the Acknowledge feature,
 *  the states are used to resend the Broadcast message.
 */
static void send_pairing_request(void)
{
    swc_error_t err;

    swc_connection_send(coord_to_node_conn, (uint8_t *)&pairing_request_message, sizeof(pairing_request_message), &err);

    apply_new_state(PAIRING_STATE_WAIT_FOR_PAIRING_RESPONSE);
}

/** @brief Used by the node to wait for a pairing request from the coordinator.
 *
 *  The node knows the PAN ID, the coordinator ID and it assigned node ID
 *  once it has received the pairing request. It reconfigures the SWC
 *  with the coordinator's address and reconfigures its own address.
 */
static void wait_for_pairing_request(void)
{
    uint16_t pan_id;
    uint8_t coordinator_id;
    uint8_t node_device_id;

    /* Give the coordinator device some time to send request. */
    swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);

    /* Node received the broadcast. */
    if (received_pairing_command == PAIRING_COMMAND_SENT_REQUEST) {
        received_pairing_command = PAIRING_COMMAND_NONE;

        /* Store the received payload bytes. */
        pan_id = SWC_CONCAT_8B_TO_16B(received_payload[PAIRING_REQUEST_BYTE_PAN_ID_MSB],
                                      received_payload[PAIRING_REQUEST_BYTE_PAN_ID_LSB]);
        coordinator_id = received_payload[PAIRING_REQUEST_BYTE_COORD_ID];
        node_device_id = received_payload[PAIRING_REQUEST_BYTE_NODE_ID];

        /* Reconfigure the coordinator and node addresses once they have been learned. */
        local_pairing.pan_id = pan_id;
        local_pairing.coordinator_address = coordinator_id;
        local_pairing.node_address = node_device_id;

        /* Prepare the node to respond. */
        apply_new_state(PAIRING_STATE_PREPARE_PAIRING_RESPONSE);
    }
}

/** @brief Used by the node to prepare a pairing response message after receiving a pairing request.
 */
static void prepare_pairing_response(void)
{
    uint8_t device_role;
    uint64_t unique_id;

    device_role = local_pairing.device_role;
    unique_id = get_radio_chip_id();

    /* Prepare the node pairing response message. */
    pairing_response_message.pairing_code_msb = EXTRACT_BYTE(PAIRING_CODE, 1);
    pairing_response_message.pairing_code_lsb = EXTRACT_BYTE(PAIRING_CODE, 0);
    pairing_response_message.pairing_command  = (uint8_t)PAIRING_COMMAND_SENT_RESPONSE;
    pairing_response_message.device_role      = device_role;
    pairing_response_message.unique_id_0      = EXTRACT_BYTE(unique_id, 4);
    pairing_response_message.unique_id_1      = EXTRACT_BYTE(unique_id, 3);
    pairing_response_message.unique_id_2      = EXTRACT_BYTE(unique_id, 2);
    pairing_response_message.unique_id_3      = EXTRACT_BYTE(unique_id, 1);
    pairing_response_message.unique_id_4      = EXTRACT_BYTE(unique_id, 0);

    apply_new_state(PAIRING_STATE_SEND_PAIRING_RESPONSE);
}

/** @brief Used by the node to send a pairing response message to the coordinator.
 */
static void send_pairing_response(void)
{
    swc_error_t err;

    /* Sending response message. */
    swc_connection_send(node_to_coord_conn, (uint8_t *)&pairing_response_message, sizeof(pairing_response_message), &err);
    swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);
}

/** @brief Used by the coordinator to wait for a pairing response from the node.
 *
 *  The coordinator knows the node's application role and its Unique ID
 *  once it has received the pairing response. The coordinator will save this
 *  information in its pairing list.
 */
static void wait_for_pairing_response(void)
{
    uint64_t unique_id;

    /* Give the node device some time to receive the sent request. */
    swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);

    if (received_pairing_command == PAIRING_COMMAND_SENT_RESPONSE) {
        received_pairing_command = PAIRING_COMMAND_NONE;

        /* Store the device application role. */
        local_pairing.device_role = received_payload[PAIRING_RESPONSE_BYTE_DEVICE_ROLE];

        /* Store the unique ID. */
        unique_id = (((uint64_t)(received_payload[PAIRING_RESPONSE_BYTE_UNIQUE_ID_0]) << (8 * 4)) |
                     ((uint64_t)(received_payload[PAIRING_RESPONSE_BYTE_UNIQUE_ID_1]) << (8 * 3)) |
                     ((uint64_t)(received_payload[PAIRING_RESPONSE_BYTE_UNIQUE_ID_2]) << (8 * 2)) |
                     ((uint64_t)(received_payload[PAIRING_RESPONSE_BYTE_UNIQUE_ID_3]) << (8 * 1)) |
                     ((uint64_t)(received_payload[PAIRING_RESPONSE_BYTE_UNIQUE_ID_4])));

        /* Add the node to the pairing device list. */
        add_node_to_paired_devices_list(local_pairing.device_role,
                                        local_pairing.node_address,
                                        unique_id);

        apply_new_state(PAIRING_STATE_PREPARE_PAIRING_CONFIRMED);

    } else {
        /* Resend the pairing request message since no ACK is possible with a broadcast. */
        apply_new_state(PAIRING_STATE_SEND_PAIRING_REQUEST);
    }
}

/** @brief Used by the coordinator to prepare the pairing confirmed message.
 */
static void prepare_pairing_confirmed(void)
{
    /* Prepare the response payload, sent on the SWC. */
    pairing_confirmed_message.pairing_code_msb = EXTRACT_BYTE(PAIRING_CODE, 1);
    pairing_confirmed_message.pairing_code_lsb = EXTRACT_BYTE(PAIRING_CODE, 0);
    pairing_confirmed_message.pairing_command  = (uint8_t)PAIRING_COMMAND_SENT_CONFIRMATION;

    apply_new_state(PAIRING_STATE_SEND_PAIRING_CONFIRMED);
}

/** @brief Used by the coordinator to notify the node it is paired.
 */
static void send_pairing_confirmed(void)
{
    swc_error_t err;

    swc_connection_send(coord_to_node_conn, (uint8_t *)&pairing_confirmed_message, sizeof(pairing_confirmed_message), &err);
    swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);

    apply_new_state(PAIRING_STATE_WAIT_FOR_PAIRING_CONFIRMED_ACK);
}

/** @brief Used by the coordinator to wait for the node's acknowledge.
 */
static void wait_for_pairing_confirmed_ack(void)
{
    swc_error_t err;

    swc_connection_send(coord_to_node_conn, (uint8_t *)&pairing_confirmed_message, sizeof(pairing_confirmed_message), &err);
    swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);
}

/** @brief Used by the node to wait for the coordinator's pairing confirmation.
 *
 *  The node is successfully paired when the confirmation is received.
 */
static void wait_for_pairing_confirmed(void)
{
    /* Give the coordinator device some time to receive the sent response. */
    swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);

    if (received_pairing_command == PAIRING_COMMAND_SENT_CONFIRMATION) {
        received_pairing_command = PAIRING_COMMAND_NONE;

        /* Give the node device some time to finish sending acks. */
        swc_hal->radio_hal->delay_ms(PAIRING_WAIT_DELAY_MS);

        apply_new_state(PAIRING_STATE_PAIRING_COMPLETE);
    }
}

/** @brief The device is paired and is ready to start the application.
 *
 *  This function is used as a transition between the pairing Process and
 *  the user application.
 */
static void pairing_complete(void)
{
    swc_error_t err;

    app_pairing->pan_id = local_pairing.pan_id;
    app_pairing->coordinator_address = local_pairing.coordinator_address;
    app_pairing->node_address = local_pairing.node_address;
    app_pairing->device_role = local_pairing.device_role;
    memcpy(app_pairing->paired_device, &local_pairing.paired_device, sizeof(local_pairing.paired_device));

    memset(&local_pairing, 0, sizeof(swc_pairing_t));

    /* Free the memory before returning to the application. */
    swc_disconnect(&err);
    swc_free_memory();

    is_pairing_success = true;

    apply_new_state(PAIRING_STATE_EXIT_PAIRING);
}

/** @brief Execute the current state machine function.
 *
 *  @param[in] state  The state to be processed.
 */
static void pairing_execute_state_function(uint8_t state)
{
    uint8_t i = 0;
    bool found = false;
    uint8_t table_len = sizeof(pairing_state_machine_function_table) / sizeof(state_machine_function_link_t);

    while (!found && i < table_len) {
        if (pairing_state_machine_function_table[i].state == state) {
            found = true;
            if (pairing_state_machine_function_table[i].fp_state_machine_callback != NULL) {
                pairing_state_machine_function_table[i].fp_state_machine_callback();
            }
        }
        i++;
    }
}

/** @brief Get the radio chip ID.
 *
 *  @return The device's radio chip ID.
 */
static uint64_t get_radio_chip_id(void)
{
    return node->wps_radio_handle->spectral_calib_vars->chip_id;
}

/** @brief Generate an address from the SPARK radio's chip ID.
 *
 *  @return The device's serialized address.
 */
static uint32_t generate_serialized_address(void)
{
    uint8_t byte_array[GENERATE_SERIALIZED_LEN];
    uint64_t seed = 0;
    uint32_t result = GENERATE_SERIALIZED_CRC_CCITT_RELOAD;
    uint32_t crc = 0;
    uint8_t result_syncword;
    uint8_t result_network;
    uint8_t result_address;
    bool result_is_valid = false;

    /* Generate a seed from the SPARK radio's chip ID. */
    seed = get_radio_chip_id();

    do {
        /* Split it into bytes in a table. */
        memcpy(byte_array, &seed, GENERATE_SERIALIZED_LEN);

        for (uint8_t i = 0; i < GENERATE_SERIALIZED_LEN; i++) {
            crc = result;
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80000) {  /* Most significant byte. */
                    crc = (crc << 1) ^ GENERATE_SERIALIZED_CRC_POLY;
                } else {
                    crc <<= 1;
                }
            }
            result = crc ^ byte_array[i];
        }

        /* Only keep 3 bytes for the PAN ID (12-bits) and the coordinator address (8bits). */
        result = result & 0xFFFFF;

        result_syncword = EXTRACT_BYTE(result, 2);
        result_network  = EXTRACT_BYTE(result, 1);
        result_address  = EXTRACT_BYTE(result, 0);

        /* Verify if the result lands on a reserved address, if so generate new seed. */
        if (result_syncword == 0x00 || result_syncword == 0x0F ||
            result_network == 0x00 || result_network == 0xFF ||
            result_address == 0x00 || result_address == 0xFF) {
            result_is_valid = false;
            seed += 1;
        } else {
            result_is_valid = true;
        }

    } while (!result_is_valid);

    return result;
}

/** @brief Look in the paired device list for an available Device ID.
 *
 *  It first increments the coordinator ID then looks into the list
 *  if this node ID is available. If not, the ID will increment until one
 *  is available.
 *
 *  @param[in] coordinator_address  The base address used to find an available address.
 *  @return An available node ID.
 */
static uint8_t get_available_node_id(uint16_t coordinator_address)
{
    bool address_is_available;
    uint8_t result_id;

    result_id = ((uint8_t)coordinator_address) + 1;

    do {
        address_is_available = true;

        for (uint8_t i = 0; i < SWC_PAIRING_DEVICE_LIST_MAX_COUNT; i++) {
            if (result_id == ((uint8_t)local_pairing.paired_device[i].node_address)) {
                address_is_available = false;
            }
        }

        /* If the address already exists, increment again. */
        if (!address_is_available) {
            if (result_id == 0xFE) {
                /* Prevent changing the result to a reserved address. */
                result_id = 0x01;
            } else {
                result_id++;
            }
        }
    } while (!address_is_available);

    return result_id;
}

/** @brief Add a node into the paired device list.
 *
 *  @param[in] index      The paired device's position in the list.
 *  @param[in] address    The address stored in the list.
 *  @param[in] unique_id  The Unique ID stored in the list.
 */
static void add_node_to_paired_devices_list(uint8_t index, uint16_t address, uint64_t unique_id)
{
    if (index < SWC_PAIRING_DEVICE_LIST_MAX_COUNT) {
        /* Add the address and unique ID to the paired device list. */
        local_pairing.paired_device[index].node_address = address;
        local_pairing.paired_device[index].unique_id = unique_id;
    }
}

/** @brief Wrapper function to apply a new state.
 *
 *  @param[in] pairing_state  The new effective pairing state.
 */
static void apply_new_state(pairing_state_t pairing_state)
{
    current_pairing_state = pairing_state;
}