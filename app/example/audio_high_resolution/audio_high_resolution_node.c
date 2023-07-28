/** @file  audio_high_resolution_node.c
 *  @brief This application creates a unidirectional uncompressed high resolution
 *         audio stream from the MAX98091 audio codec Line-In of the coordinator to the
 *         Headphone-Out of the node.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES ******************************************************************/
#include <stdio.h>
#include "sac_packing.h"
#include "iface_audio_high_resolution.h"
#include "iface_audio.h"
#include "iface_wireless.h"
#include "sac_api.h"
#include "sac_cdc.h"
#include "sac_stats.h"
#include "sac_volume.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SAC_MEM_POOL_SIZE         9000
#define SAC_PRODUCER_PAYLOAD_SIZE 78
#define SAC_CONSUMER_PAYLOAD_SIZE 104 /* 24-bit samples in 32-bit words */
#define SAC_LATENCY_QUEUE_SIZE    32
#define SWC_MEM_POOL_SIZE         3700
#define STATS_ARRAY_LENGTH        1500

/* PRIVATE GLOBALS ************************************************************/
/* ** Audio Core ** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
static sac_hal_t sac_hal;
static sac_pipeline_t *sac_pipeline;

static sac_volume_instance_t volume_instance;
static sac_processing_t *volume_processing;

static sac_packing_instance_t audio_packing_instance;
static sac_processing_t *sac_packing_processing;

static sac_cdc_instance_t cdc_instance;
static sac_processing_t *cdc_processing;

static sac_endpoint_t *max98091_consumer;

static ep_swc_instance_t swc_producer_instance;
static sac_endpoint_t *swc_producer;

/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *rx_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_timeslots[] = RX_TIMESLOTS;

/* ** Application Specific ** */
static volatile bool print_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_rx_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void app_audio_core_volume_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_packing_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_cdc_interface_init(sac_processing_interface_t *iface);
static void audio_i2s_tx_complete_cb(void);

static void volume_up(void);
static void volume_down(void);

static void audio_process_callback(void);
static void print_stats(void);
static void stats_callback(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    swc_error_t swc_err;
    sac_error_t sac_err;

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    app_audio_core_init(&sac_err);
    if (sac_err != SAC_ERR_NONE) {
        while (1);
    }

    iface_audio_node_init();

    sac_pipeline_start(sac_pipeline, &sac_err);

    swc_connect(&swc_err);

    iface_audio_process_timer_init(100);
    iface_audio_process_set_timer_callback(audio_process_callback);
    iface_audio_process_timer_start();

    iface_stats_timer_init(1000);
    iface_stats_set_timer_callback(stats_callback);
    iface_stats_timer_start();

    while (1) {
        if (print_stats_now) {
            print_stats();
            print_stats_now = false;
        }
        iface_button_handling(volume_up, volume_down);
    }

    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[out] err  Wireless Core error code.
 */
static void app_swc_core_init(swc_error_t *err)
{
    iface_swc_hal_init(&swc_hal);
    iface_swc_handlers_init();

    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE
    };
    swc_init(core_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = PAN_ID,
        .coordinator_address = COORDINATOR_ADDRESS,
        .local_address = LOCAL_ADDRESS,
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
    swc_node_add_radio(node, radio_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    /* ** RX Connection ** */
    swc_connection_cfg_t rx_conn_cfg = {
        .name = "RX Connection",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = SAC_PRODUCER_PAYLOAD_SIZE + sizeof(sac_header_t),
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    rx_conn = swc_connection_init(node, rx_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_conn, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_conn, conn_rx_success_callback, err);

    swc_setup(node, err);
}

/** @brief Callback function when a frame has been successfully received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_success_callback(void *conn)
{
    (void)conn;
    sac_error_t sac_err;

    iface_rx_conn_status();

    sac_pipeline_produce(sac_pipeline, &sac_err);
}

/** @brief Initialize the Audio Core.
 *
 *  @param[out] err  Audio Core error code.
 */
static void app_audio_core_init(sac_error_t *sac_err)
{
    sac_endpoint_interface_t max98091_consumer_iface;
    sac_endpoint_interface_t swc_producer_iface;
    sac_processing_interface_t volume_iface;
    sac_processing_interface_t packing_iface;
    sac_processing_interface_t cdc_iface;

    iface_sac_hal_init(&sac_hal);
    iface_audio_swc_endpoint_init(&swc_producer_iface, NULL);
    iface_audio_max98091_endpoint_init(NULL, &max98091_consumer_iface);
    iface_set_sai_complete_callback(audio_i2s_tx_complete_cb, NULL);

    app_audio_core_volume_interface_init(&volume_iface);
    app_audio_core_packing_interface_init(&packing_iface);
    app_audio_core_cdc_interface_init(&cdc_iface);

    swc_producer_instance.connection = rx_conn;

    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE
    };
    sac_init(core_cfg, &sac_hal, sac_err);

    /*
     * Audio Pipeline
     * ==============
     *
     * Input:      Mono stream of 26 samples @ 96 kHz/24 bits is received over the air from the Coordinator.
     * Processing: Unpacking from 24 bits aligned to 32 bits aligned audio samples.
     * Processing: Digital volume control followed by clock drift compensation.
     * Output:     Mono stream of 26 samples @ 96 kHz/24 bits (stored in 32-bit words) is played using the MAX98091 hardware audio codec.
     *
     * +-----+    +----------------+    +-----+    +-------+
     * | SWC | -> | Digital Volume | -> | CDC | -> | Codec |
     * +-----+    +----------------+    +-----+    +-------+
     */
    sac_endpoint_cfg_t swc_producer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_24BITS,
        .audio_payload_size = SAC_PRODUCER_PAYLOAD_SIZE,
        .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE,
    };
    swc_producer = sac_endpoint_init((void *)&swc_producer_instance, "SWC EP (Producer)",
                                     swc_producer_iface, swc_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    audio_packing_instance.packing_mode = SAC_UNPACK_24BITS;
    sac_packing_processing = sac_processing_stage_init((void *)&audio_packing_instance, "Audio Unpacking", packing_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    volume_instance.initial_volume_level = 100;
    volume_instance.bit_depth = SAC_24BITS;
    volume_processing = sac_processing_stage_init((void *)&volume_instance, "Digital Volume Control", volume_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    cdc_instance.cdc_resampling_length = 1440;
    cdc_instance.cdc_queue_avg_size = 1000;
    cdc_processing = sac_processing_stage_init((void *)&cdc_instance, "CDC", cdc_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t max98091_consumer_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = true,
        .channel_count      = 1,
        .bit_depth          = SAC_24BITS,
        .audio_payload_size = SAC_CONSUMER_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };
    max98091_consumer = sac_endpoint_init(NULL, "MAX98091 EP (Consumer)",
                                          max98091_consumer_iface, max98091_consumer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t pipeline_cfg = {
        .cdc_enable = true,
        .do_initial_buffering = false,
    };
    sac_pipeline = sac_pipeline_init("SWC -> Codec", swc_producer,
                                       pipeline_cfg, max98091_consumer, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline, sac_packing_processing, sac_err);
    sac_pipeline_add_processing(sac_pipeline, volume_processing, sac_err);
    sac_pipeline_add_processing(sac_pipeline, cdc_processing, sac_err);

    sac_pipeline_setup(sac_pipeline, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }
}

/** @brief Initialize the digital volume control audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_volume_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_volume_init;
    iface->ctrl = sac_volume_ctrl;
    iface->process = sac_volume_process;
    iface->gate = NULL;
}

/** @brief Initialize the audio packing processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_packing_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_packing_init;
    iface->ctrl = sac_packing_ctrl;
    iface->process = sac_packing_process;
    iface->gate = NULL;
}

/** @brief Initialize the clock drift compensation audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_cdc_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_cdc_init;
    iface->ctrl = sac_cdc_ctrl;
    iface->process = sac_cdc_process;
    iface->gate = NULL;
}

/** @brief Increase the audio output volume level.
 *
 *  This affects the audio pipeline the digital volume
 *  processing stage is added to.
 */
static void volume_up(void)
{
    sac_error_t err;

    sac_processing_ctrl(volume_processing, sac_pipeline, SAC_VOLUME_INCREASE, SAC_NO_ARG, &err);
}

/** @brief Decrease the audio output volume level.
 *
 *  This affects the audio pipeline the digital volume
 *  processing stage is added to.
 */
static void volume_down(void)
{
    sac_error_t err;

    sac_processing_ctrl(volume_processing, sac_pipeline, SAC_VOLUME_DECREASE, SAC_NO_ARG, &err);
}

/** @brief SAI DMA TX complete callback.
 *
 *  This feeds the codec with audio packets. It needs to be
 *  executed every time a DMA transfer to the codec is completed
 *  in order to keep the audio playing.
 */
static void audio_i2s_tx_complete_cb(void)
{
    sac_error_t sac_err;

    sac_pipeline_consume(sac_pipeline, &sac_err);
}

/** @brief Callback handling the audio process that triggers with the app timer.
 */
static void audio_process_callback(void)
{
    sac_error_t sac_err;

    sac_pipeline_process(sac_pipeline, &sac_err);
}

/** @brief Print the audio and wireless statistics.
 */
static void print_stats(void)
{
    static char stats_string[STATS_ARRAY_LENGTH];
    int string_length = 0;

    const char *device_str = "\n<   NODE   >\n\r";
    const char *audio_stats_str = "\n<<  Audio Core Statistics  >>\n\r";
    const char *wireless_stats_str = "\n<<  Wireless Core Statistics  >>\n\r";

    /* Device Prelude */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

    /* Audio statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, audio_stats_str);
    sac_pipeline_update_stats(sac_pipeline);
    string_length += sac_pipeline_format_stats(sac_pipeline, stats_string + string_length, sizeof(stats_string) - string_length);

    /* Wireless statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, wireless_stats_str);
    swc_connection_update_stats(rx_conn);
    string_length += swc_connection_format_stats(rx_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);

    iface_print_string(stats_string);
}

/** @brief Callback handling when the stats have to be printed.
 */
static void stats_callback(void)
{
    print_stats_now = true;
}

