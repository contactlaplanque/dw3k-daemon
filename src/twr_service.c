/*
 * DS-TWR service layer shared by dw3k-daemon. It exposes role-agnostic
 * helpers so the daemon can switch between initiator/responder modes without
 * recompiling dedicated test binaries.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "twr_service.h"

/*============================================================================
 * Constants
 *============================================================================*/

/* Message types */
#define MSG_TYPE_POLL       0x21U
#define MSG_TYPE_RESPONSE   0x10U
#define MSG_TYPE_FINAL      0x23U
#define MSG_TYPE_REPORT     0x24U
#define MSG_TYPE_RESULT     0x42U
#define MSG_TYPE_STOP       0x50U
#define MSG_TYPE_ANY        0xFFU

/* Result status codes */
#define RESULT_STATUS_OK        0x00U
#define RESULT_STATUS_OUTLIER   0x01U
#define RESULT_STATUS_ERROR     0xFFU

/* Frame offsets */
#define FRAME_SEQ_OFFSET    2U
#define FRAME_TYPE_OFFSET   9U
#define FRAME_DATA_OFFSET   10U

/* Frame sizes (excluding FCS which is auto-appended) */
#define POLL_FRAME_LEN      10U
#define RESPONSE_FRAME_LEN  10U
#define FINAL_FRAME_LEN     10U
#define REPORT_FRAME_LEN    25U
#define RESULT_FRAME_LEN    18U
#define STOP_FRAME_LEN      10U

/* Timing - conservative for stable, accurate measurements */
#define RX_TIMEOUT_UUS          30000U  /* 30ms RX timeout - tested reliable */
#define TX_TO_TX_DELAY_US       3000U   /* 3ms delay between Final and Report - critical for accuracy */
#define MAX_STAGE_RETRIES       2U
#define MAX_WAIT_RETRIES        2U
#define FAST_RANGING_INTERVAL_US 30000U  /* 30ms - allows clean hardware reset between exchanges */
#define RESPONDER_IDLE_TIMEOUT_UUS 15000000U  /* 15s timeout after polling starts */
#define DEFAULT_WAIT_TIMEOUT_US (RX_TIMEOUT_UUS * 3U)  /* 90ms - generous for stable timing */

/* Physics */
#define SPEED_OF_LIGHT_M_S  299702547.0
#define DWT_TIME_UNITS      (1.0 / 499.2e6 / 128.0)

/* Default antenna delay (DWT units) */
#define DEFAULT_ANT_DLY     16385U

/* Filtering - use dynamic allocation to handle any measurement count */
#define MAX_MEASUREMENTS        1000U  /* Maximum samples we can handle */
#define OUTLIER_THRESHOLD_SIGMA  2.5   /* Reject samples beyond 2.5 sigma (~98.8% confidence, more aggressive) */

/* RX buffer */
#define RX_BUFFER_LEN       128U

/*============================================================================
 * Types
 *============================================================================*/

typedef enum {
    ROLE_INITIATOR,
    ROLE_RESPONDER
} role_t;

/* Simple container for raw measurements - filtering done separately */
typedef struct {
    double *samples;
    size_t count;
    size_t capacity;
} measurement_buffer_t;

static void init_frames(role_t role);
static void configure_device(role_t role);

/*============================================================================
 * Global State
 *============================================================================*/

static uint8_t g_rx_buffer[RX_BUFFER_LEN];
static uint8_t g_seq_num = 0;
static twr_service_config_t g_service_cfg = {
    .pan_id = 0xDECAu,
    .local_id = 0,
    .peer_id = 0,
    .antenna_delay = DEFAULT_ANT_DLY,
};

static bool g_log_enabled = false;
static role_t g_active_role = ROLE_INITIATOR;

void twr_service_set_logging(bool enable) {
    g_log_enabled = enable;
}

static void twr_log(bool force, const char *fmt, ...) {
    if (!g_log_enabled && !force) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
}

#define LOG_INFO(...) twr_log(false, __VA_ARGS__)
#define LOG_WARN(...) twr_log(true, __VA_ARGS__)

void twr_service_set_config(const twr_service_config_t *cfg) {
    if (!cfg) {
        return;
    }
    g_service_cfg = *cfg;
    g_seq_num = 0;
}

void twr_service_get_config(twr_service_config_t *cfg) {
    if (!cfg) {
        return;
    }
    *cfg = g_service_cfg;
}

void twr_service_prepare_frames(twr_role_t role) {
    role_t internal_role = (role == TWR_ROLE_RESPONDER) ? ROLE_RESPONDER : ROLE_INITIATOR;
    g_active_role = internal_role;
    init_frames(internal_role);
    configure_device(internal_role);
}

/* Frame templates (will be populated with actual addresses) */
static uint8_t g_poll_frame[POLL_FRAME_LEN];
static uint8_t g_response_frame[RESPONSE_FRAME_LEN];
static uint8_t g_final_frame[FINAL_FRAME_LEN];
static uint8_t g_report_frame[REPORT_FRAME_LEN];
static uint8_t g_result_frame[RESULT_FRAME_LEN];
static uint8_t g_stop_frame[STOP_FRAME_LEN];

/*============================================================================
 * Helper Functions
 *============================================================================*/

/* Convert carrier integrator reading to ppm clock offset on channel 9.
 * Positive ppm means local RX clock is slower than remote TX (per API doc). */
static double read_ppm_from_carrier_integrator_ch9(void) {
    int32_t ci = dwt_readcarrierintegrator();
    double freq_hz = (double)ci * FREQ_OFFSET_MULTIPLIER;
    double ppm = freq_hz * HERTZ_TO_PPM_MULTIPLIER_CHAN_9;
    return ppm;
}

/* Compute scaling ratio to convert responder times into initiator timebase */
static double clock_ratio_from_ppm(double ppm) {
    return 1.0 + (ppm * 1e-6);
}

/* Simple piecewise-linear bias table vs RSL (dBm). Values are placeholders;
 * tune with empirical calibration for your setup. */
typedef struct {
    double rssi_dbm;
    double bias_m;
} rsl_bias_point_t;

static const rsl_bias_point_t k_rsl_bias_table[] = {
    { -95.0, 0.000 },
    { -85.0, 0.000 },
    { -80.0, 0.005 },
    { -75.0, 0.010 },
    { -70.0, 0.020 },
    { -65.0, 0.030 },
};

static double estimate_range_bias_from_rssi(double rssi_dbm) {
    const size_t n = sizeof(k_rsl_bias_table) / sizeof(k_rsl_bias_table[0]);
    if (n == 0) return 0.0;
    if (rssi_dbm <= k_rsl_bias_table[0].rssi_dbm) return k_rsl_bias_table[0].bias_m;
    if (rssi_dbm >= k_rsl_bias_table[n - 1].rssi_dbm) return k_rsl_bias_table[n - 1].bias_m;
    for (size_t i = 1; i < n; i++) {
        if (rssi_dbm < k_rsl_bias_table[i].rssi_dbm) {
            double x0 = k_rsl_bias_table[i - 1].rssi_dbm;
            double y0 = k_rsl_bias_table[i - 1].bias_m;
            double x1 = k_rsl_bias_table[i].rssi_dbm;
            double y1 = k_rsl_bias_table[i].bias_m;
            double t = (rssi_dbm - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    return 0.0;
}

static void timestamp_to_bytes(uint64_t ts, uint8_t *bytes) {
    for (int i = 0; i < 5; i++) {
        bytes[i] = (uint8_t)(ts >> (i * 8));
    }
}

static uint64_t bytes_to_timestamp(const uint8_t *bytes) {
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++) {
        ts |= (uint64_t)bytes[i] << (i * 8);
    }
    return ts;
}

static uint64_t read_tx_timestamp(void) {
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);
    return bytes_to_timestamp(ts);
}

static uint64_t read_rx_timestamp(void) {
    uint8_t ts[5];
    dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
    return bytes_to_timestamp(ts);
}

static void int32_to_bytes(int32_t value, uint8_t *bytes) {
    for (int i = 0; i < 4; i++) {
        bytes[i] = (uint8_t)(value >> (i * 8));
    }
}

static int32_t bytes_to_int32(const uint8_t *bytes) {
    return (int32_t)((uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                     ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24));
}

/*============================================================================
 * Robust Statistical Filtering
 *============================================================================*/

/* Comparison function for qsort */
static int compare_doubles(const void *a, const void *b) {
    double diff = *(const double *)a - *(const double *)b;
    return (diff > 0) - (diff < 0);
}

/* Calculate median of array */
static double calculate_median(const double *data, size_t count) {
    if (count == 0) return 0.0;
    if (count == 1) return data[0];
    
    /* Create sorted copy */
    double *sorted = malloc(count * sizeof(double));
    memcpy(sorted, data, count * sizeof(double));
    qsort(sorted, count, sizeof(double), compare_doubles);
    
    double median;
    if (count % 2 == 0) {
        median = (sorted[count/2 - 1] + sorted[count/2]) / 2.0;
    } else {
        median = sorted[count/2];
    }
    
    free(sorted);
    return median;
}

/* Calculate mean */
static double calculate_mean(const double *data, size_t count) {
    if (count == 0) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < count; i++) {
        sum += data[i];
    }
    return sum / (double)count;
}

/* Calculate standard deviation */
static double calculate_stddev(const double *data, size_t count, double mean) {
    if (count < 2) return 0.0;
    double variance = 0.0;
    for (size_t i = 0; i < count; i++) {
        double diff = data[i] - mean;
        variance += diff * diff;
    }
    return sqrt(variance / (double)count);
}

/* Robust outlier rejection using iterative 3-sigma clipping */
static size_t robust_filter(const double *raw_data, size_t raw_count,
                           double *filtered_data, size_t max_filtered) {
    if (raw_count == 0) return 0;
    
    /* Start with all data */
    size_t filtered_count = (raw_count < max_filtered) ? raw_count : max_filtered;
    memcpy(filtered_data, raw_data, filtered_count * sizeof(double));
    
    /* Need at least 10 samples for meaningful statistics */
    if (raw_count < 10) return filtered_count;
    
    /* Iterative 3-sigma clipping (max 3 iterations) */
    for (int iteration = 0; iteration < 3; iteration++) {
        double mean = calculate_mean(filtered_data, filtered_count);
        double stddev = calculate_stddev(filtered_data, filtered_count, mean);
        
        /* If stddev is tiny, data is very clean - stop filtering */
        if (stddev < 0.001) break;
        
        double threshold = OUTLIER_THRESHOLD_SIGMA * stddev;
        
        /* Filter out outliers */
        size_t new_count = 0;
        for (size_t i = 0; i < filtered_count; i++) {
            if (fabs(filtered_data[i] - mean) <= threshold) {
                filtered_data[new_count++] = filtered_data[i];
            }
        }
        
        /* If no samples rejected, filtering converged */
        if (new_count == filtered_count) break;
        
        filtered_count = new_count;
        
        /* Don't over-filter - keep at least 60% of original data */
        if (filtered_count < (raw_count * 6 / 10)) break;
    }
    
    return filtered_count;
}

/*============================================================================
 * Frame TX/RX
 *============================================================================*/

static bool send_frame(uint8_t *frame, size_t len, uint64_t *tx_ts, bool rx_after) {
    dwt_forcetrxoff();
    dwt_writetxdata((uint16_t)len, frame, 0);
    dwt_writetxfctrl((uint16_t)(len + 2), 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        return false;
    }

    /* Wait for TX complete */
    for (int i = 0; i < 10000; i++) {
        uint32_t status = dwt_readsysstatuslo();
        if (status & DWT_INT_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            if (tx_ts) *tx_ts = read_tx_timestamp();
            if (rx_after) {
                dwt_forcetrxoff();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
            return true;
        }
        usleep(1);
    }
    return false;
}

static bool wait_for_frame(uint8_t expected_type, uint64_t *rx_ts, uint8_t *received_type, uint32_t timeout_us) {
    uint32_t elapsed = 0;

    while (timeout_us == 0 || elapsed < timeout_us) {
        uint32_t status = dwt_readsysstatuslo();

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            if (rx_ts) *rx_ts = read_rx_timestamp();
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            uint8_t rng = 0;
            uint16_t len = dwt_getframelength(&rng);
            if (len > 0 && len <= RX_BUFFER_LEN) {
                dwt_readrxdata(g_rx_buffer, len, 0);

                uint8_t frame_type = g_rx_buffer[FRAME_TYPE_OFFSET];
                bool type_matches = (expected_type == MSG_TYPE_ANY) || (frame_type == expected_type);
                if (!type_matches) {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }

                uint16_t src_addr = (uint16_t)g_rx_buffer[7] | ((uint16_t)g_rx_buffer[8] << 8);
                bool from_initiator = (frame_type == MSG_TYPE_POLL ||
                                       frame_type == MSG_TYPE_FINAL ||
                                       frame_type == MSG_TYPE_REPORT ||
                                       frame_type == MSG_TYPE_STOP);
                uint16_t expected_id = g_service_cfg.peer_id;
                if (expected_id == 0 || src_addr == expected_id) {
                    if (received_type) {
                        *received_type = frame_type;
                    }
                    return true;
                }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        if (status & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            /* Hardware RX timeout/error - not fatal if we have software time left */
            /* Re-enable RX and keep waiting until software timeout expires */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        usleep(100);
        elapsed += 100;
    }
    return false;
}

static void reset_rx_state(void) {
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void send_stop_signal(void) {
    g_stop_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
    if (!send_frame(g_stop_frame, STOP_FRAME_LEN, NULL, false)) {
        LOG_WARN("WARN: failed to send STOP frame\n");
    }
}

/*============================================================================
 * DS-TWR Initiator
 *============================================================================*/

int twr_service_run_initiator(uint32_t num_measurements, twr_measurement_stats_t *stats) {
    LOG_INFO("Starting TWR initiator run: %u samples\n", num_measurements);
    
    uint32_t exchange_count = 0;
    uint32_t success_count = 0;
    
    /* Allocate buffers for raw and filtered measurements */
    size_t max_samples = (num_measurements < MAX_MEASUREMENTS) ? num_measurements : MAX_MEASUREMENTS;
    double *raw_distances = malloc(max_samples * sizeof(double));
    double *filtered_distances = malloc(max_samples * sizeof(double));
    size_t raw_count = 0;

    while (exchange_count < num_measurements) {
        exchange_count++;
        
        uint64_t t1 = 0, t4 = 0, t5 = 0;

        /* [1] Send POLL and wait for RESPONSE (with retries) */
        {
            bool response_ok = false;
            for (uint32_t attempt = 0; attempt < MAX_STAGE_RETRIES; ++attempt) {
                g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
                dwt_setrxtimeout(RX_TIMEOUT_UUS);
                if (!send_frame(g_poll_frame, POLL_FRAME_LEN, &t1, true)) {
                    continue;
                }
                if (wait_for_frame(MSG_TYPE_RESPONSE, &t4, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                    response_ok = true;
                    break;
                }
                reset_rx_state();
            }
            if (!response_ok) {
                if (exchange_count < num_measurements) {
                    usleep(FAST_RANGING_INTERVAL_US);
                }
                continue;
            }
        }

        /* [2] Send FINAL */
        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        if (!send_frame(g_final_frame, FINAL_FRAME_LEN, &t5, false)) {
            if (exchange_count < num_measurements) {
                usleep(FAST_RANGING_INTERVAL_US);
            }
            continue;
        }

        /* Small delay to let responder switch to RX mode */
        usleep(TX_TO_TX_DELAY_US);

        /* [3] Send REPORT and wait for RESULT (with retries) */
        {
            bool result_ok = false;
            for (uint32_t attempt = 0; attempt < MAX_STAGE_RETRIES; ++attempt) {
                g_report_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
                timestamp_to_bytes(t1, &g_report_frame[FRAME_DATA_OFFSET]);
                timestamp_to_bytes(t4, &g_report_frame[FRAME_DATA_OFFSET + 5]);
                timestamp_to_bytes(t5, &g_report_frame[FRAME_DATA_OFFSET + 10]);
                dwt_setrxtimeout(RX_TIMEOUT_UUS);
                if (!send_frame(g_report_frame, REPORT_FRAME_LEN, NULL, true)) {
                    continue;
                }
                if (wait_for_frame(MSG_TYPE_RESULT, NULL, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                    result_ok = true;
                    break;
                }
                reset_rx_state();
            }
            if (!result_ok) {
                if (exchange_count < num_measurements) {
                    usleep(FAST_RANGING_INTERVAL_US);
                }
                continue;
            }
        }

        /* Parse result */
        double distance = (double)bytes_to_int32(&g_rx_buffer[FRAME_DATA_OFFSET]) / 1000.0;
        uint8_t status = g_rx_buffer[FRAME_DATA_OFFSET + 4];

        if (status == RESULT_STATUS_OK) {
            success_count++;
            /* Store raw distance */
            if (raw_count < max_samples) {
                raw_distances[raw_count++] = distance;
            }
        }

        /* Small delay between measurements */
        if (exchange_count < num_measurements) {
            usleep(FAST_RANGING_INTERVAL_US);
        }
    }

    /* Notify responder that measurements are complete */
    send_stop_signal();

    /* Process results */
    if (raw_count > 0) {
        /* Calculate raw statistics */
        double raw_mean = calculate_mean(raw_distances, raw_count);
        double raw_stddev = calculate_stddev(raw_distances, raw_count, raw_mean);
        double raw_median = calculate_median(raw_distances, raw_count);
        double raw_min = raw_distances[0], raw_max = raw_distances[0];
        for (size_t i = 1; i < raw_count; i++) {
            if (raw_distances[i] < raw_min) raw_min = raw_distances[i];
            if (raw_distances[i] > raw_max) raw_max = raw_distances[i];
        }
        
        /* Apply robust filtering */
        size_t filtered_count = robust_filter(raw_distances, raw_count, filtered_distances, max_samples);
        
        /* Calculate filtered statistics */
        double filt_mean = (filtered_count > 0) ? calculate_mean(filtered_distances, filtered_count) : 0.0;
        double filt_stddev = (filtered_count > 0) ? calculate_stddev(filtered_distances, filtered_count, filt_mean) : 0.0;
        double filt_median = (filtered_count > 0) ? calculate_median(filtered_distances, filtered_count) : 0.0;
        double filt_min = (filtered_count > 0) ? filtered_distances[0] : 0.0;
        double filt_max = (filtered_count > 0) ? filtered_distances[0] : 0.0;
        for (size_t i = 1; i < filtered_count; i++) {
            if (filtered_distances[i] < filt_min) filt_min = filtered_distances[i];
            if (filtered_distances[i] > filt_max) filt_max = filtered_distances[i];
        }
        
        if (stats) {
            stats->attempts = exchange_count;
            stats->successes = success_count;
            stats->raw_count = raw_count;
            stats->filtered_count = filtered_count;
            stats->rejected_count = raw_count - filtered_count;
            stats->raw_mean = raw_mean;
            stats->raw_stddev = raw_stddev;
            stats->raw_min = raw_min;
            stats->raw_max = raw_max;
            stats->raw_median = raw_median;
            stats->filtered_mean = filt_mean;
            stats->filtered_stddev = filt_stddev;
            stats->filtered_min = filt_min;
            stats->filtered_max = filt_max;
            stats->filtered_median = filt_median;
        }
        
        free(raw_distances);
        free(filtered_distances);
        return 0;
    }

    LOG_WARN("ERROR: No successful measurements\n");
    if (stats) {
        memset(stats, 0, sizeof(*stats));
        stats->attempts = exchange_count;
    }
    free(raw_distances);
    free(filtered_distances);
    return 1;
}

/*============================================================================
 * DS-TWR Responder
 *============================================================================*/

int twr_service_run_responder(uint32_t expected_polls) {
    if (expected_polls > 0) {
        LOG_INFO("Waiting for %u polls...\n", expected_polls);
    } else {
        LOG_INFO("Waiting for polls...\n");
    }
    
    uint32_t poll_count = 0;
    bool first_poll_received = false;

    while (true) {
        uint64_t t2 = 0, t3 = 0, t6 = 0;
        uint64_t t1 = 0, t4 = 0, t5 = 0;
        double ppm_on_poll = 0.0, ppm_on_final = 0.0;

        /* [1] Wait for POLL */
        dwt_forcetrxoff();
        if (!first_poll_received) {
            /* Wait indefinitely for first poll */
            dwt_setrxtimeout(0);
        } else {
            /* After first poll, set HW timeout to 1 second */
            /* Software loop will handle the full 15s wait by re-enabling RX on HW timeout */
            dwt_setrxtimeout(1000000U);  /* 1 second */
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint8_t frame_type = 0;
        uint32_t poll_timeout = first_poll_received ? RESPONDER_IDLE_TIMEOUT_UUS : 0;
        if (!wait_for_frame(first_poll_received ? MSG_TYPE_ANY : MSG_TYPE_POLL,
                            &t2, &frame_type, poll_timeout)) {
            /* Timeout after first poll means initiator is done */
            if (first_poll_received) {
                LOG_INFO("Measurement complete. Total polls: %u\n", poll_count);
                break;
            }
            continue;
        }

        if (frame_type == MSG_TYPE_STOP) {
            LOG_INFO("Stop frame received after %u polls. Exiting.\n", poll_count);
            break;
        }

        if (frame_type != MSG_TYPE_POLL) {
            /* Ignore unexpected frames */
            continue;
        }

        /* CFO estimate from initiator (from POLL) */
        ppm_on_poll = read_ppm_from_carrier_integrator_ch9();

        if (!first_poll_received) {
            LOG_INFO("Polling received. Measurement started.\n");
            first_poll_received = true;
        }

        poll_count++;
        
        /* Log progress every 10 polls */
        if (poll_count % 10 == 0) {
            if (expected_polls > 0) {
                LOG_INFO("Polls received: %u/%u\n", poll_count, expected_polls);
            } else {
                LOG_INFO("Polls received: %u\n", poll_count);
            }
        }

        /* [2] Send RESPONSE, auto-enable RX for Final */
        g_response_frame[FRAME_SEQ_OFFSET] = g_rx_buffer[FRAME_SEQ_OFFSET];
        dwt_setrxtimeout(RX_TIMEOUT_UUS);
        if (!send_frame(g_response_frame, RESPONSE_FRAME_LEN, &t3, true)) {
            continue;
        }

        /* [3] Wait for FINAL with retries */
        bool final_ok = false;
        for (uint32_t attempt = 0; attempt < MAX_WAIT_RETRIES; ++attempt) {
            if (wait_for_frame(MSG_TYPE_FINAL, &t6, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                final_ok = true;
                break;
            }
            reset_rx_state();
        }
        if (!final_ok) {
            continue;
        }

        /* CFO estimate from initiator (from FINAL) */
        ppm_on_final = read_ppm_from_carrier_integrator_ch9();

        /* [4] Wait for REPORT - immediately re-enable RX after FINAL */
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        dwt_setrxtimeout(RX_TIMEOUT_UUS);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        bool report_ok = false;
        for (uint32_t attempt = 0; attempt < MAX_WAIT_RETRIES; ++attempt) {
            if (wait_for_frame(MSG_TYPE_REPORT, NULL, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                report_ok = true;
                break;
            }
            dwt_forcetrxoff();
            dwt_setrxtimeout(RX_TIMEOUT_UUS);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        if (!report_ok) {
            continue;
        }

        /* Extract T1, T4, T5 from report */
        t1 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        t4 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        t5 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 10]);

        /* Calculate DS-TWR distance with CFO correction */
        int64_t ra = (int64_t)(t4 - t1);
        int64_t rb = (int64_t)(t6 - t3);
        int64_t da = (int64_t)(t5 - t4);
        int64_t db = (int64_t)(t3 - t2);

        double distance = -1.0;
        uint8_t status = RESULT_STATUS_ERROR;

        if (ra > 0 && rb > 0 && da > 0 && db > 0) {
            /* Convert responder-measured intervals to initiator timebase using averaged ppm */
            double avg_ppm = (ppm_on_poll + ppm_on_final) * 0.5;
            double ratio = clock_ratio_from_ppm(avg_ppm);
            double rb_corr = (double)rb * ratio;
            double db_corr = (double)db * ratio;

            double num = (double)ra * rb_corr - (double)da * db_corr;
            double den = (double)ra + rb_corr + (double)da + db_corr;

            if (den > 0.0 && num >= 0.0) {
                double tof = num / den;
                distance = tof * DWT_TIME_UNITS * SPEED_OF_LIGHT_M_S;
                if (distance >= 0.0 && distance < 200.0) {
                    status = RESULT_STATUS_OK;
                }
            }
        }

        /* Multipath/NLOS gating and RSL-based bias compensation (use diagnostics from FINAL) */
        if (status == RESULT_STATUS_OK) {
            dwt_cirdiags_t diag = { 0 };
            if (dwt_readdiagnostics_acc(&diag, DWT_ACC_IDX_IP_M) == DWT_SUCCESS) {
                int16_t rssi_q8 = 0;
                int16_t fp_q8 = 0;
                if (dwt_calculate_rssi(&diag, DWT_ACC_IDX_IP_M, &rssi_q8) == DWT_SUCCESS &&
                    dwt_calculate_first_path_power(&diag, DWT_ACC_IDX_IP_M, &fp_q8) == DWT_SUCCESS) {
                    double rssi_dbm = (double)rssi_q8 / 256.0;
                    double fp_dbm = (double)fp_q8 / 256.0;
                    double diff_db = rssi_dbm - fp_dbm;
                    /* Gate out heavy multipath where first path is much weaker than total */
                    if (diff_db > 12.0) {
                        status = RESULT_STATUS_OUTLIER;
                    } else {
                        /* Apply small bias correction vs RSL */
                        double bias_m = estimate_range_bias_from_rssi(rssi_dbm);
                        distance -= bias_m;
                    }
                }
            }
        }

        /* [5] Send RESULT (responder doesn't filter - initiator does that) */
        g_result_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        int32_to_bytes((int32_t)llround(distance * 1000.0), &g_result_frame[FRAME_DATA_OFFSET]);
        g_result_frame[FRAME_DATA_OFFSET + 4] = status;
        g_result_frame[FRAME_DATA_OFFSET + 5] = (uint8_t)poll_count;

        dwt_forcetrxoff();
        send_frame(g_result_frame, RESULT_FRAME_LEN, NULL, false);
        
        /* Exit if we've completed expected number of polls */
        if (expected_polls > 0 && poll_count >= expected_polls) {
            LOG_INFO("Measurement complete. Completed all %u expected polls.\n", poll_count);
            break;
        }
    }

    return 0;
}

/*============================================================================
 * Initialization
 *============================================================================*/

static void init_frames(role_t role) {
    (void)role;
    uint16_t my_addr = g_service_cfg.local_id;
    uint16_t peer_addr = g_service_cfg.peer_id;

    /* POLL frame: initiator -> responder */
    g_poll_frame[0] = 0x41;
    g_poll_frame[1] = 0x88;
    g_poll_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_poll_frame[3] = (uint8_t)(g_service_cfg.pan_id & 0xFF);
    g_poll_frame[4] = (uint8_t)(g_service_cfg.pan_id >> 8);
    g_poll_frame[5] = (peer_addr & 0xFF);
    g_poll_frame[6] = (peer_addr >> 8);
    g_poll_frame[7] = (my_addr & 0xFF);
    g_poll_frame[8] = (my_addr >> 8);
    g_poll_frame[9] = MSG_TYPE_POLL;

    /* RESPONSE frame: responder -> initiator */
    g_response_frame[0] = 0x41;
    g_response_frame[1] = 0x88;
    g_response_frame[2] = 0x00;  /* Seq (will be copied from poll) */
    g_response_frame[3] = (uint8_t)(g_service_cfg.pan_id & 0xFF);
    g_response_frame[4] = (uint8_t)(g_service_cfg.pan_id >> 8);
    g_response_frame[5] = (peer_addr & 0xFF);
    g_response_frame[6] = (peer_addr >> 8);
    g_response_frame[7] = (my_addr & 0xFF);
    g_response_frame[8] = (my_addr >> 8);
    g_response_frame[9] = MSG_TYPE_RESPONSE;

    /* FINAL frame: initiator -> responder */
    g_final_frame[0] = 0x41;
    g_final_frame[1] = 0x88;
    g_final_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_final_frame[3] = (uint8_t)(g_service_cfg.pan_id & 0xFF);
    g_final_frame[4] = (uint8_t)(g_service_cfg.pan_id >> 8);
    g_final_frame[5] = (peer_addr & 0xFF);
    g_final_frame[6] = (peer_addr >> 8);
    g_final_frame[7] = (my_addr & 0xFF);
    g_final_frame[8] = (my_addr >> 8);
    g_final_frame[9] = MSG_TYPE_FINAL;

    /* REPORT frame: initiator -> responder */
    g_report_frame[0] = 0x41;
    g_report_frame[1] = 0x88;
    g_report_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_report_frame[3] = (uint8_t)(g_service_cfg.pan_id & 0xFF);
    g_report_frame[4] = (uint8_t)(g_service_cfg.pan_id >> 8);
    g_report_frame[5] = (peer_addr & 0xFF);
    g_report_frame[6] = (peer_addr >> 8);
    g_report_frame[7] = (my_addr & 0xFF);
    g_report_frame[8] = (my_addr >> 8);
    g_report_frame[9] = MSG_TYPE_REPORT;
    /* T1, T4, T5 will be filled in during transmission */

    /* RESULT frame: responder -> initiator */
    g_result_frame[0] = 0x41;
    g_result_frame[1] = 0x88;
    g_result_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_result_frame[3] = (uint8_t)(g_service_cfg.pan_id & 0xFF);
    g_result_frame[4] = (uint8_t)(g_service_cfg.pan_id >> 8);
    g_result_frame[5] = (peer_addr & 0xFF);
    g_result_frame[6] = (peer_addr >> 8);
    g_result_frame[7] = (my_addr & 0xFF);
    g_result_frame[8] = (my_addr >> 8);
    g_result_frame[9] = MSG_TYPE_RESULT;
    /* Distance, status, count will be filled in during transmission */

    /* STOP frame: initiator -> responder */
    g_stop_frame[0] = 0x41;
    g_stop_frame[1] = 0x88;
    g_stop_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_stop_frame[3] = (uint8_t)(g_service_cfg.pan_id & 0xFF);
    g_stop_frame[4] = (uint8_t)(g_service_cfg.pan_id >> 8);
    g_stop_frame[5] = (peer_addr & 0xFF);
    g_stop_frame[6] = (peer_addr >> 8);
    g_stop_frame[7] = (my_addr & 0xFF);
    g_stop_frame[8] = (my_addr >> 8);
    g_stop_frame[9] = MSG_TYPE_STOP;
}

static void configure_device(role_t role) {
    (void)role;
    uint16_t addr = g_service_cfg.local_id;
    dwt_setaddress16(addr);
    dwt_setpanid(g_service_cfg.pan_id);
    dwt_setrxantennadelay(g_service_cfg.antenna_delay);
    dwt_settxantennadelay(g_service_cfg.antenna_delay);
}
