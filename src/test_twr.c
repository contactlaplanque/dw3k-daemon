/*
 * test_twr.c – Double-Sided Two-Way Ranging (DS-TWR) for distance measurement
 *
 * Usage:
 *   ./test_twr initiator              - Run as initiator (starts ranging)
 *   ./test_twr responder              - Run as responder (replies to polls)
 *   ./test_twr calibrate <distance_m> - Calibration mode (run as initiator)
 *
 * DS-TWR 5-Message Protocol (all timestamps are ACTUAL, read post-TX/RX):
 *
 *   Initiator (A)                      Responder (B)
 *        |                                   |
 *   T1 --|-------- [1] POLL ---------------->|-- T2
 *        |                                   |
 *   T4 --|<------- [2] RESPONSE -------------|-- T3
 *        |                                   |
 *   T5 --|-------- [3] FINAL --------------->|-- T6
 *        |                                   |
 *        |-------- [4] REPORT -------------->|
 *        |         (T1, T4, T5 actual)       |
 *        |                                   |
 *        |<------- [5] RESULT ---------------|
 *        |         (calculated distance)     |
 *
 * DS-TWR Formula:
 *   ToF = (Ra × Rb - Da × Db) / (Ra + Rb + Da + Db)
 *   where:
 *     Ra = T4 - T1 (round trip at initiator)
 *     Rb = T6 - T3 (round trip at responder)
 *     Da = T5 - T4 (reply delay at initiator)
 *     Db = T3 - T2 (reply delay at responder)
 */

#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "init.h"

/*============================================================================
 * Constants
 *============================================================================*/

/* Addresses */
#define INITIATOR_ADDR      0x1001U
#define RESPONDER_ADDR      0x2001U
#define PAN_ID              0xDECAU

/* Message types */
#define MSG_TYPE_POLL       0x21U
#define MSG_TYPE_RESPONSE   0x10U
#define MSG_TYPE_FINAL      0x23U
#define MSG_TYPE_REPORT     0x24U
#define MSG_TYPE_RESULT     0x42U

/* Result status codes */
#define RESULT_STATUS_OK        0x00U
#define RESULT_STATUS_OUTLIER   0x01U
#define RESULT_STATUS_ERROR     0xFFU

/* Frame offsets */
#define FRAME_SEQ_OFFSET    2U
#define FRAME_TYPE_OFFSET   9U
#define FRAME_DATA_OFFSET   10U

/* Frame sizes (excluding FCS which is auto-appended) */
#define POLL_FRAME_LEN      10U   /* FC(2) + Seq(1) + PAN(2) + Dest(2) + Src(2) + Type(1) */
#define RESPONSE_FRAME_LEN  10U   /* Same as poll - no embedded data needed */
#define FINAL_FRAME_LEN     10U   /* Same as poll */
#define REPORT_FRAME_LEN    25U   /* Poll + T1(5) + T4(5) + T5(5) */
#define RESULT_FRAME_LEN    18U   /* Poll + distance(4) + status(1) + count(1) + reserved(2) */

/* Timing */
#define RX_TIMEOUT_UUS          15000U  /* RX timeout (μs) */
#define TX_TO_TX_DELAY_US       2000U   /* Delay between consecutive TX (μs) */
#define RANGING_INTERVAL_MS     500U    /* Time between ranging exchanges */

/* Physics */
#define SPEED_OF_LIGHT_M_S  299702547.0
#define DWT_TIME_UNITS      (1.0 / 499.2e6 / 128.0)  /* ~15.65 ps per DWT unit */

/* Default antenna delay (DWT units) */
#define DEFAULT_ANT_DLY     16446U

/* Filtering */
#define FILTER_WINDOW_SIZE      16U
#define FILTER_WARMUP_COUNT      4U
#define OUTLIER_THRESHOLD_M    0.30

/* RX buffer */
#define RX_BUFFER_LEN       128U

/*============================================================================
 * Types
 *============================================================================*/

typedef enum {
    ROLE_INITIATOR,
    ROLE_RESPONDER,
    ROLE_CALIBRATE
} role_t;

typedef struct {
    double samples[FILTER_WINDOW_SIZE];
    size_t count;
    size_t index;
    double sum;
    double min;
    double max;
} filter_t;

typedef struct {
    double distance_m;
    uint8_t status;
    uint8_t sample_count;
} result_t;

/*============================================================================
 * Global State
 *============================================================================*/

static volatile sig_atomic_t g_running = 1;
static uint8_t g_rx_buffer[RX_BUFFER_LEN];
static uint8_t g_seq_num = 0;
static uint16_t g_ant_dly = DEFAULT_ANT_DLY;
static double g_calibration_distance = 0.0;

/* Frame templates */
static uint8_t g_poll_frame[POLL_FRAME_LEN] = {
    0x41, 0x88, 0x00,
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),
    MSG_TYPE_POLL
};

static uint8_t g_response_frame[RESPONSE_FRAME_LEN] = {
    0x41, 0x88, 0x00,
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),
    MSG_TYPE_RESPONSE
};

static uint8_t g_final_frame[FINAL_FRAME_LEN] = {
    0x41, 0x88, 0x00,
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),
    MSG_TYPE_FINAL
};

static uint8_t g_report_frame[REPORT_FRAME_LEN] = {
    0x41, 0x88, 0x00,
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),
    MSG_TYPE_REPORT,
    0, 0, 0, 0, 0,  /* T1 */
    0, 0, 0, 0, 0,  /* T4 */
    0, 0, 0, 0, 0   /* T5 */
};

static uint8_t g_result_frame[RESULT_FRAME_LEN] = {
    0x41, 0x88, 0x00,
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),
    MSG_TYPE_RESULT,
    0, 0, 0, 0,     /* Distance (mm) */
    0,              /* Status */
    0,              /* Sample count */
    0, 0            /* Reserved */
};

/*============================================================================
 * Helper Functions
 *============================================================================*/

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
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
 * Filter Functions
 *============================================================================*/

static void filter_reset(filter_t *f) {
    memset(f, 0, sizeof(*f));
    f->min = 1e9;
    f->max = -1e9;
}

static void filter_add(filter_t *f, double sample) {
    if (f->count < FILTER_WINDOW_SIZE) {
        f->samples[f->count] = sample;
        f->sum += sample;
        f->count++;
    } else {
        f->sum -= f->samples[f->index];
        f->samples[f->index] = sample;
        f->sum += sample;
        f->index = (f->index + 1) % FILTER_WINDOW_SIZE;
    }
    if (sample < f->min) f->min = sample;
    if (sample > f->max) f->max = sample;
}

static double filter_mean(const filter_t *f) {
    return (f->count > 0) ? (f->sum / (double)f->count) : 0.0;
}

static size_t filter_count(const filter_t *f) {
    return f->count;
}

static double filter_stddev(const filter_t *f) {
    if (f->count < 2) return 0.0;
    double mean = filter_mean(f);
    double variance = 0.0;
    size_t n = (f->count < FILTER_WINDOW_SIZE) ? f->count : FILTER_WINDOW_SIZE;
    for (size_t i = 0; i < n; i++) {
        double diff = f->samples[i] - mean;
        variance += diff * diff;
    }
    return sqrt(variance / (double)n);
}

static bool filter_is_outlier(const filter_t *f, double sample) {
    if (f->count < FILTER_WARMUP_COUNT) return false;
    return fabs(sample - filter_mean(f)) > OUTLIER_THRESHOLD_M;
}

static void filter_print_stats(const filter_t *f) {
    if (f->count < 2) return;
    printf("  Stats: mean=%.3f m | std=%.3f m | range=[%.3f, %.3f] m\n",
           filter_mean(f), filter_stddev(f), f->min, f->max);
}

/*============================================================================
 * Frame TX/RX
 *============================================================================*/

/* Send frame and optionally enable RX immediately after */
static bool send_frame(uint8_t *frame, size_t len, uint64_t *tx_ts, bool rx_after) {
    dwt_forcetrxoff();
    dwt_writetxdata((uint16_t)len, frame, 0);
    dwt_writetxfctrl((uint16_t)(len + 2), 0, 1);

    /* Use TX_IMMEDIATE with optional RX_AFTER to minimize turnaround time */
    int mode = DWT_START_TX_IMMEDIATE;
    if (rx_after) {
        mode |= DWT_RESPONSE_EXPECTED;  /* Auto-enable RX after TX */
    }

    if (dwt_starttx(mode) != DWT_SUCCESS) {
        return false;
    }

    /* Wait for TX complete */
    for (int i = 0; i < 10000; i++) {
        uint32_t status = dwt_readsysstatuslo();
        if (status & DWT_INT_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            if (tx_ts) *tx_ts = read_tx_timestamp();
            return true;
        }
        usleep(1);
    }
    return false;
}

static bool wait_for_frame(uint8_t expected_type, uint64_t *rx_ts) {
    uint32_t timeout = RX_TIMEOUT_UUS * 3;
    uint32_t elapsed = 0;

    while (elapsed < timeout && g_running) {
        uint32_t status = dwt_readsysstatuslo();

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            if (rx_ts) *rx_ts = read_rx_timestamp();
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            uint8_t rng = 0;
            uint16_t len = dwt_getframelength(&rng);
            if (len > 0 && len <= RX_BUFFER_LEN) {
                dwt_readrxdata(g_rx_buffer, len, 0);
                if (g_rx_buffer[FRAME_TYPE_OFFSET] == expected_type) {
                    return true;
                }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        if (status & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            return false;
        }

        usleep(100);
        elapsed += 100;
    }
    return false;
}

/*============================================================================
 * DS-TWR Initiator
 *============================================================================*/

static void run_initiator(void) {
    printf("Starting DS-TWR initiator. Press Ctrl+C to stop.\n\n");

    filter_t filter;
    filter_reset(&filter);
    uint32_t exchange_count = 0;
    uint32_t success_count = 0;

    while (g_running) {
        exchange_count++;
        printf("--- Exchange #%u ---\n", exchange_count);

        uint64_t t1 = 0, t4 = 0, t5 = 0;
        const char *fail_reason = NULL;

        /* [1] Send POLL, auto-enable RX for Response */
        g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
        if (!send_frame(g_poll_frame, POLL_FRAME_LEN, &t1, true)) {
            fail_reason = "Poll TX failed";
            goto exchange_failed;
        }

        /* [2] Wait for RESPONSE (RX already enabled) */
        if (!wait_for_frame(MSG_TYPE_RESPONSE, &t4)) {
            fail_reason = "Response timeout";
            dwt_forcetrxoff();
            goto exchange_failed;
        }

        /* [3] Send FINAL, auto-enable RX (responder needs time to process) */
        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        if (!send_frame(g_final_frame, FINAL_FRAME_LEN, &t5, false)) {
            fail_reason = "Final TX failed";
            goto exchange_failed;
        }

        /* Small delay to let responder switch to RX mode after processing Final */
        usleep(TX_TO_TX_DELAY_US);

        /* [4] Send REPORT with actual T1, T4, T5, auto-enable RX for Result */
        g_report_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        timestamp_to_bytes(t1, &g_report_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t4, &g_report_frame[FRAME_DATA_OFFSET + 5]);
        timestamp_to_bytes(t5, &g_report_frame[FRAME_DATA_OFFSET + 10]);
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
        if (!send_frame(g_report_frame, REPORT_FRAME_LEN, NULL, true)) {
            fail_reason = "Report TX failed";
            goto exchange_failed;
        }

        /* [5] Wait for RESULT (RX already enabled) */
        if (!wait_for_frame(MSG_TYPE_RESULT, NULL)) {
            fail_reason = "Result timeout";
            dwt_forcetrxoff();
            goto exchange_failed;
        }

        /* Parse result */
        result_t result;
        result.distance_m = (double)bytes_to_int32(&g_rx_buffer[FRAME_DATA_OFFSET]) / 1000.0;
        result.status = g_rx_buffer[FRAME_DATA_OFFSET + 4];
        result.sample_count = g_rx_buffer[FRAME_DATA_OFFSET + 5];

        if (result.status == RESULT_STATUS_ERROR) {
            printf("  [FAIL] Responder calculation error\n");
        } else if (result.status == RESULT_STATUS_OUTLIER) {
            printf("  [INFO] Responder marked as outlier: %.3f m\n", result.distance_m);
        } else {
            /* Valid result */
            double distance = result.distance_m;

            if (filter_is_outlier(&filter, distance)) {
                printf("  [SKIP] Local filter rejected %.3f m\n", distance);
            } else {
                filter_add(&filter, distance);
                success_count++;

                printf("  T1=0x%010" PRIX64 " T4=0x%010" PRIX64 " T5=0x%010" PRIX64 "\n", t1, t4, t5);
                printf("  >>> DISTANCE: %.3f m <<<\n", distance);
                printf("  Filtered avg (n=%zu): %.3f m\n", filter_count(&filter), filter_mean(&filter));
                filter_print_stats(&filter);
            }
        }

        printf("\n");
        usleep(RANGING_INTERVAL_MS * 1000);
        continue;

exchange_failed:
        printf("  [FAIL] %s\n\n", fail_reason);
        usleep(RANGING_INTERVAL_MS * 1000);
    }

    printf("\n=== Summary ===\n");
    printf("Exchanges: %u | Successful: %u (%.1f%%)\n", exchange_count, success_count,
           exchange_count > 0 ? 100.0 * success_count / exchange_count : 0.0);
    if (filter_count(&filter) > 0) {
        printf("Final: mean=%.3f m | std=%.3f m | range=[%.3f, %.3f] m (n=%zu)\n",
               filter_mean(&filter), filter_stddev(&filter), filter.min, filter.max, filter_count(&filter));
    }
}

/*============================================================================
 * DS-TWR Responder
 *============================================================================*/

static void run_responder(void) {
    printf("Starting DS-TWR responder. Waiting for polls...\n\n");

    filter_t filter;
    filter_reset(&filter);
    uint32_t poll_count = 0;
    uint32_t success_count = 0;

    while (g_running) {
        uint64_t t2 = 0, t3 = 0, t6 = 0;
        uint64_t t1 = 0, t4 = 0, t5 = 0;

        /* [1] Wait for POLL */
        dwt_forcetrxoff();
        dwt_setrxtimeout(0);  /* Wait indefinitely */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        if (!wait_for_frame(MSG_TYPE_POLL, &t2)) {
            continue;
        }

        poll_count++;
        printf("--- Poll #%u ---\n", poll_count);
        printf("  T2=0x%010" PRIX64 "\n", t2);

        /* [2] Send RESPONSE, auto-enable RX for Final */
        g_response_frame[FRAME_SEQ_OFFSET] = g_rx_buffer[FRAME_SEQ_OFFSET];
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
        if (!send_frame(g_response_frame, RESPONSE_FRAME_LEN, &t3, true)) {
            printf("  [FAIL] Response TX failed\n\n");
            continue;
        }
        printf("  T3=0x%010" PRIX64 "\n", t3);

        /* [3] Wait for FINAL (RX already enabled) */
        if (!wait_for_frame(MSG_TYPE_FINAL, &t6)) {
            printf("  [FAIL] Final timeout\n\n");
            dwt_forcetrxoff();
            continue;
        }
        printf("  T6=0x%010" PRIX64 "\n", t6);

        /* [4] Wait for REPORT - re-enable RX with longer timeout */
        dwt_forcetrxoff();
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 3));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (!wait_for_frame(MSG_TYPE_REPORT, NULL)) {
            printf("  [FAIL] Report timeout\n\n");
            dwt_forcetrxoff();
            continue;
        }

        /* Extract T1, T4, T5 from report */
        t1 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        t4 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        t5 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 10]);
        printf("  T1=0x%010" PRIX64 " T4=0x%010" PRIX64 " T5=0x%010" PRIX64 "\n", t1, t4, t5);

        /* Calculate DS-TWR distance */
        int64_t ra = (int64_t)(t4 - t1);
        int64_t rb = (int64_t)(t6 - t3);
        int64_t da = (int64_t)(t5 - t4);
        int64_t db = (int64_t)(t3 - t2);

        double distance = -1.0;
        uint8_t status = RESULT_STATUS_ERROR;

        if (ra > 0 && rb > 0 && da > 0 && db > 0) {
            double num = (double)ra * (double)rb - (double)da * (double)db;
            double den = (double)ra + (double)rb + (double)da + (double)db;

            if (den > 0.0 && num >= 0.0) {
                double tof = num / den;
                distance = tof * DWT_TIME_UNITS * SPEED_OF_LIGHT_M_S;
                if (distance >= 0.0 && distance < 200.0) {
                    status = RESULT_STATUS_OK;
                }
            }
        }

        /* Apply filter */
        if (status == RESULT_STATUS_OK) {
            if (filter_is_outlier(&filter, distance)) {
                status = RESULT_STATUS_OUTLIER;
                printf("  [SKIP] Filter rejected %.3f m\n", distance);
            } else {
                filter_add(&filter, distance);
                success_count++;
                printf("  >>> DISTANCE: %.3f m <<<\n", distance);
                printf("  Filtered avg (n=%zu): %.3f m\n", filter_count(&filter), filter_mean(&filter));
                filter_print_stats(&filter);
            }
        } else {
            printf("  [FAIL] Invalid calculation\n");
        }

        /* [5] Send RESULT (no RX after - exchange complete) */
        g_result_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        int32_to_bytes((int32_t)llround(distance * 1000.0), &g_result_frame[FRAME_DATA_OFFSET]);
        g_result_frame[FRAME_DATA_OFFSET + 4] = status;
        g_result_frame[FRAME_DATA_OFFSET + 5] = (uint8_t)filter_count(&filter);

        dwt_forcetrxoff();
        if (!send_frame(g_result_frame, RESULT_FRAME_LEN, NULL, false)) {
            printf("  [WARN] Result TX failed\n");
        }

        printf("\n");
    }

    printf("\n=== Summary ===\n");
    printf("Polls: %u | Successful: %u (%.1f%%)\n", poll_count, success_count,
           poll_count > 0 ? 100.0 * success_count / poll_count : 0.0);
    if (filter_count(&filter) > 0) {
        printf("Final: mean=%.3f m | std=%.3f m | range=[%.3f, %.3f] m (n=%zu)\n",
               filter_mean(&filter), filter_stddev(&filter), filter.min, filter.max, filter_count(&filter));
    }
}

/*============================================================================
 * Calibration Mode
 *============================================================================*/

static void run_calibration(void) {
    printf("=== Antenna Delay Calibration (DS-TWR) ===\n");
    printf("True distance: %.3f m\n", g_calibration_distance);
    printf("Current antenna delay: %u DWT (%.2f ns)\n",
           g_ant_dly, g_ant_dly * DWT_TIME_UNITS * 1e9);
    printf("Responder must be running: ./test_twr responder\n");
    printf("Collecting 50 measurements...\n\n");

    filter_t filter;
    filter_reset(&filter);
    double sum = 0.0;
    uint32_t success = 0;
    const uint32_t target = 50;

    while (g_running && success < target) {
        uint64_t t1 = 0, t4 = 0, t5 = 0;

        /* Run 5-message exchange */
        g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
        if (!send_frame(g_poll_frame, POLL_FRAME_LEN, &t1, true)) continue;

        if (!wait_for_frame(MSG_TYPE_RESPONSE, &t4)) {
            dwt_forcetrxoff();
            continue;
        }

        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        if (!send_frame(g_final_frame, FINAL_FRAME_LEN, &t5, false)) continue;

        usleep(TX_TO_TX_DELAY_US);

        g_report_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        timestamp_to_bytes(t1, &g_report_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t4, &g_report_frame[FRAME_DATA_OFFSET + 5]);
        timestamp_to_bytes(t5, &g_report_frame[FRAME_DATA_OFFSET + 10]);
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
        if (!send_frame(g_report_frame, REPORT_FRAME_LEN, NULL, true)) continue;

        if (!wait_for_frame(MSG_TYPE_RESULT, NULL)) {
            dwt_forcetrxoff();
            continue;
        }

        double distance = (double)bytes_to_int32(&g_rx_buffer[FRAME_DATA_OFFSET]) / 1000.0;
        uint8_t status = g_rx_buffer[FRAME_DATA_OFFSET + 4];

        if (status == RESULT_STATUS_OK && !filter_is_outlier(&filter, distance)) {
            filter_add(&filter, distance);
            sum += distance;
            success++;
            printf("[%2u/%u] %.3f m\n", success, target, distance);
        }

        usleep(200000);
    }

    if (success < 10) {
        printf("\nERROR: Not enough measurements (%u)\n", success);
        return;
    }

    double mean = sum / (double)success;
    double error = mean - g_calibration_distance;
    double delta_dtu = error / (SPEED_OF_LIGHT_M_S * DWT_TIME_UNITS);
    int32_t new_delay = (int32_t)g_ant_dly - (int32_t)llround(delta_dtu);
    if (new_delay < 0) new_delay = 0;
    if (new_delay > 65535) new_delay = 65535;

    printf("\n=== Results ===\n");
    printf("Measurements: %u\n", success);
    printf("Mean measured: %.4f m\n", mean);
    printf("True distance: %.4f m\n", g_calibration_distance);
    printf("Error: %+.4f m (%+.1f cm)\n", error, error * 100.0);
    printf("\n>>> Recommended antenna delay: %d DWT <<<\n", new_delay);
    printf("\nUpdate DEFAULT_ANT_DLY in test_twr.c to %d\n", new_delay);
}

/*============================================================================
 * Argument Parsing & Main
 *============================================================================*/

static void print_usage(const char *prog) {
    fprintf(stderr, "Usage: %s <mode> [options]\n", prog);
    fprintf(stderr, "\nModes:\n");
    fprintf(stderr, "  initiator              Run as ranging initiator\n");
    fprintf(stderr, "  responder              Run as ranging responder\n");
    fprintf(stderr, "  calibrate <dist_m>     Calibration mode at known distance\n");
    fprintf(stderr, "\nExample:\n");
    fprintf(stderr, "  %s responder           # On device B\n", prog);
    fprintf(stderr, "  %s initiator           # On device A\n", prog);
    fprintf(stderr, "  %s calibrate 1.0       # Calibrate at 1.0 meter\n", prog);
}

static role_t parse_args(int argc, char **argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        exit(1);
    }

    if (strcmp(argv[1], "initiator") == 0) {
        return ROLE_INITIATOR;
    } else if (strcmp(argv[1], "responder") == 0) {
        return ROLE_RESPONDER;
    } else if (strcmp(argv[1], "calibrate") == 0) {
        if (argc < 3) {
            fprintf(stderr, "ERROR: calibrate requires distance argument\n");
            exit(1);
        }
        g_calibration_distance = atof(argv[2]);
        if (g_calibration_distance <= 0.0) {
            fprintf(stderr, "ERROR: invalid distance: %s\n", argv[2]);
            exit(1);
        }
        return ROLE_CALIBRATE;
    }

    fprintf(stderr, "ERROR: unknown mode: %s\n", argv[1]);
    print_usage(argv[0]);
    exit(1);
}

static void configure_device(role_t role) {
    uint16_t addr = (role == ROLE_RESPONDER) ? RESPONDER_ADDR : INITIATOR_ADDR;
    dwt_setaddress16(addr);
    dwt_setpanid(PAN_ID);
    dwt_setrxantennadelay(g_ant_dly);
    dwt_settxantennadelay(g_ant_dly);

    const char *role_str = (role == ROLE_RESPONDER) ? "Responder" :
                           (role == ROLE_CALIBRATE) ? "Calibration" : "Initiator";
    printf("Role: %s | Address: 0x%04X | Antenna delay: %u DWT\n\n", role_str, addr, g_ant_dly);
}

int main(int argc, char **argv) {
    role_t role = parse_args(argc, argv);

    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        perror("signal");
        return 1;
    }

    printf("=== DW3000 DS-TWR Ranging ===\n\n");

    if (chip_init() != 0) {
        fprintf(stderr, "ERROR: chip initialization failed\n");
        return 1;
    }

    configure_device(role);

    switch (role) {
        case ROLE_INITIATOR: run_initiator(); break;
        case ROLE_RESPONDER: run_responder(); break;
        case ROLE_CALIBRATE: run_calibration(); break;
    }

    printf("\nShutting down...\n");
    chip_shutdown();
    return 0;
}
