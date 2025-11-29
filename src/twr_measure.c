/*
 * twr_measure.c – Production DS-TWR distance measurement
 *
 * Usage:
 *   ./twr_measure initiator <initiator_id> <responder_id> <num_measurements>
 *   ./twr_measure responder <responder_id> <initiator_id>
 *
 * This program performs N fast distance measurements and exits with statistics.
 * Minimal output - designed for programmatic use.
 */

#include <inttypes.h>
#include <math.h>
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

#define PAN_ID              0xDECAU

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

/* Timing */
#define RX_TIMEOUT_UUS          30000U
#define TX_TO_TX_DELAY_US       3000U
#define MAX_STAGE_RETRIES       2U
#define MAX_WAIT_RETRIES        2U
#define FAST_RANGING_INTERVAL_US 80000U  /* 80ms between measurements - safe for reliability */
#define RESPONDER_IDLE_TIMEOUT_UUS 15000000U  /* 15s timeout after polling starts */
#define DEFAULT_WAIT_TIMEOUT_US (RX_TIMEOUT_UUS * 3U)  /* ~90ms */

/* Physics */
#define SPEED_OF_LIGHT_M_S  299702547.0
#define DWT_TIME_UNITS      (1.0 / 499.2e6 / 128.0)

/* Default antenna delay (DWT units) */
#define DEFAULT_ANT_DLY     16388U

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
    ROLE_RESPONDER
} role_t;

typedef struct {
    double samples[FILTER_WINDOW_SIZE];
    size_t count;
    size_t index;
    double sum;
    double min;
    double max;
} filter_t;

/*============================================================================
 * Global State
 *============================================================================*/

static uint8_t g_rx_buffer[RX_BUFFER_LEN];
static uint8_t g_seq_num = 0;
static uint16_t g_ant_dly = DEFAULT_ANT_DLY;
static uint16_t g_initiator_id = 0;
static uint16_t g_responder_id = 0;

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
                uint16_t expected_id = from_initiator ? g_initiator_id : g_responder_id;

                if (src_addr == expected_id) {
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
        fprintf(stderr, "WARN: failed to send STOP frame\n");
    }
}

/*============================================================================
 * DS-TWR Initiator
 *============================================================================*/

static int run_initiator(uint32_t num_measurements) {
    printf("Running measurement: %u samples\n", num_measurements);
    fflush(stdout);
    
    filter_t filter;
    filter_reset(&filter);
    uint32_t exchange_count = 0;
    uint32_t success_count = 0;

    while (exchange_count < num_measurements) {
        exchange_count++;
        
        /* Log progress every 10 measurements */
        if (exchange_count % 10 == 0 || exchange_count == 1 || exchange_count == num_measurements) {
            printf("Progress: %u/%u (success: %u)\n", exchange_count, num_measurements, success_count);
            fflush(stdout);
        }

        uint64_t t1 = 0, t4 = 0, t5 = 0;
        const char *fail_reason = NULL;

        /* [1] Send POLL and wait for RESPONSE (with retries) */
        {
            bool response_ok = false;
            for (uint32_t attempt = 0; attempt < MAX_STAGE_RETRIES; ++attempt) {
                g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
                dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
                if (!send_frame(g_poll_frame, POLL_FRAME_LEN, &t1, true)) {
                    fail_reason = "Poll TX failed";
                    continue;
                }
                if (wait_for_frame(MSG_TYPE_RESPONSE, &t4, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                    response_ok = true;
                    break;
                }
                fail_reason = "Response timeout";
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
                dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
                if (!send_frame(g_report_frame, REPORT_FRAME_LEN, NULL, true)) {
                    fail_reason = "Report TX failed";
                    continue;
                }
                if (wait_for_frame(MSG_TYPE_RESULT, NULL, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                    result_ok = true;
                    break;
                }
                fail_reason = "Result timeout";
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
            if (!filter_is_outlier(&filter, distance)) {
                filter_add(&filter, distance);
                success_count++;
            }
        }

        /* Small delay between measurements */
        if (exchange_count < num_measurements) {
            usleep(FAST_RANGING_INTERVAL_US);
        }
    }

    /* Notify responder that measurements are complete */
    send_stop_signal();

    /* Output results */
    if (filter_count(&filter) > 0) {
        size_t filtered = filter_count(&filter);
        printf("Completed %u attempts. Raw successes: %u. Filtered samples kept: %zu.\n",
               exchange_count, success_count, filtered);
        printf("Stats -> mean: %.3f m | stddev: %.3f m | min: %.3f m | max: %.3f m\n",
               filter_mean(&filter), filter_stddev(&filter), filter.min, filter.max);
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u\n",
               filter_mean(&filter),
               filter_stddev(&filter),
               filter.min,
               filter.max,
               filter_mean(&filter),  /* median approximation (using mean) */
               (uint32_t)filtered,
               exchange_count);
        return 0;
    } else {
        fprintf(stderr, "ERROR: No successful measurements\n");
        return 1;
    }
}

/*============================================================================
 * DS-TWR Responder
 *============================================================================*/

static int run_responder(void) {
    printf("Waiting for polls...\n");
    fflush(stdout);
    
    filter_t filter;
    filter_reset(&filter);
    uint32_t poll_count = 0;
    bool first_poll_received = false;

    while (true) {
        uint64_t t2 = 0, t3 = 0, t6 = 0;
        uint64_t t1 = 0, t4 = 0, t5 = 0;

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
                printf("Measurement complete. Total polls: %u\n", poll_count);
                fflush(stdout);
                break;
            }
            continue;
        }

        if (frame_type == MSG_TYPE_STOP) {
            printf("Stop frame received after %u polls. Exiting.\n", poll_count);
            fflush(stdout);
            break;
        }

        if (frame_type != MSG_TYPE_POLL) {
            /* Ignore unexpected frames */
            continue;
        }

        if (!first_poll_received) {
            printf("Polling received. Measurement started.\n");
            fflush(stdout);
            first_poll_received = true;
        }

        poll_count++;

        /* Log progress every 10 polls */
        if (poll_count % 10 == 0) {
            printf("Polls received: %u\n", poll_count);
            fflush(stdout);
        }

        /* [2] Send RESPONSE, auto-enable RX for Final */
        g_response_frame[FRAME_SEQ_OFFSET] = g_rx_buffer[FRAME_SEQ_OFFSET];
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
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

        /* [4] Wait for REPORT - immediately re-enable RX after FINAL */
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        bool report_ok = false;
        for (uint32_t attempt = 0; attempt < MAX_WAIT_RETRIES; ++attempt) {
            if (wait_for_frame(MSG_TYPE_REPORT, NULL, NULL, DEFAULT_WAIT_TIMEOUT_US)) {
                report_ok = true;
                break;
            }
            dwt_forcetrxoff();
            dwt_setrxtimeout((uint32_t)(RX_TIMEOUT_UUS * 2));
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        if (!report_ok) {
            continue;
        }

        /* Extract T1, T4, T5 from report */
        t1 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        t4 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        t5 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 10]);

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
            if (!filter_is_outlier(&filter, distance)) {
                filter_add(&filter, distance);
            }
        }

        /* [5] Send RESULT */
        g_result_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        int32_to_bytes((int32_t)llround(distance * 1000.0), &g_result_frame[FRAME_DATA_OFFSET]);
        g_result_frame[FRAME_DATA_OFFSET + 4] = status;
        g_result_frame[FRAME_DATA_OFFSET + 5] = (uint8_t)filter_count(&filter);

        dwt_forcetrxoff();
        send_frame(g_result_frame, RESULT_FRAME_LEN, NULL, false);
    }

    return 0;
}

/*============================================================================
 * Initialization
 *============================================================================*/

static void init_frames(role_t role) {
    uint16_t my_addr = (role == ROLE_RESPONDER) ? g_responder_id : g_initiator_id;
    uint16_t peer_addr = (role == ROLE_RESPONDER) ? g_initiator_id : g_responder_id;

    /* POLL frame: initiator -> responder */
    g_poll_frame[0] = 0x41;
    g_poll_frame[1] = 0x88;
    g_poll_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_poll_frame[3] = (PAN_ID & 0xFF);
    g_poll_frame[4] = (PAN_ID >> 8);
    g_poll_frame[5] = (peer_addr & 0xFF);
    g_poll_frame[6] = (peer_addr >> 8);
    g_poll_frame[7] = (my_addr & 0xFF);
    g_poll_frame[8] = (my_addr >> 8);
    g_poll_frame[9] = MSG_TYPE_POLL;

    /* RESPONSE frame: responder -> initiator */
    g_response_frame[0] = 0x41;
    g_response_frame[1] = 0x88;
    g_response_frame[2] = 0x00;  /* Seq (will be copied from poll) */
    g_response_frame[3] = (PAN_ID & 0xFF);
    g_response_frame[4] = (PAN_ID >> 8);
    g_response_frame[5] = (peer_addr & 0xFF);
    g_response_frame[6] = (peer_addr >> 8);
    g_response_frame[7] = (my_addr & 0xFF);
    g_response_frame[8] = (my_addr >> 8);
    g_response_frame[9] = MSG_TYPE_RESPONSE;

    /* FINAL frame: initiator -> responder */
    g_final_frame[0] = 0x41;
    g_final_frame[1] = 0x88;
    g_final_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_final_frame[3] = (PAN_ID & 0xFF);
    g_final_frame[4] = (PAN_ID >> 8);
    g_final_frame[5] = (peer_addr & 0xFF);
    g_final_frame[6] = (peer_addr >> 8);
    g_final_frame[7] = (my_addr & 0xFF);
    g_final_frame[8] = (my_addr >> 8);
    g_final_frame[9] = MSG_TYPE_FINAL;

    /* REPORT frame: initiator -> responder */
    g_report_frame[0] = 0x41;
    g_report_frame[1] = 0x88;
    g_report_frame[2] = 0x00;  /* Seq (will be set per transmission) */
    g_report_frame[3] = (PAN_ID & 0xFF);
    g_report_frame[4] = (PAN_ID >> 8);
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
    g_result_frame[3] = (PAN_ID & 0xFF);
    g_result_frame[4] = (PAN_ID >> 8);
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
    g_stop_frame[3] = (PAN_ID & 0xFF);
    g_stop_frame[4] = (PAN_ID >> 8);
    g_stop_frame[5] = (peer_addr & 0xFF);
    g_stop_frame[6] = (peer_addr >> 8);
    g_stop_frame[7] = (my_addr & 0xFF);
    g_stop_frame[8] = (my_addr >> 8);
    g_stop_frame[9] = MSG_TYPE_STOP;
}

static void configure_device(role_t role) {
    uint16_t addr = (role == ROLE_RESPONDER) ? g_responder_id : g_initiator_id;
    dwt_setaddress16(addr);
    dwt_setpanid(PAN_ID);
    dwt_setrxantennadelay(g_ant_dly);
    dwt_settxantennadelay(g_ant_dly);
}

static void print_usage(const char *prog) {
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s initiator <initiator_id> <responder_id> <num_measurements>\n", prog);
    fprintf(stderr, "  %s responder <responder_id> <initiator_id>\n", prog);
    fprintf(stderr, "\n");
    fprintf(stderr, "IDs are 16-bit addresses (0x0001 to 0xFFFF)\n");
    fprintf(stderr, "Output format (initiator): mean,stddev,min,max,median,success_count,total_count\n");
}

int main(int argc, char **argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    role_t role;
    if (strcmp(argv[1], "initiator") == 0) {
        role = ROLE_INITIATOR;
        if (argc != 5) {
            print_usage(argv[0]);
            return 1;
        }
        g_initiator_id = (uint16_t)strtoul(argv[2], NULL, 0);
        g_responder_id = (uint16_t)strtoul(argv[3], NULL, 0);
    } else if (strcmp(argv[1], "responder") == 0) {
        role = ROLE_RESPONDER;
        if (argc != 4) {
            print_usage(argv[0]);
            return 1;
        }
        g_responder_id = (uint16_t)strtoul(argv[2], NULL, 0);
        g_initiator_id = (uint16_t)strtoul(argv[3], NULL, 0);
    } else {
        print_usage(argv[0]);
        return 1;
    }

    if (chip_init() != 0) {
        fprintf(stderr, "ERROR: chip initialization failed\n");
        return 1;
    }

    configure_device(role);
    init_frames(role);

    int result;
    if (role == ROLE_INITIATOR) {
        uint32_t num_measurements = (uint32_t)strtoul(argv[4], NULL, 0);
        if (num_measurements == 0 || num_measurements > 1000) {
            fprintf(stderr, "ERROR: num_measurements must be 1-1000\n");
            result = 1;
        } else {
            result = run_initiator(num_measurements);
        }
    } else {
        result = run_responder();
    }

    chip_shutdown();
    return result;
}

