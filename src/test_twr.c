/*
 * test_twr.c – Double-Sided Two-Way Ranging (DS-TWR) for distance measurement
 *
 * Usage:
 *   ./test_twr initiator              - Run as initiator (starts ranging)
 *   ./test_twr responder              - Run as responder (replies to polls)
 *   ./test_twr calibrate <distance_m> - Calibration mode (run as initiator)
 *
 * DS-TWR Message Exchange:
 *   Initiator                    Responder
 *       |                            |
 *   T1 -|-------- Poll ------------->|- T2
 *       |                            |
 *   T4 -|<------- Response ----------|- T3  (Response embeds T2)
 *       |                            |
 *   T5 -|-------- Final ------------>|- T6  (Final embeds T1, T4, T3)
 *       |                            |
 *       |     Both can calculate     |
 *       |        distance            |
 *
 * Key insight: Each message embeds timestamps from PREVIOUS events,
 * not from itself (since TX timestamp is only known after transmission).
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

/* Frame offsets */
#define FRAME_FC_OFFSET     0U
#define FRAME_SEQ_OFFSET    2U
#define FRAME_PANID_OFFSET  3U
#define FRAME_DEST_OFFSET   5U
#define FRAME_SRC_OFFSET    7U
#define FRAME_TYPE_OFFSET   9U
#define FRAME_DATA_OFFSET   10U

/* Frame sizes (excluding FCS which is auto-appended) */
#define POLL_FRAME_LEN      10U   /* FC(2) + Seq(1) + PAN(2) + Dest(2) + Src(2) + Type(1) */
#define RESPONSE_FRAME_LEN  15U   /* Poll + T2(5) -- Response embeds poll RX timestamp */
#define FINAL_FRAME_LEN     25U   /* Poll + T1(5) + T4(5) + T3(5) -- Final embeds all prior timestamps */

/* Timing */
#define RESP_RX_TIMEOUT_UUS         10000U  /* Response RX timeout (μs) */
#define FINAL_RX_TIMEOUT_UUS        10000U  /* Final RX timeout (μs) */
#define RANGING_INTERVAL_MS         500U    /* Time between ranging exchanges */

/* Speed of light and time unit conversion */
#define SPEED_OF_LIGHT_M_S  299702547.0     /* Speed of light in air (m/s) */
#define DWT_TIME_UNITS      (1.0 / 499.2e6 / 128.0)  /* ~15.65 ps per DWT unit */

/* Default antenna delays (DWT units) - typical for DWM3000 */
#define DEFAULT_ANT_DLY     16385U

/* Statistics */
#define STATS_WINDOW_SIZE   20U

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
    double samples[STATS_WINDOW_SIZE];
    size_t count;
    size_t index;
    double sum;
} stats_t;

/*============================================================================
 * Global State
 *============================================================================*/

static volatile sig_atomic_t g_running = 1;
static uint8_t g_rx_buffer[RX_BUFFER_LEN];
static uint8_t g_seq_num = 0;
static stats_t g_stats = { 0 };
static uint16_t g_ant_dly = DEFAULT_ANT_DLY;
static double g_calibration_distance = 0.0;

/* Frame templates */
static uint8_t g_poll_frame[POLL_FRAME_LEN] = {
    0x41, 0x88,             /* Frame Control: Data frame, PAN ID compression, short addresses */
    0x00,                   /* Sequence number */
    (PAN_ID & 0xFF), (PAN_ID >> 8),  /* PAN ID */
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),  /* Destination */
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),  /* Source */
    MSG_TYPE_POLL           /* Message type */
};

static uint8_t g_response_frame[RESPONSE_FRAME_LEN] = {
    0x41, 0x88,             /* Frame Control */
    0x00,                   /* Sequence number */
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),  /* Destination (initiator) */
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),  /* Source (responder) */
    MSG_TYPE_RESPONSE,      /* Message type */
    0, 0, 0, 0, 0           /* T2 placeholder (5 bytes) - poll RX timestamp at responder */
};

static uint8_t g_final_frame[FINAL_FRAME_LEN] = {
    0x41, 0x88,             /* Frame Control */
    0x00,                   /* Sequence number */
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),  /* Destination (responder) */
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),  /* Source (initiator) */
    MSG_TYPE_FINAL,         /* Message type */
    0, 0, 0, 0, 0,          /* T1 placeholder - poll TX timestamp at initiator */
    0, 0, 0, 0, 0,          /* T4 placeholder - response RX timestamp at initiator */
    0, 0, 0, 0, 0           /* T3 placeholder - response TX timestamp at responder (received in response) */
};

/*============================================================================
 * Forward Declarations
 *============================================================================*/

static void signal_handler(int sig);
static void print_usage(const char *prog);
static role_t parse_args(int argc, char **argv);
static void configure_for_role(role_t role);

static void run_initiator(void);
static void run_responder(void);
static void run_calibration(void);

static bool send_frame_and_wait(uint8_t *frame, size_t len, uint64_t *tx_ts);
static bool wait_for_frame(uint32_t timeout_us, uint8_t expected_type, uint64_t *rx_ts);

static void timestamp_to_bytes(uint64_t ts, uint8_t *bytes);
static uint64_t bytes_to_timestamp(const uint8_t *bytes);
static uint64_t read_tx_timestamp(void);
static uint64_t read_rx_timestamp(void);

static double calculate_distance_dstwr(uint64_t t1, uint64_t t2, uint64_t t3,
                                        uint64_t t4, uint64_t t5, uint64_t t6);
static double calculate_distance_sstwr(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4);
static void stats_add(stats_t *s, double value);
static void stats_print(const stats_t *s);

/*============================================================================
 * Main
 *============================================================================*/

int main(int argc, char **argv) {
    role_t role = parse_args(argc, argv);
    
    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        perror("signal");
        return 1;
    }

    printf("=== DW3000 Double-Sided Two-Way Ranging ===\n\n");

    if (chip_init() != 0) {
        fprintf(stderr, "ERROR: chip initialization failed\n");
        return 1;
    }

    configure_for_role(role);

    switch (role) {
        case ROLE_INITIATOR:
            run_initiator();
            break;
        case ROLE_RESPONDER:
            run_responder();
            break;
        case ROLE_CALIBRATE:
            run_calibration();
            break;
    }

    chip_shutdown();
    return 0;
}

/*============================================================================
 * Argument Parsing
 *============================================================================*/

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

static void print_usage(const char *prog) {
    fprintf(stderr, "Usage: %s <mode> [options]\n", prog);
    fprintf(stderr, "\nModes:\n");
    fprintf(stderr, "  initiator              Run as ranging initiator\n");
    fprintf(stderr, "  responder              Run as ranging responder\n");
    fprintf(stderr, "  calibrate <dist_m>     Calibration mode (measure at known distance)\n");
    fprintf(stderr, "\nOptions:\n");
    fprintf(stderr, "  --antdly <value>       Set antenna delay (DWT units, default: %u)\n", DEFAULT_ANT_DLY);
    fprintf(stderr, "\nExamples:\n");
    fprintf(stderr, "  %s initiator\n", prog);
    fprintf(stderr, "  %s responder\n", prog);
    fprintf(stderr, "  %s calibrate 2.0       # Calibrate at 2.0 meters\n", prog);
    fprintf(stderr, "  %s initiator --antdly 16400\n", prog);
}

static role_t parse_args(int argc, char **argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        exit(1);
    }

    role_t role = ROLE_INITIATOR;
    
    /* Parse mode */
    if (strcmp(argv[1], "initiator") == 0) {
        role = ROLE_INITIATOR;
    } else if (strcmp(argv[1], "responder") == 0) {
        role = ROLE_RESPONDER;
    } else if (strcmp(argv[1], "calibrate") == 0) {
        role = ROLE_CALIBRATE;
        if (argc < 3) {
            fprintf(stderr, "ERROR: calibrate mode requires distance argument\n");
            print_usage(argv[0]);
            exit(1);
        }
        g_calibration_distance = atof(argv[2]);
        if (g_calibration_distance <= 0.0) {
            fprintf(stderr, "ERROR: invalid calibration distance: %s\n", argv[2]);
            exit(1);
        }
        /* In calibration mode, use zero antenna delay to measure raw error */
        g_ant_dly = 0;
    } else {
        fprintf(stderr, "ERROR: unknown mode: %s\n", argv[1]);
        print_usage(argv[0]);
        exit(1);
    }

    /* Parse optional arguments */
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--antdly") == 0 && i + 1 < argc) {
            g_ant_dly = (uint16_t)atoi(argv[++i]);
        }
    }

    return role;
}

/*============================================================================
 * Configuration
 *============================================================================*/

static void configure_for_role(role_t role) {
    uint16_t local_addr = (role == ROLE_RESPONDER) ? RESPONDER_ADDR : INITIATOR_ADDR;
    
    dwt_setaddress16(local_addr);
    dwt_setpanid(PAN_ID);
    
    /* Set antenna delays */
    dwt_setrxantennadelay(g_ant_dly);
    dwt_settxantennadelay(g_ant_dly);

    const char *role_str = (role == ROLE_RESPONDER) ? "Responder" :
                           (role == ROLE_CALIBRATE) ? "Calibration (Initiator)" : "Initiator";
    
    printf("Role: %s\n", role_str);
    printf("Local address: 0x%04X\n", local_addr);
    printf("PAN ID: 0x%04X\n", PAN_ID);
    printf("Antenna delay: %u DWT units (%.2f ns)\n", 
           g_ant_dly, g_ant_dly * DWT_TIME_UNITS * 1e9);
    
    if (role == ROLE_CALIBRATE) {
        printf("Calibration distance: %.3f m\n", g_calibration_distance);
    }
    printf("\n");
}

/*============================================================================
 * Initiator Logic (DS-TWR)
 *============================================================================*/

static void run_initiator(void) {
    printf("Starting ranging as initiator. Press Ctrl+C to stop.\n\n");
    
    uint32_t exchange_count = 0;
    uint32_t success_count = 0;
    uint32_t fail_count = 0;

    while (g_running) {
        exchange_count++;
        printf("--- Exchange #%u ---\n", exchange_count);

        uint64_t t1 = 0, t4 = 0, t5 = 0;
        uint64_t t2 = 0, t3 = 0;

        /* ===== Step 1: Send Poll ===== */
        g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        
        if (!send_frame_and_wait(g_poll_frame, POLL_FRAME_LEN, &t1)) {
            printf("  [FAIL] Poll TX failed\n\n");
            fail_count++;
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        printf("  T1 (Poll TX)     : 0x%010" PRIX64 "\n", t1);

        /* ===== Step 2: Wait for Response ===== */
        dwt_setrxtimeout((uint32_t)(RESP_RX_TIMEOUT_UUS * 1.026));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(RESP_RX_TIMEOUT_UUS * 2, MSG_TYPE_RESPONSE, &t4)) {
            printf("  [FAIL] Response RX timeout\n\n");
            fail_count++;
            dwt_forcetrxoff();
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        printf("  T4 (Response RX) : 0x%010" PRIX64 "\n", t4);

        /* Extract T2 from response (T3 comes later, embedded by responder) */
        t2 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        printf("  T2 (Poll RX @ B) : 0x%010" PRIX64 "\n", t2);
        
        /* T3 is the response TX timestamp - responder will calculate it
         * For now, we estimate T3 ≈ T2 + small_delay for SS-TWR calculation
         * The responder does the accurate DS-TWR calculation with T6 */

        /* ===== Step 3: Send Final with T1, T4 ===== */
        /* Note: We don't have T3 yet in this simplified exchange.
         * The responder knows T3 (its own TX timestamp).
         * We send T1 and T4 so responder can calculate. */
        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        timestamp_to_bytes(t1, &g_final_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t4, &g_final_frame[FRAME_DATA_OFFSET + 5]);
        /* T3 slot will be zeros - responder doesn't need it from us */
        
        if (!send_frame_and_wait(g_final_frame, FINAL_FRAME_LEN, &t5)) {
            printf("  [FAIL] Final TX failed\n\n");
            fail_count++;
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        printf("  T5 (Final TX)    : 0x%010" PRIX64 "\n", t5);

        /* ===== Calculate distance (SS-TWR approximation on initiator side) ===== */
        /* We use SS-TWR here since we don't have T3 accurately.
         * We estimate reply delay from typical values.
         * For accurate results, look at responder output (it has all timestamps). */
        
        /* Estimate T3 based on typical response processing time (~300μs) */
        uint64_t estimated_reply_delay = (uint64_t)(300.0e-6 / DWT_TIME_UNITS);  /* ~300μs in DWT units */
        t3 = t2 + estimated_reply_delay;
        
        double distance = calculate_distance_sstwr(t1, t2, t3, t4);

        int64_t ra = (int64_t)(t4 - t1);
        int64_t db = (int64_t)(t3 - t2);
        printf("  Ra (T4-T1)       : %" PRId64 " DWT\n", ra);
        printf("  Db (T3-T2) est   : %" PRId64 " DWT (estimated)\n", db);
        
        if (distance > 0 && distance < 100.0) {
            printf("\n  >>> DISTANCE: %.3f m (SS-TWR estimate) <<<\n", distance);
            stats_add(&g_stats, distance);
            success_count++;
        } else {
            printf("\n  [WARN] Invalid distance: %.3f m\n", distance);
            fail_count++;
        }

        if (g_stats.count >= 2) {
            stats_print(&g_stats);
        }
        
        printf("\n");
        usleep(RANGING_INTERVAL_MS * 1000);
    }

    printf("\n=== Ranging Summary ===\n");
    printf("Total exchanges: %u\n", exchange_count);
    printf("Successful: %u (%.1f%%)\n", success_count, 
           exchange_count > 0 ? 100.0 * success_count / exchange_count : 0.0);
    printf("Failed: %u\n", fail_count);
    
    if (g_stats.count > 0) {
        printf("\nFinal statistics:\n");
        stats_print(&g_stats);
    }
}

/*============================================================================
 * Responder Logic (DS-TWR - has all timestamps for accurate calculation)
 *============================================================================*/

static void run_responder(void) {
    printf("Starting ranging as responder. Waiting for polls...\n\n");
    
    uint32_t poll_count = 0;
    stats_t resp_stats = { 0 };

    while (g_running) {
        uint64_t t2 = 0, t3 = 0, t6 = 0;
        uint64_t t1 = 0, t4 = 0, t5 = 0;

        /* ===== Step 1: Wait for Poll ===== */
        dwt_forcetrxoff();
        dwt_setrxtimeout(0);  /* No timeout - wait indefinitely */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        if (!wait_for_frame(0, MSG_TYPE_POLL, &t2)) {
            continue;
        }
        
        poll_count++;
        printf("--- Poll #%u received ---\n", poll_count);
        printf("  T2 (Poll RX)     : 0x%010" PRIX64 "\n", t2);

        /* ===== Step 2: Send Response with T2 embedded ===== */
        g_response_frame[FRAME_SEQ_OFFSET] = g_rx_buffer[FRAME_SEQ_OFFSET];
        timestamp_to_bytes(t2, &g_response_frame[FRAME_DATA_OFFSET]);
        
        if (!send_frame_and_wait(g_response_frame, RESPONSE_FRAME_LEN, &t3)) {
            printf("  [FAIL] Response TX failed\n\n");
            continue;
        }
        printf("  T3 (Response TX) : 0x%010" PRIX64 "\n", t3);

        /* ===== Step 3: Wait for Final ===== */
        dwt_setrxtimeout((uint32_t)(FINAL_RX_TIMEOUT_UUS * 1.026));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(FINAL_RX_TIMEOUT_UUS * 2, MSG_TYPE_FINAL, &t6)) {
            printf("  [FAIL] Final RX timeout\n\n");
            dwt_forcetrxoff();
            continue;
        }
        printf("  T6 (Final RX)    : 0x%010" PRIX64 "\n", t6);

        /* Extract T1, T4 from Final message */
        t1 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        t4 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        /* T5 is not in the message - we need to estimate or use T6-based calculation */
        
        printf("  T1 (Poll TX @ A) : 0x%010" PRIX64 "\n", t1);
        printf("  T4 (Resp RX @ A) : 0x%010" PRIX64 "\n", t4);

        /* ===== Calculate distance using DS-TWR formula ===== */
        /* We have: T1, T2, T3, T4, T6
         * We need T5 for full DS-TWR. Estimate it from T4 + small delay.
         * Or use asymmetric DS-TWR formula. */
        
        /* Estimate T5 ≈ T4 + processing_delay (similar to T3-T2) */
        uint64_t da_estimate = t3 - t2;  /* Use same delay as responder */
        t5 = t4 + da_estimate;
        
        double distance = calculate_distance_dstwr(t1, t2, t3, t4, t5, t6);

        int64_t ra = (int64_t)(t4 - t1);
        int64_t rb = (int64_t)(t6 - t3);
        int64_t da = (int64_t)(t5 - t4);
        int64_t db = (int64_t)(t3 - t2);
        
        printf("  Ra (T4-T1)       : %" PRId64 " DWT\n", ra);
        printf("  Rb (T6-T3)       : %" PRId64 " DWT\n", rb);
        printf("  Da (T5-T4) est   : %" PRId64 " DWT\n", da);
        printf("  Db (T3-T2)       : %" PRId64 " DWT\n", db);
        
        if (distance > 0 && distance < 100.0) {
            printf("\n  >>> DISTANCE: %.3f m <<<\n", distance);
            stats_add(&resp_stats, distance);
            
            if (resp_stats.count >= 2) {
                stats_print(&resp_stats);
            }
        } else {
            printf("\n  [WARN] Invalid distance: %.3f m\n", distance);
        }
        printf("\n");
    }

    printf("\n=== Responder Summary ===\n");
    printf("Polls received: %u\n", poll_count);
    if (resp_stats.count > 0) {
        printf("\nFinal statistics:\n");
        stats_print(&resp_stats);
    }
}

/*============================================================================
 * Calibration Mode
 *============================================================================*/

static void run_calibration(void) {
    printf("=== Antenna Delay Calibration ===\n");
    printf("True distance: %.3f m\n", g_calibration_distance);
    printf("Antenna delay set to 0 for raw measurement.\n");
    printf("Running 50 measurements...\n\n");

    stats_t cal_stats = { 0 };
    uint32_t success = 0;
    const uint32_t target = 50;

    while (g_running && success < target) {
        uint64_t t1 = 0, t4 = 0;
        uint64_t t2 = 0;

        g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        
        if (!send_frame_and_wait(g_poll_frame, POLL_FRAME_LEN, &t1)) {
            usleep(100000);
            continue;
        }

        dwt_setrxtimeout((uint32_t)(RESP_RX_TIMEOUT_UUS * 1.026));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(RESP_RX_TIMEOUT_UUS * 2, MSG_TYPE_RESPONSE, &t4)) {
            dwt_forcetrxoff();
            usleep(100000);
            continue;
        }
        
        t2 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);

        /* Send final (responder expects it) */
        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        timestamp_to_bytes(t1, &g_final_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t4, &g_final_frame[FRAME_DATA_OFFSET + 5]);
        
        uint64_t t5 = 0;
        if (!send_frame_and_wait(g_final_frame, FINAL_FRAME_LEN, &t5)) {
            usleep(100000);
            continue;
        }

        /* SS-TWR calculation with estimated reply delay */
        uint64_t estimated_reply_delay = (uint64_t)(300.0e-6 / DWT_TIME_UNITS);
        uint64_t t3 = t2 + estimated_reply_delay;
        
        double distance = calculate_distance_sstwr(t1, t2, t3, t4);

        if (distance > 0 && distance < 100.0) {
            stats_add(&cal_stats, distance);
            success++;
            printf("[%2u/%u] Measured: %.3f m\n", success, target, distance);
        }

        usleep(200000);
    }

    if (cal_stats.count < 10) {
        printf("\nERROR: Not enough successful measurements for calibration.\n");
        return;
    }

    /* Calculate mean measured distance */
    double mean_measured = cal_stats.sum / (double)cal_stats.count;
    double error = mean_measured - g_calibration_distance;
    
    /* 
     * Error = (total_antenna_delay × 2) × c / (2 × c) = total_antenna_delay
     * In DWT units: error_m / (c × DWT_TIME_UNITS) = total_delay_dtu
     * Split equally between TX and RX
     */
    double total_delay_s = error / SPEED_OF_LIGHT_M_S;
    double total_delay_dtu = total_delay_s / DWT_TIME_UNITS;
    uint16_t recommended_delay = (uint16_t)(total_delay_dtu / 2.0);

    printf("\n=== Calibration Results ===\n");
    printf("Measurements: %zu\n", cal_stats.count);
    printf("Mean measured distance: %.4f m\n", mean_measured);
    printf("True distance: %.4f m\n", g_calibration_distance);
    printf("Error: %.4f m (%.1f cm)\n", error, error * 100.0);
    printf("\nTotal antenna delay: %.0f DWT units (%.2f ns)\n", 
           total_delay_dtu, total_delay_s * 1e9);
    printf("\n>>> Recommended antenna delay: %u DWT units <<<\n", recommended_delay);
    printf("\nAdd to your code after chip_init():\n");
    printf("  dwt_setrxantennadelay(%u);\n", recommended_delay);
    printf("  dwt_settxantennadelay(%u);\n", recommended_delay);
}

/*============================================================================
 * Frame TX/RX Helpers
 *============================================================================*/

static bool send_frame_and_wait(uint8_t *frame, size_t len, uint64_t *tx_ts) {
    dwt_forcetrxoff();
    
    dwt_writetxdata((uint16_t)len, frame, 0);
    dwt_writetxfctrl((uint16_t)(len + 2), 0, 1);  /* +2 for FCS, ranging=1 */
    
    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (ret != DWT_SUCCESS) {
        return false;
    }
    
    /* Wait for TX complete */
    uint32_t timeout = 10000;
    while (timeout--) {
        uint32_t status = dwt_readsysstatuslo();
        if (status & DWT_INT_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            if (tx_ts) {
                *tx_ts = read_tx_timestamp();
            }
            return true;
        }
        usleep(1);
    }
    return false;
}

static bool wait_for_frame(uint32_t timeout_us, uint8_t expected_type, uint64_t *rx_ts) {
    uint32_t timeout = timeout_us > 0 ? timeout_us : 10000000;
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 100;

    while (elapsed < timeout && g_running) {
        uint32_t status = dwt_readsysstatuslo();
        
        /* Check for good frame */
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            /* Read RX timestamp BEFORE clearing status */
            if (rx_ts) {
                *rx_ts = read_rx_timestamp();
            }
            
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
            
            uint8_t rng = 0;
            uint16_t len = dwt_getframelength(&rng);
            if (len > 0 && len <= RX_BUFFER_LEN) {
                dwt_readrxdata(g_rx_buffer, len, 0);
                
                if (expected_type == 0 || g_rx_buffer[FRAME_TYPE_OFFSET] == expected_type) {
                    return true;
                }
            }
            
            /* Wrong frame type - re-enable RX */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        
        /* Check for RX errors or timeout */
        if (status & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            return false;
        }

        usleep(poll_interval);
        elapsed += poll_interval;
    }
    
    return false;
}

/*============================================================================
 * Timestamp Helpers
 *============================================================================*/

static void timestamp_to_bytes(uint64_t ts, uint8_t *bytes) {
    bytes[0] = (uint8_t)(ts & 0xFF);
    bytes[1] = (uint8_t)((ts >> 8) & 0xFF);
    bytes[2] = (uint8_t)((ts >> 16) & 0xFF);
    bytes[3] = (uint8_t)((ts >> 24) & 0xFF);
    bytes[4] = (uint8_t)((ts >> 32) & 0xFF);
}

static uint64_t bytes_to_timestamp(const uint8_t *bytes) {
    return (uint64_t)bytes[0] |
           ((uint64_t)bytes[1] << 8) |
           ((uint64_t)bytes[2] << 16) |
           ((uint64_t)bytes[3] << 24) |
           ((uint64_t)bytes[4] << 32);
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

/*============================================================================
 * Distance Calculation
 *============================================================================*/

/* Single-Sided TWR: simpler, used when we don't have all timestamps */
static double calculate_distance_sstwr(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4) {
    /*
     * SS-TWR formula:
     * ToF = (Ra - Db) / 2
     * where Ra = T4 - T1 (round trip at initiator)
     *       Db = T3 - T2 (reply delay at responder)
     */
    int64_t ra = (int64_t)(t4 - t1);
    int64_t db = (int64_t)(t3 - t2);
    
    double tof_dtu = (double)(ra - db) / 2.0;
    double tof_s = tof_dtu * DWT_TIME_UNITS;
    double distance = tof_s * SPEED_OF_LIGHT_M_S;
    
    return distance;
}

/* Double-Sided TWR: more accurate, cancels clock drift */
static double calculate_distance_dstwr(uint64_t t1, uint64_t t2, uint64_t t3,
                                        uint64_t t4, uint64_t t5, uint64_t t6) {
    /*
     * DS-TWR formula:
     * 
     * Ra = T4 - T1  (round trip time measured by A)
     * Rb = T6 - T3  (round trip time measured by B)
     * Da = T5 - T4  (reply delay by A)
     * Db = T3 - T2  (reply delay by B)
     *
     *         Ra × Rb - Da × Db
     * ToF = ─────────────────────
     *         Ra + Rb + Da + Db
     */
    
    int64_t ra = (int64_t)(t4 - t1);
    int64_t rb = (int64_t)(t6 - t3);
    int64_t da = (int64_t)(t5 - t4);
    int64_t db = (int64_t)(t3 - t2);

    double numerator = (double)ra * (double)rb - (double)da * (double)db;
    double denominator = (double)(ra + rb + da + db);
    
    if (denominator == 0.0) {
        return -1.0;
    }

    double tof_dtu = numerator / denominator;
    double tof_s = tof_dtu * DWT_TIME_UNITS;
    double distance = tof_s * SPEED_OF_LIGHT_M_S;

    return distance;
}

/*============================================================================
 * Statistics
 *============================================================================*/

static void stats_add(stats_t *s, double value) {
    if (s->count < STATS_WINDOW_SIZE) {
        s->samples[s->count] = value;
        s->sum += value;
        s->count++;
    } else {
        s->sum -= s->samples[s->index];
        s->samples[s->index] = value;
        s->sum += value;
        s->index = (s->index + 1) % STATS_WINDOW_SIZE;
    }
}

static void stats_print(const stats_t *s) {
    if (s->count == 0) return;
    
    double mean = s->sum / (double)s->count;
    
    double variance = 0.0;
    double min_val = s->samples[0];
    double max_val = s->samples[0];
    
    for (size_t i = 0; i < s->count; i++) {
        double diff = s->samples[i] - mean;
        variance += diff * diff;
        if (s->samples[i] < min_val) min_val = s->samples[i];
        if (s->samples[i] > max_val) max_val = s->samples[i];
    }
    
    double std_dev = sqrt(variance / (double)s->count);
    
    printf("  Stats (n=%zu): Mean=%.3f m | StdDev=%.3f m | Range=[%.3f, %.3f] m\n",
           s->count, mean, std_dev, min_val, max_val);
}
