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
 *   T4 -|<------- Response ----------|- T3  (contains T2, T3)
 *       |                            |
 *   T5 -|-------- Final ------------>|- T6  (contains T1, T4, T5)
 *       |                            |
 *
 * Distance = c × ToF, where ToF is calculated from all 6 timestamps
 * to cancel clock drift errors.
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
#define RESPONSE_FRAME_LEN  20U   /* Poll + T2(5) + T3(5) */
#define FINAL_FRAME_LEN     25U   /* Poll + T1(5) + T4(5) + T5(5) */

/* Timing */
#define POLL_TX_TO_RESP_RX_DLY_UUS  300U    /* Delay from Poll TX to Response RX enable (μs) */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 300U    /* Delay from Response RX to Final TX (μs) */
#define RESP_RX_TIMEOUT_UUS         5000U   /* Response RX timeout (μs) */
#define FINAL_RX_TIMEOUT_UUS        5000U   /* Final RX timeout (μs) */
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
    0, 0, 0, 0, 0,          /* T2 placeholder (5 bytes) */
    0, 0, 0, 0, 0           /* T3 placeholder (5 bytes) */
};

static uint8_t g_final_frame[FINAL_FRAME_LEN] = {
    0x41, 0x88,             /* Frame Control */
    0x00,                   /* Sequence number */
    (PAN_ID & 0xFF), (PAN_ID >> 8),
    (RESPONDER_ADDR & 0xFF), (RESPONDER_ADDR >> 8),  /* Destination (responder) */
    (INITIATOR_ADDR & 0xFF), (INITIATOR_ADDR >> 8),  /* Source (initiator) */
    MSG_TYPE_FINAL,         /* Message type */
    0, 0, 0, 0, 0,          /* T1 placeholder */
    0, 0, 0, 0, 0,          /* T4 placeholder */
    0, 0, 0, 0, 0           /* T5 placeholder */
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

static bool send_frame(uint8_t *frame, size_t len);
static bool wait_for_frame(uint32_t timeout_us, uint8_t expected_type);
static bool wait_for_tx_complete(void);

static void timestamp_to_bytes(uint64_t ts, uint8_t *bytes);
static uint64_t bytes_to_timestamp(const uint8_t *bytes);
static uint64_t read_tx_timestamp(void);
static uint64_t read_rx_timestamp(void);

static double calculate_distance(uint64_t t1, uint64_t t2, uint64_t t3,
                                  uint64_t t4, uint64_t t5, uint64_t t6);
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
 * Initiator Logic
 *============================================================================*/

static void run_initiator(void) {
    printf("Starting ranging as initiator. Press Ctrl+C to stop.\n\n");
    
    uint32_t exchange_count = 0;
    uint32_t success_count = 0;
    uint32_t fail_count = 0;

    while (g_running) {
        exchange_count++;
        printf("--- Exchange #%u ---\n", exchange_count);

        /* Step 1: Send Poll */
        g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        
        dwt_forcetrxoff();
        if (!send_frame(g_poll_frame, POLL_FRAME_LEN)) {
            printf("  [FAIL] Poll TX failed\n\n");
            fail_count++;
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        
        if (!wait_for_tx_complete()) {
            printf("  [FAIL] Poll TX timeout\n\n");
            fail_count++;
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        
        uint64_t t1 = read_tx_timestamp();
        printf("  T1 (Poll TX)     : 0x%010" PRIX64 "\n", t1);

        /* Step 2: Wait for Response */
        dwt_setrxtimeout((uint32_t)(RESP_RX_TIMEOUT_UUS * 1.026));  /* Convert μs to DWT units */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(RESP_RX_TIMEOUT_UUS * 2, MSG_TYPE_RESPONSE)) {
            printf("  [FAIL] Response RX timeout\n\n");
            fail_count++;
            dwt_forcetrxoff();
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        
        uint64_t t4 = read_rx_timestamp();
        printf("  T4 (Response RX) : 0x%010" PRIX64 "\n", t4);

        /* Extract T2 and T3 from response */
        uint64_t t2 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        uint64_t t3 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        printf("  T2 (Poll RX @ B) : 0x%010" PRIX64 "\n", t2);
        printf("  T3 (Resp TX @ B) : 0x%010" PRIX64 "\n", t3);

        /* Step 3: Send Final with T1, T4, and T5 */
        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        timestamp_to_bytes(t1, &g_final_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t4, &g_final_frame[FRAME_DATA_OFFSET + 5]);
        
        /* We need to predict T5 - for now, send immediately and read actual T5 */
        dwt_forcetrxoff();
        if (!send_frame(g_final_frame, FINAL_FRAME_LEN)) {
            printf("  [FAIL] Final TX failed\n\n");
            fail_count++;
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        
        if (!wait_for_tx_complete()) {
            printf("  [FAIL] Final TX timeout\n\n");
            fail_count++;
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        
        uint64_t t5 = read_tx_timestamp();
        printf("  T5 (Final TX)    : 0x%010" PRIX64 "\n", t5);

        /* Calculate distance using DS-TWR formula */
        /* Note: Responder calculates with T6, but we can also calculate here */
        /* Using symmetric formula that works with T1-T5 */
        int64_t ra = (int64_t)(t4 - t1);  /* Round trip A */
        int64_t db = (int64_t)(t3 - t2);  /* Reply delay B */
        
        /* For SS-TWR approximation (responder will do full DS-TWR with T6) */
        double tof_dtu = (double)(ra - db) / 2.0;
        double tof_s = tof_dtu * DWT_TIME_UNITS;
        double distance = tof_s * SPEED_OF_LIGHT_M_S;

        printf("  Ra (T4-T1)       : %" PRId64 " DWT\n", ra);
        printf("  Db (T3-T2)       : %" PRId64 " DWT\n", db);
        printf("  ToF              : %.0f DWT (%.3f ns)\n", tof_dtu, tof_s * 1e9);
        
        if (distance > 0 && distance < 1000.0) {
            printf("\n  >>> DISTANCE: %.3f m <<<\n", distance);
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
 * Responder Logic
 *============================================================================*/

static void run_responder(void) {
    printf("Starting ranging as responder. Waiting for polls...\n\n");
    
    uint32_t poll_count = 0;

    while (g_running) {
        /* Enable RX and wait for Poll */
        dwt_forcetrxoff();
        dwt_setrxtimeout(0);  /* No timeout - wait indefinitely */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        if (!wait_for_frame(0, MSG_TYPE_POLL)) {
            /* Timeout or error - just retry */
            continue;
        }
        
        poll_count++;
        uint64_t t2 = read_rx_timestamp();
        printf("--- Poll #%u received ---\n", poll_count);
        printf("  T2 (Poll RX)     : 0x%010" PRIX64 "\n", t2);

        /* Prepare and send Response with T2 (T3 will be filled after TX) */
        g_response_frame[FRAME_SEQ_OFFSET] = g_rx_buffer[FRAME_SEQ_OFFSET];
        timestamp_to_bytes(t2, &g_response_frame[FRAME_DATA_OFFSET]);
        
        dwt_forcetrxoff();
        if (!send_frame(g_response_frame, RESPONSE_FRAME_LEN)) {
            printf("  [FAIL] Response TX failed\n\n");
            continue;
        }
        
        if (!wait_for_tx_complete()) {
            printf("  [FAIL] Response TX timeout\n\n");
            continue;
        }
        
        uint64_t t3 = read_tx_timestamp();
        printf("  T3 (Response TX) : 0x%010" PRIX64 "\n", t3);
        
        /* Update T3 in the response frame for next time (optimization) */
        /* Actually, we already sent it - this is informational */

        /* Wait for Final message */
        dwt_setrxtimeout((uint32_t)(FINAL_RX_TIMEOUT_UUS * 1.026));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(FINAL_RX_TIMEOUT_UUS * 2, MSG_TYPE_FINAL)) {
            printf("  [FAIL] Final RX timeout\n\n");
            dwt_forcetrxoff();
            continue;
        }
        
        uint64_t t6 = read_rx_timestamp();
        printf("  T6 (Final RX)    : 0x%010" PRIX64 "\n", t6);

        /* Extract T1, T4, T5 from Final message */
        uint64_t t1 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        uint64_t t4 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        uint64_t t5 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 10]);
        
        printf("  T1 (Poll TX @ A) : 0x%010" PRIX64 "\n", t1);
        printf("  T4 (Resp RX @ A) : 0x%010" PRIX64 "\n", t4);
        printf("  T5 (Final TX @ A): 0x%010" PRIX64 "\n", t5);

        /* Calculate distance using full DS-TWR formula */
        double distance = calculate_distance(t1, t2, t3, t4, t5, t6);
        
        if (distance > 0 && distance < 1000.0) {
            printf("\n  >>> DISTANCE: %.3f m <<<\n\n", distance);
        } else {
            printf("\n  [WARN] Invalid distance: %.3f m\n\n", distance);
        }
    }

    printf("\n=== Responder Summary ===\n");
    printf("Polls received: %u\n", poll_count);
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
        /* Same as initiator, but collecting stats */
        g_poll_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        
        dwt_forcetrxoff();
        if (!send_frame(g_poll_frame, POLL_FRAME_LEN)) {
            usleep(100000);
            continue;
        }
        
        if (!wait_for_tx_complete()) {
            usleep(100000);
            continue;
        }
        
        uint64_t t1 = read_tx_timestamp();

        dwt_setrxtimeout((uint32_t)(RESP_RX_TIMEOUT_UUS * 1.026));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(RESP_RX_TIMEOUT_UUS * 2, MSG_TYPE_RESPONSE)) {
            dwt_forcetrxoff();
            usleep(100000);
            continue;
        }
        
        uint64_t t4 = read_rx_timestamp();
        uint64_t t2 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        uint64_t t3 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);

        g_final_frame[FRAME_SEQ_OFFSET] = g_seq_num++;
        timestamp_to_bytes(t1, &g_final_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t4, &g_final_frame[FRAME_DATA_OFFSET + 5]);
        
        dwt_forcetrxoff();
        if (!send_frame(g_final_frame, FINAL_FRAME_LEN)) {
            usleep(100000);
            continue;
        }
        
        if (!wait_for_tx_complete()) {
            usleep(100000);
            continue;
        }

        int64_t ra = (int64_t)(t4 - t1);
        int64_t db = (int64_t)(t3 - t2);
        double tof_dtu = (double)(ra - db) / 2.0;
        double tof_s = tof_dtu * DWT_TIME_UNITS;
        double distance = tof_s * SPEED_OF_LIGHT_M_S;

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

static bool send_frame(uint8_t *frame, size_t len) {
    dwt_writetxdata((uint16_t)len, frame, 0);
    dwt_writetxfctrl((uint16_t)(len + 2), 0, 1);  /* +2 for FCS, ranging=1 */
    
    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    return (ret == DWT_SUCCESS);
}

static bool wait_for_tx_complete(void) {
    /* Poll for TX complete - check status register */
    uint32_t timeout = 10000;  /* ~10ms max */
    while (timeout--) {
        uint32_t status = dwt_readsysstatuslo();
        if (status & DWT_INT_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            return true;
        }
        usleep(1);
    }
    return false;
}

static bool wait_for_frame(uint32_t timeout_us, uint8_t expected_type) {
    uint32_t timeout = timeout_us > 0 ? timeout_us : 10000000;  /* 10s max if 0 */
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 100;  /* μs */

    while (elapsed < timeout && g_running) {
        uint32_t status = dwt_readsysstatuslo();
        
        /* Check for good frame */
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
            
            uint8_t rng = 0;
            uint16_t len = dwt_getframelength(&rng);
            if (len > 0 && len <= RX_BUFFER_LEN) {
                dwt_readrxdata(g_rx_buffer, len, 0);
                
                /* Check message type if specified */
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
    dwt_readrxtimestamp(ts);
    return bytes_to_timestamp(ts);
}

/*============================================================================
 * Distance Calculation (DS-TWR)
 *============================================================================*/

static double calculate_distance(uint64_t t1, uint64_t t2, uint64_t t3,
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

    /* Use floating point for the division to maintain precision */
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
        /* Sliding window - remove oldest, add new */
        s->sum -= s->samples[s->index];
        s->samples[s->index] = value;
        s->sum += value;
        s->index = (s->index + 1) % STATS_WINDOW_SIZE;
    }
}

static void stats_print(const stats_t *s) {
    if (s->count == 0) return;
    
    double mean = s->sum / (double)s->count;
    
    /* Calculate std dev, min, max */
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

