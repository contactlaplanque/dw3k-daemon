/*
 * test_twr.c – Single-Sided Two-Way Ranging (SS-TWR) for distance measurement
 *
 * Usage:
 *   ./test_twr initiator              - Run as initiator (starts ranging)
 *   ./test_twr responder              - Run as responder (replies to polls)
 *   ./test_twr calibrate <distance_m> - Calibration mode (run as initiator)
 *
 * SS-TWR Message Exchange:
 *   Initiator                    Responder
 *       |                            |
 *   T1 -|-------- Poll ------------->|- T2
 *       |                            |
 *   T4 -|<------- Response ----------|- T3  (Response embeds T2 and T3)
 *       |                            |
 *
 * Distance = c × ToF, where ToF = (Ra - Db) / 2
 *   Ra = T4 - T1 (round trip measured by initiator)
 *   Db = T3 - T2 (reply delay measured by responder)
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
#define RESPONSE_FRAME_LEN  20U   /* Poll frame + T2(5) + T3(5) */

/* Timing */
#define RESP_RX_TIMEOUT_UUS         10000U  /* Response RX timeout (μs) - added to reply delay */
#define REPLY_DELAY_UUS             3000U   /* Responder reply delay (μs) - must match responder */
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
    0, 0, 0, 0, 0,          /* T2 placeholder (5 bytes) - poll RX timestamp */
    0, 0, 0, 0, 0           /* T3 placeholder (5 bytes) - response TX timestamp */
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

static double calculate_distance(int64_t ra, int64_t db);
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

    printf("=== DW3000 Single-Sided Two-Way Ranging ===\n\n");

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
 * Initiator Logic (SS-TWR)
 *============================================================================*/

static void run_initiator(void) {
    printf("Starting ranging as initiator. Press Ctrl+C to stop.\n\n");
    
    uint32_t exchange_count = 0;
    uint32_t success_count = 0;
    uint32_t fail_count = 0;

    while (g_running) {
        exchange_count++;
        printf("--- Exchange #%u ---\n", exchange_count);

        uint64_t t1 = 0, t4 = 0;
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
        /* Timeout = reply delay + margin for TX time + propagation */
        uint32_t rx_timeout = (uint32_t)((REPLY_DELAY_UUS + RESP_RX_TIMEOUT_UUS) * 1.026);
        dwt_setrxtimeout(rx_timeout);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        if (!wait_for_frame(RESP_RX_TIMEOUT_UUS * 2, MSG_TYPE_RESPONSE, &t4)) {
            printf("  [FAIL] Response RX timeout\n\n");
            fail_count++;
            dwt_forcetrxoff();
            usleep(RANGING_INTERVAL_MS * 1000);
            continue;
        }
        printf("  T4 (Response RX) : 0x%010" PRIX64 "\n", t4);

        /* Extract T2 and T3 from response */
        t2 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET]);
        t3 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);
        printf("  T2 (Poll RX @ B) : 0x%010" PRIX64 "\n", t2);
        printf("  T3 (Resp TX @ B) : 0x%010" PRIX64 "\n", t3);

        /* ===== Calculate distance using SS-TWR ===== */
        int64_t ra = (int64_t)(t4 - t1);  /* Round trip (initiator clock) */
        int64_t db = (int64_t)(t3 - t2);  /* Reply delay (responder clock) */
        
        printf("  Ra (T4-T1)       : %" PRId64 " DWT (%.3f μs)\n", ra, ra * DWT_TIME_UNITS * 1e6);
        printf("  Db (T3-T2)       : %" PRId64 " DWT (%.3f μs)\n", db, db * DWT_TIME_UNITS * 1e6);

        double distance = calculate_distance(ra, db);
        
        double tof_dtu = (double)(ra - db) / 2.0;
        printf("  ToF              : %.0f DWT (%.3f ns)\n", tof_dtu, tof_dtu * DWT_TIME_UNITS * 1e9);
        
        if (distance >= 0 && distance < 100.0) {
            printf("\n  >>> DISTANCE: %.3f m <<<\n", distance);
            stats_add(&g_stats, distance);
            success_count++;
            
            if (g_stats.count >= 2) {
                stats_print(&g_stats);
            }
        } else {
            printf("\n  [WARN] Invalid distance: %.3f m\n", distance);
            fail_count++;
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
 * Responder Logic (SS-TWR)
 * 
 * The responder needs to send T2 (poll RX timestamp) and T3 (response TX timestamp)
 * to the initiator. The challenge is that T3 is only known AFTER transmission.
 * 
 * Solution: Use delayed TX with a generous delay to ensure we have time to:
 * 1. Read T2
 * 2. Compute T3 = T2 + delay
 * 3. Write the frame with T2 and T3 embedded
 * 4. Start delayed TX before T3 arrives
 *============================================================================*/

static void run_responder(void) {
    printf("Starting ranging as responder. Waiting for polls...\n\n");
    
    uint32_t poll_count = 0;

    /* 
     * Reply delay in DWT units.
     * Must be long enough for:
     * - Reading RX timestamp (~10μs)
     * - Computing and writing response frame (~100μs)  
     * - SPI transfers (~50μs)
     * - Safety margin
     * 
     * 3ms is very conservative but guarantees success.
     * Uses REPLY_DELAY_UUS macro defined at top of file.
     */
    const uint64_t reply_delay_dtu = (uint64_t)(REPLY_DELAY_UUS * 1e-6 / DWT_TIME_UNITS);
    const uint64_t tx_ant_delay_dtu = g_ant_dly;
    
    printf("Reply delay: %u μs (%llu DWT units)\n\n", 
           REPLY_DELAY_UUS, 
           (unsigned long long)reply_delay_dtu);
    
    while (g_running) {
        uint64_t t2 = 0, t3_scheduled = 0;

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

        /* ===== Step 2: Calculate scheduled T3 and prepare response ===== */
        /* Schedule TX at T2 + reply_delay */
        t3_scheduled = t2 + reply_delay_dtu;
        
        /* Embed T2 and T3 in response */
        uint64_t t3_actual_target = t3_scheduled; /* Actual air time after TX antenna delay */
        uint64_t dx_time_target = t3_actual_target - tx_ant_delay_dtu; /* Programmed value excludes tx antenna delay */

        g_response_frame[FRAME_SEQ_OFFSET] = g_rx_buffer[FRAME_SEQ_OFFSET];
        timestamp_to_bytes(t2, &g_response_frame[FRAME_DATA_OFFSET]);
        timestamp_to_bytes(t3_actual_target, &g_response_frame[FRAME_DATA_OFFSET + 5]);

        /* ===== Step 3: Send response at scheduled time T3 ===== */
        dwt_forcetrxoff();
        dwt_writetxdata((uint16_t)RESPONSE_FRAME_LEN, g_response_frame, 0);
        dwt_writetxfctrl((uint16_t)(RESPONSE_FRAME_LEN + 2), 0, 1);
        
        /* 
         * Set delayed TX time. 
         * The DW3000 uses bits [8:39] of the 40-bit timestamp for delayed TX.
         * Program DX_TIME with the desired start minus TX antenna delay so that
         * the actual airborne timestamp matches t3_actual_target.
         */
        uint32_t tx_time = (uint32_t)(((dx_time_target) >> 8) & 0xFFFFFFFE);  /* Must be even */
        dwt_setdelayedtrxtime(tx_time);
        
        int ret = dwt_starttx(DWT_START_TX_DELAYED);
        if (ret == DWT_SUCCESS) {
            /* Wait for TX complete */
            uint32_t timeout = 50000;  /* 50ms max wait */
            bool tx_done = false;
            while (timeout-- && !tx_done) {
                uint32_t status = dwt_readsysstatuslo();
                if (status & DWT_INT_TXFRS_BIT_MASK) {
                    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                    tx_done = true;
                }
                usleep(10);
            }
            
            if (tx_done) {
                /* Read actual TX timestamp to verify */
                uint64_t actual_t3 = read_tx_timestamp();
                int64_t db = (int64_t)(actual_t3 - t2);
                
                printf("  T3 (Response TX) : 0x%010" PRIX64 "\n", actual_t3);
                printf("  Db (T3-T2)       : %" PRId64 " DWT (%.3f μs)\n", 
                       db, db * DWT_TIME_UNITS * 1e6);
                
                /* Check if actual matches scheduled (within 1 DWT unit = ~15ps) */
                int64_t diff = (int64_t)(actual_t3 - t3_actual_target);
                if (diff < -10 || diff > 10) {
                    printf("  [WARN] T3 drift: %+" PRId64 " DWT from scheduled\n", diff);
                }
            } else {
                printf("  [FAIL] Response TX timeout (no TXFRS)\n");
            }
        } else {
            /* Delayed TX failed - scheduled time already passed */
            printf("  [FAIL] Delayed TX late! Processing took too long.\n");
            printf("  Scheduled T3: 0x%010" PRIX64 "\n", t3_scheduled);
            
            /* Don't send - the initiator will timeout and retry */
        }
        
        printf("\n");
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
    printf("Using antenna delay: %u DWT (%.2f ns)\n",
           g_ant_dly, g_ant_dly * DWT_TIME_UNITS * 1e9);
    printf("Make sure the responder runs with the same delay "
           "(e.g. ./test_twr responder --antdly %u).\n", g_ant_dly);
    printf("Running 50 measurements...\n\n");

    stats_t cal_stats = { 0 };
    uint32_t success = 0;
    const uint32_t target = 50;

    while (g_running && success < target) {
        uint64_t t1 = 0, t4 = 0;
        uint64_t t2 = 0, t3 = 0;

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
        t3 = bytes_to_timestamp(&g_rx_buffer[FRAME_DATA_OFFSET + 5]);

        int64_t ra = (int64_t)(t4 - t1);
        int64_t db = (int64_t)(t3 - t2);
        
        double distance = calculate_distance(ra, db);

        if (distance >= 0 && distance < 100.0) {
            stats_add(&cal_stats, distance);
            success++;
            printf("[%2u/%u] Measured: %.3f m (Ra=%.1fμs, Db=%.1fμs)\n", 
                   success, target, distance,
                   ra * DWT_TIME_UNITS * 1e6,
                   db * DWT_TIME_UNITS * 1e6);
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
     * The antenna delay adds to the measured distance.
     * error = measured - true = (antenna_delay_A + antenna_delay_B) * c
     * Since both devices contribute equally:
     * single_device_delay = error / (2 * c) in seconds
     * In DWT units: delay_dtu = single_device_delay / DWT_TIME_UNITS
     */
    double delta_dtu = error / (SPEED_OF_LIGHT_M_S * DWT_TIME_UNITS);
    double recommended_delay_f = (double)g_ant_dly - delta_dtu;
    if (recommended_delay_f < 0.0) {
        recommended_delay_f = 0.0;
    }
    if (recommended_delay_f > 65535.0) {
        recommended_delay_f = 65535.0;
    }
    uint16_t recommended_delay = (uint16_t)llround(recommended_delay_f);
    int32_t delta_dly = (int32_t)recommended_delay - (int32_t)g_ant_dly;

    printf("\n=== Calibration Results ===\n");
    printf("Measurements: %zu\n", cal_stats.count);
    printf("Mean measured distance: %.4f m\n", mean_measured);
    printf("True distance: %.4f m\n", g_calibration_distance);
    printf("Error: %+.4f m (%+.1f cm)\n", error, error * 100.0);
    
    if (error > 0) {
        printf("\nMeasured > True: decrease antenna delay\n");
    } else if (error < 0) {
        printf("\nMeasured < True: increase antenna delay\n");
    } else {
        printf("\nMeasured equals true distance.\n");
    }
    
    printf("\n>>> Recommended antenna delay: %u DWT units (%.2f ns) "
           "[delta %+d DWT] <<<\n", 
           recommended_delay, recommended_delay * DWT_TIME_UNITS * 1e9,
           delta_dly);
    printf("\nAdd to your code after chip_init():\n");
    printf("  dwt_setrxantennadelay(%u);\n", recommended_delay);
    printf("  dwt_settxantennadelay(%u);\n", recommended_delay);
    printf("\nOr run with: ./test_twr initiator --antdly %u\n", recommended_delay);
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
 * Distance Calculation (SS-TWR)
 *============================================================================*/

static double calculate_distance(int64_t ra, int64_t db) {
    /*
     * SS-TWR formula:
     * ToF = (Ra - Db) / 2
     * Distance = ToF × c
     *
     * where:
     *   Ra = T4 - T1 (round trip measured by initiator)
     *   Db = T3 - T2 (reply delay measured by responder)
     */
    double tof_dtu = (double)(ra - db) / 2.0;
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
