/*
 * test_beacon.c – Transmit periodic UWB beacons for scan testing
 *
 * Run this on one board while running test_scan on another.
 * Both must use the same channel and preamble codes (configured in init.c).
 */
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "init.h"

/* Beacon interval in microseconds (500 ms) */
#define BEACON_INTERVAL_US 500000U

static volatile sig_atomic_t keep_running = 1;

static void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

/*
 * Minimal IEEE 802.15.4 beacon frame structure:
 *
 * Offset  Field                Size    Description
 * ------  -----                ----    -----------
 * 0-1     Frame Control        2       Frame type, addressing modes, etc.
 * 2       Sequence Number      1       Incremented each transmission
 * 3-4     Source PAN ID        2       Our network ID (little-endian)
 * 5-6     Source Short Addr    2       Our short address (little-endian)
 * 7+      Payload              N       Application data
 * (FCS)   Frame Check Seq      2       CRC-16 (auto-appended by DW3000)
 *
 * Frame Control breakdown (0x8000):
 *   Bits 0-2:   Frame Type = 0b000 (Beacon)
 *   Bit 3:      Security Enabled = 0
 *   Bit 4:      Frame Pending = 0
 *   Bit 5:      AR (Ack Request) = 0
 *   Bit 6:      PAN ID Compression = 0
 *   Bits 7-9:   Reserved = 0
 *   Bits 10-11: Dest Addr Mode = 0b00 (not present)
 *   Bits 12-13: Frame Version = 0b00 (802.15.4-2003)
 *   Bits 14-15: Src Addr Mode = 0b10 (short address)
 */
static uint8_t beacon_frame[] = {
    /* Frame Control: Beacon, src addr mode = short (0b10 << 14 = 0x8000) */
    0x00, 0x80,
    /* Sequence number (updated each TX) */
    0x00,
    /* Source PAN ID: 0xDECA (little-endian) – must match scanner's PAN */
    0xCA, 0xDE,
    /* Source short address: 0x2001 (little-endian) – different from scanner */
    0x01, 0x20,
    /* Payload: identifier string */
    'D', 'W', '3', 'K', '-', 'B', 'E', 'A', 'C', 'O', 'N'
};
#define BEACON_FRAME_LEN sizeof(beacon_frame)

/* Print the beacon frame structure for educational purposes */
static void explain_beacon_frame(void) {
    printf("Beacon frame structure (%zu bytes + 2 FCS):\n", BEACON_FRAME_LEN);
    printf("  [0-1]  Frame Control : 0x%02X%02X (Beacon, src=short)\n",
           beacon_frame[1], beacon_frame[0]);
    printf("  [2]    Sequence      : 0x%02X\n", beacon_frame[2]);
    printf("  [3-4]  Source PAN    : 0x%02X%02X\n",
           beacon_frame[4], beacon_frame[3]);
    printf("  [5-6]  Source Addr   : 0x%02X%02X\n",
           beacon_frame[6], beacon_frame[5]);
    printf("  [7+]   Payload       : \"%.*s\"\n",
           (int)(BEACON_FRAME_LEN - 7), &beacon_frame[7]);
    printf("  (FCS auto-appended by DW3000)\n\n");
}

static void print_local_identity(void) {
    uint8_t eui[8] = { 0 };
    dwt_geteui(eui);

    printf("Beacon transmitter identity:\n");
    printf("  Device ID : 0x%08" PRIX32 "\n", dwt_readdevid());
    printf("  EUI-64    : %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
           eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
    printf("  PAN ID    : 0xDECA\n");
    printf("  Short Addr: 0x2001\n\n");
}

int main(void) {
    if (signal(SIGINT, handle_sigint) == SIG_ERR) {
        perror("signal");
        return 1;
    }

    printf("=== DW3000 Beacon Transmitter ===\n\n");

    if (chip_init() != 0) {
        fprintf(stderr, "ERROR: chip initialization failed\n");
        return 1;
    }

    /*
     * Override the short address to distinguish this device from the scanner.
     * The scanner uses 0x1000 (set in chip_init), we use 0x2001.
     */
    dwt_setaddress16(0x2001);

    print_local_identity();
    explain_beacon_frame();

    printf("Transmitting beacons every %u ms. Press Ctrl+C to stop.\n\n",
           BEACON_INTERVAL_US / 1000);

    uint8_t seq = 0;
    uint32_t tx_ok = 0;
    uint32_t tx_fail = 0;

    while (keep_running) {
        /* Update sequence number in the frame */
        beacon_frame[2] = seq;

        /*
         * Load frame data into the DW3000 TX buffer.
         * The chip has a 1024-byte TX buffer starting at offset 0.
         */
        dwt_writetxdata((uint16_t)BEACON_FRAME_LEN, beacon_frame, 0);

        /*
         * Set the TX frame control register:
         * - Frame length = data + 2 bytes for FCS (auto-appended)
         * - Buffer offset = 0
         * - Ranging bit = 0 (not a ranging frame)
         */
        dwt_writetxfctrl((uint16_t)(BEACON_FRAME_LEN + 2), 0, 0);

        /*
         * Start transmission immediately.
         * DWT_START_TX_IMMEDIATE = 0x00: transmit now
         * Other options include delayed TX for precise timing.
         */
        int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

        if (ret == DWT_SUCCESS) {
            tx_ok++;
            printf("[%3u] Beacon sent (total: %" PRIu32 ")\n", seq, tx_ok);
        } else {
            tx_fail++;
            fprintf(stderr, "[%3u] TX failed (errors: %" PRIu32 ")\n", seq, tx_fail);
            /*
             * If TX fails, the transceiver may be in an unexpected state.
             * Force it off and try again next iteration.
             */
            dwt_forcetrxoff();
        }

        seq++;  /* Will wrap around at 256, which is fine for sequence numbers */

        /* Wait before next beacon */
        usleep(BEACON_INTERVAL_US);
    }

    printf("\n=== Beacon Summary ===\n");
    printf("Transmitted: %" PRIu32 "\n", tx_ok);
    printf("Failed     : %" PRIu32 "\n", tx_fail);

    chip_shutdown();
    return 0;
}

