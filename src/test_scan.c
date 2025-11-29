#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "init.h"

#define RX_BUFFER_LEN RX_BUFFER_MAX_LEN
#define IRQ_POLL_INTERVAL_US 2000U
#define UNKNOWN_PAN_ID 0xFFFFU

struct ieee154_addr {
    bool present;
    uint16_t pan;
    uint8_t value[8];
    size_t len;
};

static volatile sig_atomic_t keep_running = 1;
static uint32_t frames_ok = 0;
static uint32_t frames_err = 0;
static uint32_t frames_timeout = 0;
static uint8_t rx_buffer[RX_BUFFER_LEN];
static struct timeval scan_start = { 0 };

static void handle_sigint(int sig);
static unsigned int parse_duration(int argc, char **argv);
static void print_local_identity(void);
static bool fetch_device_eui(uint8_t *eui, bool *synthetic);
static void configure_scan_mode(void);
static void scan_loop(unsigned int duration_s);
static bool process_rx_status(void);
static void log_frame(const uint8_t *frame, uint16_t len);
static const char *frame_type_to_string(uint16_t frame_control);
static size_t consume_address(const uint8_t *payload,
                              size_t remaining,
                              uint8_t mode,
                              bool include_pan,
                              struct ieee154_addr *addr);
static void format_address(const struct ieee154_addr *addr, char *buffer, size_t len);
static void print_address_line(const char *label, const struct ieee154_addr *addr);
static double seconds_since_start(void);

int main(int argc, char **argv) {
    unsigned int duration_s = parse_duration(argc, argv);

    if (signal(SIGINT, handle_sigint) == SIG_ERR) {
        perror("signal");
        return 1;
    }

    if (chip_init() != 0) {
        fprintf(stderr, "ERROR: chip initialization failed\n");
        return 1;
    }

    print_local_identity();
    gettimeofday(&scan_start, NULL);
    configure_scan_mode();

    printf("Starting passive scan%s. Press Ctrl+C to stop.\n",
           duration_s ? " with duration limit" : "");
    if (duration_s) {
        printf("Scan will stop after %u second(s) unless interrupted earlier.\n", duration_s);
    }

    scan_loop(duration_s);

    printf("\nScan summary: %" PRIu32 " frame(s), %" PRIu32 " error(s), %" PRIu32 " timeout(s)\n",
           frames_ok, frames_err, frames_timeout);

    chip_shutdown();
    return 0;
}

static void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

static unsigned int parse_duration(int argc, char **argv) {
    if (argc < 2) {
        return 0;
    }

    char *end = NULL;
    unsigned long value = strtoul(argv[1], &end, 10);
    if (argv[1][0] != '\0' && end != argv[1] && *end == '\0') {
        return (unsigned int)value;
    }

    fprintf(stderr, "WARNING: Invalid duration argument '%s'; running until Ctrl+C\n", argv[1]);
    return 0;
}

static void print_local_identity(void) {
    uint8_t eui[8] = { 0 };
    bool synthetic = false;
    fetch_device_eui(eui, &synthetic);

    printf("Local UWB identity:\n");
    printf("  Device ID: 0x%08" PRIX32 "\n", dwt_readdevid());
    printf("  EUI-64   : %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X%s\n",
           eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7],
           synthetic ? " (derived from lot/part IDs)" : "");
    printf("  Short addr / PAN ID follow the values you configure in chip_init().\n\n");
}

static bool fetch_device_eui(uint8_t *eui, bool *synthetic) {
    uint8_t tmp[8] = { 0 };
    dwt_geteui(tmp);

    bool all_zero = true;
    for (size_t i = 0; i < sizeof(tmp); i++) {
        if (tmp[i] != 0U) {
            all_zero = false;
            break;
        }
    }

    if (!all_zero) {
        memcpy(eui, tmp, sizeof(tmp));
        if (synthetic != NULL) {
            *synthetic = false;
        }
        return true;
    }

    uint64_t lot = dwt_getlotid();
    uint32_t part = dwt_getpartid();
    uint64_t derived = ((lot & 0x0000FFFFFFFFFFFFULL) << 16) | (uint64_t)(part & 0xFFFFU);

    for (size_t i = 0; i < sizeof(tmp); i++) {
        eui[i] = (uint8_t)((derived >> (8U * (7U - i))) & 0xFFU);
    }

    if (synthetic != NULL) {
        *synthetic = true;
    }
    return false;
}

static void configure_scan_mode(void) {
    const uint32_t rx_events =
        (uint32_t)DWT_INT_RXFCG_BIT_MASK |
        (uint32_t)DWT_INT_RXFR_BIT_MASK |
        (uint32_t)SYS_STATUS_ALL_RX_ERR |
        (uint32_t)SYS_STATUS_ALL_RX_TO;

    dwt_setinterrupt(rx_events, 0, DWT_ENABLE_INT);
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);

    dwt_forcetrxoff();
    dwt_configureframefilter(DWT_FF_DISABLE, 0);
    dwt_setrxtimeout(0);              // wait indefinitely
    dwt_setpreambledetecttimeout(0);  // disable preamble timeout
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void scan_loop(unsigned int duration_s) {
    while (keep_running) {
        bool handled = process_rx_status();

        if (duration_s > 0) {
            double elapsed = seconds_since_start();
            if (elapsed >= (double)duration_s) {
                break;
            }
        }

        if (!handled) {
            usleep(IRQ_POLL_INTERVAL_US);
        }
    }
}

static bool process_rx_status(void) {
    uint32_t status = dwt_readsysstatuslo();
    uint32_t clear_mask = 0U;
    bool handled = false;

    if ((status & SYS_STATUS_ALL_RX_GOOD) != 0U) {
        uint8_t rng = 0U;
        uint16_t length = dwt_getframelength(&rng);
        if (length > RX_BUFFER_LEN) {
            length = RX_BUFFER_LEN;
        }

        dwt_readrxdata(rx_buffer, length, 0);
        frames_ok++;
        log_frame(rx_buffer, length);
        clear_mask |= SYS_STATUS_ALL_RX_GOOD;
        (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
        handled = true;
    }

    if ((status & SYS_STATUS_ALL_RX_ERR) != 0U) {
        frames_err++;
        clear_mask |= SYS_STATUS_ALL_RX_ERR;
        dwt_forcetrxoff();
        (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
        handled = true;
    }

    if ((status & SYS_STATUS_ALL_RX_TO) != 0U) {
        frames_timeout++;
        clear_mask |= SYS_STATUS_ALL_RX_TO;
        dwt_forcetrxoff();
        (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
        handled = true;
    }

    if (clear_mask != 0U) {
        dwt_writesysstatuslo(clear_mask);
    }

    return handled;
}

static const char *frame_type_to_string(uint16_t frame_control) {
    switch (frame_control & 0x7U) {
        case 0x0: return "Beacon";
        case 0x1: return "Data";
        case 0x2: return "Ack";
        case 0x3: return "MAC Command";
        case 0x4: return "Reserved";
        case 0x5: return "Multipurpose";
        case 0x6: return "Fragment";
        case 0x7: return "Extended";
        default:  return "Unknown";
    }
}

static void log_frame(const uint8_t *frame, uint16_t len) {
    if (len < 3U) {
        printf("[%.3f s] RX frame too short (%u byte(s))\n", seconds_since_start(), (unsigned)len);
        return;
    }

    uint16_t frame_control = (uint16_t)frame[0] | ((uint16_t)frame[1] << 8);
    uint8_t sequence = frame[2];
    bool security = ((frame_control >> 3) & 0x1U) != 0U;
    bool ack_request = ((frame_control >> 5) & 0x1U) != 0U;
    bool pan_compression = ((frame_control >> 6) & 0x1U) != 0U;

    uint8_t dest_mode = (frame_control >> 10) & 0x3U;
    uint8_t frame_version = (frame_control >> 12) & 0x3U;
    uint8_t src_mode = (frame_control >> 14) & 0x3U;

    size_t index = 3U;
    size_t remaining = len - index;
    struct ieee154_addr dest = { 0 };
    struct ieee154_addr src = { 0 };

    if (dest_mode != 0U) {
        size_t consumed = consume_address(frame + index, remaining, dest_mode, true, &dest);
        if (consumed > remaining) {
            consumed = remaining;
        }
        index += consumed;
        remaining = (index <= len) ? (len - index) : 0U;
    }

    bool include_src_pan = !(pan_compression && dest.present);
    if (src_mode != 0U) {
        size_t consumed = consume_address(frame + index, remaining, src_mode, include_src_pan, &src);
        if (!include_src_pan && dest.present) {
            src.pan = dest.pan;
        }
        if (consumed > remaining) {
            consumed = remaining;
        }
        index += consumed;
        remaining = (index <= len) ? (len - index) : 0U;
    }

    printf("[%.3f s] RX %s frame seq=%u len=%u ver=%u%s%s\n",
           seconds_since_start(),
           frame_type_to_string(frame_control),
           (unsigned)sequence,
           (unsigned)len,
           (unsigned)frame_version,
           security ? " SEC" : "",
           ack_request ? " ACKREQ" : "");

    print_address_line("Dest", &dest);
    print_address_line("Src ", &src);
    if (remaining > 0U) {
        printf("    Payload bytes remaining after headers: %zu\n", remaining);
    }
}

static size_t consume_address(const uint8_t *payload,
                              size_t remaining,
                              uint8_t mode,
                              bool include_pan,
                              struct ieee154_addr *addr) {
    memset(addr, 0, sizeof(*addr));
    if (mode == 0U) {
        addr->present = false;
        addr->pan = UNKNOWN_PAN_ID;
        return 0U;
    }

    addr->present = true;
    size_t offset = 0U;

    if (include_pan) {
        if (remaining < 2U) {
            addr->present = false;
            addr->pan = UNKNOWN_PAN_ID;
            return remaining;
        }
        addr->pan = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        offset += 2U;
        remaining -= 2U;
    } else {
        addr->pan = UNKNOWN_PAN_ID;
    }

    size_t needed = (mode == 2U) ? 2U : 8U;
    if (remaining < needed) {
        addr->present = false;
        addr->len = 0U;
        return offset + remaining;
    }

    memcpy(addr->value, payload + offset, needed);
    addr->len = needed;
    offset += needed;
    return offset;
}

static void format_address(const struct ieee154_addr *addr, char *buffer, size_t len) {
    if (!addr->present || addr->len == 0U) {
        (void)snprintf(buffer, len, "--");
        return;
    }

    if (addr->len == 2U) {
        uint16_t short_addr = (uint16_t)addr->value[0] | ((uint16_t)addr->value[1] << 8);
        (void)snprintf(buffer, len, "0x%04X", short_addr);
        return;
    }

    size_t written = 0U;
    for (size_t i = 0; i < addr->len && written < len; i++) {
        int ret = snprintf(buffer + written,
                           len - written,
                           (i + 1U == addr->len) ? "%02X" : "%02X:",
                           addr->value[i]);
        if (ret < 0) {
            break;
        }
        written += (size_t)ret;
    }
}

static void print_address_line(const char *label, const struct ieee154_addr *addr) {
    char formatted[48];
    format_address(addr, formatted, sizeof(formatted));

    if (!addr->present) {
        printf("    %s: --\n", label);
        return;
    }

    if (addr->pan == UNKNOWN_PAN_ID) {
        printf("    %s: PAN=-- Addr=%s\n", label, formatted);
    } else {
        printf("    %s: PAN=0x%04X Addr=%s\n", label, addr->pan, formatted);
    }
}

static double seconds_since_start(void) {
    struct timeval now;
    gettimeofday(&now, NULL);

    double seconds = (double)(now.tv_sec - scan_start.tv_sec);
    seconds += (double)(now.tv_usec - scan_start.tv_usec) / 1000000.0;
    return seconds;
}

