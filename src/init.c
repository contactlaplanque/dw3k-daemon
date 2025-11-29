#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

/* Qorvo API header - found via -Idwt_uwb_driver */
#include "deca_device_api.h"

/* Platform glue */
#include "qorvo_wrapper.h"
#include "spi_driver.h"

/* Optional: If your API version requires dwt_context_t */
/* Uncomment if you get compilation errors about missing dwt_context */
/* dwt_context_t dwt_context; */

int chip_init(void) {
    printf("=== DW3000 Initialization ===\n");

    // 1. Start SPI at low speed (4 MHz required during initialization)
    if (spi_init("/dev/spidev0.0", 4000000) != 0) {
        fprintf(stderr, "ERROR: SPI initialization failed\n");
        return -1;
    }
    printf("[SPI] Initialized at 4 MHz\n");

    if (dw3k_driver_setup("/dev/spidev0.0") != 0) {
        fprintf(stderr, "ERROR: DW3K driver probe failed\n");
        return -1;
    }

    // 2. Wait for chip ready (IDLERC state)
    printf("Waiting for chip ready...\n");
    int wait = 0;
    while (!dwt_checkidlerc() && wait++ < 100) {
        usleep(10000);  // 10 ms
    }
    if (wait >= 100) {
        fprintf(stderr, "ERROR: Chip did not reach IDLERC\n");
        return -1;
    }
    printf("Chip is ready\n");

    // 3. Initialize the device (loads calibration from OTP)
    printf("Initializing device...\n");
    int ret = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID |
                             DWT_READ_OTP_BAT | DWT_READ_OTP_TMP);
    if (ret == DWT_ERROR) {
        fprintf(stderr, "ERROR: dwt_initialise failed\n");
        return -1;
    }
    printf("Device initialized\n");

    // 4. Check device ID
    uint32_t dev_id = dwt_readdevid();
    printf("Device ID: 0x%08X\n", dev_id);

    if (dwt_check_dev_id() != DWT_SUCCESS) {
        fprintf(stderr, "ERROR: Device ID check failed\n");
        return -1;
    }
    printf("Device ID verified\n");

    // 5. Configure for operation
    printf("Configuring device...\n");
    dwt_config_t config = {
        .chan = 5,                  // Channel 5
        .txPreambLength = DWT_PLEN_128,
        .rxPAC = DWT_PAC8,
        .txCode = 4,
        .rxCode = 4,
        .sfdType = 0,
        .dataRate = DWT_BR_6M8,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (128 + 1 + 8 - 8),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = 0,
        .pdoaMode = DWT_PDOA_M0,
    };

    if (dwt_configure(&config) != DWT_SUCCESS) {
        fprintf(stderr, "ERROR: dwt_configure failed\n");
        return -1;
    }
    printf("Device configured\n");

    // 6. Set addresses
    dwt_setpanid(0xDECA);
    dwt_setaddress16(0x1000);  // Our address
    printf("Addresses set (PAN: 0xDECA, Addr: 0x1000)\n");

    // 7. Switch to fast SPI speed (20 MHz after initialization)
    if (spi_init("/dev/spidev0.0", 20000000) != 0) {
        fprintf(stderr, "WARNING: Failed to switch to fast SPI speed\n");
        // Don't fail here, continue with 4 MHz
    } else {
        printf("[SPI] Switched to 20 MHz\n");
    }

    printf("=== Initialization Complete ===\n\n");
    return 0;
}

void chip_shutdown(void) {
    printf("Shutting down...\n");
    dwt_forcetrxoff();  // Turn off RX/TX
    spi_close();
    printf("Shutdown complete\n");
}