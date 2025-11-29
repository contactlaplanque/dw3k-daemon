#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "spi_driver.h"

int chip_init(void) {
    printf("=== DW3000 Initialization ===\n");

    // 1. Initialize SPI at low speed
    if (spi_init("/dev/spidev0.0", 4000000) != 0) {
        fprintf(stderr, "ERROR: SPI initialization failed\n");
        return -1;
    }

    // 2. Wait for chip ready
    printf("Waiting for chip ready...\n");
    int wait = 0;
    while (!dwt_checkidlerc() && wait++ < 100) {
        usleep(10000);
    }
    if (wait >= 100) {
        fprintf(stderr, "ERROR: Chip did not reach IDLERC\n");
        return -1;
    }
    printf("Chip is ready\n");

    // 3. Initialize device
    printf("Initializing device...\n");
    int ret = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID |
                             DWT_READ_OTP_BAT | DWT_READ_OTP_TMP);
    if (ret == DWT_ERROR) {
        fprintf(stderr, "ERROR: dwt_initialise failed\n");
        return -1;
    }

    // 4. Verify device ID
    uint32_t dev_id = dwt_readdevid();
    printf("Device ID: 0x%08X\n", dev_id);

    if (dwt_check_dev_id() != DWT_SUCCESS) {
        fprintf(stderr, "ERROR: Device ID check failed\n");
        return -1;
    }

    // 5. Configure device
    printf("Configuring device...\n");
    dwt_config_t config = {
        .chan = 5,
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

    // 6. Set addresses
    dwt_setpanid(0xDECA);
    dwt_setaddress16(0x1000);

    // 7. Switch to fast SPI
    spi_init("/dev/spidev0.0", 20000000);

    printf("=== Initialization Complete ===\n\n");
    return 0;
}

void chip_shutdown(void) {
    dwt_forcetrxoff();
    spi_close();
}