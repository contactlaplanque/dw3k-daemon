#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "deca_interface.h"
#include "qorvo_wrapper.h"
#include "spi_driver.h"

#define DEFAULT_SPI_DEVICE "/dev/spidev0.0"
#define SPI_SLOW_HZ 4000000U
#define SPI_FAST_HZ 20000000U

extern const struct dwt_driver_s dw3000_driver;

static char spi_device_path[128] = DEFAULT_SPI_DEVICE;
static bool driver_probed = false;

static struct dwt_driver_s *driver_list[] = {
    (struct dwt_driver_s *)&dw3000_driver,
};

static int32_t dw_spi_write_internal(uint16_t headerLength,
                                     const uint8_t *headerBuffer,
                                     uint16_t bodyLength,
                                     const uint8_t *bodyBuffer,
                                     bool append_crc,
                                     uint8_t crc8) {
    size_t total = (size_t)headerLength + (size_t)bodyLength + (append_crc ? 1U : 0U);
    if (total == 0U) {
        return DWT_SUCCESS;
    }
    if (total > UINT_MAX) {
        return DWT_ERROR;
    }

    uint8_t *tx = calloc(total, sizeof(uint8_t));
    if (tx == NULL) {
        return DWT_ERROR;
    }

    if (headerLength > 0U && headerBuffer != NULL) {
        memcpy(tx, headerBuffer, headerLength);
    }
    if (bodyLength > 0U && bodyBuffer != NULL) {
        memcpy(tx + headerLength, bodyBuffer, bodyLength);
    }
    if (append_crc) {
        tx[total - 1U] = crc8;
    }

    int ret = spi_transfer(tx, NULL, (unsigned int)total);
    free(tx);
    return (ret == 0) ? DWT_SUCCESS : DWT_ERROR;
}

static int32_t dw_spi_read_internal(uint16_t headerLength,
                                    uint8_t *headerBuffer,
                                    uint16_t readLength,
                                    uint8_t *readBuffer) {
    size_t total = (size_t)headerLength + (size_t)readLength;
    if (total == 0U) {
        return DWT_SUCCESS;
    }
    if (total > UINT_MAX) {
        return DWT_ERROR;
    }

    uint8_t *tx = calloc(total, sizeof(uint8_t));
    uint8_t *rx = calloc(total, sizeof(uint8_t));
    if (tx == NULL || rx == NULL) {
        free(tx);
        free(rx);
        return DWT_ERROR;
    }

    if (headerLength > 0U && headerBuffer != NULL) {
        memcpy(tx, headerBuffer, headerLength);
    }

    int ret = spi_transfer(tx, rx, (unsigned int)total);
    if (ret == 0 && readLength > 0U && readBuffer != NULL) {
        memcpy(readBuffer, rx + headerLength, readLength);
    }

    free(tx);
    free(rx);

    return (ret == 0) ? DWT_SUCCESS : DWT_ERROR;
}

static int32_t dw_readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                              uint16_t readLength, uint8_t *readBuffer) {
    return dw_spi_read_internal(headerLength, headerBuffer, readLength, readBuffer);
}

static int32_t dw_writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
                             uint16_t bodyLength, const uint8_t *bodyBuffer) {
    return dw_spi_write_internal(headerLength, headerBuffer, bodyLength, bodyBuffer, false, 0U);
}

static int32_t dw_writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                                    uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8) {
    return dw_spi_write_internal(headerLength, headerBuffer, bodyLength, bodyBuffer, true, crc8);
}

static void dw_setslowrate(void) {
    (void)spi_init(spi_device_path, SPI_SLOW_HZ);
}

static void dw_setfastrate(void) {
    (void)spi_init(spi_device_path, SPI_FAST_HZ);
}

static void dw_wakeup_device_with_io(void) {
    /* Basic wake-up: short delay to ensure device is ready */
    usleep(2000);
}

static struct dwt_spi_s spi_ops = {
    .readfromspi = dw_readfromspi,
    .writetospi = dw_writetospi,
    .writetospiwithcrc = dw_writetospiwithcrc,
    .setslowrate = dw_setslowrate,
    .setfastrate = dw_setfastrate,
};

int dw3k_driver_setup(const char *device_path) {
    if (device_path != NULL) {
        strncpy(spi_device_path, device_path, sizeof(spi_device_path) - 1U);
        spi_device_path[sizeof(spi_device_path) - 1U] = '\0';
    }

    if (driver_probed) {
        return 0;
    }

    struct dwt_probe_s probe = {
        .dw = NULL,
        .spi = (void *)&spi_ops,
        .wakeup_device_with_io = dw_wakeup_device_with_io,
        .driver_list = driver_list,
        .dw_driver_num = (uint8_t)(sizeof(driver_list) / sizeof(driver_list[0])),
    };

    int32_t ret = dwt_probe(&probe);
    if (ret != DWT_SUCCESS) {
        fprintf(stderr, "ERROR: dwt_probe failed (%d)\n", (int)ret);
        return -1;
    }

    driver_probed = true;
    return 0;
}

/* Legacy helpers retained for compatibility */
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint32_t bodyLength, const uint8_t *bodyBuffer) {
    if (bodyLength > UINT16_MAX) {
        return -1;
    }

    return (int)dw_writetospi(headerLength, headerBuffer,
                              (uint16_t)bodyLength, bodyBuffer);
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint32_t bodyLength, uint8_t *bodyBuffer) {
    if (bodyLength > UINT16_MAX) {
        return -1;
    }

    return (int)dw_readfromspi(headerLength, headerBuffer,
                               (uint16_t)bodyLength, bodyBuffer);
}

/* Sleep function called by API */
void deca_sleep(unsigned int ms) {
    usleep(ms * 1000);
}

/* Microsecond sleep function called by API */
void deca_usleep(unsigned long time_us) {
    usleep(time_us);
}

/* Mutex functions for critical sections */
/* On Linux user space, we use simple stubs since we don't have direct hardware interrupt control */
decaIrqStatus_t decamutexon(void) {
    return 0;
}

void decamutexoff(decaIrqStatus_t s) {
    (void)s;
}