#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

static int spi_fd = -1;

int spi_init(const char *device, unsigned int speed_hz) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }

    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }
    
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
    
    return 0;
}

int spi_transfer(uint8_t *tx, uint8_t *rx, unsigned int len) {
    struct spi_ioc_transfer xfer = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .bits_per_word = 8,
    };
    
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0) {
        perror("SPI transfer failed");
        return -1;
    }
    return 0;
}

int spi_write(uint8_t *data, unsigned int len) {
    uint8_t rx_dummy[256] = {0};
    return spi_transfer(data, rx_dummy, len);
}

int spi_read(uint8_t *data, unsigned int len) {
    uint8_t tx_dummy[256] = {0};
    return spi_transfer(tx_dummy, data, len);
}

void spi_close(void) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
}