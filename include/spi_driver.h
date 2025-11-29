#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>

int spi_init(const char *device, unsigned int speed_hz);
int spi_transfer(uint8_t *tx, uint8_t *rx, unsigned int len);
int spi_write(uint8_t *data, unsigned int len);
int spi_read(uint8_t *data, unsigned int len);
void spi_close(void);

#endif