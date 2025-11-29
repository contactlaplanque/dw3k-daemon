#include <stdio.h>
#include <unistd.h>

/* Include Qorvo API header */
#include "deca_device_api.h"  // Found via -Idwt_uwb_driver

/* Include your SPI driver header */
#include "spi_driver.h"  // Found via -Iinclude

/* These are called by all dwt_xxx() functions */

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint32_t bodyLength, const uint8_t *bodyBuffer) {
    
    if (headerLength > 0) {
        spi_write((uint8_t*)headerBuffer, headerLength);
    }
    if (bodyLength > 0) {
        spi_write((uint8_t*)bodyBuffer, bodyLength);
    }
    return 0;
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint32_t bodyLength, uint8_t *bodyBuffer) {
    
    if (headerLength > 0) {
        // Send read command header
        spi_write(headerBuffer, headerLength);
    }
    if (bodyLength > 0) {
        // Read the response data
        spi_read(bodyBuffer, bodyLength);
    }
    return 0;
}

/* Sleep function called by API */
void deca_sleep(unsigned int ms) {
    usleep(ms * 1000);
}