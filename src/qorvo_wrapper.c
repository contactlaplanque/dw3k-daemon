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

/* Microsecond sleep function called by API */
void deca_usleep(unsigned long time_us) {
    usleep(time_us);
}

/* Mutex functions for critical sections */
/* On Linux user space, we use simple stubs since we don't have direct hardware interrupt control */
decaIrqStatus_t decamutexon(void) {
    /* Return 0 to indicate interrupts were enabled */
    /* In a real embedded system, this would disable interrupts and return the previous state */
    return 0;
}

void decamutexoff(decaIrqStatus_t s) {
    /* Restore interrupt state (no-op on Linux user space) */
    /* In a real embedded system, this would restore interrupts to the state saved in 's' */
    (void)s; /* Suppress unused parameter warning */
}