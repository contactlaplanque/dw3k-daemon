#include "spi_driver.h"  // For spi_init, spi_close declarations
#include "deca_device_api.h"  // For chip_init declaration

/* Forward declaration if init.c doesn't have a header */
int chip_init(void);
void chip_shutdown(void);

int main() {
    if (chip_init() != 0) {
        printf("Initialization failed\n");
        return 1;
    }

    printf("Initialization successful!\n");

    chip_shutdown();
    return 0;
}