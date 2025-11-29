#include <stdio.h>

#include "init.h"

int main() {
    if (chip_init() != 0) {
        printf("Initialization failed\n");
        return 1;
    }

    printf("Initialization successful!\n");

    chip_shutdown();
    return 0;
}