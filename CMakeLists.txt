cmake_minimum_required(VERSION 3.20)

project(dw3k_daemon C)

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_definitions(_DEFAULT_SOURCE)

option(DWT_DW3000 "Enable DW3000 driver support" ON)
option(DWT_DW3720 "Enable DW3720 driver support" OFF)

add_subdirectory(dwt_uwb_driver)

add_library(dw3k_core
    src/init.c
    src/qorvo_wrapper.c
    src/spi_driver.c
    src/twr_service.c
)

target_include_directories(dw3k_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver
)

target_link_libraries(dw3k_core
    PUBLIC
        uwb_driver
        m
)

add_executable(dw3k_daemon
    src/dw3k_daemon.c
)
target_link_libraries(dw3k_daemon
    PRIVATE
        dw3k_core
        pthread
)

install(TARGETS dw3k_daemon
    RUNTIME DESTINATION ${CMAKE_INSTALL_SBINDIR}
)

install(FILES config/dw3k-daemon.conf
    DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}
)

