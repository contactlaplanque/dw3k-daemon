cmake_minimum_required(VERSION 3.20)

project(dw3k_daemon C)

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
)

add_executable(test_init src/test_init.c)
target_link_libraries(test_init PRIVATE dw3k_core)

add_executable(test_scan src/test_scan.c)
target_link_libraries(test_scan PRIVATE dw3k_core)

add_executable(test_beacon src/test_beacon.c)
target_link_libraries(test_beacon PRIVATE dw3k_core)

