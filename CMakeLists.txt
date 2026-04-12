# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

include(pico_sdk_import.cmake)

project(hydrosertao C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
set(PICO_CYW43_FIRMWARE_SOURCE "embedded")

add_executable(hydrosertao
    src/main.c
    src/ssd1306.c
)

pico_set_program_name(hydrosertao "HydroSertao")
pico_set_program_version(hydrosertao "1.0.0")

pico_enable_stdio_uart(hydrosertao 1)
pico_enable_stdio_usb(hydrosertao 0)

target_include_directories(hydrosertao PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(hydrosertao
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_adc
    hardware_gpio
    hardware_i2c
    hardware_uart
    hardware_pwm
    hardware_irq
)

pico_add_extra_outputs(hydrosertao)