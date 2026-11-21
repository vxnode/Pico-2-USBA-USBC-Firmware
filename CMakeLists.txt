# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform")

include(pico_sdk_import.cmake)
project(pico2_mouse_passthrough C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pico-PIO-USB provides the software USB host used for the USB-A port.
# Use the copy bundled with the SDK's TinyUSB if present, otherwise download it.
if (NOT DEFINED PICO_PIO_USB_PATH)
    set(_bundled ${PICO_SDK_PATH}/lib/tinyusb/hw/mcu/raspberry_pi/Pico-PIO-USB)
    if (EXISTS ${_bundled}/src/pio_usb.c)
        set(PICO_PIO_USB_PATH ${_bundled})
    else()
        include(FetchContent)
        FetchContent_Declare(pico_pio_usb
            GIT_REPOSITORY https://github.com/sekigon-gonnoc/Pico-PIO-USB.git
            GIT_TAG 0.7.2)
        FetchContent_GetProperties(pico_pio_usb)
        if (NOT pico_pio_usb_POPULATED)
            FetchContent_Populate(pico_pio_usb)
        endif()
        set(PICO_PIO_USB_PATH ${pico_pio_usb_SOURCE_DIR})
    endif()
endif()

pico_sdk_init()

# Creates the tinyusb_pico_pio_usb target if the SDK hasn't already
if (NOT TARGET tinyusb_pico_pio_usb)
    check_and_add_pico_pio_usb_support()
endif()

add_executable(mouse_passthrough main.cpp)
target_include_directories(mouse_passthrough PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(mouse_passthrough PRIVATE
    pico_stdlib pico_multicore pico_util
    tinyusb_device tinyusb_host tinyusb_pico_pio_usb)

# Pico-PIO-USB has noisy warnings
target_compile_options(mouse_passthrough PRIVATE -Wno-error)

pico_add_extra_outputs(mouse_passthrough)
