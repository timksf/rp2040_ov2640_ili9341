# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.22)

function(add_size_report target)
    add_custom_target(${target}_size ALL
        DEPENDS ${target}.elf
        COMMAND ${CMAKE_SIZE} ${target}.elf
    )
    add_dependencies(${target}_size ${target})
endfunction()

include(pico-sdk/pico_sdk_init.cmake)

set(CMAKE_SIZE arm-none-eabi-size)
set(PICO_DEOPTIMIZED_DEBUG 1)
set(PICO_BOARD pico CACHE STRING "Board type")

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(ov2640_ili9341)
# add_compile_options(-Wall -Wextra)
# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# executables
add_executable(firmware 
    firmware.c 
    gui.c 
    rotary_encoder.c 
    util.c
    sd_config.c
)
pico_enable_stdio_uart(firmware 0)
pico_enable_stdio_usb(firmware 1)

add_executable(display_test display_main.c rotary_encoder.c gui.c) #esa_logo.c
# pico_enable_stdio_uart(display_test 1)
pico_enable_stdio_usb(display_test 1)

add_executable(framecapture framecapture.c)
pico_enable_stdio_uart(framecapture 0)
pico_enable_stdio_usb(framecapture 1)

add_executable(test_sdcard 
    test_sdcard.c
    sd_config.c
)
pico_enable_stdio_uart(test_sdcard 0)
pico_enable_stdio_usb(test_sdcard 1)

add_executable(jpeg_sd 
    jpeg_capture_sd.c
    sd_config.c
    util.c
)
pico_enable_stdio_uart(jpeg_sd 0)
pico_enable_stdio_usb(jpeg_sd 1)
add_size_report(jpeg_sd)


add_executable(error_rpe RPE_minimal.c)
pico_enable_stdio_uart(error_rpe 0)
pico_enable_stdio_usb(error_rpe 1)

add_executable(blinky blink.c)

# Add libraries
add_subdirectory(lib/ili9341)
add_subdirectory(lib/ov2640)
add_subdirectory(lib/lvgl)
add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)
add_subdirectory(lib/tjpgd)

# Add the standard library to the build
target_link_libraries(firmware 
    pico_stdlib
    ov2640
    ili9341
    lvgl
    FatFs_SPI
    tjpgd
)

target_link_libraries(framecapture 
    pico_stdlib
    ov2640
)

target_link_libraries(display_test
    pico_stdlib
    ili9341
    lvgl
)

target_link_libraries(test_sdcard 
    pico_stdlib
    FatFs_SPI
)

target_link_libraries(jpeg_sd 
    pico_stdlib
    ov2640
    ili9341
    FatFs_SPI
    tjpgd
)

target_link_libraries(error_rpe 
    pico_stdlib
    ili9341
    FatFs_SPI
    tjpgd
)

target_link_libraries(blinky 
    pico_stdlib
)

# Add the standard include files to the build
# target_include_directories(firmware PRIVATE
#   ${CMAKE_CURRENT_LIST_DIR}
#   ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
# )

pico_add_extra_outputs(firmware)
pico_add_extra_outputs(framecapture)
pico_add_extra_outputs(display_test)
pico_add_extra_outputs(blinky)
pico_add_extra_outputs(jpeg_sd)
pico_add_extra_outputs(error_rpe)