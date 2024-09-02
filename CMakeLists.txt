# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.22)

include(pico-sdk/pico_sdk_init.cmake)

set(PICO_BOARD pico CACHE STRING "Board type")

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(ov2640_ili9341)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# executables
add_executable(firmware main.c)
pico_enable_stdio_uart(firmware 0)
pico_enable_stdio_usb(firmware 1)

add_executable(display_test display_main.c)
pico_enable_stdio_uart(display_test 1)
pico_enable_stdio_usb(display_test 0)

add_subdirectory(lib/ili9341)
add_subdirectory(lib/ov2640)

# Add the standard library to the build
target_link_libraries(firmware 
    pico_stdlib
    ov2640
    ili9341
)

target_link_libraries(display_test 
    pico_stdlib
    ili9341
)

# Add the standard include files to the build
# target_include_directories(firmware PRIVATE
#   ${CMAKE_CURRENT_LIST_DIR}
#   ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
# )

pico_add_extra_outputs(firmware)
pico_add_extra_outputs(display_test)

