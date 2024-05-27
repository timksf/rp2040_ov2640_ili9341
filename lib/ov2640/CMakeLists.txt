add_library(ov2640
    ov2640.c
)

target_include_directories(ov2640 PUBLIC
	.
)

pico_generate_pio_header(ov2640 ${CMAKE_CURRENT_LIST_DIR}/image.pio)

target_link_libraries(ov2640 
    pico_stdlib 
    hardware_i2c 
    hardware_dma 
    hardware_pio
    hardware_pwm
)