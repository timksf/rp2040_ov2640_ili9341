#include <stdio.h>
#include <tusb.h>
#include <unistd.h>
#include "pico/stdlib.h"
#include "ov2640.h"
#include "ov2640_regs.h"
#include "hardware/dma.h"

#include "main.h"

uint8_t frame_buf[BUF_SIZE];

volatile dma_debug_hw_t *volatile dma_dbg = dma_debug_hw;

//forward declaration of callbacks  
bool timer_callback(__unused struct repeating_timer *t);

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file

#ifndef CUSTOM_BOARD
    uart_init(UART_INST, 1000000);
    gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_INST, false, false);
    uart_set_format(UART_INST, 8, 1, UART_PARITY_NONE);
    uart_puts(UART_INST, "Hello from UART");
#else 
    //turn-off auto translation of data bytes
    stdio_set_translate_crlf(&stdio_usb, false);
#endif //CUSTOM_BOARD

    ov2640_config_t cam_config;

    cam_config.sccb = CAM_I2C_INST;
    cam_config.pin_sioc = PIN_CAM_SIOC;
    cam_config.pin_siod = PIN_CAM_SIOD;
    cam_config.pin_resetb = PIN_CAM_RESETB;
    cam_config.pin_xclk = PIN_CAM_XCLK;
    cam_config.pin_vsync = PIN_CAM_VSYNC;
    cam_config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;
    cam_config.pio = CAM_PIO_INST;
    cam_config.frame_sm = CAM_FRAME_SM;
    cam_config.frame_sized_sm = CAM_FRAME_SIZED_SM;
    cam_config.dma_channel = dma_claim_unused_channel(true);
    cam_config.image_buf = (uint8_t*)&frame_buf[0];
    cam_config.image_buf_size = sizeof(frame_buf);
    cam_config.frame_size_bytes = 0; //this will be used for transfers and is set by ov2640_set_framesize
    cam_config.color_format = COLOR_FORMAT_RGB565;

    // after setup wait for serial terminal connection on USB port
    while (!tud_cdc_connected()) { 
        sleep_ms(100);
    }
    printf("Booted successfully!\n");

    ov2640_init(&cam_config);
    uint16_t cam_id = ov2640_read_id(&cam_config);

    printf("Camera ID: %02x\n", (int)cam_id);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer);

    //frame streaming up to 320x240
    ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);
    ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);
    // ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);

    while(1) {
        tud_task();
        uint8_t cmd = 0x0;
#ifndef CUSTOM_BOARD
        uart_read_blocking(UART_INST, &cmd, 1);
        uart_write_blocking(UART_INST, &cmd, 1);
#else   
        if(tud_cdc_available()) {
            cmd = tud_cdc_read_char();
        }
#endif //CUSTOM_BOARD

        if (cmd == CMD_CAPTURE) {
            // ov2640_set_color_format(&cam_config, COLOR_FORMAT_RGB565);
            // ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);
            // sleep_ms(100);
            ov2640_frame_capture_single(&cam_config, true);
#ifndef CUSTOM_BOARD
            uart_write_blocking(UART_INST, cam_config.image_buf, cam_config.frame_size_bytes);
#else 
            write(1, cam_config.image_buf, cam_config.frame_size_bytes);
#endif //CUSTOM_BOARD
        } else if(cmd == CMD_CAPTURE_JPEG) {
            ov2640_set_color_format(&cam_config, COLOR_FORMAT_JPEG);
            ov2640_set_framesize(&cam_config, FRAMESIZE_UXGA);
            ov2640_frame_capture_single(&cam_config, true);
            //for JPEG, either search for EOI or simply transmit entire buffer and let recipient search
#ifndef CUSTOM_BOARD
            uart_write_blocking(UART_INST, cam_config.image_buf, cam_config.image_buf_size);
#else 
            write(1, cam_config.image_buf, cam_config.image_buf_size);
#endif //CUSTOM_BOARD
        }

        // ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);
        // tight_loop_contents();
    }
}

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}