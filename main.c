#include <stdio.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "ov2640.h"
#include "ov2640_regs.h"
#include "ili9341.h"
#include "hardware/dma.h"

#include "main.h"
// #include "img.h"

uint16_t frame_buf[FRAME_WIDTH*FRAME_HEIGHT];

//forward declaration of callbacks  
bool timer_callback(__unused struct repeating_timer *t);

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file

    uart_init(UART_INST, 500000);
    gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_INST, false, false);
    uart_set_format(UART_INST, 8, 1, UART_PARITY_NONE);
    uart_puts(UART_INST, "Hello from UART");

    for(int i = 0; i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++) 
        frame_buf[i] = ILI9341_CYAN;
    
    struct ili9341_config lcd_config;
    struct ov2640_config cam_config;

    lcd_config.spi = LCD_SPI_INST;
    lcd_config.pin_rst = PIN_LCD_RST;
    lcd_config.pin_dc = PIN_LCD_DC;
    lcd_config.pin_cs = PIN_LCD_CS;
    lcd_config.pin_sck = PIN_LCD_SCK;
    lcd_config.pin_tx = PIN_LCD_TX;

    cam_config.sccb = CAM_I2C_INST;
    cam_config.pin_sioc = PIN_CAM_SIOC;
    cam_config.pin_siod = PIN_CAM_SIOD;
    cam_config.pin_resetb = PIN_CAM_RESETB;
    cam_config.pin_xclk = -1;
    cam_config.pin_vsync = PIN_CAM_VSYNC;
    cam_config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;
    cam_config.pio = CAM_PIO_INST;
    cam_config.frame_sm = CAM_FRAME_SM;
    cam_config.byte_sm = CAM_BYTE_SM;
    cam_config.dma_channel = dma_claim_unused_channel(true);
    cam_config.image_buf = (uint8_t*)&frame_buf[0];
    cam_config.image_buf_size = sizeof(frame_buf);

    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config,1); //"landscape mode"
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    // for(int i = 0; i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++) 
    //     frame_buf[i] = ILI9341_MAGENTA;
    
    // ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    // after setup wait for serial terminal connection on USB port
    // while (!tud_cdc_connected()) { 
    //     sleep_ms(100);
    // }

    ov2640_init(&cam_config);
    uint16_t cam_id = ov2640_read_id(&cam_config);

    printf("Booted successfully!\n");
    printf("Camera ID: %02x\n", (int)cam_id);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer);

    volatile dma_debug_hw_t *volatile dma_dbg = dma_debug_hw;

    //colorbar
    // ov2640_reg_write(&cam_config, 0xFF, 0x01);
    // uint8_t com7 = ov2640_reg_read(&cam_config, 0x12);
    // com7 |= 0x02; //enable test pattern
    // ov2640_reg_write(&cam_config, 0x12, com7);

    // //hflip
    ov2640_reg_write(&cam_config, 0xFF, 0x01);
    volatile uint8_t reg04 = ov2640_reg_read(&cam_config, 0x04);
    volatile uint8_t com3 = ov2640_reg_read(&cam_config, COM3);
    volatile uint8_t com1 = ov2640_reg_read(&cam_config, COM1);

    while(1){
        uint8_t cmd;
        uart_read_blocking(UART_INST, &cmd, 1);

        if (cmd == CMD_CAPTURE) {
            ov2640_frame_capture(&cam_config, true);
            uart_write_blocking(UART_INST, cam_config.image_buf, cam_config.image_buf_size);
        }

        // ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);
    }
}

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}