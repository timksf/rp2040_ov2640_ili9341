#include <stdio.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "ov2640.h"
#include "ili9341.h"
// #include "img.h"

#define UART_INST uart1
#define CAM_I2C_INST i2c1
#define LCD_SPI_INST spi0
#define CAM_PIO_INST pio0
#define CAM_FRAME_SM 0
#define CAM_BYTE_SM 1

const int PIN_LED = 25;
//LCD pin definitions
const int PIN_LCD_DC = 0;
const int PIN_LCD_RST = 1;
const int PIN_LCD_CS = 5; //SPI0
const int PIN_LCD_TX = 3;
const int PIN_LCD_SCK = 2;
//camera pin definitions
const int PIN_CAM_SIOC = 27; //I2C1 SCL
const int PIN_CAM_SIOD = 26; //I2C1 SDA
const int PIN_CAM_RESETB = 22;
// const int PIN_CAM_XCLK = 3;
const int PIN_CAM_VSYNC = 16;
const int PIN_CAM_Y2_PIO_BASE = 6;

uint16_t frame_buf[320*240];

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file

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
    // cam_config.pin_xclk = -1;
    cam_config.pin_vsync = PIN_CAM_VSYNC;
    cam_config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;
    cam_config.pio = CAM_PIO_INST;
    cam_config.frame_sm = CAM_FRAME_SM;
    cam_config.byte_sm = CAM_BYTE_SM;
    cam_config.dma_channel = dma_claim_unused_channel(true);
    cam_config.image_buf = (uint8_t*)&frame_buf[0];
    cam_config.image_buf_size = sizeof(frame_buf);

    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config, 1); //"landscape mode"
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    for(int i = 0; i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++) 
        frame_buf[i] = ILI9341_MAGENTA;
    
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    ov2640_init(&cam_config);
    uint16_t cam_id = ov2640_read_id(&cam_config);

    //after setup wait for serial terminal connection on USB port
    // while (!tud_cdc_connected()) { 
    //     sleep_ms(100);  
    // }

    printf("Booted successfully!\n");
    printf("Camera ID: %02x\n", (int)cam_id);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    while(1){
        gpio_put(PIN_LED, !gpio_get(PIN_LED));
        ov2640_start_frame_capture(&cam_config);
        dma_channel_wait_for_finish_blocking(cam_config.dma_channel);
        ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);
    }
}