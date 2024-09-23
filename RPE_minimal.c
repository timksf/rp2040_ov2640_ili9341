#include <stdio.h>
#include <tusb.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sd_card.h"
#include "ff.h"

#include "lib/ili9341/ili9341.h"
#include "lib/tjpgd/tjpgd.h"

#include "board_config.h"
#include "util.h"

uint8_t frame_buf[BUF_SIZE] __attribute__ ((aligned (4)));

//workspace for JPEG decoder
uint8_t jpgd_ws[TJPGD_WORKSPACE_SIZE] __attribute__((aligned(4)));

static ili9341_config_t lcd_config;

void setup_lcd();
size_t jd_input(JDEC *jdec, uint8_t *buf, size_t len);
int jd_output(JDEC *jdec, void *bitmap, JRECT *rect);

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file
    printf("Booted successfully!\n");

    // while (!tud_cdc_connected()) { 
    //     sleep_ms(100);
    // }

    setup_lcd();
    
    //init display
    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config, 1);
    for(int i = 0; i < ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH; i++) 
        ((uint16_t*)frame_buf)[i] = ILI9341_MAGENTA;
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, (uint16_t*)frame_buf);

    for(int i = 0; i < ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH; i++) 
        ((uint16_t*)frame_buf)[i] = ILI9341_BLACK;
    
    ili9341_write_frame(&lcd_config, 5, 5, 2, 2, (uint16_t*)frame_buf);
    printf("\na");

    while(true) tight_loop_contents();
}

size_t jd_input(JDEC *jdec, uint8_t *buf, size_t len) {
    return len;
}

int jd_output(JDEC *jdec, void *bitmap, JRECT *rect) {
    return 1;
}

void setup_lcd() {
    lcd_config.spi = LCD_SPI_INST;
    lcd_config.pin_rst = PIN_LCD_RST;
    lcd_config.pin_dc = PIN_LCD_DC;
    lcd_config.pin_cs = PIN_LCD_CS;
    lcd_config.pin_sck = PIN_LCD_SCK;
    lcd_config.pin_tx = PIN_LCD_TX;
    lcd_config.dma_data_chan = dma_claim_unused_channel(true);
    lcd_config.dma_ctrl_chan = dma_claim_unused_channel(true);
    lcd_config.frame_buf = (uint16_t*) frame_buf;
}
