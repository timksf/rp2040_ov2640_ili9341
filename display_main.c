#include <stdio.h>
#include "pico/stdlib.h"
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

    lcd_config.spi = LCD_SPI_INST;
    lcd_config.pin_rst = PIN_LCD_RST;
    lcd_config.pin_dc = PIN_LCD_DC;
    lcd_config.pin_cs = PIN_LCD_CS;
    lcd_config.pin_sck = PIN_LCD_SCK;
    lcd_config.pin_tx = PIN_LCD_TX;

    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config,1); //"landscape mode"
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    sleep_ms(1000);

    for(int i = 0; i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++) 
        frame_buf[i] = ILI9341_MAGENTA;
    
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    printf("Booted successfully!\n");

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer);

    while(1){
        // ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);
    }
}

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}