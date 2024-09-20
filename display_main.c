#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
// #include "lvgl.h"

#include "board_config.h"
#include "ili9341.h"
#include "rotary_encoder.h"

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240

// LV_IMAGE_DECLARE(logo);

uint16_t frame_buf[FRAME_WIDTH*FRAME_HEIGHT];

ili9341_config_t lcd_config;
rotary_encoder_t nav_encoder;

//lvgl related globals
const int32_t GL_UPDATE_INTERVAL = -5; //call update function every 5ms
// lv_display_t *display;
uint8_t gl_buf[FRAME_HEIGHT*FRAME_WIDTH*2 / 10]; //1/10 the size of actual frame is recommended

struct repeating_timer gl_update_timer;
struct repeating_timer blink_timer;

//forward declaration of callbacks
void gpio_callback(uint gpio, uint32_t events);
bool blink_callback(__unused struct repeating_timer *t);
bool gl_update_callback(__unused struct repeating_timer *t);
// void gl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pxmap);

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file

#ifndef CUSTOM_BOARD
    uart_init(UART_INST, 500000);
    gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_INST, false, false);
    uart_set_format(UART_INST, 8, 1, UART_PARITY_NONE);
    uart_puts(UART_INST, "Hello from UART");
#else
    stdio_set_translate_crlf(&stdio_usb, true);
    printf("Hello World\n");
#endif

    for(int i = 0; i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++) 
        frame_buf[i] = ILI9341_WHITE;

    //configure LCD
    lcd_config.spi = LCD_SPI_INST;
    lcd_config.pin_rst = PIN_LCD_RST;
    lcd_config.pin_dc = PIN_LCD_DC;
    lcd_config.pin_cs = PIN_LCD_CS;
    lcd_config.pin_sck = PIN_LCD_SCK;
    lcd_config.pin_tx = PIN_LCD_TX;

    nav_encoder.pin_a = PIN_ENC_A;
    nav_encoder.pin_b = PIN_ENC_B;
    nav_encoder.pin_btn = PIN_ENC_BTN;

    //LVGL setup
    // lv_init();
    // display = lv_display_create(FRAME_WIDTH, FRAME_HEIGHT);
    // lv_display_set_buffers(display, gl_buf, NULL, sizeof(gl_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
    // lv_display_set_default(display);
    // lv_display_set_flush_cb(display, gl_flush);
    // lv_obj_t *esa_icon = lv_image_create(lv_screen_active());
    // lv_image_set_src(esa_icon, &logo);
    // lv_obj_set_pos(esa_icon, (FRAME_WIDTH-logo.header.w) / 2, (FRAME_HEIGHT-logo.header.h) / 2);

    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config,1); //"landscape mode"
    //write white frame to clear the screen
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, frame_buf);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    //register timers
    add_repeating_timer_ms(-1000, blink_callback, NULL, &blink_timer);
    add_repeating_timer_ms(GL_UPDATE_INTERVAL, gl_update_callback, NULL, &gl_update_timer);

    //GPIO user input 
    gpio_init(PIN_USR_BTN0);
    gpio_set_dir(PIN_USR_BTN0, false);
    gpio_disable_pulls(PIN_USR_BTN0);

    gpio_init(PIN_USR_BTN1);
    gpio_set_dir(PIN_USR_BTN1, false);
    gpio_disable_pulls(PIN_USR_BTN1);

    //rotary encoder
    rotary_encoder_init(&nav_encoder);
    
    //GPIO interrupts
    gpio_set_irq_enabled(PIN_USR_BTN0, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_USR_BTN1, GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    printf("Booted successfully!\n");

    while(1){
        tight_loop_contents();
    }
}

bool blink_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}

bool gl_update_callback(__unused struct repeating_timer *t) {
    // lv_timer_handler();
    return true;
}

// void gl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pxmap) {
//     int32_t x, y;
//     uint16_t *buf16 = (uint16_t *)pxmap; //RGB565 display
//     for(y = area->y1; y <= area->y2; y++) {
//         for(x = area->x1; x <= area->x2; x++) {
//             ili9341_write_pix(&lcd_config, x, y, *buf16);
//             buf16++;
//         }
//     }
//     // lv_display_flush_ready(disp);
// }

void gpio_callback(uint gpio, uint32_t events) {
    switch(gpio) {
        case PIN_USR_BTN0: printf("Pressed button 0\n"); break;
        case PIN_USR_BTN1: printf("Pressed button 1\n"); break;
        case PIN_ENC_A:
        case PIN_ENC_B:
            rotary_encoder_update_pos(&nav_encoder, gpio_get(nav_encoder.pin_a), gpio_get(nav_encoder.pin_b));
            break;
        case PIN_ENC_BTN:
            if(events & GPIO_IRQ_EDGE_RISE)
                rotary_encoder_update_btn(&nav_encoder, false);
            else if(events & GPIO_IRQ_EDGE_FALL)
                rotary_encoder_update_btn(&nav_encoder, true);
            break;
        default: break;
    }
    if(nav_encoder.new_pos) {
        printf("Encoder pos: %0d\n", nav_encoder.pos);
        nav_encoder.new_pos = false;
    }
}
