#include <stdio.h>
#include <tusb.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sd_card.h"
#include "ff.h"

#include "lib/ov2640/ov2640.h"
#include "lib/ili9341/ili9341.h"
#include "lib/tjpgd/tjpgd.h"

#include "board_config.h"
#include "util.h"

uint8_t frame_buf[BUF_SIZE] __attribute__ ((aligned (4)));

//workspace for JPEG decoder
uint8_t jpgd_ws[TJPGD_WORKSPACE_SIZE] __attribute__((aligned(4)));

static bool running = true;
static bool pending_capture = false;
static struct repeating_timer blink_timer;
static ov2640_config_t cam_config;
static ili9341_config_t lcd_config;

void setup_gpio();
void setup_cam();
void setup_lcd();

//jpeg decoder input function
size_t jd_input(JDEC *jdec, uint8_t *buf, size_t len);
int jd_output(JDEC *jdec, void *bitmap, JRECT *rect);

//pointer for current position in JPEG data
static uint8_t *decode_p;
static uint8_t *decode_start;
static uint8_t *decode_end;
static const uint8_t jd_scale = 3; //scale of 3 to render decoded jpeg scaled by 1/8

typedef struct {
    uint size;
    uint index;
} jpgd_session_t;

//callbacks
void gpio_callback(uint gpio, uint32_t events);
bool timer_callback(__unused struct repeating_timer *t);

int main() {

    FRESULT fr;
    FATFS fs;
    char filename[] = "test02.txt";

    stdio_init_all(); //will only init what is enabled in the CMakeLists file

#ifndef CUSTOM_BOARD
    uart_init(UART_INST, 1000000);
    gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_INST, false, false);
    uart_set_format(UART_INST, 8, 1, UART_PARITY_NONE);
    uart_puts(UART_INST, "Hello from UART");
#else 
    //turn-off/on auto translation of data bytes
    stdio_set_translate_crlf(&stdio_usb, true);
#endif //CUSTOM_BOARD

    // after setup wait for serial terminal connection on USB port
    // while (!tud_cdc_connected()) { 
    //     sleep_ms(100);
    // }
    printf("Booted successfully!\n");

    setup_gpio();
    setup_cam();
    setup_lcd();

    //init camera
    ov2640_init(&cam_config);
    uint16_t cam_id = ov2640_read_id(&cam_config);
    printf("Camera ID: %02x\n", (int)cam_id);
    ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);

    //init display
    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config, 1); //"landscape mode"
    for(int i = 0; i < ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH; i++) 
        ((uint16_t*)frame_buf)[i] = ILI9341_WHITE;
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, (uint16_t*)frame_buf);

    add_repeating_timer_ms(-200, timer_callback, NULL, &blink_timer);

    jpgd_session_t jpgd_session = { .size = 0, .index = 0 };

    // //init SD card
    // if (!sd_init_driver()) {
    //     printf("ERROR: Could not initialize SD card\r\n");
    //     while (true);
    // }

    // //mount drive on sdcard
    // fr = f_mount(&fs, "0:", 1);
    // if (fr != FR_OK) {
    //     printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
    //     while (true);
    // }
    JDEC jdec;
    jdec.swap = false;
    JRESULT jr;

    ov2640_set_brightness(&cam_config, -1);
    //capture loop
    while (running) {
        if(pending_capture) {
            ov2640_set_brightness(&cam_config, -2);
            sleep_ms(100);
            ov2640_capture_jpeg(&cam_config, FRAMESIZE_UXGA);
            printf("Successfully captured JPEG image(s)\n");
            // save_jpeg_to_sd("test-2.jpg", frame_buf, BUF_SIZE);

            //display preview
            find_next_jpeg(frame_buf, BUF_SIZE, &decode_start, &decode_end);
            decode_p = decode_start;
            jpgd_session.size = decode_end - decode_start + 1;
            jpgd_session.index = 0;

            jr = jd_prepare(&jdec, &jd_input, jpgd_ws, TJPGD_WORKSPACE_SIZE, (void*)&jpgd_session);
            if(jr != JDR_OK) {
                printf("Failed to prepare JPEG decoder %d\n", jr);
                while(true);
            }
            // decode_p = decode_start; no need to reset pointer
            jr = jd_decomp(&jdec, &jd_output, jd_scale);
            if(jr != JDR_OK) {
                printf("Failed to run JPEG decoder %d\n", jr);
                // while(true);
            }

            pending_capture = false;
        }
    }

    //unmount drive before stopping
    // f_unmount("0:");

    while(true) tight_loop_contents();
}

size_t jd_input(JDEC *jdec, uint8_t *buf, size_t len) {
    //the input function is used to fill the JPEG decoder input stream
    //simply copy from frame buffer where the JPEG is located
    jpgd_session_t *session = (jpgd_session_t*) jdec->device;
    if(session->index + len > session->size)
        len = session->size - session->index;

    if(buf) //copy bytes into buf if not NULL
        for(uint i = 0; i < len; i++)
            buf[i] = decode_p[session->index+i];

    session->index += len; //move index further in any case
    return len;
}

int jd_output(JDEC *jdec, void *bitmap, JRECT *rect) {
    
    (void) jdec;

    volatile uint16_t x = rect->left + 0;
    volatile uint16_t y = rect->top  + 0;
    volatile uint16_t w = rect->right  + 1 - rect->left;
    volatile uint16_t h = rect->bottom + 1 - rect->top;

    ili9341_write_frame(&lcd_config, x, y, w, h, (uint16_t*) bitmap);
    printf("rect: x=%d, y=%d, w=%d, h=%d\n", x, y, w, h);
    return 1;
}

void setup_gpio() {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    //GPIO user input
    gpio_init(PIN_USR_BTN0);
    gpio_set_dir(PIN_USR_BTN0, false);
    gpio_disable_pulls(PIN_USR_BTN0);

    gpio_init(PIN_USR_BTN1);
    gpio_set_dir(PIN_USR_BTN1, false);
    gpio_disable_pulls(PIN_USR_BTN1);

    gpio_set_irq_enabled(PIN_USR_BTN0, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_USR_BTN1, GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void setup_cam() {
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
    cam_config.dma_data_chan = dma_claim_unused_channel(true);
    cam_config.dma_ctrl_chan = dma_claim_unused_channel(true);
    cam_config.image_buf = (uint8_t*)&frame_buf[0];
    cam_config.image_buf_size = sizeof(frame_buf);
    cam_config.frame_size_bytes = 0; //this will be used for transfers and is set by ov2640_set_framesize
    cam_config.color_format = COLOR_FORMAT_RGB565;
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

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}

void gpio_callback(uint gpio, uint32_t events) {
    switch(gpio) {
        case PIN_USR_BTN0: 
            printf("Pressed stop button\n");
            running = false;
            break;
        case PIN_USR_BTN1: 
            printf("Pressed trigger button\n"); 
            pending_capture = true;
            break;
        default: break;
    }
}
