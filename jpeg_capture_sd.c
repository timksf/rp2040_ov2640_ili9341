#include <stdio.h>
#include <tusb.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "sd_card.h"
#include "ff.h"

#include "lib/ov2640/ov2640.h"

#include "board_config.h"

uint8_t frame_buf[BUF_SIZE] __attribute__ ((aligned (4)));

static const uint16_t SOI = 0xFFD8;
static const uint16_t EOI = 0xFFD9;

static bool running = true;
static bool pending_capture = false;
static struct repeating_timer blink_timer;
static ov2640_config_t cam_config;

void save_bin(const char *filename, uint8_t *d, uint size);
void save_jpeg_to_sd(const char *filename, uint8_t *start, uint size);
void setup_gpio();
void setup_cam();

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
    while (!tud_cdc_connected()) { 
        sleep_ms(100);
    }
    printf("Booted successfully!\n");

    setup_gpio();
    setup_cam();

    ov2640_init(&cam_config);
    uint16_t cam_id = ov2640_read_id(&cam_config);
    printf("Camera ID: %02x\n", (int)cam_id);
    ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);


    //init SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        while (true);
    }

    //mount drive on sdcard
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        while (true);
    }

    //capture loop
    while (running) {
        if(pending_capture) {
            ov2640_set_color_format(&cam_config, COLOR_FORMAT_JPEG);
            ov2640_set_framesize(&cam_config, FRAMESIZE_UXGA);
            ov2640_frame_capture_single(&cam_config, true);
            printf("Successfully captured JPEG image(s)\n");

            //look for SOI and EOI markers
            uint8_t *p = frame_buf;
            uint8_t *s = NULL, *e = NULL;
            while(p < &frame_buf[BUF_SIZE-1]) {
                uint16_t m = *p<<8 | *(p+1);
                if(m == SOI && s == NULL) {
                    s = p;
                }
                if(m == EOI && e == NULL && s != NULL) {
                    e = p+1;
                    break;
                }
                p++;
            }

            uint sz = e - s + 1;

            if(s != NULL && e != NULL && sz > 0) {
                printf("Valid JPEG file at %p, %p, with size %u\n", s, e, sz);
                save_jpeg_to_sd("test.jpg", (uint8_t*)s, sz);
                //also save the entire buffer as a test
                // save_bin("buffer.bin", frame_buf, sizeof(frame_buf));
                // printf("Saved frame buffer to file\n");
            } else
                printf("No valid JPEG file in buffer\n");

            pending_capture = false;
        }
    }

    //unmount drive before stopping
    f_unmount("0:");

    while(true) tight_loop_contents();
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

    add_repeating_timer_ms(-1000, timer_callback, NULL, &blink_timer);
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

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}

void save_jpeg_to_sd(const char *filename, uint8_t *start, uint size) {
    save_bin(filename, start, size);
}

void save_bin(const char *filename, uint8_t *d, uint size) {
    FRESULT fr;
    FIL f;
    FIL *fp = &f;
    
    //try to open file
    fr = f_open(fp, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        while (true);
    }

    uint bytes_written = 0;

    fr = f_write(fp, d, size, &bytes_written);
    if (fr != FR_OK) {
        printf("ERROR: Could not write to file (%d)\r\n", fr);
        while (true);
    } else {
        printf("Wrote %u bytes\n", bytes_written);
    }

    fr = f_close(fp);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true);
    }
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
