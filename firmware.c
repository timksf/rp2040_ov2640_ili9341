#include <stdio.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "sd_card.h"
#include "ff.h"

#include "board_config.h"
#include "lib/lvgl/lvgl.h"
#include "lib/tjpgd/tjpgd.h"
#include "lib/ov2640/ov2640.h"
#include "lib/ili9341/ili9341.h"
#include "rotary_encoder.h"
#include "gui.h"
#include "util.h"

#define FRAME_WIDTH ILI9341_TFTHEIGHT
#define FRAME_HEIGHT ILI9341_TFTWIDTH

typedef struct {
    uint size;
    uint index;
    uint x_offs;
    uint y_offs;
    uint width_out;
    uint height_out;
} jpgd_session_t;

uint8_t frame_buf[BUF_SIZE]; //holds up to exactly one LCD frame
uint8_t jpgd_ws[TJPGD_WORKSPACE_SIZE];

ov2640_config_t cam_config;
ili9341_config_t lcd_config;
rotary_encoder_t nav_encoder;

static bool pending_capture = false;
static uint8_t *decode_p;
static uint8_t *decode_start;
static uint8_t *decode_end;
static const uint8_t jd_scale = 3; //scale of 3 to render decoded jpeg scaled by 1/8
static jpgd_session_t jpgd_session = { 0 };

//lvgl related globals
uint8_t gl_buf[FRAME_HEIGHT*FRAME_WIDTH*2 / 10]; //1/10 the size of actual frame is recommended
const int32_t GL_UPDATE_INTERVAL = -10; //call update function every few ms
const int32_t GL_TICK_INTERVAL = -5; //increase tick count independent of timer handler
lv_display_t *display;

struct repeating_timer gl_update_timer;
struct repeating_timer gl_tick_timer;
struct repeating_timer blink_timer;

void setup_gpio();
void setup_cam();
void setup_lcd();

//jpeg decoder input function
size_t jd_input(JDEC *jdec, uint8_t *buf, size_t len);
int jd_output(JDEC *jdec, void *bitmap, JRECT *rect);

//forward declaration of callbacks
void gpio_callback(uint gpio, uint32_t events);
bool blink_callback(__unused struct repeating_timer *t);
void gl_encoder_read(lv_indev_t*, lv_indev_data_t*);
bool gl_update_callback(__unused struct repeating_timer *t);
bool gl_inc_tick(__unused struct repeating_timer *t);
void gl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pxmap);

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file

    while (!tud_cdc_connected()) { 
        sleep_ms(100);
    }

    setup_lcd();
    setup_cam();
    setup_gpio();

    //LCD init
    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config, 1); //"landscape mode"
    ili9341_write_frame_mono(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, ILI9341_WHITE);

    //camera init
    ov2640_init(&cam_config);
    uint16_t cam_id = ov2640_read_id(&cam_config);
    printf("Camera ID: %02x\n", (int)cam_id);
    ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);

    //register timers
    add_repeating_timer_ms(-1000, blink_callback, NULL, &blink_timer);
    add_repeating_timer_ms(GL_UPDATE_INTERVAL, gl_update_callback, NULL, &gl_update_timer);
    add_repeating_timer_ms(GL_TICK_INTERVAL, gl_inc_tick, NULL, &gl_tick_timer);

    //LVGL setup
    lv_init();
    display = lv_display_create(FRAME_WIDTH, FRAME_HEIGHT);
    lv_display_set_buffers(display, gl_buf, NULL, sizeof(gl_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_default(display);
    lv_display_set_flush_cb(display, gl_flush);

    lv_group_set_default(lv_group_create());
    lv_indev_t *encoder = lv_indev_create();
    lv_indev_set_type(encoder, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_display(encoder, display);
    lv_indev_set_group(encoder, lv_group_get_default());
    lv_indev_set_read_cb(encoder, gl_encoder_read);
    lv_indev_set_mode(encoder, LV_INDEV_MODE_TIMER);

    lv_obj_t *menu_screen = lv_obj_create(NULL);
    create_menu(menu_screen);
    lv_scr_load(menu_screen);

    //jpeg decoder related
    JDEC jdec;
    jdec.swap = false;
    JRESULT jr;

    FRESULT fr;
    FATFS fs;
    char filename[64];

    //init SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        cancel_repeating_timer(&blink_timer);
        add_repeating_timer_ms(-200, blink_callback, NULL, &blink_timer);
    }

    //mount drive on sdcard
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        cancel_repeating_timer(&blink_timer);
        add_repeating_timer_ms(-200, blink_callback, NULL, &blink_timer);
    }

    printf("Booted successfully!\n");

    // ov2640_set_vflip(&cam_config, true);
    while(1) {
        if(disp_camera_preview()) {
            ov2640_frame_capture_single(&cam_config, true);
            ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, (uint16_t*)frame_buf);
            if(pending_capture) {
                sleep_ms(100);
                ov2640_capture_jpeg(&cam_config, FRAMESIZE_SXGA);
                printf("Successfully captured JPEG image(s)\n");
                
                //setup SPI for SD operation (LCD takes care of that itself)
                //writing to the SD has to happen before the decoder runs, otherwise the JPG might be overwritten
                snprintf(filename, 14, "%010d.jpg", to_ms_since_boot(get_absolute_time()));
                spi_set_baudrate(LCD_SPI_INST, 100 * 1000);
                spi_set_format(LCD_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
                save_jpeg_to_sd(filename, frame_buf, BUF_SIZE);

                find_next_jpeg(frame_buf, BUF_SIZE, &decode_start, &decode_end);
                jpgd_session.size = decode_end - decode_start + 1;
                jpgd_session.index = 0;

                //move JPEG to end of buffer for decoding
                memmove(frame_buf+BUF_SIZE-jpgd_session.size, decode_start, jpgd_session.size);
                decode_start = frame_buf+BUF_SIZE-jpgd_session.size;
                decode_end = decode_start + jpgd_session.size - 1;
                decode_p = decode_start;

                jr = jd_prepare(&jdec, &jd_input, jpgd_ws, TJPGD_WORKSPACE_SIZE, (void*)&jpgd_session);
                if(jr != JDR_OK) {
                    printf("Failed to prepare JPEG decoder %d\n", jr);
                    while(true);
                }

                uint output_width = jdec.width / 8;
                uint output_height = jdec.height / 8;
                jpgd_session.x_offs = (FRAME_WIDTH - output_width) >> 1;
                jpgd_session.y_offs = (FRAME_HEIGHT - output_height) >> 1;

                if((BUF_SIZE-jpgd_session.size) < output_width*output_height*2) {
                    printf("WARN: Decoding will overlap! (Might still work)");
                }
                // decode_p = decode_start; no need to reset pointer
                jr = jd_decomp(&jdec, &jd_output, jd_scale);
                if(jr != JDR_OK) {
                    printf("Failed to run JPEG decoder %d\n", jr);
                    // while(true);
                }
                printf("Decoded JPEG! w: %0d, h:%0d\n", output_width, output_height);
                ili9341_write_frame_mono(&lcd_config, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, ILI9341_WHITE);
                ili9341_write_frame_mono(&lcd_config, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, ILI9341_WHITE);
                ili9341_write_frame(&lcd_config, jpgd_session.x_offs, jpgd_session.y_offs, output_width, output_height, (uint16_t*)frame_buf);
                pending_capture = false;

                //wait for encoder-knob acknowledgement to return to preview
                while(!nav_encoder.pressed) tight_loop_contents();

                //reset camera settings for preview
                ov2640_set_color_format(&cam_config, COLOR_FORMAT_RGB565);
                ov2640_set_framesize(&cam_config, FRAMESIZE_QVGA);
                ov2640_reset_cif(&cam_config);
                sleep_ms(200);
            }
        }
    }
    f_unmount("0:");
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

    jpgd_session_t *session = (jpgd_session_t*) jdec->device;

    volatile uint16_t x = rect->left;
    volatile uint16_t y = rect->top;
    volatile uint16_t w = rect->right  + 1 - rect->left;
    volatile uint16_t h = rect->bottom + 1 - rect->top;

    // ili9341_write_frame(&lcd_config, x, y, w, h, (uint16_t*) bitmap);
    // sleep_ms(1);
    // printf("rect: x=%d, y=%d, w=%d, h=%d\n", x, y, w, h);
    for(uint i = 0; i < h; i++) 
        for(uint j = 0; j < w; j++)
            ((uint16_t*)frame_buf)[(y+i)*jdec->width/8+(x+j)] = ((uint16_t*)bitmap)[i*w+j];

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

    //handles rotary encoder GPIOs
    nav_encoder.pin_a = PIN_ENC_A;
    nav_encoder.pin_b = PIN_ENC_B;
    nav_encoder.pin_btn = PIN_ENC_BTN;
    rotary_encoder_init(&nav_encoder);

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
    lcd_config.baudrate = 1000 * 40000;
}


bool blink_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}

void gl_encoder_read(lv_indev_t *indev, lv_indev_data_t *data) {
    static int last_pos = 0;
    int diff = last_pos - nav_encoder.pos;
    data->enc_diff = -diff;

    if(nav_encoder.pressed)
        data->state = LV_INDEV_STATE_PRESSED;
    else
        data->state = LV_INDEV_STATE_RELEASED;

    last_pos = nav_encoder.pos;
    // printf("Encoder read callback! Diff:%d, Pressed: %d\n", -diff, nav_encoder.pressed);
}

bool gl_inc_tick(__unused struct repeating_timer *t) {
    lv_tick_inc(-GL_TICK_INTERVAL);
}

bool gl_update_callback(__unused struct repeating_timer *t) {
    lv_timer_handler();
    return true;
}

void gl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pxmap) {
    uint16_t *buf16 = (uint16_t *)pxmap; //RGB565 display
    uint x = area->x1;
    uint y = area->y1;
    uint width = area->x2-area->x1+1;
    uint height = area->y2-area->y1+1;
    ili9341_write_frame(&lcd_config, x, y, width, height, buf16);
    lv_display_flush_ready(disp);
}

void gpio_callback(uint gpio, uint32_t events) {
    switch(gpio) {
        case PIN_USR_BTN0: 
            load_main_page();
            pending_capture = false;
            break;
        case PIN_USR_BTN1: 
            printf("Pressed trigger button\n");
            if(disp_camera_preview()) //only capture an image if in preview
                pending_capture = true;
            break;
        case PIN_ENC_A:
        case PIN_ENC_B:
            rotary_encoder_update_pos(&nav_encoder, gpio_get(nav_encoder.pin_a), gpio_get(nav_encoder.pin_b));
            break;
        case PIN_ENC_BTN:
            rotary_encoder_update_btn(&nav_encoder, gpio_get(nav_encoder.pin_btn));
            break;
        default: break;
    }
    // if(nav_encoder.new_pos) {
    //     printf("Encoder pos: %0d\n", nav_encoder.pos);
    //     nav_encoder.new_pos = false;
    // }
}
