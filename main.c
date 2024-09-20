#include <stdio.h>
#include <tusb.h>
#include <unistd.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "ov2640.h"
#include "ov2640_regs.h"
#include "ili9341.h"
#include "hardware/dma.h"

#include "board_config.h"

uint8_t frame_buf[BUF_SIZE];
//used for uart DMA transfer
typedef struct {
    uint32_t len;
    void *buf;
} uart_dma_ctrl_block_t;

const uart_dma_ctrl_block_t uart_dma_idle_ctrl = { 0, NULL };

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
    //turn-off/on auto translation of data bytes
    stdio_set_translate_crlf(&stdio_usb, true);
#endif //CUSTOM_BOARD

    for(int i = 0; i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++) 
        ((uint16_t*)frame_buf)[i] = ILI9341_MAGENTA;
    
    ili9341_config_t lcd_config;
    ov2640_config_t cam_config;

    lcd_config.spi = LCD_SPI_INST;
    lcd_config.pin_rst = PIN_LCD_RST;
    lcd_config.pin_dc = PIN_LCD_DC;
    lcd_config.pin_cs = PIN_LCD_CS;
    lcd_config.pin_sck = PIN_LCD_SCK;
    lcd_config.pin_tx = PIN_LCD_TX;
    lcd_config.dma_data_chan = dma_claim_unused_channel(true);
    lcd_config.dma_ctrl_chan = dma_claim_unused_channel(true);
    lcd_config.frame_buf = (uint16_t*) &frame_buf[0];

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

    ili9341_init(&lcd_config);
    ili9341_set_rotation(&lcd_config,1); //"landscape mode"
    ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, (uint16_t*)frame_buf);

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

    uint32_t time, last_time;
    last_time = to_ms_since_boot(get_absolute_time());

    // ov2640_frame_capture_continuous(&cam_config, false);

    while(1) {
        ov2640_frame_capture_single(&cam_config, true);
        ili9341_write_frame(&lcd_config, 0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, (uint16_t*)frame_buf);
        time = to_ms_since_boot(get_absolute_time());
        uint32_t frame_time = time - last_time;
        last_time = time;
        printf("Main loop time: %" PRIuFAST32 " ms\n", frame_time);
        tight_loop_contents();
    }
}

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}

    /*
        We use several DMA channels with a specific chained configuration:

        CTRL->CAM_DATA->UART_SETUP->UART_DATA->UART_RESET->...

        CTRL: setup cam data channel by updating the write address to the frame buffer location
        CAM_DATA: transfer data corresponding to one frame from PIO to memory
        UART_SETUP: write transfer count from control block to UART_DATA DMA channel
        UART_DATA: transfer nothing or a whole frame over uart, depending on the setup
        UART_RESET: set transfer size in control block to 0, so that a single command 
            corresponds to a single transferred frame
    */

    // uint uart_ctrl_dma_channel = dma_claim_unused_channel(true);
    // uint uart_dma_channel = dma_claim_unused_channel(true);
    // uint uart_ctrl_rst_dma_channel = dma_claim_unused_channel(true);

    // //modifying this control block will allow using the uart dma data channel
    // //start with a length of 0, since no transfer is pending yet
    // uart_dma_ctrl_block_t uart_dma_ctrl_block = { 0, frame_buf };

    // //ctrl channel for uart channel setup
    // //this is used to trigger the uart channel from external events, such as received commands 
    // dma_channel_config c = dma_channel_get_default_config(uart_ctrl_dma_channel);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, true);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    // channel_config_set_ring(uart_ctrl_dma_channel, false, 1 << 2); //write wrap around after 1 32bit word
    // channel_config_set_ring(uart_ctrl_dma_channel, true, 1 << 3); //read wrap around after 2 32bit words 
    // dma_channel_configure(
    //     uart_ctrl_dma_channel, &c,
    //     //this will write the trans_count and then trigger the data channel by writing the read address
    //     &dma_hw->ch[uart_dma_channel].al3_transfer_count, 
    //     &uart_dma_ctrl_block,
    //     2, //2 words are transferred, transfer count and read pointer
    //     false
    // );

    // //uart transfer DMA channel
    // c = dma_channel_get_default_config(uart_dma_channel);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, false);
    // channel_config_set_dreq(&c, uart_get_dreq(UART_INST, true));
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // dma_channel_configure(
    //     uart_dma_channel, &c,
    //     &uart_get_hw(UART_INST)->dr,
    //     uart_dma_ctrl_block.buf,
    //     uart_dma_ctrl_block.len,
    //     false
    // );

    // //uart transfer reset channel
    // //this channel simply copies the IDLE config to the currently active config
    // c = dma_channel_get_default_config(uart_ctrl_rst_dma_channel);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, true);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    // //both read and write wrap around after two words, so 8 bytes
    // channel_config_set_ring(uart_ctrl_dma_channel, false, 1 << 3);
    // channel_config_set_ring(uart_ctrl_dma_channel, true, 1 << 3);
    // dma_channel_configure(
    //     uart_ctrl_rst_dma_channel, &c,
    //     &uart_dma_ctrl_block,           //write to active config
    //     &uart_dma_idle_ctrl,            //read from idle config
    //     2,
    //     false
    // );