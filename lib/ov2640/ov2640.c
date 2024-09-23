#include <stdio.h>
#include <inttypes.h>

#include "ov2640.h"
#include "ov2640_init.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "image.pio.h"
#include "pico/time.h"

#define PIO_IRQ_NUM(pio, irq_index) ((pio == pio0) ? PIO0_IRQ_0 + (irq_index) : PIO1_IRQ_0 + (irq_index))

static const uint8_t OV2640_ADDR = 0x60 >> 1;
ov2640_config_t *volatile irq_context;
static uint32_t last_frame = 0;

static void frame_sync_isr(void) {
    uint32_t time = to_ms_since_boot(get_absolute_time());
    uint32_t frame_time = time - last_frame;
    last_frame = time;
    printf("Frame time: %" PRIuFAST32 " ms\n", frame_time);

    pio_interrupt_clear(irq_context->pio, 2); //interrupt is SM number here
    irq_clear(PIO_IRQ_NUM(irq_context->pio, IRQ_FRAME_SYNC));

    // pio_sm_set_enabled(irq_context->pio, irq_context->frame_sm, false);
    // pio_sm_clear_fifos(irq_context->pio, irq_context->frame_sm);

    // pio_sm_restart(irq_context->pio, irq_context->frame_sm);
    // pio_sm_exec(irq_context->pio, irq_context->frame_sm, pio_encode_jmp(irq_context->frame_prog_offs));
    // pio_sm_set_enabled(irq_context->pio, irq_context->frame_sm, true);

    //clear DMA interrupt
    // dma_hw->ints0 = dma_hw->ints0;
}

void ov2640_init(ov2640_config_t *config) {
    // XCLK generation
    if(config->pin_xclk >= 0) {
        gpio_set_function(config->pin_xclk, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(config->pin_xclk);
        // 6 cycles (0 to 5), 125 MHz / 6 = ~20.83 MHz wrap rate
        pwm_set_wrap(slice_num, 5);
        pwm_set_gpio_level(config->pin_xclk, 3);
        pwm_set_enabled(slice_num, true);
    }

    // SCCB I2C @ 100 kHz
    gpio_set_function(config->pin_sioc, GPIO_FUNC_I2C);
    gpio_set_function(config->pin_siod, GPIO_FUNC_I2C);
    i2c_init(config->sccb, 100 * 1000);

    // Initialise reset pin
    gpio_init(config->pin_resetb);
    gpio_set_dir(config->pin_resetb, GPIO_OUT);

    // Reset camera, and give it some time to wake back up
    gpio_put(config->pin_resetb, 0);
    sleep_ms(100);
    gpio_put(config->pin_resetb, 1);
    sleep_ms(100);
    //perform soft reset after hard reset
    ov2640_sreset(config);
    sleep_ms(100);

    // Initialize the camera itself over SCCB
    ov2640_regs_write(config, ov2640_settings_cif);
    ov2640_set_framesize(config, FRAMESIZE_CIF);

    // Enable image RX PIO
    config->frame_prog_offs = pio_add_program(config->pio, &image_frame_capture_single_program);
    config->frame_sized_prog_offs = pio_add_program(config->pio, &image_frame_capture_sized_program);

    //setup the GPIOS via a PIO program executed on any non-init'ed PIO SM
    camera_pio_setup_gpios(config->pio, config->frame_sm, config->pin_y2_pio_base);
    gpio_pull_up(config->pin_vsync);

    //enable PIO interrupts for frame capture SM
    irq_set_enabled(PIO_IRQ_NUM(config->pio, IRQ_FRAME_DONE), true);

    image_frame_capture_single_init(config->pio, config->frame_sm, config->frame_prog_offs, config->pin_y2_pio_base);
    image_frame_capture_sized_init(config->pio, config->frame_sized_sm, config->frame_sized_prog_offs, config->pin_y2_pio_base);

    ///-------------TESTING-------------
    // irq_context = config;    
    // irq_set_enabled(PIO0_IRQ_1, true);
    // irq_set_exclusive_handler(PIO0_IRQ_1, frame_sync_isr);
    // uint offs = pio_add_program(config->pio, &frame_sync_program);
    // frame_sync_init(config->pio, 2, offs, config->pin_y2_pio_base);
    // pio_sm_set_jmp_pin(config->pio, 2, config->pin_y2_pio_base + OFFS_VSYNC);
}

void ov2640_frame_capture_single(ov2640_config_t *config, bool blocking) {
    uint sm = config->frame_sized_sm;

    dma_channel_config c = dma_channel_get_default_config(config->dma_data_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(config->pio, sm, false));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

    dma_channel_configure(
        config->dma_data_chan, &c,
        config->image_buf,
        &config->pio->rxf[sm],
        config->frame_size_bytes,
        false
    );

    dma_channel_start(config->dma_data_chan);
    trigger_frame_capture(config->pio, sm, config->frame_size_bytes-1);
    dma_channel_wait_for_finish_blocking(config->dma_data_chan);

    // pio_sm_set_enabled(config->pio, sm, false);
    // pio_sm_clear_fifos(config->pio, sm);

    // pio_sm_restart(config->pio, sm);
    // pio_sm_exec(config->pio, sm, pio_encode_jmp(config->frame_prog_offs));

    // pio_sm_set_enabled(config->pio, sm, true);
    // dma_channel_wait_for_finish_blocking(config->dma_channel);
}

void ov2640_frame_capture_continuous(ov2640_config_t *config, uint chain_to) {
    //This function sets up two DMA channels
    //One DMA channel is responsible for moving the data from the PIO
    //image capture SM to main memory
    //The other DMA channel is responsible of restarting the other one, when it finishes
    //It simply writes to the control registers of the other DMA channel when
    //this channel triggers its "finish" interrupt

    //first, construct the control DMA 
    dma_channel_config c = dma_channel_get_default_config(config->dma_ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(
        config->dma_ctrl_chan, &c, 
        &dma_hw->ch[config->dma_data_chan].al2_write_addr_trig, //reset the write address after each completed DMA transfer
        &config->image_buf, //the write_addr of the data channel is reset to the beginning of the frame buffer
        1, //write a single address, so a single word
        false
    );

    //chaining from ctrl to data channel is not needed as we use a *triggering* write 

    //now, construct the data DMA, same as for capturing single frames
    dma_channel_config c1 = dma_channel_get_default_config(config->dma_data_chan);
    channel_config_set_read_increment(&c1, false);
    channel_config_set_write_increment(&c1, true);
    channel_config_set_dreq(&c1, pio_get_dreq(config->pio, config->frame_sm, false));
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_8);
    
    //optionally chain to a different channel
    if(chain_to >= NUM_DMA_CHANNELS || !dma_channel_is_claimed(chain_to))
        channel_config_set_chain_to(&c1, config->dma_ctrl_chan);
    else
        channel_config_set_chain_to(&c1, chain_to);

    dma_channel_configure(
        config->dma_data_chan, &c1,
        config->image_buf,
        &config->pio->rxf[config->frame_sm],
        config->frame_size_bytes,
        false
    );


    //raise DMA_IRQ_0 when data channel finishes
    // dma_channel_set_irq0_enabled(config->dma_ctrl_chan, true);
    // irq_set_enabled(DMA_IRQ_0, true);
    // irq_set_exclusive_handler(DMA_IRQ_0, frame_sync_isr);

    //start control channel!
    dma_channel_start(config->dma_ctrl_chan);
}

void ov2640_reg_write(ov2640_config_t *config, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    int err = i2c_write_blocking(config->sccb, OV2640_ADDR, data, sizeof(data), false);
    if(err == PICO_ERROR_GENERIC){
        printf("I2C error\n");
    }
}

uint8_t ov2640_reg_read(ov2640_config_t *config, uint8_t reg) {
    i2c_write_blocking(config->sccb, OV2640_ADDR, &reg, 1, false);

    uint8_t value;
    i2c_read_blocking(config->sccb, OV2640_ADDR, &value, 1, false);

    return value;
}

void ov2640_regs_write(ov2640_config_t *config, const uint8_t (*regs_list)[2]) {
    while (1) {
        uint8_t reg = (*regs_list)[0];
        uint8_t value = (*regs_list)[1];

        if (reg == 0xff && value == 0xff) {
            //ENDMARKER detected
            break;
        }

        ov2640_reg_write(config, reg, value);

        regs_list++;
    }
}

void ov2640_sreset(ov2640_config_t *config) {
    ov2640_reg_write(config, 0xFF, 0x01); //select camera registers
    ov2640_reg_write(config, 0x12, 0x80); //soft reset via COM7 register
    sleep_ms(10);
}

uint16_t ov2640_read_id(ov2640_config_t *config) {
    ov2640_reg_write(config, 0xFF, 0x01);
    uint8_t pidh = ov2640_reg_read(config, 0x0A);
    uint8_t pidl = ov2640_reg_read(config, 0x0B);
    uint16_t pid = ((uint16_t) pidh) << 8 | pidl;
    return pid;
}

void ov2640_set_color_format(ov2640_config_t *config, color_format_t color_format) {
    config->color_format = color_format;
    switch(color_format) {
        case COLOR_FORMAT_RGB565:
            ov2640_regs_write(config, ov2640_settings_rgb565);
            break;
        case COLOR_FORMAT_JPEG:
            ov2640_regs_write(config, ov2640_settings_jpeg);
            break;
        default: break; //TODO: return false/-1
    }
    sleep_ms(15);
}

void ov2640_set_framesize(ov2640_config_t *config, framesize_t framesize) {
    int ret = 0;
    const uint8_t (*regs)[2];
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    aspect_ratio_t ratio = resolution[framesize].aspect_ratio;
    uint16_t max_x = ratio_table[ratio].max_x;
    uint16_t max_y = ratio_table[ratio].max_y;
    uint16_t offset_x = ratio_table[ratio].offset_x;
    uint16_t offset_y = ratio_table[ratio].offset_y;
    ov2640_sensor_mode_t mode = OV2640_MODE_UXGA;
    uint8_t clkrc, pclk_div;

    //the ordering here is achieved with the correct order of definitions in the enum
    if (framesize <= FRAMESIZE_CIF) {
        mode = OV2640_MODE_CIF;
        //for CIF shrink the active sensor area by a factor of 4
        max_x /= 4;
        max_y /= 4;
        offset_x /= 4;
        offset_y /= 4;
        if(max_y > 296){
            max_y = 296;
        }
    } else if (framesize <= FRAMESIZE_SVGA) {
        mode = OV2640_MODE_SVGA;
        //for SVGA shrink the active sensor area by a factor of 2
        max_x /= 2;
        max_y /= 2;
        offset_x /= 2;
        offset_y /= 2;
    }

    //now adjust the sensor registers to materialize the new framesize
#define VAL_SET(x, mask, rshift, lshift) ((((x) >> (rshift)) & mask) << (lshift))

    uint8_t ov2640_size_regs[][2] = {
        { BANK_SEL, BANK_SEL_DSP },
        { RESET, RESET_DVP },
        // { SIZEL, (((max_x >> 11) & 0b1) << 6) | ((max_x & 0b111) << 3) | (max_y & 0b111) },
        // { HSIZE8, max_x >> 3 }, //HSIZE8 = hsize/8
        // { VSIZE8, max_y >> 3 }, //VSIZE8 = vsize/8
        { HSIZE, (max_x >> 2) & 0xFF }, //HSIZE = hsize/4
        { VSIZE, (max_y >> 2) & 0xFF }, //VSIZE = vsize/4
        { XOFFL, offset_x & 0xFF },
        { YOFFL, offset_y & 0xFF },
        { VHYX, VAL_SET(max_y, 0b1, 8+2, 7) |       //V_SIZE[8]
                VAL_SET(offset_y, 0b111, 8, 4) |     //OFFSET_Y[10:8]
                VAL_SET(max_x, 0b1, 8+2, 3) |       //H_SIZE[8]
                VAL_SET(offset_x, 0b111, 8, 0) },    //OFFSET_X[10:8]
        { TEST, (max_x >> (9+2)) << 7},
        { ZMOW, (w >> 2) & 0xFF },
        { ZMOH, (h >> 2) & 0xFF },
        { ZMHH, (((h >> 10) & 0b1) << 2) | ((w >> 10) & 0b11)},
        ENDMARKER,
    };

    if(config->color_format == COLOR_FORMAT_JPEG) {
        clkrc = 0x00;
        pclk_div = 8;
        if(mode == OV2640_MODE_UXGA){
            pclk_div = 12;
        }
    } else {

        pclk_div = R_DVP_SP_AUTO_MODE;
        // pclk_div |= 8;

        clkrc = CLKRC_2X | CLKRC_DIV(8);

        if (mode == OV2640_MODE_CIF) {
            clkrc = CLKRC_2X | CLKRC_DIV(2);
        } else if(mode == OV2640_MODE_UXGA) {
            pclk_div = 12;
        }
    }

    if (mode == OV2640_MODE_CIF) {
        regs = ov2640_settings_to_cif;
    } else if (mode == OV2640_MODE_SVGA) {
        regs = ov2640_settings_to_svga;
    } else {
        regs = ov2640_settings_to_uxga;
    }

    printf("CLKRC: %0x R_DVP_SP: %0x\n", clkrc, pclk_div);

    //disable DSP while changing settings
    ov2640_reg_write(config, BANK_SEL, BANK_SEL_DSP);
    ov2640_reg_write(config, R_BYPASS, R_BYPASS_DSP_BYPAS);

    ov2640_regs_write(config, regs);
    ov2640_regs_write(config, ov2640_size_regs);

    ov2640_reg_write(config, BANK_SEL, BANK_SEL_SENSOR);
    ov2640_reg_write(config, CLKRC, clkrc);

    ov2640_reg_write(config, BANK_SEL, BANK_SEL_DSP);
    ov2640_reg_write(config, R_DVP_SP, pclk_div);
    ov2640_reg_write(config, R_BYPASS, R_BYPASS_DSP_EN);

    sleep_ms(100);

    //set pixel format registers again (since DSP was resetted)
    ov2640_set_color_format(config, config->color_format);

    switch(config->color_format){
        case COLOR_FORMAT_RGB565: config->frame_size_bytes = 2*h*w; break;
        case COLOR_FORMAT_JPEG: config->frame_size_bytes = config->image_buf_size; break; //h*w / 5 see adafruit driver, probably accurate approximation
        default: config->frame_prog_offs = 0; break;
    }

    if(config->frame_size_bytes > config->image_buf_size) {
        //TODO: error!!!!!
    }

    uint sm = config->frame_sized_sm;
    pio_sm_set_enabled(config->pio, sm, false);
    pio_sm_clear_fifos(config->pio, sm);

    pio_sm_restart(config->pio, sm);
    pio_sm_exec(config->pio, sm, pio_encode_jmp(config->frame_sized_prog_offs));

    pio_sm_set_enabled(config->pio, sm, true);
}

void ov2640_enable_test_pattern(ov2640_config_t *config) {
    ov2640_reg_write(config, BANK_SEL, BANK_SEL_SENSOR);
    uint8_t com7 = ov2640_reg_read(config, COM7);
    com7 |= COM7_COLOR_BAR; //enable test pattern
    ov2640_reg_write(config, COM7, com7);
}

void ov2640_set_brightness(ov2640_config_t *config, int8_t level) {
    ov2640_reg_write(config, BANK_SEL, BANK_SEL_DSP);

    level += 3;
    if (level <= 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return;
    }

    for (int i=0; i<5; i++) {
        ov2640_reg_write(config, brightness_regs[0][i], brightness_regs[level][i]);
    }

}

void ov2640_capture_jpeg(ov2640_config_t *config, framesize_t framesize) {
    ov2640_set_color_format(config, COLOR_FORMAT_JPEG);
    ov2640_set_framesize(config, framesize);
    sleep_ms(100);
    ov2640_frame_capture_single(config, true);
}