#include <stdio.h>

#include "ov2640.h"
#include "ov2640_init.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "image.pio.h"

static const uint8_t OV2640_ADDR = 0x60 >> 1;
struct ov2640_config *volatile irq_context;

static void frame_done_isr(void) {
    if(irq_context == NULL) 
        return;
    //restart byte capture SM 
    pio_sm_restart(irq_context->pio, irq_context->byte_sm);
    pio_sm_clear_fifos(irq_context->pio, irq_context->byte_prog_offs);
    //jump to beginning so that byte SM waits for interrupt
    pio_sm_exec(irq_context->pio, irq_context->byte_sm, 0x0000 | irq_context->byte_prog_offs);

    irq_context->pending_capture = false;
}

void ov2640_init(struct ov2640_config *config) {
    // XCLK generation (~20.83 MHz)
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

    // Initialise the camera itself over SCCB
    ov2640_regs_write(config, ov2640_qvga);

    // Set RGB565 output mode
    ov2640_reg_write(config, 0xff, 0x00);
    ov2640_reg_write(config, 0xDA, (ov2640_reg_read(config, 0xDA) & 0xC) | 0x8);

    // Enable image RX PIO
    config->frame_prog_offs = pio_add_program(config->pio, &image_frame_capture_program);
    config->byte_prog_offs = pio_add_program(config->pio, &image_byte_capture_program);

    camera_pio_setup_gpios(config->pio, config->frame_sm, config->pin_y2_pio_base); //any non-init'ed SM works

    image_frame_capture_init(config->pio, config->frame_sm, offset_frame_capture, config->pin_y2_pio_base);
    image_byte_capture_init(config->pio, config->byte_sm, config->byte_prog_offs, config->pin_y2_pio_base);

    irq_set_enabled(PIO0_IRQ_0, true);
    irq_set_enabled(PIO0_IRQ_1, true);

    irq_context = config; //any better way to do this?
    irq_set_exclusive_handler(PIO0_IRQ_1, frame_done_isr);
    config->pending_capture = false;
}

void ov2640_frame_capture(struct ov2640_config *config, bool blocking) {
    dma_channel_config c = dma_channel_get_default_config(config->dma_channel);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(config->pio, config->byte_sm, false));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    
    dma_channel_configure(
        config->dma_channel, &c,
        config->image_buf,
        &config->pio->rxf[config->byte_sm], //the byte SM outputs the data
        config->image_buf_size,
        true //start right away, frame synchronization is handled by PIO
    );

    // Wait for vsync rising edge to start frame
    // while (gpio_get(config->pin_vsync) == true);
    // while (gpio_get(config->pin_vsync) == false);

    //trigger_frame_capture clears IRQ_FRAME_REQ, starting the capture of a frame
    trigger_frame_capture(config->pio);
    irq_context->pending_capture = true;

    if(blocking) { //wait until IRQ_FRAME_DONE is triggered
        while(config->pending_capture)
            tight_loop_contents();
    }

}

void ov2640_reg_write(struct ov2640_config *config, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    int err = i2c_write_blocking(config->sccb, OV2640_ADDR, data, sizeof(data), false);
    if(err == PICO_ERROR_GENERIC){
        printf("I2C error\n");
    }
}

uint8_t ov2640_reg_read(struct ov2640_config *config, uint8_t reg) {
    i2c_write_blocking(config->sccb, OV2640_ADDR, &reg, 1, false);

    uint8_t value;
    i2c_read_blocking(config->sccb, OV2640_ADDR, &value, 1, false);

    return value;
}

void ov2640_regs_write(struct ov2640_config *config, const uint8_t (*regs_list)[2]) {
    // for(uint32_t idx = 0; idx < sizeof(regs_list)/2; idx++) {
    //     uint8_t reg = regs_list[idx][0];
    //     uint8_t value = regs_list[idx][1];
    //     ov2640_reg_write(config, reg, value);
    // }
    while (1) {
		uint8_t reg = (*regs_list)[0];
		uint8_t value = (*regs_list)[1];

		if (reg == 0x00 && value == 0x00) {
			break;
		}

		ov2640_reg_write(config, reg, value);

		regs_list++;
	}
}

void ov2640_sreset(struct ov2640_config *config) {
    ov2640_reg_write(config, 0xFF, 0x01); //select camera registers
    ov2640_reg_write(config, 0x12, 0x80); //soft reset via COM7 register
    sleep_ms(10);
}

uint16_t ov2640_read_id(struct ov2640_config *config) {
    ov2640_reg_write(config, 0xFF, 0x01);
    uint8_t pidh = ov2640_reg_read(config, 0x0A);
    uint8_t pidl = ov2640_reg_read(config, 0x0B);
    uint16_t pid = ((uint16_t) pidh) << 8 | pidl;
    return pid;
}