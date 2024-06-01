#ifndef OV2640_H
#define OV2640_H
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

struct ov2640_config {
    i2c_inst_t *sccb;
    uint pin_sioc;
    uint pin_siod;
    
    uint pin_resetb;
    int pin_xclk;
    uint pin_vsync;
    // Y2, Y3, Y4, Y5, Y6, Y7, Y8, PCLK, HREF
    uint pin_y2_pio_base;

    PIO pio;
    uint frame_sm;
    uint byte_sm;

    uint frame_prog_offs;
    uint byte_prog_offs;

    bool pending_capture;

    uint dma_channel;
    uint8_t *image_buf;
    size_t image_buf_size;
};

void ov2640_init(struct ov2640_config *config);

void ov2640_frame_capture(struct ov2640_config *config, bool blocking);
void ov2640_sreset(struct ov2640_config *config);

void ov2640_reg_write(struct ov2640_config *config, uint8_t reg, uint8_t value);
uint8_t ov2640_reg_read(struct ov2640_config *config, uint8_t reg);
void ov2640_regs_write(struct ov2640_config *config, const uint8_t (*regs_list)[2]);
uint16_t ov2640_read_id(struct ov2640_config *config);

#endif
