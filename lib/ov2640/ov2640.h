#ifndef OV2640_H
#define OV2640_H
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include "sensor.h"

typedef enum {
    OV2640_MODE_UXGA, OV2640_MODE_SVGA, OV2640_MODE_CIF, OV2640_MODE_MAX
} ov2640_sensor_mode_t;

typedef struct {
    uint16_t offset_x;
    uint16_t offset_y;
    uint16_t max_x;
    uint16_t max_y;
} ov2640_ratio_settings_t;

typedef struct {
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
    uint frame_sized_sm;

    uint frame_prog_offs;
    uint frame_sized_prog_offs;

    bool pending_capture;

    uint dma_channel;
    uint8_t *image_buf;
    size_t image_buf_size;
    size_t frame_size_bytes;

    color_format_t color_format;
} ov2640_config_t;

//it's important these are in the same order as the aspect ratio definitions
//the offsets place the image in the center of the 1600x1200 area
static const ov2640_ratio_settings_t ratio_table[] = {
    // offs_x, offs_y, max_x, max_y
    {   0,   0, 1600, 1200 }, //4x3
    {   8,  72, 1584, 1056 }, //3x2
    {   0, 100, 1600, 1000 }, //16x10
    {   0, 120, 1600,  960 }, //5x3
    {   0, 150, 1600,  900 }, //16x9
    {   2, 258, 1596,  684 }, //21x9
    {  50,   0, 1500, 1200 }, //5x4
    { 200,   0, 1200, 1200 }, //1x1
    { 462,   0,  676, 1200 }  //9x16
};

void ov2640_init(ov2640_config_t *config);

void ov2640_frame_capture_single(ov2640_config_t *config, bool blocking);
void ov2640_frame_capture_continuous(ov2640_config_t *config);

void ov2640_sreset(ov2640_config_t *config);

void ov2640_reg_write(ov2640_config_t *config, uint8_t reg, uint8_t value);
uint8_t ov2640_reg_read(ov2640_config_t *config, uint8_t reg);
void ov2640_regs_write(ov2640_config_t *config, const uint8_t (*regs_list)[2]);
uint16_t ov2640_read_id(ov2640_config_t *config);

void ov2640_set_framesize(ov2640_config_t *config, framesize_t framesize);
void ov2640_set_color_format(ov2640_config_t *config, color_format_t color_format_t);

void ov2640_enable_test_pattern(ov2640_config_t *config);

#endif
