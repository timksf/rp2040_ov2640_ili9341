#ifndef OV2640_INIT_H
#define OV2640_INIT_H
#include <stdint.h>

#include "ov2640_regs.h"

static const uint8_t ov2640_vga[][2] = {
    {0xff, 0x00},  /* Device control register list Table 12 */
    {0x2c, 0xff},  /* Reserved                              */
    {0x2e, 0xdf},  /* Reserved                              */
    {0xff, 0x01},  /* Device control register list Table 13 */
    {0x3c, 0x32},  /* Reserved                              */
    {0x11, 0x80},  /* Clock Rate Control                    */
    {0x09, 0x02},  /* Common control 2                      */
    {0x04, 0xA8},  /* Mirror                                */
    {0x13, 0xe5},  /* Common control 8                      */
    {0x14, 0x48},  /* Common control 9                      */
    {0x2c, 0x0c},  /* Reserved                              */
    {0x33, 0x78},  /* Reserved                              */
    {0x3a, 0x33},  /* Reserved                              */
    {0x3b, 0xfB},  /* Reserved                              */
    {0x3e, 0x00},  /* Reserved                              */
    {0x43, 0x11},  /* Reserved                              */
    {0x16, 0x10},  /* Reserved                              */
    {0x4a, 0x81},  /* Reserved                              */
    {0x21, 0x99},  /* Reserved                              */
    {0x24, 0x40},  /* Luminance signal High range           */
    {0x25, 0x38},  /* Luminance signal low range            */
    {0x26, 0x82},  /*                                       */
    {0x5c, 0x00},  /* Reserved                              */
    {0x63, 0x00},  /* Reserved                              */
    {0x46, 0x3f},  /* Frame length adjustment               */
    {0x0c, 0x3c},  /* Common control 3                      */
    {0x61, 0x70},  /* Histogram algo low level              */
    {0x62, 0x80},  /* Histogram algo high level             */
    {0x7c, 0x05},  /* Reserved                              */
    {0x20, 0x80},  /* Reserved                              */
    {0x28, 0x30},  /* Reserved                              */
    {0x6c, 0x00},  /* Reserved                              */
    {0x6d, 0x80},  /* Reserved                              */
    {0x6e, 0x00},  /* Reserved                              */
    {0x70, 0x02},  /* Reserved                              */
    {0x71, 0x94},  /* Reserved                              */
    {0x73, 0xc1},  /* Reserved                              */
    {0x3d, 0x34},  /* Reserved                              */
    {0x5a, 0x57},  /* Reserved                              */
    {0x12, 0x00},  /* Common control 7                      */
    {0x11, 0x00},  /* Clock Rate Control                   2*/
    {0x17, 0x11},  /* Horiz window start MSB 8bits          */
    {0x18, 0x75},  /* Horiz window end MSB 8bits            */
    {0x19, 0x01},  /* Vert window line start MSB 8bits      */
    {0x1a, 0x97},  /* Vert window line end MSB 8bits        */
    {0x32, 0x36},
    {0x03, 0x0f},
    {0x37, 0x40},
    {0x4f, 0xbb},
    {0x50, 0x9c},
    {0x5a, 0x57},
    {0x6d, 0x80},
    {0x6d, 0x38},
    {0x39, 0x02},
    {0x35, 0x88},
    {0x22, 0x0a},
    {0x37, 0x40},
    {0x23, 0x00},
    {0x34, 0xa0},
    {0x36, 0x1a},
    {0x06, 0x02},
    {0x07, 0xc0},
    {0x0d, 0xb7},
    {0x0e, 0x01},
    {0x4c, 0x00},
    {0xff, 0x00},
    {0xe5, 0x7f},
    {0xf9, 0xc0},
    {0x41, 0x24},
    {0xe0, 0x14},
    {0x76, 0xff},
    {0x33, 0xa0},
    {0x42, 0x20},
    {0x43, 0x18},
    {0x4c, 0x00},
    {0x87, 0xd0},
    {0x88, 0x3f},
    {0xd7, 0x03},
    {0xd9, 0x10},
    {0xd3, 0x82},
    {0xc8, 0x08},
    {0xc9, 0x80},
    {0x7d, 0x00},
    {0x7c, 0x03},
    {0x7d, 0x48},
    {0x7c, 0x08},
    {0x7d, 0x20},
    {0x7d, 0x10},
    {0x7d, 0x0e},
    {0x90, 0x00},
    {0x91, 0x0e},
    {0x91, 0x1a},
    {0x91, 0x31},
    {0x91, 0x5a},
    {0x91, 0x69},
    {0x91, 0x75},
    {0x91, 0x7e},
    {0x91, 0x88},
    {0x91, 0x8f},
    {0x91, 0x96},
    {0x91, 0xa3},
    {0x91, 0xaf},
    {0x91, 0xc4},
    {0x91, 0xd7},
    {0x91, 0xe8},
    {0x91, 0x20},
    {0x92, 0x00},
    {0x93, 0x06},
    {0x93, 0xe3},
    {0x93, 0x02},
    {0x93, 0x02},
    {0x93, 0x00},
    {0x93, 0x04},
    {0x93, 0x00},
    {0x93, 0x03},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x96, 0x00},
    {0x97, 0x08},
    {0x97, 0x19},
    {0x97, 0x02},
    {0x97, 0x0c},
    {0x97, 0x24},
    {0x97, 0x30},
    {0x97, 0x28},
    {0x97, 0x26},
    {0x97, 0x02},
    {0x97, 0x98},
    {0x97, 0x80},
    {0x97, 0x00},
    {0x97, 0x00},
    {0xc3, 0xef},
    {0xff, 0x00},
    {0xba, 0xdc},
    {0xbb, 0x08},
    {0xb6, 0x24},
    {0xb8, 0x33},
    {0xb7, 0x20},
    {0xb9, 0x30},
    {0xb3, 0xb4},
    {0xb4, 0xca},
    {0xb5, 0x43},
    {0xb0, 0x5c},
    {0xb1, 0x4f},
    {0xb2, 0x06},
    {0xc7, 0x00},
    {0xc6, 0x51},
    {0xc5, 0x11},
    {0xc4, 0x9c},
    {0xbf, 0x00},
    {0xbc, 0x64},
    {0xa6, 0x00},
    {0xa7, 0x1e},
    {0xa7, 0x6b},
    {0xa7, 0x47},
    {0xa7, 0x33},
    {0xa7, 0x00},
    {0xa7, 0x23},
    {0xa7, 0x2e},
    {0xa7, 0x85},
    {0xa7, 0x42},
    {0xa7, 0x33},
    {0xa7, 0x00},
    {0xa7, 0x23},
    {0xa7, 0x1b},
    {0xa7, 0x74},
    {0xa7, 0x42},
    {0xa7, 0x33},
    {0xa7, 0x00},
    {0xa7, 0x23},
    {0xc0, 0xc8},
    {0xc1, 0x96},
    {0x8c, 0x00},
    {0x86, 0x3d},
    {0x50, 0x92},
    {0x51, 0x90},
    {0x52, 0x2c},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x88},
    {0x5a, 0x50},
    {0x5b, 0x3c},
    {0x5c, 0x00},
    {0xd3, 0x04},
    {0x7f, 0x00},
    {0xda, 0x00},
    {0xe5, 0x1f},
    {0xe1, 0x67},
    {0xe0, 0x00},
    {0xdd, 0x7f},
    {0x05, 0x00},
    {0xff, 0x00},
    {0xe0, 0x04},
    {0xc0, 0xc8},
    {0xc1, 0x96},
    {0x86, 0x3d},
    {0x50, 0x92},
    {0x51, 0x90},
    {0x52, 0x2c},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x88},
    {0x57, 0x00},
    {0x5a, 0x50},
    {0x5b, 0x3c},
    {0x5c, 0x00},
    {0xd3, 0x04},
    {0xe0, 0x00},
    {0xFF, 0x00},
    {0x05, 0x00},
    {0xDA, 0x08},
    {0xda, 0x09},
    {0x98, 0x00},
    {0x99, 0x00},
    {0x00, 0x00},
    {0xff, 0x00},
    {0xe0, 0x04},
    {0xc0, 0xc8},
    {0xc1, 0x96},
    {0x86, 0x3d},
    {0x50, 0x89},
    {0x51, 0x90},
    {0x52, 0x2c},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x88},
    {0x57, 0x00},
    {0x5a, 0xA0},
    {0x5b, 0x78},
    {0x5c, 0x00},
    {0xd3, 0x02},
    {0xe0, 0x00},
    //stop
    {0xff, 0xff}
};

static const uint8_t ov2640_uxga_cif[][2] = {   
    {0xff, 0x00},   
    {0xe0, 0x04},   
    {0xc0, 0xc8},   
    {0xc1, 0x96},   
    {0x86, 0x35},   
    {0x50, 0x92},   
    {0x51, 0x90},   
    {0x52, 0x2c},   
    {0x53, 0x00},   
    {0x54, 0x00},   
    {0x55, 0x88},   
    {0x57, 0x00},   
    {0x5a, 0x58},   
    {0x5b, 0x48},   
    {0x5c, 0x00},   
    {0xd3, 0x10},   
    {0xe0, 0x00},
    //stop
    {0xff, 0xff}
};   

/* Initialization sequence for QVGA resolution (320x240) */
static const uint8_t ov2640_qvga[][2] = {
    {0xff, 0x00},
    {0x2c, 0xff},
    {0x2e, 0xdf},
    {0xff, 0x01},
    {0x3c, 0x32},
    {0x11, 0x00},
    {0x09, 0x02},
    {0x04, 0xA8},
    {0x13, 0xe5},
    {0x14, 0x48},
    {0x2c, 0x0c},
    {0x33, 0x78},
    {0x3a, 0x33},
    {0x3b, 0xfB},
    {0x3e, 0x00},
    {0x43, 0x11},
    {0x16, 0x10},
    {0x4a, 0x81},
    {0x21, 0x99},
    {0x24, 0x40},
    {0x25, 0x38},
    {0x26, 0x82},
    {0x5c, 0x00},
    {0x63, 0x00},
    {0x46, 0x3f},
    {0x0c, 0x3c},
    {0x61, 0x70},
    {0x62, 0x80},
    {0x7c, 0x05},
    {0x20, 0x80},
    {0x28, 0x30},
    {0x6c, 0x00},
    {0x6d, 0x80},
    {0x6e, 0x00},
    {0x70, 0x02},
    {0x71, 0x94},
    {0x73, 0xc1},
    {0x3d, 0x34},
    {0x5a, 0x57},
    {0x12, 0x00},
    {0x11, 0x00},
    {0x17, 0x11},
    {0x18, 0x75},
    {0x19, 0x01},
    {0x1a, 0x97},
    {0x32, 0x36},
    {0x03, 0x0f},
    {0x37, 0x40},
    {0x4f, 0xbb},
    {0x50, 0x9c},
    {0x5a, 0x57},
    {0x6d, 0x80},
    {0x6d, 0x38},
    {0x39, 0x02},
    {0x35, 0x88},
    {0x22, 0x0a},
    {0x37, 0x40},
    {0x23, 0x00},
    {0x34, 0xa0},
    {0x36, 0x1a},
    {0x06, 0x02},
    {0x07, 0xc0},
    {0x0d, 0xb7},
    {0x0e, 0x01},
    {0x4c, 0x00},
    {0xff, 0x00},
    {0xe5, 0x7f},
    {0xf9, 0xc0},
    {0x41, 0x24},
    {0xe0, 0x14},
    {0x76, 0xff},
    {0x33, 0xa0},
    {0x42, 0x20},
    {0x43, 0x18},
    {0x4c, 0x00},
    {0x87, 0xd0},
    {0x88, 0x3f},
    {0xd7, 0x03},
    {0xd9, 0x10},
    {0xd3, 0x82},
    {0xc8, 0x08},
    {0xc9, 0x80},
    {0x7d, 0x00},
    {0x7c, 0x03},
    {0x7d, 0x48},
    {0x7c, 0x08},
    {0x7d, 0x20},
    {0x7d, 0x10},
    {0x7d, 0x0e},
    {0x90, 0x00},
    {0x91, 0x0e},
    {0x91, 0x1a},
    {0x91, 0x31},
    {0x91, 0x5a},
    {0x91, 0x69},
    {0x91, 0x75},
    {0x91, 0x7e},
    {0x91, 0x88},
    {0x91, 0x8f},
    {0x91, 0x96},
    {0x91, 0xa3},
    {0x91, 0xaf},
    {0x91, 0xc4},
    {0x91, 0xd7},
    {0x91, 0xe8},
    {0x91, 0x20},
    {0x92, 0x00},
    {0x93, 0x06},
    {0x93, 0xe3},
    {0x93, 0x02},
    {0x93, 0x02},
    {0x93, 0x00},
    {0x93, 0x04},
    {0x93, 0x00},
    {0x93, 0x03},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x96, 0x00},
    {0x97, 0x08},
    {0x97, 0x19},
    {0x97, 0x02},
    {0x97, 0x0c},
    {0x97, 0x24},
    {0x97, 0x30},
    {0x97, 0x28},
    {0x97, 0x26},
    {0x97, 0x02},
    {0x97, 0x98},
    {0x97, 0x80},
    {0x97, 0x00},
    {0x97, 0x00},
    {0xc3, 0xef},
    {0xff, 0x00},
    {0xba, 0xdc},
    {0xbb, 0x08},
    {0xb6, 0x24},
    {0xb8, 0x33},
    {0xb7, 0x20},
    {0xb9, 0x30},
    {0xb3, 0xb4},
    {0xb4, 0xca},
    {0xb5, 0x43},
    {0xb0, 0x5c},
    {0xb1, 0x4f},
    {0xb2, 0x06},
    {0xc7, 0x00},
    {0xc6, 0x51},
    {0xc5, 0x11},
    {0xc4, 0x9c},
    {0xbf, 0x00},
    {0xbc, 0x64},
    {0xa6, 0x00},
    {0xa7, 0x1e},
    {0xa7, 0x6b},
    {0xa7, 0x47},
    {0xa7, 0x33},
    {0xa7, 0x00},
    {0xa7, 0x23},
    {0xa7, 0x2e},
    {0xa7, 0x85},
    {0xa7, 0x42},
    {0xa7, 0x33},
    {0xa7, 0x00},
    {0xa7, 0x23},
    {0xa7, 0x1b},
    {0xa7, 0x74},
    {0xa7, 0x42},
    {0xa7, 0x33},
    {0xa7, 0x00},
    {0xa7, 0x23},
    {0xc0, 0xc8},
    {0xc1, 0x96},
    {0x8c, 0x00},
    {0x86, 0x3d},
    {0x50, 0x92},
    {0x51, 0x90},
    {0x52, 0x2c},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x88},
    {0x5a, 0x50},
    {0x5b, 0x3c},
    {0x5c, 0x00},
    {0xd3, 0x04},
    {0x7f, 0x00},
    {0xda, 0x00},
    {0xe5, 0x1f},
    {0xe1, 0x67},
    {0xe0, 0x00},
    {0xdd, 0x7f},
    {0x05, 0x00},
    {0xff, 0x00},
    {0xe0, 0x04},
    {0xc0, 0xc8},
    {0xc1, 0x96},
    {0x86, 0x3d},
    {0x50, 0x92},
    {0x51, 0x90},
    {0x52, 0x2c},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x88},
    {0x57, 0x00},
    {0x5a, 0x50},
    {0x5b, 0x3C},
    {0x5c, 0x00},
    {0xd3, 0x08},
    {0xe0, 0x00},
    {0xFF, 0x00},
    {0x05, 0x00},
    {0xDA, 0x08},
    {0xda, 0x09},
    {0x98, 0x00},
    {0x99, 0x00},
    //stop
    {0xff, 0xff}
};

static const uint8_t ov2640_init_regs[][2] = {
    { BANK_SEL, BANK_SEL_DSP },
    { 0x2c,   0xff },
    { 0x2e,   0xdf },
    { BANK_SEL, BANK_SEL_SENSOR },
    { 0x3c,   0x32 },
    { CLKRC,  CLKRC_2X_CIF },
    { COM2,   COM2_OUT_DRIVE_4x },
    { REG04,  REG04_DEFAULT | REG04_HREF_EN },
    { COM8,   COM8_DEFAULT | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN },
    { COM9,   COM9_AGC_GAIN_8x | 0x08},
    { 0x2c,   0x0c },
    { 0x33,   0x78 },
    { 0x3a,   0x33 },
    { 0x3b,   0xfb },
    { 0x3e,   0x00 },
    { 0x43,   0x11 },
    { 0x16,   0x10 },
    { 0x39,   0x02 },
    { 0x35,   0x88 },
    { 0x22,   0x0a },
    { 0x37,   0x40 },
    { 0x23,   0x00 },
    { ARCOM2, 0xa0 },
    { 0x06,   0x02 },
    { 0x06,   0x88 },
    { 0x07,   0xc0 },
    { 0x0d,   0xb7 },
    { 0x0e,   0x01 },
    { 0x4c,   0x00 },
    { 0x4a,   0x81 },
    { 0x21,   0x99 },
    { AEW,    0x40 },
    { AEB,    0x38 },
    { VV,     VV_AGC_TH_SET(0x08, 0x02) },
    { 0x5c,   0x00 },
    { 0x63,   0x00 },
    { FLL,    0x22 },
    { COM3,   0x38 | COM3_BAND_AUTO },
    { REG5D,  0x55 },
    { REG5E,  0x7d },
    { REG5F,  0x7d },
    { REG60,  0x55 },
    { HISTO_LOW,   0x70 },
    { HISTO_HIGH,  0x80 },
    { 0x7c,   0x05 },
    { 0x20,   0x80 },
    { 0x28,   0x30 },
    { 0x6c,   0x00 },
    { 0x6d,   0x80 },
    { 0x6e,   0x00 },
    { 0x70,   0x02 },
    { 0x71,   0x94 },
    { 0x73,   0xc1 },
    { 0x3d,   0x34 },
    { COM7,   COM7_RES_UXGA | COM7_ZOOM_EN },
    { 0x5A,   0x57 },		/* 0x57 */
    { COM25,  0x00 },	/* 0x00 */
    { BD50,   0xbb },	/* 0xbb */
    { BD60,   0x9C },	/* 0x9c */
    { BANK_SEL,  BANK_SEL_DSP },
    { 0xe5,   0x7f },
    { MC_BIST,  MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL },
    { 0x41,   0x24 },
    { RESET,  RESET_JPEG | RESET_DVP },
    { 0x76,   0xff },
    { 0x33,   0xa0 },
    { 0x42,   0x20 },
    { 0x43,   0x18 },
    { 0x4c,   0x00 },
    { CTRL3,  CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 },
    { 0x88,   0x3f },
    { 0xd7,   0x03 },
    { 0xd9,   0x10 },
    { R_DVP_SP,  R_DVP_SP_AUTO_MODE | 0x2 }, //48MHz / 2 = 24MHz
    { 0xc8,   0x08 },
    { 0xc9,   0x80 },
    { BPADDR, 0x00 },
    { BPDATA, 0x00 },
    { BPADDR, 0x03 },
    { BPDATA, 0x48 },
    { BPDATA, 0x48 },
    { BPADDR, 0x08 },
    { BPDATA, 0x20 },
    { BPDATA, 0x10 },
    { BPDATA, 0x0e },
    { 0x90,   0x00 },
    { 0x91,   0x0e },
    { 0x91,   0x1a },
    { 0x91,   0x31 },
    { 0x91,   0x5a },
    { 0x91,   0x69 },
    { 0x91,   0x75 },
    { 0x91,   0x7e },
    { 0x91,   0x88 },
    { 0x91,   0x8f },
    { 0x91,   0x96 },
    { 0x91,   0xa3 },
    { 0x91,   0xaf },
    { 0x91,   0xc4 },
    { 0x91,   0xd7 },
    { 0x91,   0xe8 },
    { 0x91,   0x20 },
    { 0x92,   0x00 },
    { 0x93,   0x06 },
    { 0x93,   0xe3 },
    { 0x93,   0x03 },
    { 0x93,   0x03 },
    { 0x93,   0x00 },
    { 0x93,   0x02 },
    { 0x93,   0x00 },
    { 0x93,   0x00 },
    { 0x93,   0x00 },
    { 0x93,   0x00 },
    { 0x93,   0x00 },
    { 0x93,   0x00 },
    { 0x93,   0x00 },
    { 0x96,   0x00 },
    { 0x97,   0x08 },
    { 0x97,   0x19 },
    { 0x97,   0x02 },
    { 0x97,   0x0c },
    { 0x97,   0x24 },
    { 0x97,   0x30 },
    { 0x97,   0x28 },
    { 0x97,   0x26 },
    { 0x97,   0x02 },
    { 0x97,   0x98 },
    { 0x97,   0x80 },
    { 0x97,   0x00 },
    { 0x97,   0x00 },
    { 0xa4,   0x00 },
    { 0xa8,   0x00 },
    { 0xc5,   0x11 },
    { 0xc6,   0x51 },
    { 0xbf,   0x80 },
    { 0xc7,   0x10 },	/* simple AWB */
    { 0xb6,   0x66 },
    { 0xb8,   0xA5 },
    { 0xb7,   0x64 },
    { 0xb9,   0x7C },
    { 0xb3,   0xaf },
    { 0xb4,   0x97 },
    { 0xb5,   0xFF },
    { 0xb0,   0xC5 },
    { 0xb1,   0x94 },
    { 0xb2,   0x0f },
    { 0xc4,   0x5c },
    { 0xa6,   0x00 },
    { 0xa7,   0x20 },
    { 0xa7,   0xd8 },
    { 0xa7,   0x1b },
    { 0xa7,   0x31 },
    { 0xa7,   0x00 },
    { 0xa7,   0x18 },
    { 0xa7,   0x20 },
    { 0xa7,   0xd8 },
    { 0xa7,   0x19 },
    { 0xa7,   0x31 },
    { 0xa7,   0x00 },
    { 0xa7,   0x18 },
    { 0xa7,   0x20 },
    { 0xa7,   0xd8 },
    { 0xa7,   0x19 },
    { 0xa7,   0x31 },
    { 0xa7,   0x00 },
    { 0xa7,   0x18 },
    { 0x7f,   0x00 },
    { 0xe5,   0x1f },
    { 0xe1,   0x77 },
    { 0xdd,   0x7f },
    { CTRL0,  CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },
    ENDMARKER,
};

// static const struct regval_list ov2640_size_change_preamble_regs[] = {
// 	{ BANK_SEL, BANK_SEL_DSP },
// 	{ RESET, RESET_DVP },
// 	{ SIZEL, SIZEL_HSIZE8_11_SET(UXGA_WIDTH) |
// 		 SIZEL_HSIZE8_SET(UXGA_WIDTH) |
// 		 SIZEL_VSIZE8_SET(UXGA_HEIGHT) },
// 	{ HSIZE8, HSIZE8_SET(UXGA_WIDTH) },
// 	{ VSIZE8, VSIZE8_SET(UXGA_HEIGHT) },
// 	{ CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN |
// 		 CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
// 	{ HSIZE, HSIZE_SET(UXGA_WIDTH) },
// 	{ VSIZE, VSIZE_SET(UXGA_HEIGHT) },
// 	{ XOFFL, XOFFL_SET(0) },
// 	{ YOFFL, YOFFL_SET(0) },
// 	{ VHYX, VHYX_HSIZE_SET(UXGA_WIDTH) | VHYX_VSIZE_SET(UXGA_HEIGHT) |
// 		VHYX_XOFF_SET(0) | VHYX_YOFF_SET(0)},
// 	{ TEST, TEST_HSIZE_SET(UXGA_WIDTH) },
// 	ENDMARKER,
// };

// #define PER_SIZE_REG_SEQ(x, y, v_div, h_div, pclk_div)	\
// 	{ CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(v_div) |	\
// 		 CTRLI_H_DIV_SET(h_div)},		\
// 	{ ZMOW, ZMOW_OUTW_SET(x) },			\
// 	{ ZMOH, ZMOH_OUTH_SET(y) },			\
// 	{ ZMHH, ZMHH_OUTW_SET(x) | ZMHH_OUTH_SET(y) },	\
// 	{ R_DVP_SP, pclk_div },				\
// 	{ RESET, 0x00}

const uint8_t ov2640_settings_cif[][2] = {
    {BANK_SEL, BANK_SEL_DSP},
    {0x2c, 0xff},
    {0x2e, 0xdf},
    {BANK_SEL, BANK_SEL_SENSOR},
    // {ADDVSL, 8},
    {0x46, 0x3f},
    {0x3c, 0x32},
    {COM1, 0x06},
    {CLKRC, CLKRC_DIV(1)}, //12MHz internal clock for DIV=1
    {COM2, COM2_OUT_DRIVE_3x},
    {COM3, 0x38 | COM3_BAND_AUTO},
    {REG04, REG04_DEFAULT},
    {COM8, COM8_DEFAULT | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN},
    {COM9, COM9_AGC_SET(COM9_AGC_GAIN_8x)},
    {0x2c, 0x0c},
    {0x33, 0x78},
    {0x3a, 0x33},
    {0x3b, 0xfB},
    {0x3e, 0x00},
    {0x43, 0x11},
    {0x16, 0x10},
    {0x39, 0x92},
    {0x35, 0xda},
    {0x22, 0x1a},
    {0x37, 0xc3},
    {0x23, 0x00},
    {ARCOM2, 0xc0},
    {0x06, 0x88},
    {0x07, 0xc0},
    {COM4, 0x87},
    {0x0e, 0x41},
    {0x4c, 0x00},
    {0x4a, 0x81},
    {0x21, 0x99},
    {AEW, 0x40},
    {AEB, 0x38},
    {VV, VV_AGC_TH_SET(8,2)},
    {0x5c, 0x00},
    {0x63, 0x00},
    // {FLH, 0x02},
    {HISTO_LOW, 0x70},
    {HISTO_HIGH, 0x80},
    {0x7c, 0x05},
    {0x20, 0x80},
    {0x28, 0x30},
    {0x6c, 0x00},
    {0x6d, 0x80},
    {0x6e, 0x00},
    {0x70, 0x02},
    {0x71, 0x94},
    {0x73, 0xc1},
    {0x3d, 0x34},
    {0x5a, 0x57},
    {BD50, 0xbb},
    {BD60, 0x9c},
    {COM7, COM7_RES_CIF},
    {HSTART, 0x11},
    {HSTOP, 0x43},
    {VSTART, 0x00},
    {VSTOP, 0x25},
    {REG32, REG32_CIF},
    {0x37, 0xc0},
    {BD50, 0xca},
    {BD60, 0xa8},
    {0x6d, 0x00},
    {0x3d, 0x38},
    {BANK_SEL, BANK_SEL_DSP},
    {0xe5, 0x7f},
    {MC_BIST, MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL},
    {0x41, 0x24},
    {RESET, RESET_JPEG | RESET_DVP},
    {0x76, 0xff},
    {0x33, 0xa0},
    {0x42, 0x20},
    {0x43, 0x18},
    {0x4c, 0x00},
    {CTRL3, CTRL3_WPC_EN | 0x10 },
    {0x88, 0x3f},
    {0xd7, 0x03},
    {0xd9, 0x10},
    {R_DVP_SP, R_DVP_SP_AUTO_MODE},
    {0xc8, 0x08},
    {0xc9, 0x80},
    {BPADDR, 0x00},
    {BPDATA, 0x00},
    {BPADDR, 0x03},
    {BPDATA, 0x48},
    {BPDATA, 0x48},
    {BPADDR, 0x08},
    {BPDATA, 0x20},
    {BPDATA, 0x10},
    {BPDATA, 0x0e},
    {0x90, 0x00},
    {0x91, 0x0e},
    {0x91, 0x1a},
    {0x91, 0x31},
    {0x91, 0x5a},
    {0x91, 0x69},
    {0x91, 0x75},
    {0x91, 0x7e},
    {0x91, 0x88},
    {0x91, 0x8f},
    {0x91, 0x96},
    {0x91, 0xa3},
    {0x91, 0xaf},
    {0x91, 0xc4},
    {0x91, 0xd7},
    {0x91, 0xe8},
    {0x91, 0x20},
    {0x92, 0x00},
    {0x93, 0x06},
    {0x93, 0xe3},
    {0x93, 0x05},
    {0x93, 0x05},
    {0x93, 0x00},
    {0x93, 0x04},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x96, 0x00},
    {0x97, 0x08},
    {0x97, 0x19},
    {0x97, 0x02},
    {0x97, 0x0c},
    {0x97, 0x24},
    {0x97, 0x30},
    {0x97, 0x28},
    {0x97, 0x26},
    {0x97, 0x02},
    {0x97, 0x98},
    {0x97, 0x80},
    {0x97, 0x00},
    {0x97, 0x00},
    {0xa4, 0x00},
    {0xa8, 0x00},
    {0xc5, 0x11},
    {0xc6, 0x51},
    {0xbf, 0x80},
    {0xc7, 0x10},
    {0xb6, 0x66},
    {0xb8, 0xA5},
    {0xb7, 0x64},
    {0xb9, 0x7C},
    {0xb3, 0xaf},
    {0xb4, 0x97},
    {0xb5, 0xFF},
    {0xb0, 0xC5},
    {0xb1, 0x94},
    {0xb2, 0x0f},
    {0xc4, 0x5c},
    {CTRL1, 0xfd},
    {0x7f, 0x00},
    {0xe5, 0x1f},
    {0xe1, 0x67},
    {0xdd, 0x7f},
    {IMAGE_MODE, 0x00},
    {RESET, 0x00},
    {R_BYPASS, R_BYPASS_DSP_EN},
    ENDMARKER
};

static const uint8_t ov2640_settings_rgb565[][2] = {
    {BANK_SEL, BANK_SEL_DSP},
    {RESET, RESET_DVP},
    {IMAGE_MODE, IMAGE_MODE_RGB565},
    {0xD7, 0x03},
    {0xE1, 0x77},
    {RESET, 0x00},
    ENDMARKER
};

#define CIF_WIDTH 400
#define CIF_HEIGHT 296

static const uint8_t ov2640_size_cif[][2] = {
    { BANK_SEL, BANK_SEL_DSP },
    { RESET, RESET_DVP },
    { SIZEL, (((352 >> 11) & 0b1) << 7) | ((352 & 0b111) << 3) | (288 & 0b111) },
    { HSIZE8, 352 >> 3 },
    { VSIZE8, 288 >> 3 },
    // { CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN |
    //      CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
    { HSIZE, 352 >> 2 },
    { VSIZE, 288 >> 2 },
    { XOFFL, 0x00 },
    { YOFFL, 0x00 },
    { VHYX, ((288 >> (8+2)) << 7) | ((352 >> (8+2)) << 3) },
    { TEST, (352 >> (9+2)) << 7},
    { ZMOW, 352 >> 2 },
    { ZMOW, 288 >> 2 },
    { ZMHH, (((288 >> 10) & 0b1) << 2) | ((352 >> 10) & 0b11)},
    ENDMARKER,
};

#endif
