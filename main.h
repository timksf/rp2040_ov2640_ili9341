#ifndef __MAIN_H__
#define __MAIN_H__

#define UART_INST uart1
#define CAM_I2C_INST i2c1
#define LCD_SPI_INST spi0
#define CAM_PIO_INST pio0
#define CAM_FRAME_SM 0
#define CAM_BYTE_SM 1

#define FRAME_WIDTH 352
#define FRAME_HEIGHT 288

const int PIN_LED = 25;
//LCD pin definitions
const int PIN_LCD_DC = 0;
const int PIN_LCD_RST = 1;
const int PIN_LCD_CS = 5; //SPI0
const int PIN_LCD_TX = 3;
const int PIN_LCD_SCK = 2;
//camera pin definitions
const int PIN_CAM_SIOC = 27; //I2C1 SCL
const int PIN_CAM_SIOD = 26; //I2C1 SDA
const int PIN_CAM_RESETB = 22;
// const int PIN_CAM_XCLK = 3;
const int PIN_CAM_VSYNC = 16;
const int PIN_CAM_Y2_PIO_BASE = 6;

const uint8_t CMD_REG_WRITE = 0xAA;
const uint8_t CMD_REG_READ = 0xBB;
const uint8_t CMD_CAPTURE = 0xCC;

#endif //__MAIN_H__