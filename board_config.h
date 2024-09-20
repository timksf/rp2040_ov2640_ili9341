#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#define UART_INST uart1
#define CAM_PIO_INST pio0
#define CAM_FRAME_SM 0
#define CAM_FRAME_SIZED_SM 1

#define BUF_SIZE 200 * 1024 //200KB image/frame buffer

#define CUSTOM_BOARD

#ifndef CUSTOM_BOARD
#define CAM_I2C_INST i2c1
#define LCD_SPI_INST spi0
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
const int PIN_CAM_XCLK = -1;
const int PIN_CAM_VSYNC = 16;
const int PIN_CAM_Y2_PIO_BASE = 6;
#else 
#define CAM_I2C_INST i2c0
#define LCD_SPI_INST spi0
const int PIN_LED = 2;
//LCD pin definitions
const int PIN_LCD_DC = 19;
const int PIN_LCD_RST = 18;
const int PIN_LCD_CS = 21; //SPI0
const int PIN_LCD_TX = 23;
const int PIN_LCD_SCK = 22;
//camera pin definitions
const int PIN_CAM_SIOC = 5; //I2C1 SCL
const int PIN_CAM_SIOD = 4; //I2C1 SDA
const int PIN_CAM_RESETB = 3;
const int PIN_CAM_XCLK = 25;
const int PIN_CAM_VSYNC = 16;
const int PIN_CAM_Y2_PIO_BASE = 6;
//user GPIO
const uint PIN_USR_BTN0 = 0;
const uint PIN_USR_BTN1 = 1;
const uint PIN_ENC_BTN = 27;
const uint PIN_ENC_A = 28;
const uint PIN_ENC_B = 29;
#endif //CUSTOM_BOARD

const uint8_t CMD_REG_WRITE = 0xAA;
const uint8_t CMD_REG_READ = 0xBB;
const uint8_t CMD_CAPTURE = 0xCC;
const uint8_t CMD_CAPTURE_JPEG = 0xDD;

#endif //__BOARD_CONFIG_H__