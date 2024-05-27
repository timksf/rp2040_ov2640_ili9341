#include <stdio.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "ov2640.h"
#include "ili9341.h"

#define UART_INST uart1
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
const int PIN_CAM_XCLK = 3;
const int PIN_CAM_VSYNC = 16;
const int PIN_CAM_Y2_PIO_BASE = 6;

int main() {
    stdio_init_all(); //will only init what is enabled in the CMakeLists file

    LCD_setPins(PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST, PIN_LCD_SCK, PIN_LCD_TX);

    //after setup wait for serial terminal connection on USB port
    while (!tud_cdc_connected()) { 
        sleep_ms(100);  
    }
    printf("Booted successfully!");

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    while(1){
        gpio_put(PIN_LED, !gpio_get(PIN_LED));
        sleep_ms(1000);
    }
}