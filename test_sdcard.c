#include <stdio.h>
#include <tusb.h>
#include <unistd.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "sd_card.h"
#include "ff.h"

#include "board_config.h"

//forward declaration of callbacks  
bool timer_callback(__unused struct repeating_timer *t);

int main() {

    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    char buf[100];
    char filename[] = "test02.txt";

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

    // after setup wait for serial terminal connection on USB port
    while (!tud_cdc_connected()) { 
        sleep_ms(100);
    }
    printf("Booted successfully!\n");

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer);

    printf("\r\nSD card test. Press 'enter' to start.\r\n");
    while (true) {
        buf[0] = getchar();
        if ((buf[0] == '\r') || (buf[0] == '\n')) {
            break;
        }
    }

    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        while (true);
    }

        // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        while (true);
    }

    // Open file for writing ()
    fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        while (true);
    }

    // Write something to file
    ret = f_printf(&fil, "This is another test\r\n");
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        while (true);
    }
    ret = f_printf(&fil, "of writing to an SD card.\r\n");
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        while (true);
    }

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true);
    }

    // Open file for reading
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        while (true);
    }

    // Print every line in file over serial
    printf("Reading from file '%s':\r\n", filename);
    printf("---\r\n");
    while (f_gets(buf, sizeof(buf), &fil)) {
        printf(buf);
    }
    printf("\r\n---\r\n");

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true);
    }

    // Unmount drive
    f_unmount("0:");

    // Loop forever doing nothing
    while (true) {
        sleep_ms(1000);
    }

}

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}