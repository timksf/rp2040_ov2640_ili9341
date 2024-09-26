#include "util.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "sd_card.h"
#include "ff.h"

void find_next_jpeg(uint8_t *buf, uint size, uint8_t **s, uint8_t **e) {
    //look for SOI and EOI markers and return start and end byte address
    uint8_t *p = buf;
    *s = NULL; *e = NULL;
    while(p < &buf[size-1]) {
        uint16_t m = *p<<8 | *(p+1);
        if(m == SOI && *s == NULL) {
            *s = p;
        }
        if(m == EOI && *e == NULL && *s != NULL) {
            *e = p+1;
            break;
        }
        p++;
    }
}

void save_jpeg_to_sd(const char *filename, uint8_t *buf, uint size) {

    uint8_t *s, *e;
    find_next_jpeg(buf, size, &s, &e);
    uint sz = e - s + 1;

    if(s != NULL && e != NULL && sz > 0) {
        printf("Valid JPEG file at %p, %p, with size %u\n", s, e, sz);
        save_bin_to_sd(filename, s, sz);
        //also save the entire buffer as a test
        // save_bin("buffer.bin", frame_buf, sizeof(frame_buf));
        // printf("Saved frame buffer to file\n");
    } else
        printf("No valid JPEG file in buffer\n");
}

void save_bin_to_sd(const char *filename, uint8_t *d, uint size) {
    FRESULT fr;
    FIL f;
    FIL *fp = &f;
    
    //try to open file
    fr = f_open(fp, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        while (true);
    }

    uint bytes_written = 0;
    fr = f_write(fp, d, size, &bytes_written);
    fr = f_sync(fp);

    if (fr != FR_OK) {
        printf("ERROR: Could not write to file (%d)\r\n", fr);
        while (true);
    } else {
        printf("Wrote %u bytes\n", bytes_written);
    }

    fr = f_close(fp);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true);
    }
}

void load_bin_from_sd(const char *filename, uint8_t *buf, uint max_size) {
    FRESULT fr;
    FIL f;
    FIL *fp = &f;
    
    //try to open file
    fr = f_open(fp, filename, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        while (true);
    }
    
    uint bytes_read = 0;
    fr = f_read(fp, buf, max_size, &bytes_read);

    if (fr != FR_OK) {
        printf("ERROR: Could not write to file (%d)\r\n", fr);
        while (true);
    } else {
        printf("Read %u bytes\n", bytes_read);
    }

    fr = f_close(fp);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true);
    }
}