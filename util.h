#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>
#include "pico/stdlib.h"

static const uint16_t SOI = 0xFFD8;
static const uint16_t EOI = 0xFFD9;

/*
    Finds location of first pair of successive SOI and EOI markers and stores 
    the start and end positions in *start and *end
*/
void find_next_jpeg(uint8_t *buf, uint size, uint8_t **start, uint8_t **end);
/*
    Saves size bytes from buf to a new file `filename`
    @param filename The name of the new file
    @param buf The address of the data buffer
    @param size The size of the bytes to write
*/
void save_bin_to_sd(const char *filename, uint8_t *d, uint size);

/*
    Finds first JPEG file in byte buffer and saves it to a new file
    @param filename The filename of the JPEG output file
    @param buf The address of the data buffer
    @param size The number of bytes from buf that are searched
*/
void save_jpeg_to_sd(const char *filename, uint8_t *buf, uint size);

void load_bin_from_sd(const char *filename, uint8_t *buf, uint max_size);

#endif // __UTIL_H__