#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

#include "ili9341.h"

//based on adafruit driver

static const uint8_t initcmd[] = {
    0xEF, 3, 0x03, 0x80, 0x02,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00, 0x00,
    ILI9341_PWCTR1, 1, 0x23,	   // Power control VRH[5:0]
    ILI9341_PWCTR2, 1, 0x10,	   // Power control SAP[2:0];BT[3:0]
    ILI9341_VMCTR1, 2, 0x3e, 0x28, // VCM control
    ILI9341_VMCTR2, 1, 0x86,	   // VCM control2
    ILI9341_MADCTL, 1, 0x48,	   // Memory Access Control
    ILI9341_VSCRSADD, 1, 0x00,	   // Vertical scroll zero
    ILI9341_PIXFMT, 1, 0x55,
    ILI9341_FRMCTR1, 2, 0x00, 0x13,
    ILI9341_DFUNCTR, 3, 0x08, 0x82, 0x27,					 // Display Function Control
    0xF2, 1, 0x00,											 // 3Gamma Function Disable
    ILI9341_GAMMASET, 1, 0x01,								 // Gamma curve selected
    ILI9341_GMCTRP1, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    ILI9341_GMCTRN1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    ILI9341_SLPOUT, 0x80, // Exit Sleep
    ILI9341_DISPON, 0x80, // Display on
    0x00 // endmarker
};

void ili9341_init(ili9341_config_t *cfg) {
    if(!cfg->baudrate)
        cfg->baudrate = 1000 * 40000;
    spi_init(cfg->spi, cfg->baudrate);
    spi_set_format(cfg->spi, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(cfg->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_tx, GPIO_FUNC_SPI);

    gpio_init(cfg->pin_cs);
    gpio_set_dir(cfg->pin_cs, GPIO_OUT);
    gpio_put(cfg->pin_cs, 1);

    gpio_init(cfg->pin_dc);
    gpio_set_dir(cfg->pin_dc, GPIO_OUT);
    gpio_put(cfg->pin_dc, 1);

    if (cfg->pin_rst != -1) {
        gpio_init(cfg->pin_rst);
        gpio_set_dir(cfg->pin_rst, GPIO_OUT);
        gpio_put(cfg->pin_rst, 1);
    }

    //init display over SPI
    ili9341_select(cfg);
    if (cfg->pin_rst < 0) {
        ili9341_write_command(cfg, ILI9341_SWRESET);
        sleep_ms(150);
    } else
        ili9341_reset(cfg);

    const uint8_t *addr = initcmd;
    uint8_t cmd, n_args;
    while ((cmd = *(addr++))) {
        // printf("addr: %x", addr);
        uint8_t x = *(addr++);
        n_args = x & 0x7F; //mask out delay bit
        // printf(" cmd: %x n_args: %d\n", cmd, n_args);
        ili9341_send_command(cfg, cmd, addr, n_args);
        addr += n_args;
        if (x & 0x80)
            sleep_ms(150);
    }

    cfg->width = ILI9341_TFTWIDTH;
    cfg->height = ILI9341_TFTHEIGHT;
}

void ili9341_reset(ili9341_config_t *cfg) {
    if (cfg->pin_rst != -1) {
        gpio_put(cfg->pin_rst, 0);
        sleep_ms(5);
        gpio_put(cfg->pin_rst, 1);
        sleep_ms(150);
    }
}

void ili9341_select(ili9341_config_t *cfg) {
    asm volatile("nop\n");
    gpio_put(cfg->pin_cs, 0);
    asm volatile("nop\n");
}

void ili9341_deselect(ili9341_config_t *cfg) {
    asm volatile("nop\n");
    gpio_put(cfg->pin_cs, 1);
    asm volatile("nop\n");
}

void ili9341_announce_command(ili9341_config_t *cfg) {
    gpio_put(cfg->pin_dc, 0);
}

void ili9341_announce_data(ili9341_config_t *cfg) {
    gpio_put(cfg->pin_dc, 1);
}

void ili9341_write_command(ili9341_config_t *cfg, uint8_t cmd) {
    ili9341_announce_command(cfg);
    spi_set_format(cfg->spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_write_blocking(cfg->spi, &cmd, sizeof(cmd));
}

void ili9341_write_data(ili9341_config_t *cfg, const uint8_t *d, size_t size) {
    ili9341_announce_data(cfg);
    spi_set_format(cfg->spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_write_blocking(cfg->spi, d, size);
}

void ili9341_send_command(ili9341_config_t *cfg, uint8_t cmd, const uint8_t *args, uint8_t arg_size) {
    ili9341_select(cfg);
    ili9341_write_command(cfg, cmd);
    ili9341_write_data(cfg, args, arg_size);
    ili9341_deselect(cfg);
}

void ili9341_set_rotation(ili9341_config_t *cfg, uint8_t m) {
    uint8_t rotation = m % 4; // can't be higher than 3
    switch (rotation) {
    case 0:
        m = (MADCTL_MX | MADCTL_BGR);
        cfg->width = ILI9341_TFTWIDTH;
        cfg->height = ILI9341_TFTHEIGHT;
        break;
    case 1:
        m = (MADCTL_MV | MADCTL_BGR);
        cfg->width = ILI9341_TFTHEIGHT;
        cfg->height = ILI9341_TFTWIDTH;
        break;
    case 2:
        m = (MADCTL_MY | MADCTL_BGR);
        cfg->width = ILI9341_TFTWIDTH;
        cfg->height = ILI9341_TFTHEIGHT;
        break;
    case 3:
        m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
        cfg->width = ILI9341_TFTHEIGHT;
        cfg->height = ILI9341_TFTWIDTH;
        break;
    }
    ili9341_send_command(cfg, ILI9341_MADCTL, &m, 1);
}

void ili9341_set_addr_window(ili9341_config_t *cfg, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint32_t xa = ((uint32_t)x << 16) | (x + w - 1);
    uint32_t ya = ((uint32_t)y << 16) | (y + h - 1);

    xa = __builtin_bswap32(xa);
    ya = __builtin_bswap32(ya);

    ili9341_write_command(cfg, ILI9341_CASET);
    ili9341_write_data(cfg, (uint8_t*)&xa, sizeof(xa));

    // printf("Setting address window: x: %u, y: %u, w: %u, h: %u\n", x, y, w, h);
    // printf("xa: 0x%08X, ya: 0x%08X\n", xa, ya);

    //row address set
    ili9341_write_command(cfg, ILI9341_PASET);
    ili9341_write_data(cfg, (uint8_t*)&ya, sizeof(ya));

    //write to RAM
    ili9341_write_command(cfg, ILI9341_RAMWR);
}

void ili9341_write_frame(ili9341_config_t *cfg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *frame) {
    volatile uint b = spi_set_baudrate(cfg->spi, cfg->baudrate);
    ili9341_select(cfg);
    ili9341_set_addr_window(cfg, x, y, w, h); // Clipped area
    ili9341_announce_data(cfg);
    spi_set_format(cfg->spi, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    dma_channel_config c = dma_channel_get_default_config(cfg->dma_data_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_dreq(&c, spi_get_dreq(cfg->spi, true));
    dma_channel_configure(
        cfg->dma_data_chan,
        &c,
        &spi_get_hw(cfg->spi)->dr,
        frame,
        w * h,
        true
    );
    
    dma_channel_wait_for_finish_blocking(cfg->dma_data_chan);
    while(spi_is_busy(cfg->spi)) tight_loop_contents();
    ili9341_deselect(cfg);
}

void ili9341_write_frame_mono(ili9341_config_t *cfg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t col) {
    spi_set_baudrate(cfg->spi, cfg->baudrate);
    ili9341_select(cfg);
    ili9341_set_addr_window(cfg, x, y, w, h); // Clipped area
    ili9341_announce_data(cfg);
    spi_set_format(cfg->spi, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    dma_channel_config c = dma_channel_get_default_config(cfg->dma_data_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_dreq(&c, spi_get_dreq(cfg->spi, true));
    dma_channel_configure(
        cfg->dma_data_chan, &c,
        &spi_get_hw(cfg->spi)->dr,
        &col,
        w * h,
        true
    );
    
    dma_channel_wait_for_finish_blocking(cfg->dma_data_chan);
    while(spi_is_busy(cfg->spi)) tight_loop_contents();
    ili9341_deselect(cfg);
}

void ili9341_write_pix(ili9341_config_t *cfg, int x, int y, uint16_t col) {
    ili9341_select(cfg);
    ili9341_set_addr_window(cfg, x, y, 1, 1); // Clipped area
    ili9341_announce_data(cfg);
    spi_set_format(cfg->spi, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_write16_blocking(cfg->spi, &col, 1);
    ili9341_deselect(cfg);
}


void ili9341_stream_start(ili9341_config_t *cfg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *frame) {
    ili9341_select(cfg);
    ili9341_set_addr_window(cfg, x, y, w, h); // Clipped area
    ili9341_announce_data(cfg);
    spi_set_format(cfg->spi, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    //first, construct the control DMA, which reset the READ address 
    dma_channel_config c = dma_channel_get_default_config(cfg->dma_ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(
        cfg->dma_ctrl_chan, &c, 
        &dma_hw->ch[cfg->dma_data_chan].al3_read_addr_trig, //reset the read address after each completed DMA transfer
        &cfg->frame_buf, //the read_addr of the data channel is reset to the beginning of the frame buffer
        1, //write a single address, so a single word
        false
    );

    dma_channel_config c1 = dma_channel_get_default_config(cfg->dma_data_chan);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_16);
    channel_config_set_dreq(&c1, spi_get_dreq(cfg->spi, true));
    dma_channel_configure(
        cfg->dma_data_chan, &c1,
        &spi_get_hw(cfg->spi)->dr,
        frame,
        w * h,
        false
    );

    //the DMA ctrl channel will be started externally
}

void ili9341_stream_stop(ili9341_config_t *cfg) {
    ili9341_deselect(cfg);
}