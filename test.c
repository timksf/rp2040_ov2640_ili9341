    /*
        We use several DMA channels with a specific chained configuration:

        CTRL->CAM_DATA->UART_SETUP->UART_DATA->UART_RESET->...

        CTRL: setup cam data channel by updating the write address to the frame buffer location
        CAM_DATA: transfer data corresponding to one frame from PIO to memory
        UART_SETUP: write transfer count from control block to UART_DATA DMA channel
        UART_DATA: transfer nothing or a whole frame over uart, depending on the setup
        UART_RESET: set transfer size in control block to 0, so that a single command 
            corresponds to a single transferred frame
    */

    // uint uart_ctrl_dma_channel = dma_claim_unused_channel(true);
    // uint uart_dma_channel = dma_claim_unused_channel(true);
    // uint uart_ctrl_rst_dma_channel = dma_claim_unused_channel(true);

    // //modifying this control block will allow using the uart dma data channel
    // //start with a length of 0, since no transfer is pending yet
    // uart_dma_ctrl_block_t uart_dma_ctrl_block = { 0, frame_buf };

    // //ctrl channel for uart channel setup
    // //this is used to trigger the uart channel from external events, such as received commands 
    // dma_channel_config c = dma_channel_get_default_config(uart_ctrl_dma_channel);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, true);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    // channel_config_set_ring(uart_ctrl_dma_channel, false, 1 << 2); //write wrap around after 1 32bit word
    // channel_config_set_ring(uart_ctrl_dma_channel, true, 1 << 3); //read wrap around after 2 32bit words 
    // dma_channel_configure(
    //     uart_ctrl_dma_channel, &c,
    //     //this will write the trans_count and then trigger the data channel by writing the read address
    //     &dma_hw->ch[uart_dma_channel].al3_transfer_count, 
    //     &uart_dma_ctrl_block,
    //     2, //2 words are transferred, transfer count and read pointer
    //     false
    // );

    // //uart transfer DMA channel
    // c = dma_channel_get_default_config(uart_dma_channel);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, false);
    // channel_config_set_dreq(&c, uart_get_dreq(UART_INST, true));
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // dma_channel_configure(
    //     uart_dma_channel, &c,
    //     &uart_get_hw(UART_INST)->dr,
    //     uart_dma_ctrl_block.buf,
    //     uart_dma_ctrl_block.len,
    //     false
    // );

    // //uart transfer reset channel
    // //this channel simply copies the IDLE config to the currently active config
    // c = dma_channel_get_default_config(uart_ctrl_rst_dma_channel);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, true);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    // //both read and write wrap around after two words, so 8 bytes
    // channel_config_set_ring(uart_ctrl_dma_channel, false, 1 << 3);
    // channel_config_set_ring(uart_ctrl_dma_channel, true, 1 << 3);
    // dma_channel_configure(
    //     uart_ctrl_rst_dma_channel, &c,
    //     &uart_dma_ctrl_block,           //write to active config
    //     &uart_dma_idle_ctrl,            //read from idle config
    //     2,
    //     false
    // );