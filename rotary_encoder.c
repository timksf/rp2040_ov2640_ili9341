#include "rotary_encoder.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

void rotary_encoder_init(rotary_encoder_t *encoder) {
    gpio_init(encoder->pin_a);
    gpio_init(encoder->pin_b);
    gpio_init(encoder->pin_btn);

    gpio_set_dir(encoder->pin_a, false);
    gpio_set_dir(encoder->pin_b, false);
    gpio_set_dir(encoder->pin_btn, false);

    gpio_disable_pulls(encoder->pin_a);
    gpio_disable_pulls(encoder->pin_b);
    gpio_disable_pulls(encoder->pin_btn);

    //interrupt handlers have to be added outside of this function
    gpio_set_irq_enabled(encoder->pin_a, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(encoder->pin_b, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(encoder->pin_btn, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    encoder->state = 0;
    encoder->pos = 0;
    encoder->new_pos = false;
    encoder->pressed = false;
}

void rotary_encoder_update_pos(rotary_encoder_t *encoder, bool a, bool b) {
    uint8_t input = (a << 1) | b;

    encoder->state = RE_TABLE[(encoder->state << 2) | input];

    if(encoder->state == RE_FINALSTATE_CW) {
        encoder->pos++;
        encoder->new_pos = true;
    } else if(encoder->state == RE_FINALSTATE_CCW) {
        encoder->pos--;
        encoder->new_pos = true;
    }
    
}

void rotary_encoder_update_btn(rotary_encoder_t *encoder, bool p) {
    encoder->pressed = p;
}