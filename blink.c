#include <stdio.h>
#include "pico/stdlib.h"

#define PIN_LED 2

//forward declaration of callbacks  
bool timer_callback(__unused struct repeating_timer *t);

int main() {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    struct repeating_timer timer;
    add_repeating_timer_ms(-200, timer_callback, NULL, &timer);
    while(1){
        tight_loop_contents();
    }
}

bool timer_callback(__unused struct repeating_timer *t) {
    gpio_put(PIN_LED, !gpio_get(PIN_LED));
    return true;
}