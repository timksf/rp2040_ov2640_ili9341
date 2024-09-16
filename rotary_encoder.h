#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

#include <stdint.h>
#include <stdbool.h>

static const uint8_t RE_FINALSTATE_CW = 3;
static const uint8_t RE_FINALSTATE_CCW = 6;

//full quadrature transition table, triggered at any edge of A or B
static const int8_t RE_TABLE[] = {
    //idx 0: q0
    0, 4, 1, 0, //leave start state if A==1, B==0 or A==0, B==1
    //idx 1: q1 CW
    0, 0, 0, 2,
    //idx 2: q2 CW
    0, 3, 0, 0,
    //idx 3: q3 CW, final state for CW
    0, 4, 1, 0,
    //idx 4: q1 CCW
    0, 0, 0, 5,
    //idx 5: q2 CCW
    0, 0, 6, 0,
    //idx 6: q3 CCW
    0, 4, 1, 0
};

typedef struct {
    uint32_t pin_a;
    uint32_t pin_b;
    uint32_t pin_btn;

    uint8_t state;
    int32_t pos;
    bool new_pos;
    bool pressed;
} rotary_encoder_t;

void rotary_encoder_init(rotary_encoder_t *encoder);
void rotary_encoder_update_pos(rotary_encoder_t *encoder, bool a, bool b);
void rotary_encoder_update_btn(rotary_encoder_t *encoder, bool p);

#endif //__ROTARY_ENCODER_H__