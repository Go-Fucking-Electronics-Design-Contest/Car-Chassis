#ifndef MOTOR_ENCODER_GPIO_LOGIC_H
#define MOTOR_ENCODER_GPIO_LOGIC_H

#include <stdint.h>

static inline int8_t Motor_Encoder_DecodeTransition(
    uint8_t previous_ab,
    uint8_t current_ab)
{
    static const int8_t delta_table[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };

    return delta_table[
        ((previous_ab & 0x03u) << 2) |
        (current_ab & 0x03u)];
}

#endif
