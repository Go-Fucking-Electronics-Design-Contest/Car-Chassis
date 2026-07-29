#include <assert.h>
#include <stdint.h>

#include "motor_encoder_gpio_logic.h"

int main(void)
{
    /* Forward quadrature cycle: 00 -> 01 -> 11 -> 10 -> 00. */
    assert(Motor_Encoder_DecodeTransition(0u, 1u) == 1);
    assert(Motor_Encoder_DecodeTransition(1u, 3u) == 1);
    assert(Motor_Encoder_DecodeTransition(3u, 2u) == 1);
    assert(Motor_Encoder_DecodeTransition(2u, 0u) == 1);

    /* Reverse quadrature cycle: 00 -> 10 -> 11 -> 01 -> 00. */
    assert(Motor_Encoder_DecodeTransition(0u, 2u) == -1);
    assert(Motor_Encoder_DecodeTransition(2u, 3u) == -1);
    assert(Motor_Encoder_DecodeTransition(3u, 1u) == -1);
    assert(Motor_Encoder_DecodeTransition(1u, 0u) == -1);

    assert(Motor_Encoder_DecodeTransition(0u, 0u) == 0);
    assert(Motor_Encoder_DecodeTransition(3u, 3u) == 0);
    assert(Motor_Encoder_DecodeTransition(0u, 3u) == 0);
    assert(Motor_Encoder_DecodeTransition(3u, 0u) == 0);

    return 0;
}
