#ifndef MOTOR_SPEED_UNITS_H
#define MOTOR_SPEED_UNITS_H

#include <stdint.h>

static inline float Motor_Speed_CountsPerSecond(int32_t count_delta, float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    return ((float)count_delta) / dt_s;
}

#endif
