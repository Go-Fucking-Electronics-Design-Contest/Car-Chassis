#ifndef MOTOR_SPEED_INTERVAL_H
#define MOTOR_SPEED_INTERVAL_H

#include <stdint.h>

static inline uint8_t Motor_SpeedInterval_Accumulate(
    uint32_t *elapsed_acc_us,
    uint32_t elapsed_us,
    uint32_t period_us,
    uint32_t *completed_interval_us)
{
    if ((UINT32_MAX - *elapsed_acc_us) < elapsed_us)
    {
        *elapsed_acc_us = UINT32_MAX;
    }
    else
    {
        *elapsed_acc_us += elapsed_us;
    }

    if ((period_us == 0u) || (*elapsed_acc_us < period_us))
    {
        return 0u;
    }

    *completed_interval_us = *elapsed_acc_us;
    *elapsed_acc_us = 0u;
    return 1u;
}

#endif
