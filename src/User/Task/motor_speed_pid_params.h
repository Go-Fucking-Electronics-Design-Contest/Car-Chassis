#ifndef MOTOR_SPEED_PID_PARAMS_H
#define MOTOR_SPEED_PID_PARAMS_H

#include "pid.h"

static inline void Motor_SpeedPid_ApplyParams(
    PID_t *pid,
    const float params[PID_PARAM_COUNT])
{
    PID_SetParams(pid,
        params[PID_PARAM_KP_INDEX],
        params[PID_PARAM_KI_INDEX],
        params[PID_PARAM_KD_INDEX],
        params[PID_PARAM_OUTPUT_LIMIT_INDEX],
        params[PID_PARAM_INTEGRAL_LIMIT_INDEX],
        params[PID_PARAM_DEADBAND_INDEX],
        (uint8_t)params[PID_PARAM_ANTI_WINDUP_INDEX]);
}

#endif
