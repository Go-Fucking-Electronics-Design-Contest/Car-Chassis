#ifndef MOTOR_SPEED_WINDOW_H
#define MOTOR_SPEED_WINDOW_H

#include <stdint.h>

#include "motor_speed_units.h"

#define MOTOR_SPEED_WINDOW_SAMPLE_CAPACITY (5u)

typedef struct
{
    int32_t count_delta[MOTOR_SPEED_WINDOW_SAMPLE_CAPACITY];
    float dt_s[MOTOR_SPEED_WINDOW_SAMPLE_CAPACITY];
    int32_t count_sum;
    float time_sum_s;
    uint8_t next_index;
    uint8_t sample_count;
} Motor_SpeedWindow_t;

static inline void Motor_SpeedWindow_Reset(Motor_SpeedWindow_t *window)
{
    uint8_t index;

    for (index = 0u; index < MOTOR_SPEED_WINDOW_SAMPLE_CAPACITY; index++)
    {
        window->count_delta[index] = 0;
        window->dt_s[index] = 0.0f;
    }

    window->count_sum = 0;
    window->time_sum_s = 0.0f;
    window->next_index = 0u;
    window->sample_count = 0u;
}

static inline void Motor_SpeedWindow_Push(
    Motor_SpeedWindow_t *window,
    int32_t count_delta,
    float dt_s)
{
    uint8_t index;

    index = window->next_index;

    if (window->sample_count == MOTOR_SPEED_WINDOW_SAMPLE_CAPACITY)
    {
        window->count_sum -= window->count_delta[index];
        window->time_sum_s -= window->dt_s[index];
    }
    else
    {
        window->sample_count++;
    }

    window->count_delta[index] = count_delta;
    window->dt_s[index] = dt_s;
    window->count_sum += count_delta;
    window->time_sum_s += dt_s;

    index++;
    if (index >= MOTOR_SPEED_WINDOW_SAMPLE_CAPACITY)
    {
        index = 0u;
    }
    window->next_index = index;
}

static inline uint8_t Motor_SpeedWindow_GetSampleCount(
    const Motor_SpeedWindow_t *window)
{
    return window->sample_count;
}

static inline int32_t Motor_SpeedWindow_GetCountSum(
    const Motor_SpeedWindow_t *window)
{
    return window->count_sum;
}

static inline float Motor_SpeedWindow_GetTimeSum_s(
    const Motor_SpeedWindow_t *window)
{
    return window->time_sum_s;
}

static inline float Motor_SpeedWindow_GetCountsPerSecond(
    const Motor_SpeedWindow_t *window)
{
    return Motor_Speed_CountsPerSecond(
        window->count_sum,
        window->time_sum_s);
}

#endif
