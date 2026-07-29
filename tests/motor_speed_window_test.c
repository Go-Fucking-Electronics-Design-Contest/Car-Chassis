#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "motor_speed_window.h"

static int FloatNear(float actual, float expected)
{
    return fabsf(actual - expected) < 0.01f;
}

int main(void)
{
    Motor_SpeedWindow_t window;

    Motor_SpeedWindow_Reset(&window);
    assert(Motor_SpeedWindow_GetSampleCount(&window) == 0u);
    assert(Motor_SpeedWindow_GetCountSum(&window) == 0);
    assert(FloatNear(Motor_SpeedWindow_GetTimeSum_s(&window), 0.0f));
    assert(FloatNear(Motor_SpeedWindow_GetCountsPerSecond(&window), 0.0f));

    Motor_SpeedWindow_Push(&window, 2, 0.01f);
    assert(Motor_SpeedWindow_GetSampleCount(&window) == 1u);
    assert(Motor_SpeedWindow_GetCountSum(&window) == 2);
    assert(FloatNear(Motor_SpeedWindow_GetCountsPerSecond(&window), 200.0f));

    Motor_SpeedWindow_Push(&window, 3, 0.01f);
    Motor_SpeedWindow_Push(&window, 2, 0.01f);
    Motor_SpeedWindow_Push(&window, 3, 0.01f);
    Motor_SpeedWindow_Push(&window, 2, 0.01f);
    assert(Motor_SpeedWindow_GetSampleCount(&window) == 5u);
    assert(Motor_SpeedWindow_GetCountSum(&window) == 12);
    assert(FloatNear(Motor_SpeedWindow_GetTimeSum_s(&window), 0.05f));
    assert(FloatNear(Motor_SpeedWindow_GetCountsPerSecond(&window), 240.0f));

    Motor_SpeedWindow_Push(&window, 3, 0.01f);
    assert(Motor_SpeedWindow_GetSampleCount(&window) == 5u);
    assert(Motor_SpeedWindow_GetCountSum(&window) == 13);
    assert(FloatNear(Motor_SpeedWindow_GetTimeSum_s(&window), 0.05f));
    assert(FloatNear(Motor_SpeedWindow_GetCountsPerSecond(&window), 260.0f));

    Motor_SpeedWindow_Reset(&window);
    assert(Motor_SpeedWindow_GetSampleCount(&window) == 0u);
    assert(Motor_SpeedWindow_GetCountSum(&window) == 0);
    assert(FloatNear(Motor_SpeedWindow_GetCountsPerSecond(&window), 0.0f));

    return 0;
}
