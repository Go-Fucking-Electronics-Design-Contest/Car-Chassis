#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "motor_speed_units.h"

static int FloatNear(float actual, float expected)
{
    return fabsf(actual - expected) < 0.001f;
}

int main(void)
{
    assert(FloatNear(Motor_Speed_CountsPerSecond(10, 0.01f), 1000.0f));
    assert(FloatNear(Motor_Speed_CountsPerSecond(-5, 0.02f), -250.0f));
    assert(FloatNear(Motor_Speed_CountsPerSecond(0, 0.01f), 0.0f));
    assert(FloatNear(Motor_Speed_CountsPerSecond(10, 0.0f), 0.0f));

    return 0;
}
