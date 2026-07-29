#include <assert.h>
#include <stdint.h>

#include "motor_speed_pid_params.h"

int main(void)
{
    static const float left_params[PID_PARAM_COUNT] = {
        1.0f, 2.0f, 3.0f, 100.0f, 50.0f, 4.0f, 1.0f
    };
    static const float right_params[PID_PARAM_COUNT] = {
        5.0f, 6.0f, 7.0f, 200.0f, 80.0f, 8.0f, 0.0f
    };
    PID_t left_pid;
    PID_t right_pid;

    PID_Init(&left_pid);
    PID_Init(&right_pid);

    Motor_SpeedPid_ApplyParams(&left_pid, left_params);
    Motor_SpeedPid_ApplyParams(&right_pid, right_params);

    assert(left_pid.kp == 1.0f);
    assert(left_pid.ki == 2.0f);
    assert(left_pid.kd == 3.0f);
    assert(left_pid.output_min == -100.0f);
    assert(left_pid.output_max == 100.0f);
    assert(left_pid.integral_min == -50.0f);
    assert(left_pid.integral_max == 50.0f);
    assert(left_pid.deadband == 4.0f);
    assert(left_pid.integral_anti_windup == 1u);

    assert(right_pid.kp == 5.0f);
    assert(right_pid.ki == 6.0f);
    assert(right_pid.kd == 7.0f);
    assert(right_pid.output_min == -200.0f);
    assert(right_pid.output_max == 200.0f);
    assert(right_pid.integral_min == -80.0f);
    assert(right_pid.integral_max == 80.0f);
    assert(right_pid.deadband == 8.0f);
    assert(right_pid.integral_anti_windup == 0u);

    return 0;
}
