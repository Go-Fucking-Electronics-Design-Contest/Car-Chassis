#include <assert.h>
#include <stdint.h>

#include "motor_speed_interval.h"

int main(void)
{
    uint32_t elapsed_acc_us = 0u;
    uint32_t completed_interval_us = 0u;

    assert(Motor_SpeedInterval_Accumulate(
        &elapsed_acc_us, 10000u, 20000u, &completed_interval_us) == 0u);
    assert(elapsed_acc_us == 10000u);
    assert(completed_interval_us == 0u);

    assert(Motor_SpeedInterval_Accumulate(
        &elapsed_acc_us, 10000u, 20000u, &completed_interval_us) == 1u);
    assert(elapsed_acc_us == 0u);
    assert(completed_interval_us == 20000u);

    assert(Motor_SpeedInterval_Accumulate(
        &elapsed_acc_us, 22000u, 20000u, &completed_interval_us) == 1u);
    assert(elapsed_acc_us == 0u);
    assert(completed_interval_us == 22000u);

    return 0;
}
