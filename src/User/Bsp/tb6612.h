#ifndef TB6612_H
#define TB6612_H

#include <stdint.h>

/*
 * TB6612 PWM 命令单位：
 * -3200 表示满速反转。
 *  3200 表示满速正转。
 *     0 表示滑行停止。
 */
#define TB6612_PWM_MAX_COUNT (3200)

void TB6612_Init(void);
void TB6612_SetMotorA(int16_t pwm_count);
void TB6612_SetMotorB(int16_t pwm_count);
void TB6612_SetSpeed(int16_t motor_a_pwm_count, int16_t motor_b_pwm_count);
void TB6612_Stop(void);
void TB6612_Brake(void);

#endif
