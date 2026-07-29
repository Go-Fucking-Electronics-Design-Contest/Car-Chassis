#include "tb6612.h"
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * 引脚连接：
 * PWMA -> PB2 / TIMG6_CCP0
 * PWMB -> PB3 / TIMG6_CCP1
 * AIN2 -> PA11
 * AIN1 -> PA12
 * BIN1 -> PB16
 * BIN2 -> PB0
 * STBY -> 硬件上拉
 */

#define TB6612_PWM_PERIOD_COUNT (3200u)

static int16_t TB6612_LimitPwmCount(int16_t pwm_count);
static uint16_t TB6612_GetPwmAbsCount(int16_t pwm_count);
static void TB6612_SetMotorADir(int16_t pwm_count);
static void TB6612_SetMotorBDir(int16_t pwm_count);
static void TB6612_SetMotorPwm(uint32_t channel, uint16_t pwm_count);

/*
 * 初始化电机输出到安全状态。
 * TIMG6 PWM 和 GPIO 复用已经由 SysConfig 在本函数调用前完成。
 */
void TB6612_Init(void)
{
    TB6612_Stop();
}

/*
 * 设置 A 电机 PWM 计数值。
 * 正数正转，负数反转，0 为滑行停止。
 */
void TB6612_SetMotorA(int16_t pwm_count)
{
    pwm_count = TB6612_LimitPwmCount(pwm_count);

    TB6612_SetMotorADir(pwm_count);
    TB6612_SetMotorPwm(DL_TIMER_CC_0_INDEX,
        TB6612_GetPwmAbsCount(pwm_count));
}

/*
 * 设置 B 电机 PWM 计数值。
 * 正数正转，负数反转，0 为滑行停止。
 */
void TB6612_SetMotorB(int16_t pwm_count)
{
    pwm_count = TB6612_LimitPwmCount(pwm_count);

    TB6612_SetMotorBDir(pwm_count);
    TB6612_SetMotorPwm(DL_TIMER_CC_1_INDEX,
        TB6612_GetPwmAbsCount(pwm_count));
}

/*
 * 同时更新左右两个电机。
 * 差速控制优先使用这个接口，避免左右轮分散更新。
 */
void TB6612_SetSpeed(int16_t motor_a_pwm_count, int16_t motor_b_pwm_count)
{
    TB6612_SetMotorA(motor_a_pwm_count);
    TB6612_SetMotorB(motor_b_pwm_count);
}

/*
 * 滑行停止。
 * IN1/IN2 同时拉低，PWM 输出为 0。
 */
void TB6612_Stop(void)
{
    DL_GPIO_clearPins(GPIO_TB6612_DIR_AIN1_PORT,
        GPIO_TB6612_DIR_AIN1_PIN | GPIO_TB6612_DIR_AIN2_PIN);
    DL_GPIO_clearPins(GPIO_TB6612_DIR_BIN1_PORT,
        GPIO_TB6612_DIR_BIN1_PIN | GPIO_TB6612_DIR_BIN2_PIN);

    TB6612_SetMotorPwm(DL_TIMER_CC_0_INDEX, 0u);
    TB6612_SetMotorPwm(DL_TIMER_CC_1_INDEX, 0u);
}

/*
 * 短刹停止。
 * IN1/IN2 同时拉高，只在需要快速停车时使用。
 */
void TB6612_Brake(void)
{
    DL_GPIO_setPins(GPIO_TB6612_DIR_AIN1_PORT,
        GPIO_TB6612_DIR_AIN1_PIN | GPIO_TB6612_DIR_AIN2_PIN);
    DL_GPIO_setPins(GPIO_TB6612_DIR_BIN1_PORT,
        GPIO_TB6612_DIR_BIN1_PIN | GPIO_TB6612_DIR_BIN2_PIN);

    TB6612_SetMotorPwm(DL_TIMER_CC_0_INDEX, 0u);
    TB6612_SetMotorPwm(DL_TIMER_CC_1_INDEX, 0u);
}

/*
 * 把 PWM 命令限制在合法计数范围内。
 */
static int16_t TB6612_LimitPwmCount(int16_t pwm_count)
{
    if (pwm_count > TB6612_PWM_MAX_COUNT)
    {
        return TB6612_PWM_MAX_COUNT;
    }

    if (pwm_count < -TB6612_PWM_MAX_COUNT)
    {
        return -TB6612_PWM_MAX_COUNT;
    }

    return pwm_count;
}

/*
 * 把带方向的 PWM 命令转换成绝对计数值。
 * 方向由 IN 引脚决定，PWM 比较值只使用幅值。
 */
static uint16_t TB6612_GetPwmAbsCount(int16_t pwm_count)
{
    if (pwm_count < 0)
    {
        pwm_count = (int16_t)(-pwm_count);
    }

    if (pwm_count > TB6612_PWM_MAX_COUNT)
    {
        return TB6612_PWM_PERIOD_COUNT;
    }

    return (uint16_t)pwm_count;
}

/*
 * 设置 A 电机方向引脚。
 */
static void TB6612_SetMotorADir(int16_t pwm_count)
{
    if (pwm_count > 0)
    {
        DL_GPIO_setPins(GPIO_TB6612_DIR_AIN1_PORT, GPIO_TB6612_DIR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_DIR_AIN2_PORT, GPIO_TB6612_DIR_AIN2_PIN);
    }
    else if (pwm_count < 0)
    {
        DL_GPIO_clearPins(GPIO_TB6612_DIR_AIN1_PORT, GPIO_TB6612_DIR_AIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_DIR_AIN2_PORT, GPIO_TB6612_DIR_AIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_TB6612_DIR_AIN1_PORT,
            GPIO_TB6612_DIR_AIN1_PIN | GPIO_TB6612_DIR_AIN2_PIN);
    }
}

/*
 * 设置 B 电机方向引脚。
 */
static void TB6612_SetMotorBDir(int16_t pwm_count)
{
    /*
     * Motor B is mounted with the opposite electrical polarity to Motor A.
     * Keep the public convention consistent: positive PWM drives both wheels
     * toward the vehicle's forward direction.
     */
    if (pwm_count > 0)
    {
        DL_GPIO_clearPins(GPIO_TB6612_DIR_BIN1_PORT, GPIO_TB6612_DIR_BIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_DIR_BIN2_PORT, GPIO_TB6612_DIR_BIN2_PIN);
    }
    else if (pwm_count < 0)
    {
        DL_GPIO_setPins(GPIO_TB6612_DIR_BIN1_PORT, GPIO_TB6612_DIR_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_DIR_BIN2_PORT, GPIO_TB6612_DIR_BIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_TB6612_DIR_BIN1_PORT,
            GPIO_TB6612_DIR_BIN1_PIN | GPIO_TB6612_DIR_BIN2_PIN);
    }
}

/*
 * 写入 PWM 比较值。
 * SysConfig 当前使用边沿对齐 PWM，这里直接写有效 PWM 计数值。
 */
static void TB6612_SetMotorPwm(uint32_t channel, uint16_t pwm_count)
{
    uint16_t compare_count;

    if (pwm_count > TB6612_PWM_PERIOD_COUNT)
    {
        pwm_count = TB6612_PWM_PERIOD_COUNT;
    }

    compare_count = (uint16_t)(TB6612_PWM_PERIOD_COUNT - pwm_count);

    DL_TimerG_setCaptureCompareValue(TIMEG6_Motor_INST,
        (uint32_t)compare_count,
        (DL_TIMER_CC_INDEX)channel);
}
