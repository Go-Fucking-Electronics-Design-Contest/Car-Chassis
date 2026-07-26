#include "pid.h"

static float PID_Clamp(float value, float min, float max);
static float PID_Abs(float value);
static void PID_SetOutputLimitInternal(PID_t *pid, float min, float max);
static void PID_SetIntegralLimitInternal(PID_t *pid, float min, float max);
static void PID_SetDeadbandInternal(PID_t *pid, float deadband);
static float PID_LimitIntegral(float integral, float min, float max);
static float PID_UpdateErrorInternal(PID_t *pid, float error, float dt_s);
static uint8_t PID_ShouldStopIntegral(float unclamped_output,
                                      float output_min,
                                      float output_max,
                                      float error,
                                      uint8_t enable);

void PID_Init(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->first_update = 1u;
}

void PID_ClearIntegral(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->i_out = 0.0f;
}

void PID_SetParams(PID_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_limit_abs,
                   float integral_limit_abs,
                   float deadband,
                   uint8_t integral_anti_windup)
{
    float output_limit;
    float integral_limit;

    if (pid == 0)
    {
        return;
    }

    output_limit = PID_Abs(output_limit_abs);
    integral_limit = PID_Abs(integral_limit_abs);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    PID_SetOutputLimitInternal(pid, -output_limit, output_limit);
    PID_SetIntegralLimitInternal(pid, -integral_limit, integral_limit);
    PID_SetDeadbandInternal(pid, deadband);
    pid->integral_anti_windup = (integral_anti_windup != 0u) ? 1u : 0u;
}

float pid_clac(PID_t *pid,
               float target,
               float feedback,
               float dt_s)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return PID_UpdateErrorInternal(pid, target - feedback, dt_s);
}

static void PID_SetOutputLimitInternal(PID_t *pid, float min, float max)
{
    if (pid == 0)
    {
        return;
    }

    if (min > max)
    {
        float temp = min;
        min = max;
        max = temp;
    }

    pid->output_min = min;
    pid->output_max = max;
    pid->output = PID_Clamp(pid->output, pid->output_min, pid->output_max);
}

static void PID_SetIntegralLimitInternal(PID_t *pid, float min, float max)
{
    if (pid == 0)
    {
        return;
    }

    if (min > max)
    {
        float temp = min;
        min = max;
        max = temp;
    }

    pid->integral_min = min;
    pid->integral_max = max;
    pid->integral = PID_LimitIntegral(pid->integral,
                                      pid->integral_min,
                                      pid->integral_max);
}

static void PID_SetDeadbandInternal(PID_t *pid, float deadband)
{
    if (pid == 0)
    {
        return;
    }

    pid->deadband = PID_Abs(deadband);
}

static float PID_UpdateErrorInternal(PID_t *pid, float error, float dt_s)
{
    float derivative;
    float next_integral;
    float unclamped_output;
    uint8_t stop_integral;

    if (pid == 0)
    {
        return 0.0f;
    }
    pid->error = error;
    /*
     * 死区处理。
     * 误差很小时按 0 处理，避免中心附近的小抖动不断触发修正。
     */
    if (PID_Abs(error) <= pid->deadband)
    {
        error = 0.0f;
    }

    /*
     * dt_s 必须大于 0。
     * 如果传入 0 或负数，说明调用周期异常，本次不更新微分和积分。
     */
    if (dt_s <= 0.0f)
    {
        pid->p_out = pid->kp * error;
        pid->i_out = pid->ki * pid->integral;
        pid->d_out = 0.0f;
        pid->output = PID_Clamp(pid->p_out + pid->i_out,
                                pid->output_min,
                                pid->output_max);
        return pid->output;
    }

    /*
     * 首次计算时没有可靠的 last_error。
     * 微分项直接置 0，避免刚上电或刚 init 时出现一次很大的 D 冲击。
     */
    if (pid->first_update)
    {
        derivative = 0.0f;
        pid->first_update = 0u;
    }
    else
    {
        derivative = (error - pid->last_error) / dt_s;
    }

    /*
     * 先计算候选积分值。
     * 后面如果发现输出已经饱和且误差还在继续推向饱和方向，就丢弃这次积分。
     */
    next_integral = PID_LimitIntegral(pid->integral + error * dt_s,
                                      pid->integral_min,
                                      pid->integral_max);

    pid->p_out = pid->kp * error;
    pid->i_out = pid->ki * next_integral;
    pid->d_out = pid->kd * derivative;

    unclamped_output = pid->p_out + pid->i_out + pid->d_out;

    stop_integral = PID_ShouldStopIntegral(unclamped_output,
                                           pid->output_min,
                                           pid->output_max,
                                           error,
                                           pid->integral_anti_windup);
    if (stop_integral)
    {
        /*
         * 积分饱和保护。
         * 输出已经到上限或下限时，如果继续积分只会让饱和更严重，就保留旧积分。
         */
        pid->i_out = pid->ki * pid->integral;
        unclamped_output = pid->p_out + pid->i_out + pid->d_out;
    }
    else
    {
        pid->integral = next_integral;
    }

    pid->output = PID_Clamp(unclamped_output,
                            pid->output_min,
                            pid->output_max);

    pid->last_error = error;
    return pid->output;
}

static float PID_Clamp(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }

    if (value < min)
    {
        return min;
    }

    return value;
}

static float PID_Abs(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float PID_LimitIntegral(float integral, float min, float max)
{
    return PID_Clamp(integral, min, max);
}

static uint8_t PID_ShouldStopIntegral(float unclamped_output,
                                      float output_min,
                                      float output_max,
                                      float error,
                                      uint8_t enable)
{
    if (enable == 0u)
    {
        return 0u;
    }

    /*
     * 上限饱和：
     * 未限幅输出已经超过 output_max，而且当前误差为正。
     */
    if ((unclamped_output > output_max) && (error > 0.0f))
    {
        return 1u;
    }

    /*
     * 下限饱和：
     * 未限幅输出已经低于 output_min，而且当前误差为负。
     */
    if ((unclamped_output < output_min) && (error < 0.0f))
    {
        return 1u;
    }

    return 0u;
}
