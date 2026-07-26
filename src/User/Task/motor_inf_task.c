#include "motor_inf_task.h"

#include "ti_msp_dl_config.h"

#include "tb6612.h"
#include "ts_time.h"

#define MOTOR_INF_PERIOD_US             (10000u)
#define MOTOR_INF_DEFAULT_DT_S          (0.01f)

#define MOTOR_INF_TARGET_LIMIT          (100000.0f)

PID_t motor_left_speed_pid;
PID_t motor_right_speed_pid;

const float motor_inf_speed_pid_params[PID_PARAM_COUNT] = {
    0.8f,
    0.0f,
    0.0f,
    (float)TB6612_PWM_MAX_COUNT,
    10000.0f,
    0.0f,
    1.0f
};

volatile int32_t motor_left_encoder_count;
volatile int32_t motor_right_encoder_count;
volatile int32_t motor_left_encoder_delta;
volatile int32_t motor_right_encoder_delta;

volatile float motor_left_speed_ref;
volatile float motor_right_speed_ref;
volatile float motor_left_speed_fdb;
volatile float motor_right_speed_fdb;
volatile float motor_left_pwm;
volatile float motor_right_pwm;
volatile float motor_inf_dt_s;

static uint32_t motor_inf_last_tick;
static uint32_t motor_inf_elapsed_acc_us;
static int32_t motor_left_encoder_last;
static int32_t motor_right_encoder_last;
static uint8_t motor_left_ab_last;
static uint8_t motor_right_ab_last;
static uint8_t motor_left_ab_valid;
static uint8_t motor_right_ab_valid;
static uint16_t motor_right_qei_last;

static float Motor_Inf_LimitTarget(float value);
static int16_t Motor_Inf_PwmFloatToCount(float pwm);
static int8_t Motor_Inf_DecodeAB(uint8_t prev, uint8_t curr);
static void Motor_Inf_SetSpeedPidParams(PID_t *pid);
static void Motor_Inf_UpdateRightQei(void);
static uint8_t Motor_Inf_ReadLeftEncoderA(void);
static uint8_t Motor_Inf_ReadLeftEncoderB(void);

void Motor_Inf_Task_Init(void)
{
    PID_Init(&motor_left_speed_pid);
    PID_Init(&motor_right_speed_pid);

    Motor_Inf_SetSpeedPidParams(&motor_left_speed_pid);
    Motor_Inf_SetSpeedPidParams(&motor_right_speed_pid);

    Motor_Inf_ResetEncoder();
    Motor_Inf_Stop();

    NVIC_ClearPendingIRQ(CAPTURE_ENCODER_LEFT_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_ENCODER_LEFT_INST_INT_IRQN);

    DL_TimerG_startCounter(QEI_ENCODER_RIGHT_INST);
    motor_right_qei_last = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_RIGHT_INST);

    motor_inf_last_tick = TS_Time_Get_tick();
    motor_inf_elapsed_acc_us = 0u;
    motor_inf_dt_s = MOTOR_INF_DEFAULT_DT_S;
}

void Motor_Inf_Task_Run(void)
{
    uint32_t elapsed_us;
    int32_t left_now;
    int32_t right_now;

    elapsed_us = TS_Time_GetDelta_us(&motor_inf_last_tick);
    motor_inf_elapsed_acc_us += elapsed_us;
    if (motor_inf_elapsed_acc_us < MOTOR_INF_PERIOD_US)
    {
        return;
    }

    motor_inf_dt_s = ((float)motor_inf_elapsed_acc_us) * 0.000001f;
    motor_inf_elapsed_acc_us = 0u;
    if (motor_inf_dt_s <= 0.0f)
    {
        motor_inf_dt_s = MOTOR_INF_DEFAULT_DT_S;
    }

    Motor_Inf_UpdateRightQei();

    left_now = motor_left_encoder_count;
    right_now = motor_right_encoder_count;

    motor_left_encoder_delta = left_now - motor_left_encoder_last;
    motor_right_encoder_delta = right_now - motor_right_encoder_last;
    motor_left_encoder_last = left_now;
    motor_right_encoder_last = right_now;

    motor_left_speed_fdb = ((float)motor_left_encoder_delta) / motor_inf_dt_s;
    motor_right_speed_fdb = ((float)motor_right_encoder_delta) / motor_inf_dt_s;
//(float)line_sensor.error
    motor_left_pwm = pid_clac(&motor_left_speed_pid,
        motor_left_speed_ref,
        motor_left_speed_fdb,
        motor_inf_dt_s);
    motor_right_pwm = pid_clac(&motor_right_speed_pid,
        motor_right_speed_ref,
        motor_right_speed_fdb,
        motor_inf_dt_s);

    TB6612_SetSpeed(Motor_Inf_PwmFloatToCount(motor_left_pwm),
        Motor_Inf_PwmFloatToCount(motor_right_pwm));
}

void Motor_Inf_SetTarget(float left_speed, float right_speed)
{
    motor_left_speed_ref = Motor_Inf_LimitTarget(left_speed);
    motor_right_speed_ref = Motor_Inf_LimitTarget(right_speed);
}

void Motor_Inf_Stop(void)
{
    motor_left_speed_ref = 0.0f;
    motor_right_speed_ref = 0.0f;
    motor_left_pwm = 0.0f;
    motor_right_pwm = 0.0f;
    PID_ClearIntegral(&motor_left_speed_pid);
    PID_ClearIntegral(&motor_right_speed_pid);
    TB6612_Stop();
}

void Motor_Inf_AddEncoderDelta(Motor_Inf_Id_t motor, int32_t delta)
{
    if (motor == MOTOR_INF_LEFT)
    {
        motor_left_encoder_count += delta;
    }
    else
    {
        motor_right_encoder_count += delta;
    }
}

void Motor_Inf_EncoderABEdge(Motor_Inf_Id_t motor, uint8_t a_level, uint8_t b_level)
{
    uint8_t curr;
    uint8_t prev;
    int8_t delta;

    curr = (uint8_t)(((a_level != 0u) ? 2u : 0u) |
                     ((b_level != 0u) ? 1u : 0u));

    if (motor == MOTOR_INF_LEFT)
    {
        if (!motor_left_ab_valid)
        {
            motor_left_ab_last = curr;
            motor_left_ab_valid = 1u;
            return;
        }

        prev = motor_left_ab_last;
        motor_left_ab_last = curr;
    }
    else
    {
        if (!motor_right_ab_valid)
        {
            motor_right_ab_last = curr;
            motor_right_ab_valid = 1u;
            return;
        }

        prev = motor_right_ab_last;
        motor_right_ab_last = curr;
    }

    delta = Motor_Inf_DecodeAB(prev, curr);
    if (delta != 0)
    {
        Motor_Inf_AddEncoderDelta(motor, (int32_t)delta);
    }
}

void Motor_Inf_ResetEncoder(void)
{
    motor_left_encoder_count = 0;
    motor_right_encoder_count = 0;
    motor_left_encoder_delta = 0;
    motor_right_encoder_delta = 0;
    motor_left_encoder_last = 0;
    motor_right_encoder_last = 0;
    motor_left_speed_fdb = 0.0f;
    motor_right_speed_fdb = 0.0f;
    motor_left_ab_last = 0u;
    motor_right_ab_last = 0u;
    motor_left_ab_valid = 0u;
    motor_right_ab_valid = 0u;

    motor_left_ab_last = (uint8_t)((Motor_Inf_ReadLeftEncoderA() ? 2u : 0u) |
                                   (Motor_Inf_ReadLeftEncoderB() ? 1u : 0u));
    motor_left_ab_valid = 1u;

    motor_right_qei_last = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_RIGHT_INST);
}

static float Motor_Inf_LimitTarget(float value)
{
    if (value > MOTOR_INF_TARGET_LIMIT)
    {
        return MOTOR_INF_TARGET_LIMIT;
    }

    if (value < -MOTOR_INF_TARGET_LIMIT)
    {
        return -MOTOR_INF_TARGET_LIMIT;
    }

    return value;
}

static int16_t Motor_Inf_PwmFloatToCount(float pwm)
{
    if (pwm > (float)TB6612_PWM_MAX_COUNT)
    {
        return (int16_t)TB6612_PWM_MAX_COUNT;
    }

    if (pwm < (float)-TB6612_PWM_MAX_COUNT)
    {
        return (int16_t)-TB6612_PWM_MAX_COUNT;
    }

    if (pwm >= 0.0f)
    {
        return (int16_t)(pwm + 0.5f);
    }

    return (int16_t)(pwm - 0.5f);
}

static int8_t Motor_Inf_DecodeAB(uint8_t prev, uint8_t curr)
{
    static const int8_t table[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };

    return table[((prev & 0x03u) << 2) | (curr & 0x03u)];
}

static void Motor_Inf_SetSpeedPidParams(PID_t *pid)
{
    PID_SetParams(pid,
        motor_inf_speed_pid_params[PID_PARAM_KP_INDEX],
        motor_inf_speed_pid_params[PID_PARAM_KI_INDEX],
        motor_inf_speed_pid_params[PID_PARAM_KD_INDEX],
        motor_inf_speed_pid_params[PID_PARAM_OUTPUT_LIMIT_INDEX],
        motor_inf_speed_pid_params[PID_PARAM_INTEGRAL_LIMIT_INDEX],
        motor_inf_speed_pid_params[PID_PARAM_DEADBAND_INDEX],
        (uint8_t)motor_inf_speed_pid_params[PID_PARAM_ANTI_WINDUP_INDEX]);
}

static void Motor_Inf_UpdateRightQei(void)
{
    uint16_t now;
    int16_t delta;

    now = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_RIGHT_INST);
    delta = (int16_t)(now - motor_right_qei_last);
    motor_right_qei_last = now;

    if (delta != 0)
    {
        Motor_Inf_AddEncoderDelta(MOTOR_INF_RIGHT, (int32_t)delta);
    }
}

static uint8_t Motor_Inf_ReadLeftEncoderA(void)
{
    return (DL_GPIO_readPins(GPIO_CAPTURE_ENCODER_LEFT_C0_PORT,
        GPIO_CAPTURE_ENCODER_LEFT_C0_PIN) != 0u) ? 1u : 0u;
}

static uint8_t Motor_Inf_ReadLeftEncoderB(void)
{
    return (DL_GPIO_readPins(GPIO_CAPTURE_ENCODER_LEFT_C1_PORT,
        GPIO_CAPTURE_ENCODER_LEFT_C1_PIN) != 0u) ? 1u : 0u;
}

void CAPTURE_ENCODER_LEFT_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CAPTURE_ENCODER_LEFT_INST))
    {
        case DL_TIMERG_IIDX_CC0_DN:
        case DL_TIMERG_IIDX_CC1_DN:
            Motor_Inf_EncoderABEdge(MOTOR_INF_LEFT,
                Motor_Inf_ReadLeftEncoderA(),
                Motor_Inf_ReadLeftEncoderB());
            break;

        default:
            break;
    }
}
