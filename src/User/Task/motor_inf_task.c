#include "motor_inf_task.h"

#include "ti_msp_dl_config.h"

#include "motor_encoder_gpio_logic.h"
#include "motor_speed_interval.h"
#include "motor_speed_pid_params.h"
#include "motor_speed_units.h"
#include "tb6612.h"
#include "ts_time.h"

#define MOTOR_INF_PERIOD_US             (10000u)
#define MOTOR_INF_DEFAULT_DT_S          (0.01f)
#define MOTOR_LEFT_SPEED_PERIOD_US       (20000u)

#define MOTOR_ENCODER_PPR                   (13.0f)
#define MOTOR_GEAR_RATIO                    (28.0f)
#define MOTOR_LEFT_GPIO_EDGE_FACTOR         (4.0f)
#define MOTOR_RIGHT_QEI_QUADRATURE_FACTOR   (4.0f)
#define MOTOR_LEFT_ENCODER_COUNTS_PER_REV   \
    (MOTOR_ENCODER_PPR * MOTOR_LEFT_GPIO_EDGE_FACTOR * MOTOR_GEAR_RATIO)
#define MOTOR_RIGHT_ENCODER_COUNTS_PER_REV  \
    (MOTOR_ENCODER_PPR * MOTOR_RIGHT_QEI_QUADRATURE_FACTOR * MOTOR_GEAR_RATIO)

#define MOTOR_INF_TARGET_LIMIT_RPM      (100000.0f)

PID_t motor_left_speed_pid;
PID_t motor_right_speed_pid;

const float motor_left_speed_pid_params[PID_PARAM_COUNT] = {
//    160.0f,//160
//    0.5f,
//    1.0f,
//	//1600,
//    (float)TB6612_PWM_MAX_COUNT,
//    0.0f,
//    0.0f,
//    1.0f
	50.0f,//160
    0.5f,
    0.0f,
	//1600,
    (float)TB6612_PWM_MAX_COUNT,
    0.0f,
    0.0f,
    1.0f
};

const float motor_right_speed_pid_params[PID_PARAM_COUNT] = {
//    180.0f, //180
//    1.0f,
//    0.0f,
//	//1600,
//    (float)TB6612_PWM_MAX_COUNT,
//    0.0f,
//    0.0f,
//    1.0f
	    50.0f, //180
    0.0f,
    0.0f,
	//1600,
    (float)TB6612_PWM_MAX_COUNT,
    0.0f,
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
volatile float motor_left_speed_counts_per_s_fdb;
volatile float motor_right_speed_counts_per_s_fdb;
volatile float motor_left_pwm;
volatile float motor_right_pwm;
volatile float motor_inf_dt_s;
volatile uint32_t motor_left_speed_interval_us;

volatile int32_t motor_left_gpio_pulse_accum;
volatile uint32_t motor_diag_gpioa_isr_count;
volatile uint32_t motor_diag_gpioa_a_rise_count;
volatile uint32_t motor_diag_gpioa_b_rise_count;
volatile uint32_t motor_diag_gpioa_unhandled_count;
volatile uint8_t motor_diag_gpioa_ab_prev;
volatile uint8_t motor_diag_gpioa_ab_curr;
volatile int8_t motor_diag_gpioa_ab_delta;
volatile uint32_t motor_diag_gpioa_ab_valid_count;
volatile uint32_t motor_diag_gpioa_ab_invalid_count;
volatile uint32_t motor_diag_gpioa_ab_same_count;

volatile uint32_t motor_diag_timg7_isr_count;
volatile uint32_t motor_diag_timg7_cc0_count;
volatile uint32_t motor_diag_timg7_cc1_count;
volatile uint32_t motor_diag_timg7_unhandled_count;
volatile uint32_t motor_diag_timg7_last_iidx;
volatile uint8_t motor_diag_timg7_ab_prev;
volatile uint8_t motor_diag_timg7_ab_curr;
volatile int8_t motor_diag_timg7_ab_delta;
volatile uint32_t motor_diag_timg7_ab_valid_count;
volatile uint32_t motor_diag_timg7_ab_invalid_count;
volatile uint32_t motor_diag_timg7_ab_same_count;
volatile uint16_t motor_diag_timg8_qei_raw;
volatile int16_t motor_diag_timg8_qei_delta;

static uint32_t motor_inf_last_tick;
static uint32_t motor_inf_elapsed_acc_us;
static uint32_t motor_left_speed_elapsed_acc_us;
static int32_t motor_right_encoder_last;
static uint16_t motor_right_qei_last;
static uint8_t motor_left_ab_prev;
static uint8_t motor_left_ab_valid;

static float Motor_Inf_LimitTarget(float value);
static int16_t Motor_Inf_PwmFloatToCount(float pwm);
static void Motor_Inf_UpdateRightQei(void);
static void Motor_Inf_ConfigLeftEncoderGPIO(void);
static uint8_t Motor_Inf_ReadLeftEncoderAB(void);

void Motor_Inf_Task_Init(void)
{
    PID_Init(&motor_left_speed_pid);
    PID_Init(&motor_right_speed_pid);

    Motor_SpeedPid_ApplyParams(
        &motor_left_speed_pid,
        motor_left_speed_pid_params);
    Motor_SpeedPid_ApplyParams(
        &motor_right_speed_pid,
        motor_right_speed_pid_params);

    Motor_Inf_ResetEncoder();
    Motor_Inf_Stop();
    Motor_Inf_ConfigLeftEncoderGPIO();

    DL_TimerG_startCounter(QEI_ENCODER_RIGHT_INST);
    motor_right_qei_last = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_RIGHT_INST);

    motor_inf_last_tick = TS_Time_Get_tick();
    motor_inf_elapsed_acc_us = 0u;
    motor_inf_dt_s = MOTOR_INF_DEFAULT_DT_S;
}
 uint32_t elapsed_us;
void Motor_Inf_Task_Run(void)
{
    uint32_t primask;
    uint32_t control_interval_us;
    uint32_t left_interval_us;
    int32_t right_now;

    elapsed_us = TS_Time_GetDelta_us(&motor_inf_last_tick);
    motor_inf_elapsed_acc_us += elapsed_us;
    if (motor_inf_elapsed_acc_us < MOTOR_INF_PERIOD_US)
    {
        return;
    }

    control_interval_us = motor_inf_elapsed_acc_us;
    motor_inf_dt_s = ((float)control_interval_us) * 0.000001f;
    motor_inf_elapsed_acc_us = 0u;
    if (motor_inf_dt_s <= 0.0f)
    {
        motor_inf_dt_s = MOTOR_INF_DEFAULT_DT_S;
    }

    Motor_Inf_UpdateRightQei();

    right_now = motor_right_encoder_count;
    motor_right_encoder_delta = right_now - motor_right_encoder_last;
    motor_right_encoder_last = right_now;

    if (Motor_SpeedInterval_Accumulate(
            &motor_left_speed_elapsed_acc_us,
            control_interval_us,
            MOTOR_LEFT_SPEED_PERIOD_US,
            &left_interval_us) != 0u)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        motor_left_encoder_delta = motor_left_gpio_pulse_accum;
        motor_left_gpio_pulse_accum = 0;
        __set_PRIMASK(primask);

        motor_left_speed_interval_us = left_interval_us;
        motor_left_speed_counts_per_s_fdb =
            Motor_Speed_CountsPerSecond(
                motor_left_encoder_delta,
                ((float)left_interval_us) * 0.000001);
        motor_left_speed_fdb =
            (motor_left_speed_counts_per_s_fdb * 60.0f) /
            MOTOR_LEFT_ENCODER_COUNTS_PER_REV;
    }

		motor_right_speed_counts_per_s_fdb =
		Motor_Speed_CountsPerSecond(motor_right_encoder_delta, motor_inf_dt_s);
    motor_right_speed_fdb =
        ((float)motor_right_encoder_delta * 60.0f) /
        (MOTOR_RIGHT_ENCODER_COUNTS_PER_REV * motor_inf_dt_s);
//(float)line_sensor.error
    motor_left_pwm = pid_clac(&motor_left_speed_pid,
        motor_left_speed_ref,
        motor_left_speed_fdb,
        motor_inf_dt_s);
    motor_right_pwm = pid_clac(&motor_right_speed_pid,
        motor_right_speed_ref,
        motor_right_speed_fdb,
        motor_inf_dt_s);

    TB6612_SetSpeed(Motor_Inf_PwmFloatToCount(motor_left_pwm+1500),
        Motor_Inf_PwmFloatToCount(motor_right_pwm+1500));
//				TB6612_SetSpeed(0,
//        0);
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

void Motor_Inf_ResetEncoder(void)
{
    motor_left_encoder_count = 0;
    motor_right_encoder_count = 0;
    motor_left_encoder_delta = 0;
    motor_right_encoder_delta = 0;
    motor_right_encoder_last = 0;
    motor_left_speed_fdb = 0.0f;
    motor_right_speed_fdb = 0.0f;
    motor_left_speed_counts_per_s_fdb = 0.0f;
    motor_left_gpio_pulse_accum = 0;
    motor_left_speed_elapsed_acc_us = 0u;
    motor_left_speed_interval_us = 0u;

    motor_diag_gpioa_isr_count = 0u;
    motor_diag_gpioa_a_rise_count = 0u;
    motor_diag_gpioa_b_rise_count = 0u;
    motor_diag_gpioa_unhandled_count = 0u;
    motor_diag_gpioa_ab_curr = Motor_Inf_ReadLeftEncoderAB();
    motor_diag_gpioa_ab_prev = motor_diag_gpioa_ab_curr;
    motor_diag_gpioa_ab_delta = 0;
    motor_diag_gpioa_ab_valid_count = 0u;
    motor_diag_gpioa_ab_invalid_count = 0u;
    motor_diag_gpioa_ab_same_count = 0u;
    motor_left_ab_prev = motor_diag_gpioa_ab_curr;
    motor_left_ab_valid = 1u;

    motor_diag_timg7_isr_count = 0u;
    motor_diag_timg7_cc0_count = 0u;
    motor_diag_timg7_cc1_count = 0u;
    motor_diag_timg7_unhandled_count = 0u;
    motor_diag_timg7_last_iidx = 0u;
    motor_diag_timg7_ab_prev = 0u;
    motor_diag_timg7_ab_curr = 0u;
    motor_diag_timg7_ab_delta = 0;
    motor_diag_timg7_ab_valid_count = 0u;
    motor_diag_timg7_ab_invalid_count = 0u;
    motor_diag_timg7_ab_same_count = 0u;
    motor_diag_timg8_qei_raw = 0u;
    motor_diag_timg8_qei_delta = 0;

    motor_right_qei_last = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_RIGHT_INST);
}

static float Motor_Inf_LimitTarget(float value)
{
    if (value > MOTOR_INF_TARGET_LIMIT_RPM)
    {
        return MOTOR_INF_TARGET_LIMIT_RPM;
    }

    if (value < -MOTOR_INF_TARGET_LIMIT_RPM)
    {
        return -MOTOR_INF_TARGET_LIMIT_RPM;
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

static void Motor_Inf_UpdateRightQei(void)
{
    uint16_t now;
    int16_t delta;

    now = (uint16_t)DL_TimerG_getTimerCount(QEI_ENCODER_RIGHT_INST);
    delta = (int16_t)(now - motor_right_qei_last);
    motor_right_qei_last = now;
    motor_diag_timg8_qei_raw = now;
    motor_diag_timg8_qei_delta = delta;

    if (delta != 0)
    {
        Motor_Inf_AddEncoderDelta(MOTOR_INF_RIGHT, (int32_t)delta);
    }
}

static void Motor_Inf_ConfigLeftEncoderGPIO(void)
{
    uint32_t pins;

    DL_TimerG_disableInterrupt(CAPTURE_ENCODER_LEFT_INST,
        DL_TIMERG_INTERRUPT_CC0_UP_EVENT |
        DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
    DL_TimerG_stopCounter(CAPTURE_ENCODER_LEFT_INST);
    DL_TimerG_clearInterruptStatus(CAPTURE_ENCODER_LEFT_INST,
        DL_TIMERG_INTERRUPT_CC0_UP_EVENT |
        DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
    NVIC_ClearPendingIRQ(CAPTURE_ENCODER_LEFT_INST_INT_IRQN);
    NVIC_DisableIRQ(CAPTURE_ENCODER_LEFT_INST_INT_IRQN);

    DL_GPIO_initDigitalInput(GPIO_CAPTURE_ENCODER_LEFT_C0_IOMUX);
    DL_GPIO_initDigitalInput(GPIO_CAPTURE_ENCODER_LEFT_C1_IOMUX);

    pins = GPIO_CAPTURE_ENCODER_LEFT_C0_PIN |
        GPIO_CAPTURE_ENCODER_LEFT_C1_PIN;
    DL_GPIO_disableInterrupt(GPIOA, pins);
    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_17_EDGE_RISE_FALL |
        DL_GPIO_PIN_18_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, pins);
    motor_left_ab_prev = Motor_Inf_ReadLeftEncoderAB();
    motor_left_ab_valid = 1u;
    motor_diag_gpioa_ab_prev = motor_left_ab_prev;
    motor_diag_gpioa_ab_curr = motor_left_ab_prev;
    motor_diag_gpioa_ab_delta = 0;
    DL_GPIO_enableInterrupt(GPIOA, pins);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

static uint8_t Motor_Inf_ReadLeftEncoderAB(void)
{
    uint32_t levels;

    levels = DL_GPIO_readPins(GPIOA,
        GPIO_CAPTURE_ENCODER_LEFT_C0_PIN |
        GPIO_CAPTURE_ENCODER_LEFT_C1_PIN);

    return (uint8_t)(
        (((levels & GPIO_CAPTURE_ENCODER_LEFT_C0_PIN) != 0u) ? 2u : 0u) |
        (((levels & GPIO_CAPTURE_ENCODER_LEFT_C1_PIN) != 0u) ? 1u : 0u));
}

void Motor_Inf_LeftEncoderGPIO_IRQHandler(void)
{
    uint32_t pins;
    uint32_t pending;
    uint8_t current_ab;
    int8_t delta;

    pins = GPIO_CAPTURE_ENCODER_LEFT_C0_PIN |
        GPIO_CAPTURE_ENCODER_LEFT_C1_PIN;
    pending = DL_GPIO_getEnabledInterruptStatus(GPIOA, pins);
    motor_diag_gpioa_isr_count++;

    if ((pending & GPIO_CAPTURE_ENCODER_LEFT_C0_PIN) != 0u)
    {
        motor_diag_gpioa_a_rise_count++;
    }

    if ((pending & GPIO_CAPTURE_ENCODER_LEFT_C1_PIN) != 0u)
    {
        motor_diag_gpioa_b_rise_count++;
    }

    if ((pending & pins) != 0u)
    {
        current_ab = Motor_Inf_ReadLeftEncoderAB();
        motor_diag_gpioa_ab_prev = motor_left_ab_prev;
        motor_diag_gpioa_ab_curr = current_ab;

        if (motor_left_ab_valid == 0u)
        {
            delta = 0;
            motor_left_ab_valid = 1u;
        }
        else
        {
            delta = Motor_Encoder_DecodeTransition(
                motor_left_ab_prev,
                current_ab);
        }

        motor_diag_gpioa_ab_delta = delta;
        if (delta != 0)
        {
            motor_left_encoder_count += (int32_t)delta;
            motor_left_gpio_pulse_accum += (int32_t)delta;
            motor_diag_gpioa_ab_valid_count++;
        }
        else if (current_ab == motor_left_ab_prev)
        {
            motor_diag_gpioa_ab_same_count++;
        }
        else
        {
            motor_diag_gpioa_ab_invalid_count++;
        }

        motor_left_ab_prev = current_ab;
    }
    else
    {
        motor_diag_gpioa_unhandled_count++;
    }

    DL_GPIO_clearInterruptStatus(GPIOA, pending & pins);
}
