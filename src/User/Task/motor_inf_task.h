#ifndef MOTOR_INF_TASK_H
#define MOTOR_INF_TASK_H

#include <stdint.h>

#include "pid.h"

typedef enum
{
    MOTOR_INF_LEFT = 0,
    MOTOR_INF_RIGHT = 1
} Motor_Inf_Id_t;

/*
 * 电机信息任务：
 * - PA17/PA18 使用 GPIO 双边沿中断进行软件 4 倍频有符号计数。
 * - 左轮每 20ms 读取并清零窗口计数，速度反馈单位为 RPM。
 * - TIMG8 使用硬件 QEI，10ms 周期读取硬件计数增量。
 * - Motor_Inf_Task_Run() 每 10ms 更新一次速度 PID。
 * - 速度 PID 输出为 TB6612 PWM 计数值，范围由 TB6612_PWM_MAX_COUNT 限制。
 */
extern PID_t motor_left_speed_pid;
extern PID_t motor_right_speed_pid;
extern const float motor_left_speed_pid_params[PID_PARAM_COUNT];
extern const float motor_right_speed_pid_params[PID_PARAM_COUNT];

extern volatile int32_t motor_left_encoder_count;
extern volatile int32_t motor_right_encoder_count;
extern volatile int32_t motor_left_encoder_delta;
extern volatile int32_t motor_right_encoder_delta;

/* Wheel speed reference and feedback are both expressed in RPM. */
extern volatile float motor_left_speed_ref;
extern volatile float motor_right_speed_ref;
extern volatile float motor_left_speed_fdb;
extern volatile float motor_right_speed_fdb;
/* Left encoder speed before PPR/gear-ratio conversion, in count/s. */
extern volatile float motor_left_speed_counts_per_s_fdb;
extern volatile float motor_right_speed_counts_per_s_fdb;
extern volatile float motor_left_pwm;
extern volatile float motor_right_pwm;
extern volatile float motor_inf_dt_s;
/* Actual duration of the most recently completed left speed window. */
extern volatile uint32_t motor_left_speed_interval_us;

/* Left encoder GPIO edge window and diagnostics. */
extern volatile int32_t motor_left_gpio_pulse_accum;
extern volatile uint32_t motor_diag_gpioa_isr_count;
/* Legacy names: these counters now include both rising and falling edges. */
extern volatile uint32_t motor_diag_gpioa_a_rise_count;
extern volatile uint32_t motor_diag_gpioa_b_rise_count;
extern volatile uint32_t motor_diag_gpioa_unhandled_count;
extern volatile uint8_t motor_diag_gpioa_ab_prev;
extern volatile uint8_t motor_diag_gpioa_ab_curr;
extern volatile int8_t motor_diag_gpioa_ab_delta;
extern volatile uint32_t motor_diag_gpioa_ab_valid_count;
extern volatile uint32_t motor_diag_gpioa_ab_invalid_count;
extern volatile uint32_t motor_diag_gpioa_ab_same_count;

/*
 * Legacy TIMG7 diagnostics remain available for watch-window comparison.
 * They should stay at zero after the left encoder switches to GPIOA.
 */
extern volatile uint32_t motor_diag_timg7_isr_count;
extern volatile uint32_t motor_diag_timg7_cc0_count;
extern volatile uint32_t motor_diag_timg7_cc1_count;
extern volatile uint32_t motor_diag_timg7_unhandled_count;
extern volatile uint32_t motor_diag_timg7_last_iidx;
extern volatile uint8_t motor_diag_timg7_ab_prev;
extern volatile uint8_t motor_diag_timg7_ab_curr;
extern volatile int8_t motor_diag_timg7_ab_delta;
extern volatile uint32_t motor_diag_timg7_ab_valid_count;
extern volatile uint32_t motor_diag_timg7_ab_invalid_count;
extern volatile uint32_t motor_diag_timg7_ab_same_count;
extern volatile uint16_t motor_diag_timg8_qei_raw;
extern volatile int16_t motor_diag_timg8_qei_delta;

void Motor_Inf_Task_Init(void);
void Motor_Inf_Task_Run(void);

void Motor_Inf_SetTarget(float left_speed, float right_speed);
void Motor_Inf_Stop(void);

void Motor_Inf_AddEncoderDelta(Motor_Inf_Id_t motor, int32_t delta);
void Motor_Inf_ResetEncoder(void);
void Motor_Inf_LeftEncoderGPIO_IRQHandler(void);

#endif
