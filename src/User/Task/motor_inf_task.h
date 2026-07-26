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
 * - TIMG7 用双通道捕获中断模拟 QEI，AB 任意边沿触发后软件 4 倍频计数。
 * - TIMG8 使用硬件 QEI，10ms 周期读取硬件计数增量。
 * - Motor_Inf_Task_Run() 每 10ms 计算一次速度并更新速度 PID。
 * - 速度 PID 输出为 TB6612 PWM 计数值，范围由 TB6612_PWM_MAX_COUNT 限制。
 */
extern PID_t motor_left_speed_pid;
extern PID_t motor_right_speed_pid;
extern const float motor_inf_speed_pid_params[];

extern volatile int32_t motor_left_encoder_count;
extern volatile int32_t motor_right_encoder_count;
extern volatile int32_t motor_left_encoder_delta;
extern volatile int32_t motor_right_encoder_delta;

extern volatile float motor_left_speed_ref;
extern volatile float motor_right_speed_ref;
extern volatile float motor_left_speed_fdb;
extern volatile float motor_right_speed_fdb;
extern volatile float motor_left_pwm;
extern volatile float motor_right_pwm;
extern volatile float motor_inf_dt_s;

void Motor_Inf_Task_Init(void);
void Motor_Inf_Task_Run(void);

void Motor_Inf_SetTarget(float left_speed, float right_speed);
void Motor_Inf_Stop(void);

void Motor_Inf_AddEncoderDelta(Motor_Inf_Id_t motor, int32_t delta);
void Motor_Inf_EncoderABEdge(Motor_Inf_Id_t motor, uint8_t a_level, uint8_t b_level);
void Motor_Inf_ResetEncoder(void);

#endif
