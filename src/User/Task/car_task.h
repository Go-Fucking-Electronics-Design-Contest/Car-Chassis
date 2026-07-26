#ifndef CAR_TASK_H
#define CAR_TASK_H

#include <stdint.h>

#include "pid.h"

typedef enum
{
    CAR_STATE_LINE_FOLLOW = 0,
    CAR_STATE_TURN_PREPARE,
    CAR_STATE_TURNING_90,
    CAR_STATE_REACQUIRE_LINE,
    CAR_STATE_STOP
} Car_State_t;

/*
 * 整车任务：
 * - 只负责状态机和左右轮目标速度生成。
 * - 不直接写 PWM，最终输出统一交给 motor_inf_task 的速度闭环。
 * - 后续引入 RTOS 时，Car_Task_Run() 可以直接放入 10ms 整车控制任务。
 */
extern PID_t car_turn_pid;
extern const float car_turn_pid_params[];

extern volatile Car_State_t car_state;
extern volatile float car_base_speed;
extern volatile float car_turn_output;
extern volatile float car_left_speed_ref;
extern volatile float car_right_speed_ref;
extern volatile float car_yaw_ref;
extern volatile float car_yaw_fdb;
extern volatile float car_yaw_error;
extern volatile uint8_t car_side_count;
extern volatile uint8_t chassis_task_running;
extern volatile float car_cruise_speed;

void Car_Task_Init(void);
void Car_Task_Run(void);
void Car_Task_Start(void);
void Car_Task_Stop(void);
void Car_Task_RequestTurn(int8_t direction);
void Car_Task_SetCruiseSpeed(float speed);

#endif
