#include "car_task.h"

#include "icm42688.h"
#include "line_sensor.h"
#include "line_task.h"
#include "motor_inf_task.h"
#include "tjc_protocol.h"
#include "ts_time.h"

#define CAR_TASK_PERIOD_US              (10000u)
#define CAR_LINE_BASE_SPEED             (300.0f)
#define CAR_TASK1_CRUISE_SPEED          (300.0f)
#define CAR_TASK4_CRUISE_SPEED          (200.0f)
#define CAR_SIDES_PER_LAP               (4u)
#define CAR_REACQUIRE_SPEED             (120.0f)
#define CAR_LINE_TURN_SCALE             (1.0f)
#define CAR_TURN_ANGLE_DEG              (90.0f)
#define CAR_TURN_FINISH_DEADBAND_DEG    (3.0f)
#define CAR_TURN_FINISH_HOLD_COUNT      (8u)
#define CAR_REACQUIRE_HOLD_COUNT        (5u)
#define CAR_CORNER_BLACK_MIN_COUNT      (5u)
#define CAR_CORNER_DETECT_HOLD_COUNT    (3u)
#define CAR_CORNER_RELEASE_MAX_COUNT    (3u)
#define CAR_CORNER_RELEASE_HOLD_COUNT   (5u)
#define CAR_CORNER_TURN_DIRECTION       (1)

PID_t car_turn_pid;

const float car_turn_pid_params[PID_PARAM_COUNT] = {
    5.0f,
    0.0f,
    0.0f,
    260.0f,
    100.0f,
    0.5f,
    1.0f
};

volatile Car_State_t car_state;
volatile float car_base_speed;
volatile float car_turn_output;
volatile float car_left_speed_ref;
volatile float car_right_speed_ref;
volatile float car_yaw_ref;
volatile float car_yaw_fdb;
volatile float car_yaw_error;
volatile uint8_t car_side_count;
volatile uint8_t chassis_task_running;
volatile float car_cruise_speed;

static uint32_t car_task_last_tick;
static uint32_t car_task_elapsed_acc_us;
static uint8_t car_turn_finish_count;
static uint8_t car_reacquire_count;
static int8_t car_turn_direction;
static uint8_t car_turn_request;
static uint8_t car_corner_detect_count;
static uint8_t car_corner_release_count;
static uint8_t car_corner_marker_locked;
/* 底盘当前实际正在执行的任务号；未运行时为 0。 */
static uint16_t car_running_task_id;

static void Car_Task_UpdateSelectedTask(void);
static void Car_Task_RunLapTask(float cruise_speed, uint16_t task_id);
static void Car_Task_StopSelectedTask(void);
static void Car_Task_UpdateTurnRequest(void);
static uint8_t Car_Task_CountBits(uint8_t value);
static void Car_Task_RunState(float dt_s);
static void Car_Task_SetWheelTarget(float base_speed, float turn_speed);
static void Car_Task_SetTurnPidParams(void);
static float Car_Task_WrapAngle(float angle_deg);
static float Car_Task_AngleDiff(float target_deg, float feedback_deg);
static float Car_Task_Abs(float value);

void Car_Task_Init(void)
{
    PID_Init(&car_turn_pid);
    Car_Task_SetTurnPidParams();

    car_state = CAR_STATE_LINE_FOLLOW;
    car_base_speed = CAR_LINE_BASE_SPEED;
    car_turn_output = 0.0f;
    car_left_speed_ref = 0.0f;
    car_right_speed_ref = 0.0f;
    car_yaw_ref = 0.0f;
    car_yaw_fdb = 0.0f;
    car_yaw_error = 0.0f;
    car_side_count = 0u;
    chassis_task_running = 0u;
    car_cruise_speed = CAR_LINE_BASE_SPEED;
    car_turn_finish_count = 0u;
    car_reacquire_count = 0u;
    car_turn_direction = 1;
    car_turn_request = 0u;
    car_corner_detect_count = 0u;
    car_corner_release_count = 0u;
    car_corner_marker_locked = 0u;
    car_running_task_id = 0u;

    car_task_last_tick = TS_Time_Get_tick();
    car_task_elapsed_acc_us = 0u;
}

void Car_Task_Run(void)
{
    uint32_t elapsed_us;
    float dt_s;

    Car_Task_UpdateSelectedTask();

    elapsed_us = TS_Time_GetDelta_us(&car_task_last_tick);
    car_task_elapsed_acc_us += elapsed_us;
    if (car_task_elapsed_acc_us < CAR_TASK_PERIOD_US)
    {
        return;
    }

    dt_s = ((float)car_task_elapsed_acc_us) * 0.000001f;
    car_task_elapsed_acc_us = 0u;
    if (dt_s <= 0.0f)
    {
        dt_s = 0.01f;
    }

    Car_Task_RunState(dt_s);
}

/*
    run_enabled： 用户要求运行
    chassis_task_running：  底盘实际运行
    gimbal_task_running：云台任务是否实际运行
*/
static void Car_Task_UpdateSelectedTask(void)
{
    switch (tjc_control.task_id)
    {
        case 0u:
            /* 任务 0：空闲，底盘和云台都停止。 */
            tjc_control.run_enabled = 0u;
            tjc_control.gimbal_task_running = 0u;
            Car_Task_StopSelectedTask();
            break;

        case 1u:
            /* 任务 1：仅底盘按设定圈数循迹行驶。 */
            tjc_control.gimbal_task_running = 0u;
            if (tjc_control.run_enabled)
            {
                Car_Task_RunLapTask(CAR_TASK1_CRUISE_SPEED, 1u);
            }
            else
            {
                Car_Task_StopSelectedTask();
            }
            break;

        case 2u:
            /* 任务 2：仅运行云台瞄准任务。 */
            tjc_control.gimbal_task_running = tjc_control.run_enabled;
            Car_Task_StopSelectedTask();
            break;

        case 3u:
            /* 任务 3：仅运行云台自动瞄准任务。 */
            tjc_control.gimbal_task_running = tjc_control.run_enabled;
            Car_Task_StopSelectedTask();
            break;

        case 4u:
            /* 任务 4：底盘循迹，同时运行云台任务。 */
            tjc_control.gimbal_task_running = tjc_control.run_enabled;
            if (tjc_control.run_enabled)
            {
                Car_Task_RunLapTask(CAR_TASK4_CRUISE_SPEED, 4u);
            }
            else
            {
                Car_Task_StopSelectedTask();
            }
            break;

        default:
            tjc_control.run_enabled = 0u;
            tjc_control.gimbal_task_running = 0u;
            Car_Task_StopSelectedTask();
            break;
    }
}

static void Car_Task_RunLapTask(float cruise_speed, uint16_t task_id)
{
    if ((!chassis_task_running) || (car_running_task_id != task_id))
    {
        Car_Task_SetCruiseSpeed(cruise_speed);
        car_running_task_id = task_id;
        tjc_control.completed_laps = 0u;
        tjc_control.task_finished = 0u;
        Car_Task_Start();
    }

    tjc_control.completed_laps =
        (uint16_t)(car_side_count / CAR_SIDES_PER_LAP);

    if (tjc_control.completed_laps >= tjc_control.lap_count)
    {
        Car_Task_Stop();
        car_running_task_id = 0u;
        tjc_control.run_enabled = 0u;
        tjc_control.gimbal_task_running = 0u;
        tjc_control.task_finished = 1u;
    }
}

static void Car_Task_StopSelectedTask(void)
{
    if (chassis_task_running)
    {
        Car_Task_Stop();
    }

    car_running_task_id = 0u;
}
void Car_Task_Start(void)
{
    chassis_task_running = 1u;
    car_state = CAR_STATE_LINE_FOLLOW;
    car_side_count = 0u;
    car_turn_request = 0u;
    car_turn_finish_count = 0u;
    car_reacquire_count = 0u;
    car_corner_detect_count = 0u;
    car_corner_release_count = 0u;
    car_corner_marker_locked = 0u;
}

void Car_Task_Stop(void)
{
    chassis_task_running = 0u;
    car_state = CAR_STATE_STOP;
    car_left_speed_ref = 0.0f;
    car_right_speed_ref = 0.0f;
    Motor_Inf_Stop();
}

void Car_Task_RequestTurn(int8_t direction)
{
    car_turn_direction = (direction < 0) ? -1 : 1;
    car_turn_request = 1u;
}

void Car_Task_SetCruiseSpeed(float speed)
{
    if (speed >= 0.0f)
    {
        car_cruise_speed = speed;
    }
}

static void Car_Task_UpdateTurnRequest(void)
{
    uint8_t black_count = Car_Task_CountBits(line_sensor.black_mask);

    /* 同一条横向标志线只允许触发一次，离开后才能再次触发。 */
    if (car_corner_marker_locked)
    {
        if (black_count <= CAR_CORNER_RELEASE_MAX_COUNT)
        {
            car_corner_release_count++;
            if (car_corner_release_count >= CAR_CORNER_RELEASE_HOLD_COUNT)
            {
                car_corner_marker_locked = 0u;
                car_corner_release_count = 0u;
            }
        }
        else
        {
            car_corner_release_count = 0u;
        }
        return;
    }

    /* 至少 6 路连续 3 个周期为黑，判定为 90 度转弯标志线。 */
    if (black_count >= CAR_CORNER_BLACK_MIN_COUNT)
    {
        car_corner_detect_count++;
        if (car_corner_detect_count >= CAR_CORNER_DETECT_HOLD_COUNT)
        {
            car_corner_detect_count = 0u;
            car_corner_release_count = 0u;
            car_corner_marker_locked = 1u;
            Car_Task_RequestTurn(CAR_CORNER_TURN_DIRECTION);
        }
    }
    else
    {
        car_corner_detect_count = 0u;
    }
}

static uint8_t Car_Task_CountBits(uint8_t value)
{
    uint8_t count = 0u;

    while (value != 0u)
    {
        count = (uint8_t)(count + (value & 1u));
        value >>= 1u;
    }

    return count;
}

static void Car_Task_RunState(float dt_s)
{
    if (!chassis_task_running)
    {
        Car_Task_SetWheelTarget(50.0f, 0.0f);
        return;
    }

    car_yaw_fdb = icm42688_data.yaw;

    switch (car_state)
    {
        case CAR_STATE_LINE_FOLLOW:
            Car_Task_UpdateTurnRequest();

            if (line_sensor.line_lost)
            {
                car_state = CAR_STATE_REACQUIRE_LINE;
                car_reacquire_count = 0u;
                break;
            }

            if (car_turn_request)
            {
                car_state = CAR_STATE_TURN_PREPARE;
                break;
            }

            car_base_speed = car_cruise_speed;
            car_turn_output = line_pid_output * CAR_LINE_TURN_SCALE;
            Car_Task_SetWheelTarget(car_base_speed, car_turn_output);
            break;

        case CAR_STATE_TURN_PREPARE:
            car_turn_request = 0u;
            car_yaw_ref = Car_Task_WrapAngle(
                car_yaw_fdb + ((float)car_turn_direction * CAR_TURN_ANGLE_DEG));
            car_turn_finish_count = 0u;
            PID_Init(&car_turn_pid);
            Car_Task_SetTurnPidParams();
            car_state = CAR_STATE_TURNING_90;
            break;

        case CAR_STATE_TURNING_90:
            car_yaw_error = Car_Task_AngleDiff(car_yaw_ref, car_yaw_fdb);
            car_turn_output = pid_clac(&car_turn_pid,
                0.0f,
                -car_yaw_error,
                dt_s);
            Car_Task_SetWheelTarget(0.0f, car_turn_output);

            if (Car_Task_Abs(car_yaw_error) <= CAR_TURN_FINISH_DEADBAND_DEG)
            {
                car_turn_finish_count++;
                if (car_turn_finish_count >= CAR_TURN_FINISH_HOLD_COUNT)
                {
                    car_side_count++;
                    car_reacquire_count = 0u;
                    car_state = CAR_STATE_REACQUIRE_LINE;
                }
            }
            else
            {
                car_turn_finish_count = 0u;
            }
            break;

        case CAR_STATE_REACQUIRE_LINE:
            if (!line_sensor.line_lost)
            {
                car_reacquire_count++;
                if (car_reacquire_count >= CAR_REACQUIRE_HOLD_COUNT)
                {
                    car_state = CAR_STATE_LINE_FOLLOW;
                }
            }
            else
            {
                car_reacquire_count = 0u;
            }

            car_turn_output = line_pid_output * CAR_LINE_TURN_SCALE;
            Car_Task_SetWheelTarget(CAR_REACQUIRE_SPEED, car_turn_output);
            break;

        case CAR_STATE_STOP:
        default:
            Car_Task_SetWheelTarget(0.0f, 0.0f);
            break;
    }
}

static void Car_Task_SetWheelTarget(float base_speed, float turn_speed)
{
    car_left_speed_ref = base_speed - turn_speed;
    car_right_speed_ref = base_speed + turn_speed;
//	car_right_speed_ref = 0;
    Motor_Inf_SetTarget(car_left_speed_ref, car_right_speed_ref);
}

static void Car_Task_SetTurnPidParams(void)
{
    PID_SetParams(&car_turn_pid,
        car_turn_pid_params[PID_PARAM_KP_INDEX],
        car_turn_pid_params[PID_PARAM_KI_INDEX],
        car_turn_pid_params[PID_PARAM_KD_INDEX],
        car_turn_pid_params[PID_PARAM_OUTPUT_LIMIT_INDEX],
        car_turn_pid_params[PID_PARAM_INTEGRAL_LIMIT_INDEX],
        car_turn_pid_params[PID_PARAM_DEADBAND_INDEX],
        (uint8_t)car_turn_pid_params[PID_PARAM_ANTI_WINDUP_INDEX]);
}

static float Car_Task_WrapAngle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float Car_Task_AngleDiff(float target_deg, float feedback_deg)
{
    return Car_Task_WrapAngle(target_deg - feedback_deg);
}

static float Car_Task_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}
