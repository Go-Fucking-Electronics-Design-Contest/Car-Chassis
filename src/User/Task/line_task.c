#include "line_task.h"
#include <stdint.h>
#include "line_sensor.h"
#include "ts_time.h"

/*
 * 循迹 PID 对象。
 * 保持全局，方便 Watch 观察 P/I/D 分量、积分量和最终输出。
 */
PID_t line_pid;

/*
 * 循迹 PID 实际运行周期，单位 s。
 * 由 TS_Time_GetDelta_s() 根据两次 LineSensor_Update() 成功间隔计算。
 */
volatile float line_pid_dt_s;

/*
 * 循迹 PID 输出。
 * 当前只计算并保留结果，电机输出示例默认注释，方便后续调车时打开。
 */
volatile float line_pid_output;

/*
 * 只在本文件内部使用的 PID 时间戳。
 */
static uint32_t line_pid_last_tick;

/*
 * 初始化循迹传感器和 PID。
 */
void Line_Task_Init(void)
{
    LineSensor_Init(&line_sensor);

    PID_Init(&line_pid);
    PID_SetParams(&line_pid,
        line_sensor_pid_params[PID_PARAM_KP_INDEX],
        line_sensor_pid_params[PID_PARAM_KI_INDEX],
        line_sensor_pid_params[PID_PARAM_KD_INDEX],
        line_sensor_pid_params[PID_PARAM_OUTPUT_LIMIT_INDEX],
        line_sensor_pid_params[PID_PARAM_INTEGRAL_LIMIT_INDEX],
        line_sensor_pid_params[PID_PARAM_DEADBAND_INDEX],
        (uint8_t)line_sensor_pid_params[PID_PARAM_ANTI_WINDUP_INDEX]);

    line_pid_last_tick = TS_Time_Get_tick();
}

/*
 * 执行一次循迹任务。
 *
 * LineSensor_Update() 内部会完成：
 * - 选择 8 路灰度通道。
 * - ADC + DMA 采样。
 * - 归一化。
 * - 计算 line_sensor.error 和 line_sensor.line_lost。
 */
void Line_Task_Run(void)
{
    if (!LineSensor_Update(&line_sensor))
    {
        return;
    }

    line_pid_dt_s = TS_Time_GetDelta_s(&line_pid_last_tick);
    line_pid_output = pid_clac(&line_pid,
        0.0f,
        (float)line_sensor.error,
        line_pid_dt_s);

    if (line_sensor.line_lost)
    {
        PID_ClearIntegral(&line_pid);
        return;
    }

     /*
     * TB6612 循迹输出例程：
     * TB6612_SetSpeed() 使用 PWM 计数值，范围 -3200 ~ +3200。
     *
     * line_pid_output 是循迹差速修正量。
     * 如果 PID 输出不是 PWM 计数单位，需要先乘一个缩放系数。
     * 后续调车时，取消下面代码注释即可把 PID 输出接到左右电机。
     */
    // TB6612_SetSpeed(640 - (int16_t)(line_pid_output * 0.1f),
    //     640 + (int16_t)(line_pid_output * 0.1f));
}
