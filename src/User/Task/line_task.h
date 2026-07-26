#ifndef LINE_TASK_H
#define LINE_TASK_H

#include "pid.h"

/*
 * 循迹任务模块。
 *
 * 当前裸机工程由 main() 循环调用 Line_Task_Run()。
 * 后续加入 RTOS 后，可以直接把 Line_Task_Run() 放进循迹任务线程。
 */

/*
 * 这些变量需要放进 CCS Watch 观察，所以保持全局。
 */
extern PID_t line_pid;
extern volatile float line_pid_dt_s;
extern volatile float line_pid_output;

void Line_Task_Init(void);
void Line_Task_Run(void);

#endif
