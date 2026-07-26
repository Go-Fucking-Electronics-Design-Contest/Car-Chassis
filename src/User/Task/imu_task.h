#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>

#include "mahony.h"

/* 静止校准质量检测：0关闭，1开启；当前默认关闭。 */
#ifndef IMU_QUALITY_CHECK_ENABLE
#define IMU_QUALITY_CHECK_ENABLE (0u)
#endif

typedef struct
{
    float x;
    float y;
    float z;
} IMU_Vector3f_t;

/*
 * IMU 任务模块。
 *
 * 当前工程仍然是裸机循环，main() 会在 while 里反复调用
 * IMU_Task_Run()。
 *
 * 后续加入 FreeRTOS 后，可以把 IMU_Task_Run() 放进真正的
 * IMU 任务线程里，IMU 主逻辑不需要重新搬一遍。
 */

/*
 * 需要放进 Watch 或跨文件访问的变量保持全局。
 * 只在 imu_task.c 内部使用的变量放在 .c 文件里并加 static。
 */
extern MahonyFilter_t imu_mahony;
extern volatile uint8_t imu_init_ok;
extern volatile uint8_t imu_init_failed;
extern volatile float spi_dma_deltat;
extern uint16_t send_divider_count;
extern volatile uint8_t imu_quality_state;

/* 与chassis_open相同原理的连续Yaw角，单位deg。 */
extern volatile float Yaw_TotalAngle;
extern volatile float Yaw_AngleLast;
extern volatile int32_t Yaw_RoundCount;

/*
 * Fixed mounting transform:
 *
 *     body_vector = imu_sensor_to_body * sensor_vector
 *
 * Rows are body X/Y/Z axes and columns are sensor X/Y/Z axes.
 * Use a proper rotation matrix (orthogonal and determinant +1).
 */
extern const float imu_sensor_to_body[3][3];
extern IMU_Vector3f_t imu_body_accel_g;
extern IMU_Vector3f_t imu_body_gyro_radps;

void IMU_Task_Init(void);
void IMU_Task_Run(void);
void IMU_TransformSensorToBody(const IMU_Vector3f_t *sensor,
                               IMU_Vector3f_t *body);

#endif
