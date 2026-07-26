#ifndef __IMU_CALIB_H__
#define __IMU_CALIB_H__

#include "icm42688.h"
#include <stdint.h>

/* Maximum accepted change of adjacent one-second bias means, in rad/s^2. */
#ifndef IMU_CALIB_GYRO_BIAS_RATE_MAX_RADPS2
#define IMU_CALIB_GYRO_BIAS_RATE_MAX_RADPS2 (0.001f)
#endif

/*
 * IMU gyro 静态零偏校准模块。
 *
 * 职责：
 * 1. 上电读取Flash中上一次保存的gyro bias。
 * 2. 按上层命令执行3秒、3000点静态零偏采样。
 * 3. 把最终选定的bias写入Flash。
 * 4. 每帧把gyro bias扣到角速度数据上。
 * 5. 校准期间红灯常亮，成功保存后熄灭，致命故障时闪烁。
 */

/* Watch 用状态变量。 */
extern volatile uint8_t imu_calib_running;
extern volatile uint8_t imu_calib_failed;
extern volatile uint32_t imu_calib_sample_count;
extern volatile uint8_t imu_calib_quality_checking;
extern volatile uint8_t imu_calib_retry_count;
extern volatile float imu_calib_bias_rate_x_radps2;
extern volatile float imu_calib_bias_rate_y_radps2;
extern volatile float imu_calib_bias_rate_z_radps2;
extern volatile float imu_calib_bias_rate_max_radps2;
extern volatile uint8_t imu_calib_bias_rate_passed;

/* 上电初始化校准模块，读取 Flash 中保存的 bias，并初始化 LED 为熄灭。 */
void IMU_Calib_Init(void);

/* 开始一轮3秒静态零偏采样，红灯常亮。 */
void IMU_Calib_Start(void);

/*
 * 每处理完一帧 IMU 数据后调用一次。
 *
 * 正在校准时先累计当前未扣Bias的gyro样本，然后再对本帧应用当前Bias。
 *
 * 返回 1：本次调用刚完成一次校准，main 可以重置 Mahony。
 * 返回 0：没有新完成的校准。
 */
uint8_t IMU_Calib_Process(volatile icm42688_data_t *imu);

/* Read the mean acceleration collected during the latest completed attempt. */
uint8_t IMU_Calib_GetAccelMean(float *ax_g, float *ay_g, float *az_g);

/* 选择质量检测得到的最好Bias，只更新RAM，不写Flash。 */
void IMU_Calib_SetBias(float bias_x, float bias_y, float bias_z);

/* 将当前RAM Bias写入Flash；成功时关闭红灯。 */
uint8_t IMU_Calib_SaveCurrentBiasToFlash(void);

/* 执行一次红灯亮灭故障提示，由启动故障死循环反复调用。 */
void IMU_Calib_BlinkFault(void);

#endif
