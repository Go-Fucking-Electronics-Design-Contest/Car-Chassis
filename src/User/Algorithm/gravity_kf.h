#ifndef __GRAVITY_KF_H__
#define __GRAVITY_KF_H__

#include <stdint.h>

/*
 * GravityKF 模块用途：
 *
 * 这个模块不直接输出 roll/pitch/yaw。
 * 它只估计“机体坐标系下的重力方向向量”：
 *
 *     g_body = [x y z]^T
 *
 * main.c 再把这个重力方向作为 Mahony 的加速度参考输入。
 *
 * 这样做的目的：
 * - gyro 用来预测重力方向如何随机体转动而变化。
 * - accel 用来观测当前重力方向。
 * - 当 accel 模长明显不是 1g 时，说明存在明显线性加速度，此时跳过 accel 修正。
 */
typedef struct
{
    /*
     * KF 估计出的机体系重力方向。
     *
     * 注意：
     * - 这里保存的是方向，不是加速度大小。
     * - 正常情况下已经归一化，模长约等于 1。
     * - 可以直接作为 Mahony_Update() 的 ax/ay/az 输入。
     */
    float x;
    float y;
    float z;

    /*
     * 3x3 协方差矩阵 P。
     *
     * P 越大，表示当前重力方向估计越不确定。
     * P 越小，表示当前重力方向估计越可信。
     */
    float p[3][3];

    /*
     * 最近一次更新是否使用了 accel 观测。
     *
     * 1：accel 模长在可信范围内，本次执行了观测更新。
     * 0：accel 不可信，本次只使用 gyro 过程模型预测。
     */
    uint8_t acc_used;

    /*
     * 初始化标志。
     *
     * 第一次调用 GravityKF_Update() 时，如果 accel 有效，
     * 会用 accel 方向初始化重力方向。
     */
    uint8_t initialized;
} GravityKF_t;

/*
 * 可调参数都定义在 gravity_kf.c 顶部，声明为全局变量方便 Watch 或 VOFA 命令修改。
 */
extern volatile float gravity_kf_q;
extern volatile float gravity_kf_r;
extern volatile float gravity_kf_p0;
extern volatile float gravity_kf_acc_trust_min;
extern volatile float gravity_kf_acc_trust_max;

/*
 * 默认全局 GravityKF 实例。
 *
 * main.c 可以直接使用：
 *     GravityKF_Init(&gravity_kf);
 *     GravityKF_Update(&gravity_kf, ...);
 */
extern GravityKF_t gravity_kf;

/*
 * @brief 初始化 GravityKF。
 * @param kf GravityKF 实例指针。
 *
 * 函数会清空状态，并把协方差 P 设置成 gravity_kf_p0。
 * gyro bias 校准完成后建议重新调用一次，避免旧 bias 下的预测状态继续影响输出。
 */
void GravityKF_Init(GravityKF_t *kf);

/*
 * @brief 执行一次 GravityKF 更新。
 * @param kf GravityKF 实例指针。
 * @param ax_g ICM42688 加速度 x 轴，单位 g。
 * @param ay_g ICM42688 加速度 y 轴，单位 g。
 * @param az_g ICM42688 加速度 z 轴，单位 g。
 * @param gx_radps 已经做过零偏补偿的陀螺仪 x 轴角速度，单位 rad/s。
 * @param gy_radps 已经做过零偏补偿的陀螺仪 y 轴角速度，单位 rad/s。
 * @param gz_radps 已经做过零偏补偿的陀螺仪 z 轴角速度，单位 rad/s。
 * @param dt 采样周期，单位 s。1kHz IMU 通常传 0.001f。
 */
void GravityKF_Update(GravityKF_t *kf,
                      float ax_g,
                      float ay_g,
                      float az_g,
                      float gx_radps,
                      float gy_radps,
                      float gz_radps,
                      float dt);

#endif
