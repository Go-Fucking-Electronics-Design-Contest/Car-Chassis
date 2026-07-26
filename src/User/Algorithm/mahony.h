#ifndef __MAHONY_H__
#define __MAHONY_H__

#include <stdint.h>

/*
 * Mahony 姿态解算对外接口。
 *
 * 使用顺序：
 * 1. 定义 MahonyFilter_t 结构体变量。
 * 2. 调用 Mahony_Init(&mahony, dt) 初始化。
 * 3. 可选：直接修改 mahony_kp / mahony_ki 等全局参数。
 * 4. 每收到一帧 IMU 数据调用 Mahony_Update()。
 * 5. 从 mahony.roll / mahony.pitch / mahony.yaw 读取欧拉角，单位 deg。
 */

/* 弧度转角度系数：deg = rad * MAHONY_RAD_TO_DEG。 */
#define MAHONY_RAD_TO_DEG 57.29577951308232f

/* 角度转弧度系数：rad = deg * MAHONY_DEG_TO_RAD。 */
#define MAHONY_DEG_TO_RAD 0.017453292519943295f

extern volatile float mahony_kp;
extern volatile float mahony_ki;
extern volatile float mahony_acc_trust_min;
extern volatile float mahony_acc_trust_max;
extern volatile float mahony_kp_scale_near_1g;
extern volatile float mahony_kp_scale_mid;
extern volatile float mahony_kp_scale_far;

typedef struct
{
    /* 三轴向量通用结构，既可以保存 acc，也可以保存 gyro。 */
    float x;
    float y;
    float z;
} mahony_vec3_t;

typedef struct
{
    /*
     * 四元数
     * q0 为实部，q1/q2/q3 为虚部
     */
    float q0;
    float q1;
    float q2;
    float q3;

    /*
     * 旋转矩阵
     */
    float rMat[3][3];

    /*
     * 输入数据
     */
    mahony_vec3_t acc;   // 单位：g
    mahony_vec3_t gyro;  // 单位：rad/s

    /*
     * 叉乘误差
     */
    float ex;
    float ey;
    float ez;

    /*
     * 积分误差，用于补偿陀螺零偏
     */
    float exInt;
    float eyInt;
    float ezInt;

    /*
     * PI 参数
     */
    float Kp;
    float Ki;

    /*
     * 采样周期，单位 s
     */
    float dt;

    /*
     * 欧拉角输出，单位 deg
     */
    float roll;
    float pitch;
    float yaw;

    /*
     * 欧拉角输出，单位 rad
     */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    /*
     * 加速度可信度门限
     * acc_norm 接近 1g 才强修正
     */
    float acc_trust_min;
    float acc_trust_max;

} MahonyFilter_t;


void Mahony_Init(MahonyFilter_t *mahony, float dt);

uint8_t Mahony_InitFromAccel(MahonyFilter_t *mahony,
                             float dt,
                             float ax_g,
                             float ay_g,
                             float az_g);

void Mahony_SetParam(MahonyFilter_t *mahony, float kp, float ki);

void Mahony_Update(MahonyFilter_t *mahony,
                   float ax_g,
                   float ay_g,
                   float az_g,
                   float gx_radps,
                   float gy_radps,
                   float gz_radps);

#endif
