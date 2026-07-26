#include "mahony.h"
#include <math.h>

/*
 * mahony.c 模块说明
 *
 * 这个文件实现六轴 Mahony 姿态解算：
 * 1. 输入加速度 ax/ay/az，单位 g。
 * 2. 输入角速度 gx/gy/gz，单位 rad/s。
 * 3. 用当前四元数估计“理论重力方向”。
 * 4. 用加速度归一化后得到“测量重力方向”。
 * 5. 两个重力方向叉乘得到姿态误差 ex/ey/ez。
 * 6. 用 PI 对 gyro 做修正，再积分四元数。
 * 7. 四元数归一化后换算旋转矩阵和欧拉角 roll/pitch/yaw。
 *
 * 注意：
 * 六轴 Mahony 没有磁力计，所以 yaw 没有绝对参考，长期会漂。
 */

/*
 * ==================== Mahony 常用调参区 ====================
 * mahony_kp / mahony_ki 是全局可调参数，Watch 或 VOFA 修改后会在下一次更新生效。
 *
 * ACC_TRUST_MIN/MAX：加速度模长在这个范围内才认为加速度方向比较可信。
 * 车体动态大时可以放宽，例如 0.70f ~ 1.30f；静态或低动态可以收窄。
 *
 * KP_SCALE_*：加速度模长偏离 1g 时，自动降低 Mahony 的 Kp，减少动态加速度把姿态拉歪。
 */
volatile float mahony_kp = 0.5f;
volatile float mahony_ki = 0.0f;

/* 加速度模长可信范围。小车动态比较大时，可以放宽到 0.70f ~ 1.30f。 */
volatile float mahony_acc_trust_min = 0.75f;
volatile float mahony_acc_trust_max = 1.25f;

/* 加速度越偏离 1g，Kp 自动降得越多，减少动态加速度拉歪姿态。 */
volatile float mahony_kp_scale_near_1g = 1.0f;
volatile float mahony_kp_scale_mid = 0.5f;
volatile float mahony_kp_scale_far = 0.2f;
/* ==================== Mahony 常用调参区结束 ==================== */


static float Mahony_InvSqrt(float x)
{
    /* 防止 sqrtf() 输入非法值；返回 0 表示本次归一化失败。 */
    if (x <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(x);
}

//限幅
static float Mahony_Clamp(float x, float min_val, float max_val)
{
    /* 把 x 限制在 [min_val, max_val]，主要用于防止 asinf() 输入超过 +/-1。 */
    if (x < min_val)
    {
        return min_val;
    }

    if (x > max_val)
    {
        return max_val;
    }

    return x;
}


/*
 * 四元数归一化
 */
static void Mahony_NormalizeQuat(MahonyFilter_t *mahony)
{
    /* norm 实际保存的是 1 / 四元数模长。 */
    float norm;

    norm = Mahony_InvSqrt(mahony->q0 * mahony->q0 +
                          mahony->q1 * mahony->q1 +
                          mahony->q2 * mahony->q2 +
                          mahony->q3 * mahony->q3);

    if (norm > 0.0f)
    {
        mahony->q0 *= norm;
        mahony->q1 *= norm;
        mahony->q2 *= norm;
        mahony->q3 *= norm;
    }
}


/*
 * 由四元数计算旋转矩阵
 * 旋转矩阵每一行代表一个轴相对世界系的单位方向向量
 * rMat[2][0], rMat[2][1], rMat[2][2]
 * 就是当前四元数估计出来的重力方向。
 */
static void Mahony_UpdateRotationMatrix(MahonyFilter_t *mahony)
{
    /* 先把四元数成员取到局部变量，后面公式更清楚，也减少重复访问结构体。 */
    float q0 = mahony->q0;
    float q1 = mahony->q1;
    float q2 = mahony->q2;
    float q3 = mahony->q3;

    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    mahony->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mahony->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    mahony->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    mahony->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    mahony->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mahony->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    mahony->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    mahony->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    mahony->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}


/*
 * 从旋转矩阵提取 roll pitch yaw
 */
static void Mahony_UpdateEuler(MahonyFilter_t *mahony)
{
    /* r31 对应旋转矩阵第 3 行第 1 列，用于计算 pitch。 */
    float r31;

    r31 = Mahony_Clamp(mahony->rMat[2][0], -1.0f, 1.0f);

    mahony->pitch_rad = -asinf(r31);
    mahony->roll_rad  = atan2f(mahony->rMat[2][1], mahony->rMat[2][2]);
    mahony->yaw_rad   = atan2f(mahony->rMat[1][0], mahony->rMat[0][0]);

    mahony->roll  = mahony->roll_rad  * MAHONY_RAD_TO_DEG;
    mahony->pitch = mahony->pitch_rad * MAHONY_RAD_TO_DEG;
    mahony->yaw   = mahony->yaw_rad   * MAHONY_RAD_TO_DEG;
}


void Mahony_Init(MahonyFilter_t *mahony, float dt)
{
    if (mahony == 0)
    {
        return;
    }

    /* 四元数初始化为单位姿态：没有旋转。 */
    mahony->q0 = 1.0f;
    mahony->q1 = 0.0f;
    mahony->q2 = 0.0f;
    mahony->q3 = 0.0f;

    /* 输入缓存初始化：默认加速度指向 +Z，gyro 为 0。 */
    mahony->acc.x = 0.0f;
    mahony->acc.y = 0.0f;
    mahony->acc.z = 1.0f;

    mahony->gyro.x = 0.0f;
    mahony->gyro.y = 0.0f;
    mahony->gyro.z = 0.0f;

    /* 叉乘误差和积分误差清零，避免上一次运行残留。 */
    mahony->ex = 0.0f;
    mahony->ey = 0.0f;
    mahony->ez = 0.0f;

    mahony->exInt = 0.0f;
    mahony->eyInt = 0.0f;
    mahony->ezInt = 0.0f;

    /*
     * 当前调参起点：Kp=0.5，Ki=0。
     * 两个全局变量可以通过Watch或VOFA继续调整。
     */
    /* 默认 PI 参数来自 mahony.c 顶部的全局可调变量。 */
    mahony->Kp = mahony_kp;
    mahony->Ki = mahony_ki;

    /* 保存融合周期，单位秒。当前500Hz Mahony使用dt=0.002。 */
    mahony->dt = dt;

    /* 欧拉角输出清零。 */
    mahony->roll = 0.0f;
    mahony->pitch = 0.0f;
    mahony->yaw = 0.0f;

    mahony->roll_rad = 0.0f;
    mahony->pitch_rad = 0.0f;
    mahony->yaw_rad = 0.0f;

    /*
     * 加速度模长可信范围。
     * 小车动态比较大时，可以放宽到 0.7~1.3。
     */
    mahony->acc_trust_min = mahony_acc_trust_min;
    mahony->acc_trust_max = mahony_acc_trust_max;

    Mahony_UpdateRotationMatrix(mahony);
    Mahony_UpdateEuler(mahony);
}

/*
 * Use a stationary acceleration vector to initialize roll and pitch directly.
 * Gravity cannot observe yaw, so the initial yaw is deliberately defined as 0.
 */
uint8_t Mahony_InitFromAccel(MahonyFilter_t *mahony,
                             float dt,
                             float ax_g,
                             float ay_g,
                             float az_g)
{
    float acc_norm_sq;
    float acc_norm;
    float inv_norm;
    float roll_rad;
    float pitch_rad;
    float half_roll;
    float half_pitch;
    float sin_half_roll;
    float cos_half_roll;
    float sin_half_pitch;
    float cos_half_pitch;

    if ((mahony == 0) || !(dt > 0.0f))
    {
        return 0u;
    }

    acc_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;

    /* These comparisons also reject NaN and unreasonably large input values. */
    if (!((acc_norm_sq > 1.0e-8f) && (acc_norm_sq < 100.0f)))
    {
        return 0u;
    }

    acc_norm = sqrtf(acc_norm_sq);
    if ((acc_norm < mahony_acc_trust_min) ||
        (acc_norm > mahony_acc_trust_max))
    {
        return 0u;
    }

    inv_norm = 1.0f / acc_norm;
    ax_g *= inv_norm;
    ay_g *= inv_norm;
    az_g *= inv_norm;

    roll_rad = atan2f(ay_g, az_g);
    pitch_rad = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    half_roll = 0.5f * roll_rad;
    half_pitch = 0.5f * pitch_rad;
    sin_half_roll = sinf(half_roll);
    cos_half_roll = cosf(half_roll);
    sin_half_pitch = sinf(half_pitch);
    cos_half_pitch = cosf(half_pitch);

    Mahony_Init(mahony, dt);

    /* ZYX quaternion with yaw fixed at zero. */
    mahony->q0 = cos_half_roll * cos_half_pitch;
    mahony->q1 = sin_half_roll * cos_half_pitch;
    mahony->q2 = cos_half_roll * sin_half_pitch;
    mahony->q3 = -sin_half_roll * sin_half_pitch;
    Mahony_NormalizeQuat(mahony);

    mahony->acc.x = ax_g;
    mahony->acc.y = ay_g;
    mahony->acc.z = az_g;

    Mahony_UpdateRotationMatrix(mahony);
    Mahony_UpdateEuler(mahony);

    return 1u;
}

//叉乘误差的PI参数设置
void Mahony_SetParam(MahonyFilter_t *mahony, float kp, float ki)
{
    if (mahony == 0)
    {
        return;
    }

    /* Kp 控制加速度修正姿态的速度；Ki 控制慢速 gyro bias 补偿。 */
    mahony->Kp = kp;
    mahony->Ki = ki;
}


/*
 * Mahony 六轴更新
 *
 * 输入：
 * ax_g ay_g az_g        加速度计，单位 g
 * gx_radps gy_radps gz_radps  陀螺仪，单位 rad/s
 */
void Mahony_Update(MahonyFilter_t *mahony,
                   float ax_g,
                   float ay_g,
                   float az_g,
                   float gx_radps,
                   float gy_radps,
                   float gz_radps)
{
    float acc_norm_sq;
    float acc_norm;
    float inv_norm;

    float vx;
    float vy;
    float vz;

    float ex;
    float ey;
    float ez;

    float kp_use;

    float q0Last;
    float q1Last;
    float q2Last;
    float q3Last;
    float halfT;

    if (mahony == 0)
    {
        return;
    }

    if (mahony->dt <= 0.0f)
    {
        return;
    }

    /*
     * 这些参数是全局可调变量，Watch 或 VOFA 修改后，
     * 下一次 Mahony_Update() 会自动同步到滤波器结构体。
     */
    mahony->Kp = mahony_kp;
    mahony->Ki = mahony_ki;
    mahony->acc_trust_min = mahony_acc_trust_min;
    mahony->acc_trust_max = mahony_acc_trust_max;

    mahony->acc.x = ax_g;
    mahony->acc.y = ay_g;
    mahony->acc.z = az_g;

    mahony->gyro.x = gx_radps;
    mahony->gyro.y = gy_radps;
    mahony->gyro.z = gz_radps;

    /*
     * 1. 先根据当前四元数计算旋转矩阵
     * 第三行就是估计重力方向
     */
    Mahony_UpdateRotationMatrix(mahony);

    /*
     * 2. 加速度归一化
     */
    acc_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;

    if (acc_norm_sq > 1.0e-8f)
    {
        acc_norm = sqrtf(acc_norm_sq);

        /*
         * 3. 根据加速度模长动态决定是否相信 accel。
         *
         * 如果小车急加速/震动，|acc| 会明显偏离 1g，
         * 此时加速度方向不一定等于重力方向，所以降低 Kp。
         */
        if ((acc_norm > mahony->acc_trust_min) &&
            (acc_norm < mahony->acc_trust_max))
        {
            float acc_err;

            acc_err = fabsf(acc_norm - 1.0f);

            /*
             * 简单自适应权重：
             * acc 越接近 1g，Kp 越接近原始 Kp。
             * acc 偏离越大，Kp 越小。
             */
            if (acc_err < 0.08f)
            {
                kp_use = mahony_kp_scale_near_1g * mahony->Kp;
            }
            else if (acc_err < 0.20f)
            {
                kp_use = mahony_kp_scale_mid * mahony->Kp;
            }
            else
            {
                kp_use = mahony_kp_scale_far * mahony->Kp;
            }

            inv_norm = 1.0f / acc_norm;

            ax_g *= inv_norm;
            ay_g *= inv_norm;
            az_g *= inv_norm;

            /*
             * 4. 由当前四元数估计出来的重力方向
             *
             * vx = rMat[2][0]
             * vy = rMat[2][1]
             * vz = rMat[2][2]
             */
            vx = mahony->rMat[2][0];
            vy = mahony->rMat[2][1];
            vz = mahony->rMat[2][2];

            /*
             * 5. 加速度计测得的重力方向 与 估计重力方向做叉乘
             *
             * e = acc × gravity_est
             */
            ex = ay_g * vz - az_g * vy;
            ey = az_g * vx - ax_g * vz;
            ez = ax_g * vy - ay_g * vx;

            mahony->ex = ex;
            mahony->ey = ey;
            mahony->ez = ez;

            /*
             * 6. 积分项，用于慢慢补偿陀螺 bias
             *
             * 注意：
             * 六轴条件下主要可靠修正 roll/pitch 对应 bias。
             * yaw/gz 长期仍然会漂。
             */
            if (mahony->Ki > 0.0f)
            {
                mahony->exInt += mahony->Ki * ex * mahony->dt;
                mahony->eyInt += mahony->Ki * ey * mahony->dt;
                mahony->ezInt += mahony->Ki * ez * mahony->dt;
            }
            else
            {
                mahony->exInt = 0.0f;
                mahony->eyInt = 0.0f;
                mahony->ezInt = 0.0f;
            }

            /*
             * 7. PI 修正陀螺仪
             */
            mahony->gyro.x += kp_use * ex + mahony->exInt;
            mahony->gyro.y += kp_use * ey + mahony->eyInt;
            mahony->gyro.z += kp_use * ez + mahony->ezInt;
        }
        else
        {
            /*
             * 加速度明显不可信，不用 accel 修正。
             * 此时只靠 gyro 积分。
             */
            mahony->ex = 0.0f;
            mahony->ey = 0.0f;
            mahony->ez = 0.0f;
        }
    }

    /*
     * 8. 用修正后的 gyro 积分四元数
     *
     * q_dot = 0.5 * q ⊗ omega
     */
    q0Last = mahony->q0;
    q1Last = mahony->q1;
    q2Last = mahony->q2;
    q3Last = mahony->q3;

    halfT = 0.5f * mahony->dt;

    mahony->q0 += (-q1Last * mahony->gyro.x -
                   q2Last * mahony->gyro.y -
                   q3Last * mahony->gyro.z) * halfT;

    mahony->q1 += ( q0Last * mahony->gyro.x +
                    q2Last * mahony->gyro.z -
                    q3Last * mahony->gyro.y) * halfT;

    mahony->q2 += ( q0Last * mahony->gyro.y -
                    q1Last * mahony->gyro.z +
                    q3Last * mahony->gyro.x) * halfT;

    mahony->q3 += ( q0Last * mahony->gyro.z +
                    q1Last * mahony->gyro.y -
                    q2Last * mahony->gyro.x) * halfT;

    /*
     * 9. 四元数归一化，因为旋转矩阵表示的是纯旋转，也是单位向量
     */
    Mahony_NormalizeQuat(mahony);

    /*
     * 10. 更新旋转矩阵和欧拉角
     */
    Mahony_UpdateRotationMatrix(mahony);
    Mahony_UpdateEuler(mahony);
}
