#include "gravity_kf.h"
#include "kalman.h"

#include <math.h>
#include <string.h>

/*
 * ==================== GravityKF 可调参数 ====================
 */

/* 过程噪声 Q。值越大，越允许 gyro 预测带来的状态变化。 */
volatile float gravity_kf_q = 0.0001f;

/* 测量噪声 R。值越大，越不相信 accel 观测。 */
volatile float gravity_kf_r = 0.008f;

/* 初始协方差 P0。 */
volatile float gravity_kf_p0 = 0.5f;

/* accel 模长在该范围内时，才认为 accel 主要表示重力。 */
volatile float gravity_kf_acc_trust_min = 0.75f;
volatile float gravity_kf_acc_trust_max = 1.25f;

/* 如果实测 roll/pitch 方向反了，把 1.0f 改成 -1.0f。 */
#define GRAVITY_KF_GYRO_SIGN (1.0f)

/* 默认全局 GravityKF 输出实例。 */
GravityKF_t gravity_kf;

/*
 * gEstimateKF 是通用 KalmanFilter_t 结构体。
 * gVec 保存重力方向估计值，给 GravityKF 输出和 Mahony 使用。
 */
static KalmanFilter_t gEstimateKF;
static float gVec[3];

/*
 * ==================== gEstimateKF 矩阵数据 ====================
 *
 * KalmanFilter_t 结构体内部保存的是 mat 矩阵对象和 float* 指针，
 * 真正的矩阵数据内存由这里这些数组提供。
 *
 * 命名规则：
 * - xhat_data      ：后验估计 x(k|k)
 * - xhatminus_data ：先验估计 x(k|k-1)
 * - z_data         ：测量向量 z
 * - P/Pminus       ：后验/先验协方差
 * - F/FT           ：状态转移矩阵及其转置
 * - H/HT           ：测量矩阵及其转置
 * - Q/R            ：过程噪声/测量噪声
 * - K/S            ：Kalman 增益/残差协方差
 * - temp_*         ：通用 Kalman 更新过程里的临时矩阵和向量
 */
static float gEstimateKF_xhat_data[3];
static float gEstimateKF_xhatminus_data[3];
static float gEstimateKF_u_data[1];
static float gEstimateKF_z_data[3];

static float gEstimateKF_P_data[9];
static float gEstimateKF_Pminus_data[9];
static float gEstimateKF_F_data[9];
static float gEstimateKF_FT_data[9];
static float gEstimateKF_B_data[1];
static float gEstimateKF_H_data[9];
static float gEstimateKF_HT_data[9];
static float gEstimateKF_Q_data[9];
static float gEstimateKF_R_data[9];
static float gEstimateKF_K_data[9];
static float gEstimateKF_S_data[9];
static float gEstimateKF_temp_matrix_data[9];
static float gEstimateKF_temp_matrix_data1[9];
static float gEstimateKF_temp_vector_data[3];
static float gEstimateKF_temp_vector_data1[3];

static void GravityKF_LinkKalmanBuffers(void);
static void GravityKF_InitStateFromAccel(float ax_g, float ay_g, float az_g);
static void GravityKF_UpdateF(float gx_radps, float gy_radps, float gz_radps, float dt);
static void GravityKF_UpdateQR(void);
static void GravityKF_SyncOutputFromKalman(void);

void GravityKF_Init(GravityKF_t *kf)
{
    if (kf == 0)
    {
        return;
    }

    /*
     * 清空通用 Kalman 结构体。
     * 注意：清空的是结构体本身，不是下面那些静态数组。
     * 后面 GravityKF_LinkKalmanBuffers() 会重新把数组地址挂进去。
     */
    memset(&gEstimateKF, 0, sizeof(gEstimateKF));

    /*
     * 把上面的 xhat/F/P/Q/R 等数据数组挂到 gEstimateKF 结构体里。
     * 这一步等价于文章里把各种 *_data 指针赋给滤波器结构体。
     */
    GravityKF_LinkKalmanBuffers();

    /*
     * 状态 3 维：[gx gy gz]^T。
     * 控制 0 维：本模型没有控制输入。
     * 测量 3 维：[ax ay az]^T。
     */
    Kalman_Filter_Init(&gEstimateKF, 3u, 0u, 3u);

    /*
     * F 初始为单位阵。
     * 每次 GravityKF_Update() 时会根据 gyro 和 dt 更新 F 的非对角项。
     */
    Kalman_Matrix_SetIdentityData(gEstimateKF_F_data, 3u);

    /*
     * H = I。
     * 因为测量向量 z = [ax ay az]^T 直接观测状态 x = [gx gy gz]^T。
     */
    Kalman_Matrix_SetIdentityData(gEstimateKF_H_data, 3u);

    /*
     * P = P0 * I。
     * P0 大表示初始状态不确定度大，更容易被 accel 修正。
     */
    Kalman_Matrix_SetDiagonalData(gEstimateKF_P_data,
        3u,
        3u,
        gravity_kf_p0,
        0.0f);

    /*
     * Q/R 从全局可调参数写入矩阵。
     */
    GravityKF_UpdateQR();

    /*
     * 默认初始重力方向设为机体 z 轴。
     * 如果第一次进 GravityKF_Update() 时 accel 有效，会用 accel 方向覆盖。
     */
    gEstimateKF_xhat_data[0] = 0.0f;
    gEstimateKF_xhat_data[1] = 0.0f;
    gEstimateKF_xhat_data[2] = 1.0f;

    gVec[0] = gEstimateKF_xhat_data[0];
    gVec[1] = gEstimateKF_xhat_data[1];
    gVec[2] = gEstimateKF_xhat_data[2];

    kf->x = gVec[0];
    kf->y = gVec[1];
    kf->z = gVec[2];
    kf->acc_used = 0u;
    kf->initialized = 0u;
    Kalman_Matrix_CopyData(&kf->p[0][0], gEstimateKF_P_data, 3u, 3u);
}

void GravityKF_Update(GravityKF_t *kf,
                      float ax_g,
                      float ay_g,
                      float az_g,
                      float gx_radps,
                      float gy_radps,
                      float gz_radps,
                      float dt)
{
    float ax;
    float ay;
    float az;
    float acc_norm;

    if ((kf == 0) || (dt <= 0.0f))
    {
        return;
    }

    /*
     * ==================== 1. 首次状态初始化 ====================
     *
     * 第一次用 accel 方向初始化重力方向。
     * 后续每帧由 Kalman_Filter_Update() 推进状态。
     */
    if (!kf->initialized)
    {
        //初始化时提取有效加速度计的单位重力方向向量
        GravityKF_InitStateFromAccel(ax_g, ay_g, az_g);
        kf->initialized = 1u;
    }

    /*
     * ==================== 2. 更新状态转移矩阵 F ====================
     *
     * 这里对应文章里的 F = I + dt * C(wm)。
     */
    GravityKF_UpdateF(gx_radps, gy_radps, gz_radps, dt);

    /*
     * ==================== 3. 更新 Q/R ====================
     *
     * Q/R 是可调全局变量，允许 Watch 或 VOFA 改完后下一帧立即生效。
     */
    GravityKF_UpdateQR();

    ax = ax_g;
    ay = ay_g;
    az = az_g;
    acc_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));

    /*
     * ==================== 4. 更新测量向量 z ====================
     *
     * accel 可信时：
     * - 先归一化，只保留方向。
     * - 写入 gEstimateKF.MeasuredVector。
     * - MeasurementValidNum = 3，表示有 3 个有效测量量。
     *
     * accel 不可信时：
     * - MeasurementValidNum = 0。
     * - 通用 Kalman 只做预测，不做测量修正。
     */
    if ((acc_norm >= gravity_kf_acc_trust_min) &&
        (acc_norm <= gravity_kf_acc_trust_max) &&
        Kalman_Vec3Normalize(&ax, &ay, &az))
    {
        gEstimateKF.MeasuredVector[0] = ax;
        gEstimateKF.MeasuredVector[1] = ay;
        gEstimateKF.MeasuredVector[2] = az;
        gEstimateKF.MeasurementValidNum = 3u;
        kf->acc_used = 1u;
    }
    else
    {
        gEstimateKF.MeasurementValidNum = 0u;
        kf->acc_used = 0u;
    }

    /*
     * ==================== 5. 调用通用 Kalman 更新 ====================
     *
     * 本文件不展开 P、K、S 等矩阵公式。
     * 这些公式统一在 Kalman_Filter_Update() 中完成。
     */
    (void)Kalman_Filter_Update(&gEstimateKF);

    /*
     * ==================== 6. 提取输出 ====================
     *
     * Kalman 输出保存在 gEstimateKF.FilteredValue。
     * 这里同步到 gVec 和 gravity_kf.x/y/z，方便 Mahony 和 VOFA 使用。
     */
    GravityKF_SyncOutputFromKalman();

    kf->x = gVec[0];
    kf->y = gVec[1];
    kf->z = gVec[2];
    Kalman_Matrix_CopyData(&kf->p[0][0], gEstimateKF_P_data, 3u, 3u);
}

static void GravityKF_LinkKalmanBuffers(void)
{
    /*
     * 把所有静态数组挂接到通用 Kalman 结构体。
     *
     * Kalman_Filter_Init() 只负责根据这些指针初始化 mat 结构体；
     * 它不会自己申请内存，所以必须先完成这里的挂接。
     */
    gEstimateKF.xhat_data = gEstimateKF_xhat_data;
    gEstimateKF.xhatminus_data = gEstimateKF_xhatminus_data;
    gEstimateKF.u_data = gEstimateKF_u_data;
    gEstimateKF.z_data = gEstimateKF_z_data;

    gEstimateKF.P_data = gEstimateKF_P_data;
    gEstimateKF.Pminus_data = gEstimateKF_Pminus_data;
    gEstimateKF.F_data = gEstimateKF_F_data;
    gEstimateKF.FT_data = gEstimateKF_FT_data;
    gEstimateKF.B_data = gEstimateKF_B_data;
    gEstimateKF.H_data = gEstimateKF_H_data;
    gEstimateKF.HT_data = gEstimateKF_HT_data;
    gEstimateKF.Q_data = gEstimateKF_Q_data;
    gEstimateKF.R_data = gEstimateKF_R_data;
    gEstimateKF.K_data = gEstimateKF_K_data;
    gEstimateKF.S_data = gEstimateKF_S_data;
    gEstimateKF.temp_matrix_data = gEstimateKF_temp_matrix_data;
    gEstimateKF.temp_matrix_data1 = gEstimateKF_temp_matrix_data1;
    gEstimateKF.temp_vector_data = gEstimateKF_temp_vector_data;
    gEstimateKF.temp_vector_data1 = gEstimateKF_temp_vector_data1;
}


static void GravityKF_InitStateFromAccel(float ax_g, float ay_g, float az_g)
{
    /*
     * 初始化状态时，只使用 accel 的方向，不使用 accel 的大小。
     * 因为状态 x 表示的是重力方向单位向量，不是加速度幅值。
     */
    if (Kalman_Vec3Normalize(&ax_g, &ay_g, &az_g))
    {
        gEstimateKF.FilteredValue[0] = ax_g;
        gEstimateKF.FilteredValue[1] = ay_g;
        gEstimateKF.FilteredValue[2] = az_g;
    }
    else
    {
        gEstimateKF.FilteredValue[0] = 0.0f;
        gEstimateKF.FilteredValue[1] = 0.0f;
        gEstimateKF.FilteredValue[2] = 1.0f;
    }
}

static void GravityKF_UpdateF(float gx_radps, float gy_radps, float gz_radps, float dt)
{
    float gxdt;
    float gydt;
    float gzdt;

    gxdt = GRAVITY_KF_GYRO_SIGN * gx_radps * dt;
    gydt = GRAVITY_KF_GYRO_SIGN * gy_radps * dt;
    gzdt = GRAVITY_KF_GYRO_SIGN * gz_radps * dt;

    /*
     * 这里直接更新通用 Kalman 结构体中的 F 矩阵数据。
     *
     * F =
     * [ 1      wz*dt   -wy*dt
     *  -wz*dt  1        wx*dt
     *   wy*dt -wx*dt    1     ]
     */
    gEstimateKF.F_data[0] = 1.0f;
    gEstimateKF.F_data[1] = gzdt;
    gEstimateKF.F_data[2] = -gydt;

    gEstimateKF.F_data[3] = -gzdt;
    gEstimateKF.F_data[4] = 1.0f;
    gEstimateKF.F_data[5] = gxdt;

    gEstimateKF.F_data[6] = gydt;
    gEstimateKF.F_data[7] = -gxdt;
    gEstimateKF.F_data[8] = 1.0f;
}

static void GravityKF_UpdateQR(void)
{
    /*
     * 把全局参数写入 Q/R 对角阵     
     * 如果你在 Watch VOFA 中修gravity_kf_q / gravity_kf_r     
     * 下一帧调GravityKF_Update() 时会在这里同步到滤波器矩阵    
     */
    Kalman_Matrix_SetDiagonalData(gEstimateKF.Q_data,
        3u,
        3u,
        gravity_kf_q,
        0.0f);
    Kalman_Matrix_SetDiagonalData(gEstimateKF.R_data,
        3u,
        3u,
        gravity_kf_r,
        0.0f);
}

static void GravityKF_SyncOutputFromKalman(void)
{
    /*
     * 通用 Kalman 输出是 gEstimateKF.FilteredValue。
     * 为了让外部使用更直观，这里同步到 gVec。
     */
    gVec[0] = gEstimateKF.FilteredValue[0];
    gVec[1] = gEstimateKF.FilteredValue[1];
    gVec[2] = gEstimateKF.FilteredValue[2];

    /*
     * 状态量是方向向量，更新后必须保持单位长度。
     */
    if (Kalman_Vec3Normalize(&gVec[0], &gVec[1], &gVec[2]))
    {
        gEstimateKF.FilteredValue[0] = gVec[0];
        gEstimateKF.FilteredValue[1] = gVec[1];
        gEstimateKF.FilteredValue[2] = gVec[2];
    }
}
