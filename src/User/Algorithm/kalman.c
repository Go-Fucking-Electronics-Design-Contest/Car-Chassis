#include "kalman.h"
#include <math.h>
#include <string.h>

/*
 * 防止0 或矩阵求逆时除以非常小的数
 */
#define KALMAN_EPS          1.0e-8f

static void Kalman_CopyMatrix(const mat *src, mat *dst);


/*
 * @brief 3x3 矩阵求逆：invA = inverse(A)。
 * @param A 输入 3x3 矩阵。
 * @param invA 输出逆矩阵。
 * @return 1 表示求逆成功，0 表示矩阵接近奇异或不可逆。
 */
static uint8_t mat33_inverse(const float *A, float *invA)
{
    float det;

    /*
     * 计算 3x3 矩阵行列式
     */
    det = A[0] * (A[4] * A[8] - A[5] * A[7])
        - A[1] * (A[3] * A[8] - A[5] * A[6])
        + A[2] * (A[3] * A[7] - A[4] * A[6]);

    /*
     * 行列式太小，说明矩阵不可逆或数值不稳定
     */
    if (fabsf(det) < KALMAN_EPS)
    {
        return 0;
    }

    det = 1.0f / det;

    /*
     * 伴随矩阵 / 行列式
     */
    invA[0] =  (A[4] * A[8] - A[5] * A[7]) * det;
    invA[1] = -(A[1] * A[8] - A[2] * A[7]) * det;
    invA[2] =  (A[1] * A[5] - A[2] * A[4]) * det;

    invA[3] = -(A[3] * A[8] - A[5] * A[6]) * det;
    invA[4] =  (A[0] * A[8] - A[2] * A[6]) * det;
    invA[5] = -(A[0] * A[5] - A[2] * A[3]) * det;

    invA[6] =  (A[3] * A[7] - A[4] * A[6]) * det;
    invA[7] = -(A[0] * A[7] - A[1] * A[6]) * det;
    invA[8] =  (A[0] * A[4] - A[1] * A[3]) * det;

    return 1;
}


/*
 * @brief 将 3 维向量原地归一化。
 * @param x 向量 x 分量指针。
 * @param y 向量 y 分量指针。
 * @param z 向量 z 分量指针。
 * @return 1 表示成功，0 表示模长太小。
 */
static uint8_t vec3_normalize(float *x, float *y, float *z)
{
    float norm;

    norm = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));

    if (norm < KALMAN_EPS)
    {
        return 0;
    }

    *x /= norm;
    *y /= norm;
    *z /= norm;

    return 1;
}


/*
 * @brief 3 维向量归一化。
 * @param x 向量 x 分量指针，函数会原地修改。
 * @param y 向量 y 分量指针，函数会原地修改。
 * @param z 向量 z 分量指针，函数会原地修改。
 * @return 1 表示成功，0 表示向量模长太小。
 */
uint8_t Kalman_Vec3Normalize(float *x, float *y, float *z)
{
    return vec3_normalize(x, y, z);
}

/*
 * @brief 初始化矩阵对象。
 * @param m 矩阵对象指针。
 * @param rows 矩阵行数。
 * @param cols 矩阵列数。
 * @param data 矩阵数据首地址，按行优先存储。
 */
void Kalman_Matrix_Init(mat *m, uint16_t rows, uint16_t cols, float *data)
{
    if (m == 0)
    {
        return;
    }

    /*
     * 这个函数等价于 CMSIS DSP 的 arm_mat_init_f32()。
     * 它不分配内存，只把外部已经准备好的 float 数组挂到矩阵实例上。
     */
    m->numRows = rows;
    m->numCols = cols;
    m->pData = data;
}

/*
 * @brief 矩阵加法：out = a + b。
 * @param a 左输入矩阵。
 * @param b 右输入矩阵。
 * @param out 输出矩阵，尺寸必须和 a、b 相同。
 * @return 0 表示成功，负数表示参数或尺寸错误。
 */
int8_t Kalman_Matrix_Add(const mat *a, const mat *b, mat *out)
{
    uint16_t i;
    uint16_t count;

    if ((a == 0) || (b == 0) || (out == 0) ||
        (a->pData == 0) || (b->pData == 0) || (out->pData == 0) ||
        (a->numRows != b->numRows) || (a->numCols != b->numCols) ||
        (out->numRows != a->numRows) || (out->numCols != a->numCols))
    {
        return -1;
    }

    count = (uint16_t)(a->numRows * a->numCols);
    for (i = 0u; i < count; i++)
    {
        out->pData[i] = a->pData[i] + b->pData[i];
    }

    return 0;
}

/*
 * @brief 矩阵减法：out = a - b。
 * @param a 左输入矩阵。
 * @param b 右输入矩阵。
 * @param out 输出矩阵，尺寸必须和 a、b 相同。
 * @return 0 表示成功，负数表示参数或尺寸错误。
 */
int8_t Kalman_Matrix_Subtract(const mat *a, const mat *b, mat *out)
{
    uint16_t i;
    uint16_t count;

    if ((a == 0) || (b == 0) || (out == 0) ||
        (a->pData == 0) || (b->pData == 0) || (out->pData == 0) ||
        (a->numRows != b->numRows) || (a->numCols != b->numCols) ||
        (out->numRows != a->numRows) || (out->numCols != a->numCols))
    {
        return -1;
    }

    count = (uint16_t)(a->numRows * a->numCols);
    for (i = 0u; i < count; i++)
    {
        out->pData[i] = a->pData[i] - b->pData[i];
    }

    return 0;
}

/*
 * @brief 矩阵乘法：out = a * b。
 * @param a 左输入矩阵。
 * @param b 右输入矩阵。
 * @param out 输出矩阵，尺寸必须为 a->numRows x b->numCols。
 * @return 0 表示成功，负数表示参数、尺寸或临时缓存不足。
 */
int8_t Kalman_Matrix_Multiply(const mat *a, const mat *b, mat *out)
{
    uint16_t r;
    uint16_t c;
    uint16_t n;
    uint16_t out_count;
    /*
     * 临时数组用于支持 out 和 a/b 使用同一块内存的情况。
     * 当前工程只用 3x3、3x1 这类小矩阵，36 个 float 足够覆盖 6x6 以内结果。
     */
    float temp[36];

    if ((a == 0) || (b == 0) || (out == 0) ||
        (a->pData == 0) || (b->pData == 0) || (out->pData == 0) ||
        (a->numCols != b->numRows) ||
        (out->numRows != a->numRows) || (out->numCols != b->numCols))
    {
        return -1;
    }

    out_count = (uint16_t)(out->numRows * out->numCols);
    if (out_count > (uint16_t)(sizeof(temp) / sizeof(temp[0])))
    {
        return -1;
    }

    for (r = 0u; r < out->numRows; r++)
    {
        for (c = 0u; c < out->numCols; c++)
        {
            temp[(uint16_t)(r * out->numCols + c)] = 0.0f;
            for (n = 0u; n < a->numCols; n++)
            {
                temp[(uint16_t)(r * out->numCols + c)] +=
                    a->pData[(uint16_t)(r * a->numCols + n)] *
                    b->pData[(uint16_t)(n * b->numCols + c)];
            }
        }
    }

    memcpy(out->pData, temp, (size_t)out_count * sizeof(float));
    return 0;
}

/*
 * @brief 矩阵转置：out = a^T。
 * @param a 输入矩阵。
 * @param out 输出矩阵，尺寸必须为 a->numCols x a->numRows。
 * @return 0 表示成功，负数表示参数或尺寸错误。
 */
int8_t Kalman_Matrix_Transpose(const mat *a, mat *out)
{
    uint16_t r;
    uint16_t c;

    if ((a == 0) || (out == 0) ||
        (a->pData == 0) || (out->pData == 0) ||
        (out->numRows != a->numCols) || (out->numCols != a->numRows))
    {
        return -1;
    }

    for (r = 0u; r < a->numRows; r++)
    {
        for (c = 0u; c < a->numCols; c++)
        {
            out->pData[(uint16_t)(c * out->numCols + r)] =
                a->pData[(uint16_t)(r * a->numCols + c)];
        }
    }

    return 0;
}

/*
 * @brief 小矩阵求逆。
 * @param a 输入方阵，目前支持 1x1、2x2、3x3。
 * @param out 输出逆矩阵，尺寸必须和 a 相同。
 * @return 0 表示成功，负数表示参数、尺寸错误或矩阵不可逆。
 */
int8_t Kalman_Matrix_Inverse(const mat *a, mat *out)
{
    float det;

    if ((a == 0) || (out == 0) ||
        (a->pData == 0) || (out->pData == 0) ||
        (a->numRows != a->numCols) ||
        (out->numRows != a->numRows) || (out->numCols != a->numCols))
    {
        return -1;
    }

    if (a->numRows == 1u)
    {
        if (fabsf(a->pData[0]) < KALMAN_EPS)
        {
            return -1;
        }

        out->pData[0] = 1.0f / a->pData[0];
        return 0;
    }

    if (a->numRows == 2u)
    {
        det = (a->pData[0] * a->pData[3]) - (a->pData[1] * a->pData[2]);
        if (fabsf(det) < KALMAN_EPS)
        {
            return -1;
        }

        out->pData[0] = a->pData[3] / det;
        out->pData[1] = -a->pData[1] / det;
        out->pData[2] = -a->pData[2] / det;
        out->pData[3] = a->pData[0] / det;
        return 0;
    }

    if (a->numRows == 3u)
    {
        return mat33_inverse(a->pData, out->pData) ? 0 : -1;
    }

    return -1;
}

/*
 * @brief 将一维矩阵数据全部填成同一个值。
 * @param data 矩阵数据首地址。
 * @param rows 矩阵行数。
 * @param cols 矩阵列数。
 * @param value 要写入的值。
 */
void Kalman_Matrix_FillData(float *data,
                            uint16_t rows,
                            uint16_t cols,
                            float value)
{
    uint16_t i;
    uint16_t count;

    if (data == 0)
    {
        return;
    }

    count = (uint16_t)(rows * cols);
    for (i = 0u; i < count; i++)
    {
        data[i] = value;
    }
}

/*
 * @brief 拷贝一维矩阵数据。
 * @param dst 目标矩阵数据首地址。
 * @param src 源矩阵数据首地址。
 * @param rows 矩阵行数。
 * @param cols 矩阵列数。
 */
void Kalman_Matrix_CopyData(float *dst,
                            const float *src,
                            uint16_t rows,
                            uint16_t cols)
{
    uint16_t count;

    if ((dst == 0) || (src == 0))
    {
        return;
    }

    count = (uint16_t)(rows * cols);
    memcpy(dst, src, (size_t)count * sizeof(float));
}

/*
 * @brief 设置矩阵的对角线和非对角线元素。
 * @param data 矩阵数据首地址。
 * @param rows 矩阵行数。
 * @param cols 矩阵列数。
 * @param diagonal_value 对角线元素值。
 * @param other_value 非对角线元素值。
 */
void Kalman_Matrix_SetDiagonalData(float *data,
                                   uint16_t rows,
                                   uint16_t cols,
                                   float diagonal_value,
                                   float other_value)
{
    uint16_t r;
    uint16_t c;

    if (data == 0)
    {
        return;
    }

    for (r = 0u; r < rows; r++)
    {
        for (c = 0u; c < cols; c++)
        {
            data[(uint16_t)(r * cols + c)] =
                (r == c) ? diagonal_value : other_value;
        }
    }
}

/*
 * @brief 设置单位阵。
 * @param data 方阵数据首地址。
 * @param size 方阵阶数。
 */
void Kalman_Matrix_SetIdentityData(float *data, uint16_t size)
{
    Kalman_Matrix_SetDiagonalData(data, size, size, 1.0f, 0.0f);
}

/*
 * @brief 初始化通用 Kalman 滤波器结构体。
 * @param kf Kalman 滤波器结构体指针。
 * @param xhatSize 状态向量维度。
 * @param uSize 控制向量维度；没有控制输入时传 0。
 * @param zSize 测量向量维度。
 */
void Kalman_Filter_Init(KalmanFilter_t *kf,
                        uint8_t xhatSize,
                        uint8_t uSize,
                        uint8_t zSize)
{
    if (kf == 0)
    {
        return;
    }

    /*
     * 记录滤波器维度。
     *
     * xhatSize 是状态维度，例如 GravityKF 的 x = [gx gy gz]^T，所以是 3。
     * uSize 是控制输入维度；GravityKF 没有控制输入，所以是 0。
     * zSize 是测量维度，例如 accel 测量 z = [ax ay az]^T，所以是 3。
     */
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;
    kf->MeasurementValidNum = zSize;

    /*
     * 建立对外便捷指针。
     * 业务代码可以直接读写 FilteredValue / MeasuredVector / ControlVector，
     * 不需要关心底层矩阵结构体。
     */
    kf->FilteredValue = kf->xhat_data;
    kf->MeasuredVector = kf->z_data;
    kf->ControlVector = kf->u_data;

    /*
     * 初始化所有矩阵对象。
     * 这里仅绑定行列数和数据指针，不会清零数据。
     * 初始值由业务模块自己写入，例如 gravity_kf.c 写入 F/P/Q/R/H。
     */
    Matrix_Init(&kf->xhat, xhatSize, 1u, kf->xhat_data);
    Matrix_Init(&kf->xhatminus, xhatSize, 1u, kf->xhatminus_data);
    Matrix_Init(&kf->u, uSize, 1u, kf->u_data);
    Matrix_Init(&kf->z, zSize, 1u, kf->z_data);

    Matrix_Init(&kf->P, xhatSize, xhatSize, kf->P_data);
    Matrix_Init(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);
    Matrix_Init(&kf->F, xhatSize, xhatSize, kf->F_data);
    Matrix_Init(&kf->FT, xhatSize, xhatSize, kf->FT_data);
    Matrix_Init(&kf->B, xhatSize, uSize, kf->B_data);
    Matrix_Init(&kf->H, zSize, xhatSize, kf->H_data);
    Matrix_Init(&kf->HT, xhatSize, zSize, kf->HT_data);
    Matrix_Init(&kf->Q, xhatSize, xhatSize, kf->Q_data);
    Matrix_Init(&kf->R, zSize, zSize, kf->R_data);
    Matrix_Init(&kf->K, xhatSize, zSize, kf->K_data);
    Matrix_Init(&kf->S, zSize, zSize, kf->S_data);

    Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize, kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize, kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, xhatSize, 1u, kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, zSize, 1u, kf->temp_vector_data1);

    kf->MatStatus = 0;
}

/*
 * @brief 执行一次通用 Kalman 滤波更新。
 * @param kf Kalman 滤波器结构体指针。
 * @return 后验估计值数组首地址，也就是 kf->FilteredValue。
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    float hpminus_data[36];
    float s_inv_data[36];
    mat hpminus;
    mat s_inv;
    uint16_t hpminus_count;
    uint16_t s_count;

    if (kf == 0)
    {
        return 0;
    }

    /*
     * hpminus = H * Pminus，尺寸是 zSize x xhatSize。
     * s_inv = inv(S)，尺寸是 zSize x zSize。
     *
     * 这两个局部矩阵让通用 KF 支持“状态维度 != 测量维度”的模型。
     * 当前实现面向单片机小矩阵，36 个 float 覆盖 6x6 以内的中间结果。
     */
    hpminus_count = (uint16_t)(kf->zSize * kf->xhatSize);
    s_count = (uint16_t)(kf->zSize * kf->zSize);
    if ((hpminus_count > (uint16_t)(sizeof(hpminus_data) / sizeof(hpminus_data[0]))) ||
        (s_count > (uint16_t)(sizeof(s_inv_data) / sizeof(s_inv_data[0]))))
    {
        kf->MatStatus = -1;
        return kf->FilteredValue;
    }

    Matrix_Init(&hpminus, kf->zSize, kf->xhatSize, hpminus_data);
    Matrix_Init(&s_inv, kf->zSize, kf->zSize, s_inv_data);

    /*
     * User_Func0_f：预测前回调。
     *
     * 适合做这些事：
     * - 根据 gyro 更新时变状态转移矩阵 F。
     * - 根据 dt 或传感器状态更新 Q。
     * - 更新控制矩阵 B。
     *
     * GravityKF 当前没有用回调，而是在调用本函数前直接更新 F/Q/R。
     */
    if (kf->User_Func0_f != 0)
    {
        kf->User_Func0_f(kf);
    }

    /*
     * ==================== 1. 状态预测 ====================
     *
     * xhatminus = F * xhat
     *
     * xhat      = 上一时刻后验估计 x(k-1|k-1)
     * xhatminus = 当前时刻先验估计 x(k|k-1)
     */
    kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
    if (kf->MatStatus != 0)
    {
        return kf->FilteredValue;
    }

    /*
     * 如果存在控制输入，则：
     *
     * xhatminus = F * xhat + B * u
     *
     * GravityKF 没有控制输入，所以 uSize = 0，会跳过这一步。
     */
    if ((kf->uSize > 0u) && (kf->B.pData != 0) && (kf->u.pData != 0))
    {
        kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector);
        if (kf->MatStatus != 0)
        {
            return kf->FilteredValue;
        }

        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhatminus);
        if (kf->MatStatus != 0)
        {
            return kf->FilteredValue;
        }
    }

    /*
     * ==================== 2. 协方差预测 ====================
     *
     * Pminus = F * P * F^T + Q
     *
     * P      = 上一时刻后验协方差 P(k-1|k-1)。
     * Pminus = 当前时刻先验协方差 P(k|k-1)。
     * Q      = 过程噪声，表示模型预测本身的不确定性。
     */
    Matrix_Transpose(&kf->F, &kf->FT);
    Matrix_Multiply(&kf->F, &kf->P, &kf->temp_matrix);
    Matrix_Multiply(&kf->temp_matrix, &kf->FT, &kf->Pminus);
    Matrix_Add(&kf->Pminus, &kf->Q, &kf->Pminus);

    /*
     * User_Func1_f：预测完成、测量更新前回调。
     *
     * 适合做这些事：
     * - 根据传感器可信度调整 R。
     * - 判断测量是否有效。
     * - 修改 H 或 z。
     */
    if (kf->User_Func1_f != 0)
    {
        kf->User_Func1_f(kf);
    }

    /*
     * ==================== 3. 测量有效性判断 ====================
     *
     * MeasurementValidNum = 0 表示本次没有有效测量。
     * 例如 GravityKF 中 accel 模长明显偏离 1g 时，就只做 gyro 预测。
     */
    if (kf->MeasurementValidNum == 0u)
    {
        Kalman_CopyMatrix(&kf->xhatminus, &kf->xhat);
        Kalman_CopyMatrix(&kf->Pminus, &kf->P);
        return kf->FilteredValue;
    }

    /*
     * ==================== 4. 残差协方差 ====================
     *
     * S = H * Pminus * H^T + R
     *
     * H = 测量矩阵，把状态空间映射到测量空间。
     * R = 测量噪声，表示测量值自身的不确定性。
     */
    Matrix_Transpose(&kf->H, &kf->HT);
    Matrix_Multiply(&kf->H, &kf->Pminus, &hpminus);
    Matrix_Multiply(&hpminus, &kf->HT, &kf->S);
    Matrix_Add(&kf->S, &kf->R, &kf->S);

    if (Matrix_Inverse(&kf->S, &s_inv) != 0)
    {
        Kalman_CopyMatrix(&kf->xhatminus, &kf->xhat);
        Kalman_CopyMatrix(&kf->Pminus, &kf->P);
        return kf->FilteredValue;
    }

    /*
     * ==================== 5. Kalman 增益 ====================
     *
     * K = Pminus * H^T * inv(S)
     *
     * K 决定“预测”和“测量”各占多少权重。
     * R 大时，K 变小，更相信预测。
     * Pminus 大时，K 变大，更愿意接受测量修正。
     */
    Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->K);
    Matrix_Multiply(&kf->K, &s_inv, &kf->K);

    /*
     * ==================== 6. 测量残差 ====================
     *
     * y = z - H * xhatminus
     *
     * z 是实际测量值。
     * H*xhatminus 是根据预测状态推算出的“应该测到的值”。
     * 二者差值就是本次测量对预测的修正量。
     */
    Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector1);
    Matrix_Subtract(&kf->z, &kf->temp_vector1, &kf->temp_vector1);

    /*
     * ==================== 7. 状态修正 ====================
     *
     * xhat = xhatminus + K * y
     */
    Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);
    Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);

    /*
     * ==================== 8. 协方差修正 ====================
     *
     * P = (I - K * H) * Pminus
     *
     * temp_matrix1 复用为单位阵 I，避免额外占用 RAM。
     */
    Kalman_Matrix_SetIdentityData(kf->temp_matrix1.pData, kf->xhatSize);
    Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);
    Matrix_Subtract(&kf->temp_matrix1, &kf->temp_matrix, &kf->temp_matrix);
    Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->P);

    /*
     * User_Func2_f：完整更新后回调。
     *
     * 适合做这些事：
     * - 对状态向量归一化。
     * - 对状态做限幅。
     * - 同步输出变量。
     */
    if (kf->User_Func2_f != 0)
    {
        kf->User_Func2_f(kf);
    }

    return kf->FilteredValue;
}

/*
 * @brief 拷贝矩阵对象中的数据。
 * @param src 源矩阵对象。
 * @param dst 目标矩阵对象。
 *
 * 该函数只复制数据区，不修改矩阵对象的行列数。
 */
static void Kalman_CopyMatrix(const mat *src, mat *dst)
{
    uint16_t count;

    if ((src == 0) || (dst == 0) || (src->pData == 0) || (dst->pData == 0))
    {
        return;
    }

    count = (uint16_t)(src->numRows * src->numCols);
    memcpy(dst->pData, src->pData, (size_t)count * sizeof(float));
}
