#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <stdint.h>

/*
 * 没有 CMSIS DSP 时，用 kalman.c 里的矩阵函数模拟 CMSIS DSP 接口。
 * 后续如果引入 CMSIS DSP，只需要把这些宏切换到 arm_mat_xxx_f32 即可。
 */
typedef struct
{
    /* 矩阵行数。 */
    uint16_t numRows;

    /* 矩阵列数。 */
    uint16_t numCols;

    /* 矩阵数据首地址，按行优先存储：data[row * numCols + col]。 */
    float *pData;
} KalmanMatrix_t;

#define mat                 KalmanMatrix_t
#define Matrix_Init         Kalman_Matrix_Init
#define Matrix_Add          Kalman_Matrix_Add
#define Matrix_Subtract     Kalman_Matrix_Subtract
#define Matrix_Multiply     Kalman_Matrix_Multiply
#define Matrix_Transpose    Kalman_Matrix_Transpose
#define Matrix_Inverse      Kalman_Matrix_Inverse

typedef struct kf_t
{
    /* 后验估计值 x(k|k)，指向 xhat_data。 */
    float *FilteredValue;

    /* 测量向量 z，指向 z_data。 */
    float *MeasuredVector;

    /* 控制输入 u，指向 u_data；没有控制输入时可以不用。 */
    float *ControlVector;

    /* 状态向量维度。 */
    uint8_t xhatSize;

    /* 控制向量维度；没有控制输入时传 0。 */
    uint8_t uSize;

    /* 测量向量维度。 */
    uint8_t zSize;

    /*
     * 有效测量数量。
     * 置 0 时，本次只做预测，不做测量修正。
     */
    uint8_t MeasurementValidNum;

    mat xhat;           /* 后验估计 x(k|k)。 */
    mat xhatminus;      /* 先验估计 x(k|k-1)。 */
    mat u;              /* 控制输入 u。 */
    mat z;              /* 测量向量 z。 */
    mat P;              /* 后验协方差 P(k|k)。 */
    mat Pminus;         /* 先验协方差 P(k|k-1)。 */
    mat F;              /* 状态转移矩阵 F。 */
    mat FT;             /* F 的转置。 */
    mat B;              /* 控制矩阵 B。 */
    mat H;              /* 测量矩阵 H。 */
    mat HT;             /* H 的转置。 */
    mat Q;              /* 过程噪声矩阵 Q。 */
    mat R;              /* 测量噪声矩阵 R。 */
    mat K;              /* Kalman 增益 K。 */
    mat S;              /* 残差协方差 S。 */
    mat temp_matrix;    /* 通用临时矩阵。 */
    mat temp_matrix1;   /* 通用临时矩阵，更新 P 时也用作单位阵 I。 */
    mat temp_vector;    /* 通用临时向量。 */
    mat temp_vector1;   /* 通用临时向量，常用于残差 y。 */

    /* 最近一次矩阵运算状态，0 表示成功，负数表示失败。 */
    int8_t MatStatus;

    /*
     * 用户回调函数。
     *
     * User_Func0_f：预测前调用，适合更新时变 F、B、Q。
     * User_Func1_f：协方差预测后、测量更新前调用，适合调整 R 或测量有效性。
     * User_Func2_f：完整更新后调用，适合归一化状态或做输出限制。
     * 其余函数指针预留给更复杂模型扩展。
     */
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    /*
     * 矩阵数据区指针。
     *
     * 这些指针由具体业务模块提供实际数组，例如：
     *     static float my_kf_F_data[9];
     *
     * 然后在调用 Kalman_Filter_Init() 前把指针挂到结构体里。
     */
    float *xhat_data;
    float *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data;
    float *Pminus_data;
    float *F_data;
    float *FT_data;
    float *B_data;
    float *H_data;
    float *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data;
    float *temp_matrix_data;
    float *temp_matrix_data1;
    float *temp_vector_data;
    float *temp_vector_data1;
} KalmanFilter_t;

/*
 * @brief 初始化矩阵对象。
 * @param m 矩阵对象指针。
 * @param rows 矩阵行数。
 * @param cols 矩阵列数。
 * @param data 矩阵数据首地址，按行优先存储。
 */
void Kalman_Matrix_Init(mat *m, uint16_t rows, uint16_t cols, float *data);

/*
 * @brief 矩阵加法：out = a + b。
 * @param a 左输入矩阵。
 * @param b 右输入矩阵。
 * @param out 输出矩阵，尺寸必须和 a、b 相同。
 * @return 0 表示成功，负数表示参数或尺寸错误。
 */
int8_t Kalman_Matrix_Add(const mat *a, const mat *b, mat *out);

/*
 * @brief 矩阵减法：out = a - b。
 * @param a 左输入矩阵。
 * @param b 右输入矩阵。
 * @param out 输出矩阵，尺寸必须和 a、b 相同。
 * @return 0 表示成功，负数表示参数或尺寸错误。
 */
int8_t Kalman_Matrix_Subtract(const mat *a, const mat *b, mat *out);

/*
 * @brief 矩阵乘法：out = a * b。
 * @param a 左输入矩阵。
 * @param b 右输入矩阵。
 * @param out 输出矩阵，尺寸必须为 a->numRows x b->numCols。
 * @return 0 表示成功，负数表示参数、尺寸或临时缓存不足。
 */
int8_t Kalman_Matrix_Multiply(const mat *a, const mat *b, mat *out);

/*
 * @brief 矩阵转置：out = a^T。
 * @param a 输入矩阵。
 * @param out 输出矩阵，尺寸必须为 a->numCols x a->numRows。
 * @return 0 表示成功，负数表示参数或尺寸错误。
 */
int8_t Kalman_Matrix_Transpose(const mat *a, mat *out);

/*
 * @brief 小矩阵求逆。
 * @param a 输入方阵，目前支持 1x1、2x2、3x3。
 * @param out 输出逆矩阵，尺寸必须和 a 相同。
 * @return 0 表示成功，负数表示参数、尺寸错误或矩阵不可逆。
 */
int8_t Kalman_Matrix_Inverse(const mat *a, mat *out);

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
                            float value);

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
                            uint16_t cols);

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
                                   float other_value);

/*
 * @brief 设置单位阵。
 * @param data 方阵数据首地址。
 * @param size 方阵阶数。
 */
void Kalman_Matrix_SetIdentityData(float *data, uint16_t size);

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
                        uint8_t zSize);

/*
 * @brief 执行一次通用 Kalman 滤波更新。
 * @param kf Kalman 滤波器结构体指针。
 * @return 后验估计值数组首地址，也就是 kf->FilteredValue。
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf);

/*
 * @brief 3 维向量归一化。
 * @param x 向量 x 分量指针，函数会原地修改。
 * @param y 向量 y 分量指针，函数会原地修改。
 * @param z 向量 z 分量指针，函数会原地修改。
 * @return 1 表示成功，0 表示向量模长太小。
 */
uint8_t Kalman_Vec3Normalize(float *x, float *y, float *z);

#endif
