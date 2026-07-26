#ifndef PID_H_
#define PID_H_

#include <stdint.h>

/*
 * 通用 PID 控制器。
 *
 * 这个模块不绑定灰度传感器、电机或 IMU。
 * 循迹误差、速度误差、角度误差都可以复用同一个 PID_t。
 */
typedef struct
{
    /* 比例、积分、微分参数。 */
    float kp;
    float ki;
    float kd;

    /* 积分累计值。 */
    float integral;

    /* 上一次误差，用于计算微分项。 */
    float last_error;
    float error;
    /* 最近一次 PID 输出，方便 Watch 观察。 */
    float output;

    /* 最近一次 P/I/D 分量，方便调参时观察每一项贡献。 */
    float p_out;
    float i_out;
    float d_out;

    /*
     * 误差死区。
     * abs(error) <= deadband 时，本次误差按 0 处理。
     */
    float deadband;

    /* 输出限幅。 */
    float output_min;
    float output_max;

    /* 积分限幅，限制 integral 本身，避免积分项无限累计。 */
    float integral_min;
    float integral_max;

    /*
     * 积分饱和保护。
     * 1：输出已经到限幅时，如果误差还在把输出继续推向饱和方向，则暂停积分。
     * 0：只做 integral_min/integral_max 限幅，不做额外抗饱和判断。
     */
    uint8_t integral_anti_windup;

    /*
     * 首次运行标志。
     * 首次运行时没有 last_error，微分项强制为 0，避免上电瞬间微分冲击。
     */
    uint8_t first_update;
} PID_t;

typedef enum
{
    PID_PARAM_KP_INDEX = 0,
    PID_PARAM_KI_INDEX,
    PID_PARAM_KD_INDEX,
    PID_PARAM_OUTPUT_LIMIT_INDEX,
    PID_PARAM_INTEGRAL_LIMIT_INDEX,
    PID_PARAM_DEADBAND_INDEX,
    PID_PARAM_ANTI_WINDUP_INDEX,
    PID_PARAM_COUNT
} PID_Param_Index_t;

/*
 * @brief 初始化 PID 运行状态。
 * @param pid PID 控制器对象指针。
 *
 * 这个函数只清空 integral、last_error、output、P/I/D 分量。
 * PID 参数、限幅、死区由 PID_SetParams() 设置。
 */
void PID_Init(PID_t *pid);

/*
 * @brief 只清空积分项。
 * @param pid PID 控制器对象指针。
 */
void PID_ClearIntegral(PID_t *pid);

/*
 * @brief 设置 PID 参数。
 * @param pid PID 控制器对象指针。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @param output_limit_abs 输出限幅绝对值。
 * @param integral_limit_abs 积分限幅绝对值。
 * @param deadband 误差死区，传入负数时会自动取正。
 * @param integral_anti_windup 1 开启积分饱和保护，0 关闭。
 */
void PID_SetParams(PID_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_limit_abs,
                   float integral_limit_abs,
                   float deadband,
                   uint8_t integral_anti_windup);

/*
 * @brief 计算一次 PID 输出。
 * @param pid PID 控制器对象指针。
 * @param target 目标值。
 * @param feedback 当前反馈值。
 * @param dt_s 两次计算之间的时间，单位秒。
 * @return PID 输出。
 *
 * error = target - feedback。
 * 输出限幅、积分限幅和死区由 PID_SetParams() 设置。
 */
float pid_clac(PID_t *pid,
               float target,
               float feedback,
               float dt_s);

#endif
