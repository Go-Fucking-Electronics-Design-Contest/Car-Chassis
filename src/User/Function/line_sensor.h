#ifndef LINE_SENSOR_H_
#define LINE_SENSOR_H_

#include <stdint.h>

#include "pid.h"

/*
 * 感为无 MCU 八路灰度传感器模块。
 *
 * 当前接线按官方 MSPM0 ADC DMA 例程整理：
 * - ADDR0 -> PB0
 * - ADDR1 -> PB1
 * - ADDR2 -> PB2
 * - OUT   -> PA27 / ADC12_0 MEM0
 * - VCC   -> 5V
 * - GND   -> 控制板 GND
 *
 * 这个模块只负责：
 * - 选择 74HC4051 的 8 个模拟通道
 * - 使用 ADC + DMA 采集每一路灰度值
 * - 根据黑白校准值做归一化
 * - 计算循迹误差 error
 *
 * 电机 PID、循迹状态机、转向策略不要写在这里，后续应放到更上层的
 * line_follow / car_control 模块中。
 */

#define LINE_SENSOR_CHANNEL_COUNT       (8u)
#define LINE_SENSOR_ADC_MAX_VALUE       (4095u)

typedef struct
{
    /* 8 路原始 ADC 值，范围 0~4095。 */
    uint16_t raw[LINE_SENSOR_CHANNEL_COUNT];

    /* 归一化白度值：黑色接近 0，白色接近 4095。 */
    uint16_t normalized[LINE_SENSOR_CHANNEL_COUNT];

    /* 黑线强度：黑色越明显，数值越大，用于计算循迹误差。 */
    uint16_t black_strength[LINE_SENSOR_CHANNEL_COUNT];

    /* 白底校准值，每一路独立保存。 */
    uint16_t white[LINE_SENSOR_CHANNEL_COUNT];

    /* 黑线校准值，每一路独立保存。 */
    uint16_t black[LINE_SENSOR_CHANNEL_COUNT];

    /* 二值化阈值：高于 white_threshold 认为偏白。 */
    uint16_t white_threshold[LINE_SENSOR_CHANNEL_COUNT];

    /* 二值化阈值：低于 black_threshold 认为偏黑。 */
    uint16_t black_threshold[LINE_SENSOR_CHANNEL_COUNT];

    /*
     * 归一化比例，Q16 定点格式。
     * 使用定点数是为了减少 Cortex-M0+ 上的浮点计算开销。
     */
    uint32_t normal_factor_q16[LINE_SENSOR_CHANNEL_COUNT];

    /* bit=1 表示该通道判断为白底。 */
    uint8_t white_mask;

    /* bit=1 表示该通道判断为黑线。 */
    uint8_t black_mask;

    /*
     * 循迹误差。
     * 左侧为负，右侧为正，黑线在中心时接近 0。
     */
    int32_t error;

    /* 上一次有效误差，丢线时用于判断应该往哪边找线。 */
    int32_t last_error;

    /* 1 表示当前没有检测到有效黑线。 */
    uint8_t line_lost;

    /* 1 表示校准参数有效。 */
    uint8_t calibrated;
} LineSensor_t;



extern const uint16_t line_sensor_default_white[LINE_SENSOR_CHANNEL_COUNT];
extern const uint16_t line_sensor_default_black[LINE_SENSOR_CHANNEL_COUNT];
extern LineSensor_t line_sensor;
/*
 * 全局灰度传感器对象。
 * 需要在 Watch 里观察时，可以直接看 line_sensor.raw / line_sensor.error。
 */
extern LineSensor_t line_sensor;

extern const float line_sensor_pid_params[PID_PARAM_COUNT];

/*
 * @brief 初始化灰度传感器模块。
 * @param sensor 灰度传感器对象指针。
 *
 * 函数会写入默认黑白校准值，并配置 ADC DMA 的源地址、目标地址和搬运长度。
 */
void LineSensor_Init(LineSensor_t *sensor);

/*
 * @brief 设置 8 路黑白校准值。
 * @param sensor 灰度传感器对象指针。
 * @param white 白底校准值数组，长度为 8。
 * @param black 黑线校准值数组，长度为 8。
 *
 * white 和 black 允许传反，函数内部会自动交换，保证 white >= black。
 */
void LineSensor_SetCalibration(LineSensor_t *sensor,
                               const uint16_t *white,
                               const uint16_t *black);

/*
 * @brief 采集 8 路灰度并更新所有结果。
 * @param sensor 灰度传感器对象指针。
 * @return 1 表示更新成功，0 表示参数错误或 ADC DMA 超时。
 *
 * 成功后会更新：
 * - raw[]
 * - normalized[]
 * - black_strength[]
 * - white_mask / black_mask
 * - error / line_lost
 */
uint8_t LineSensor_Update(LineSensor_t *sensor);

/*
 * @brief 只采集 8 路原始 ADC 值。
 * @param sensor 灰度传感器对象指针。
 * @return 1 表示采集成功，0 表示参数错误或 ADC DMA 超时。
 *
 * 这个函数只更新 raw[]，不计算二值化、归一化和循迹误差。
 */
uint8_t LineSensor_ReadRaw(LineSensor_t *sensor);

/*
 * @brief 灰度 ADC DMA 完成中断处理函数。
 *
 * 这个函数由工程里的 DMA_IRQHandler() 在收到 DMA_CH0 TC 中断时调用。
 * 函数内部会停止 ADC/DMA、计算当前 40 个 ADC 样本平均值，并置位完成标志。
 */
void LineSensor_DMA_IRQHandler(void);

#endif
