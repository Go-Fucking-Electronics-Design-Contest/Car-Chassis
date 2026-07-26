#include "line_sensor.h"
#include "ti_msp_dl_config.h"

#include <string.h>

/*
 * ==================== 灰度传感器常用配置区 ====================
 */

/*
 * 默认校准值。
 * 这组值只用于让程序先跑起来，实车循迹前建议重新采集白底和黑线值。
 */
const uint16_t line_sensor_default_white[LINE_SENSOR_CHANNEL_COUNT] =
{   
    3072u, 3090u, 2939u, 2586u, 2986u, 3109u, 3030u, 3091u
};

const uint16_t line_sensor_default_black[LINE_SENSOR_CHANNEL_COUNT] =
{
    83u, 91u, 91u, 93u, 93u, 93u, 93u, 92u
};

/*
 * 通道方向。
 * 1：保持官方例程 Direction=1 的效果，把硬件通道 0 映射到 raw[7]。
 * 0：不反向，硬件通道 0 映射到 raw[0]。
 *
 * 如果你发现小车左侧传感器变化时 raw[7] 先变，而你希望 raw[0] 先变，
 * 就把这里改成 0。
 */
#define LINE_SENSOR_REVERSE_ORDER       (1u)

/* 每个通道采样次数，官方例程使用 40。 */
#define LINE_SENSOR_ADC_SAMPLE_COUNT    (40u)

/*
 * 切换 74HC4051 地址后的等待时间。
 * 作用是等待模拟通道输出稳定，避免刚切换通道就采 ADC。
 */
#define LINE_SENSOR_MUX_SETTLE_CYCLES   (320u)

/*
 * ADC DMA 等待超时。
 * 如果硬件或 DMA 配置异常，函数会返回失败，不会一直卡死在等待循环里。
 */
#define LINE_SENSOR_ADC_TIMEOUT_POLLS   (10000u)

/*
 * 黑线有效强度阈值。
 * 8 路 black_strength 总和低于这个值时，认为当前没有检测到黑线。
 */
#define LINE_SENSOR_MIN_ACTIVE_SUM      (2500)

/*
 * 8 路位置权重。
 * 左侧为负，右侧为正，中间为 0。最终 error 是黑线强度的加权平均。
 */
static const int32_t line_sensor_weight[LINE_SENSOR_CHANNEL_COUNT] =
{
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

/*
 * 循迹 PID 参数数组。
 *
 * 参数顺序定义在 line_sensor.h：
 * 0：Kp
 * 1：Ki
 * 2：Kd
 * 3：输出限幅绝对值
 * 4：积分限幅绝对值
 * 5：误差死区
 * 6：积分饱和保护开关，0 关闭，非 0 开启
 */
const float line_sensor_pid_params[PID_PARAM_COUNT] =
{
    0.2f,     /* Kp，先小一点 */
    0.0f,     /* Ki，先关掉 */
    0.02f,    /* Kd，先小一点 */
    300.0f,   /* 输出限幅，先给一个保守修正量 */
    0.0f,     /* 积分限幅，Ki=0 时无意义 */
    0.0f,    /* 死区，后面根据 error 静止抖动改 */
    1.0f
};

/* ==================== 灰度传感器常用配置区结束 ==================== */

LineSensor_t line_sensor;

/*
 * ADC DMA 目标缓冲区。
 *
 * 每次读取某一路灰度时，ADC 会连续转换 40 次；
 * DMA_CH0 会把 40 次转换结果依次搬到这个数组中。
 * ADC 是 12 位，结果范围 0~4095，因此 uint16_t 足够保存。
 */
static volatile uint16_t line_adc_dma_buf[LINE_SENSOR_ADC_SAMPLE_COUNT];

/*
 * 当前一路灰度的 40 点平均值。
 * DMA TC 中断里计算，主循环侧由 LineSensor_PollAdcAverage() 读取。
 */
static volatile uint16_t line_adc_average;

/*
 * ADC DMA 完成标志。
 * 0：当前一路采样还没完成。
 * 1：DMA 已经搬完 40 个样本，平均值已经更新到 line_adc_average。
 */
static volatile uint8_t line_adc_done;

typedef enum
{
    LINE_SENSOR_SCAN_IDLE = 0,
    LINE_SENSOR_SCAN_WAIT_DMA
} LineSensorScanState_t;

static LineSensorScanState_t line_scan_state;
static LineSensor_t *line_scan_sensor;
static uint8_t line_scan_channel;
static uint32_t line_adc_timeout_count;

/*
 * @brief 选择 74HC4051 当前导通的模拟通道。
 * @param channel 硬件通道号，范围 0~7。
 *
 * 传感器 8 路灰度共用一个 OUT 输出，所以采每一路之前都必须先切换地址线。
 */
static void LineSensor_SelectChannel(uint8_t channel);

/* @brief 启动当前已选通通道的 ADC + DMA 平均采样。 */
static void LineSensor_StartAdcAverage(void);
static uint8_t LineSensor_PollAdcAverage(uint16_t *result);
static void LineSensor_StopAdcAverage(void);
static void LineSensor_ResetScan(void);

/*
 * @brief 根据 raw[] 和阈值更新 white_mask / black_mask。
 * @param sensor 灰度传感器对象指针。
 *
 * 这里使用双阈值滞回，减少黑白边界附近的抖动。
 */
static void LineSensor_UpdateDigital(LineSensor_t *sensor);

/*
 * @brief 根据黑白校准值把 raw[] 归一化。
 * @param sensor 灰度传感器对象指针。
 *
 * 输出：
 * - normalized[]：白度，白底接近 4095，黑线接近 0。
 * - black_strength[]：黑线强度，黑线越明显数值越大。
 */
static void LineSensor_UpdateNormalized(LineSensor_t *sensor);

/*
 * @brief 根据 black_strength[] 计算循迹误差。
 * @param sensor 灰度传感器对象指针。
 *
 * 输出：
 * - error：左负右正。
 * - line_lost：是否丢线。
 */
static void LineSensor_UpdateError(LineSensor_t *sensor);

void LineSensor_Init(LineSensor_t *sensor)
{
    if (sensor == 0)
    {
        return;
    }

    /*
     * 清空对象，避免上电后结构体里残留随机值。
     * 注意：这里不会清空全局配置常量，只清空传入的运行状态对象。
     */
    memset(sensor, 0, sizeof(*sensor));

    /*
     * 写入默认校准值。
     * 如果后续做自动校准，只需要调用 LineSensor_SetCalibration() 覆盖这里的值。
     */
    LineSensor_SetCalibration(sensor, line_sensor_default_white, line_sensor_default_black);

    /*
     * ADC DMA 说明：
     * - 灰度 ADC 使用 DMA_CH0
     * - UART1 TX 使用 DMA_CH1
     * - UART1 RX 使用 DMA_CH2
     * - SPI1 RX 使用 DMA_CH3
     * - SPI1 TX 使用 DMA_CH4
     *
     * DMA_CH0 是 Full DMA Channel，支持 repeat single 模式。
     * ADC 每完成一次 MEM0 转换，就会触发 DMA 搬运一个 half-word。
     */
    /*
     * ADC 电源策略改成 manual。
     *
     * SysConfig 默认是 auto：每组转换结束后 ADC 可能自动掉电，
     * 下一次启动转换前需要重新唤醒，会让采样启动时间多一个不稳定延迟。
     *
     * manual 表示 ADC 保持上电，灰度扫描周期更稳定，适合循迹小车。
     */
    DL_ADC12_setPowerDownMode(ADC12_0_INST, DL_ADC12_POWER_DOWN_MODE_MANUAL);

    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&line_adc_dma_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, LINE_SENSOR_ADC_SAMPLE_COUNT);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);

    /*
     * 灰度 ADC 不再靠软件连续触发。
     * ADC12_0 在 SysConfig 中被配置为事件触发，TIMG0 的 ZERO_EVENT 会按固定周期触发 ADC。
     * 这里先确保定时器停止，真正采样时由 LineSensor_StartAdcAverage() 启动。
     */
    DL_TimerG_stopCounter(TIMER_ADC_GRAY_INST);
}

void LineSensor_SetCalibration(LineSensor_t *sensor,
                               const uint16_t *white,
                               const uint16_t *black)
{
    uint8_t i;
    uint16_t white_value;
    uint16_t black_value;
    uint16_t diff;

    if ((sensor == 0) || (white == 0) || (black == 0))
    {
        return;
    }

    for (i = 0u; i < LINE_SENSOR_CHANNEL_COUNT; i++)
    {
        /*
         * 每一路传感器的亮度、安装高度、反射特性都可能不同，
         * 所以白底值和黑线值都按通道分别保存。
         */
        white_value = white[i];
        black_value = black[i];

        /*
         * 正常情况下白底 ADC 值大于黑线 ADC 值。
         * 如果调用者传反了，这里自动交换，避免后面的归一化计算出错。
         */
        if (black_value > white_value)
        {
            uint16_t temp = white_value;
            white_value = black_value;
            black_value = temp;
        }

        sensor->white[i] = white_value;
        sensor->black[i] = black_value;

        /*
         *     使用两个阈值做简单滞回：
         * - 高于 white_threshold 才明确认为是白
         * - 低于 black_threshold 才明确认为是黑
         * 中间区域保持上一状态，可减少边界抖动。
                 其实就是斯密特触发器
         */
        sensor->white_threshold[i] =
            (uint16_t)(((uint32_t)white_value * 2u + black_value) / 3u);
        sensor->black_threshold[i] =
            (uint16_t)(((uint32_t)white_value + (uint32_t)black_value * 2u) / 3u);

        diff = (uint16_t)(white_value - black_value);
        if (diff == 0u)
        {
            /*
             * white == black 说明这一通道校准无效。
             * 归一化比例置 0，后面会把该通道 normalized 当作 0 处理。
             */
            sensor->normal_factor_q16[i] = 0u;
        }
        else
        {
            /*
             * normal_factor_q16 = 4095 / (white - black)
             *
             * 这里左移 16 位，把比例放大为 Q16 定点数。
             * 后续归一化时再右移 16 位，避免每次都做浮点除法。
             */
            sensor->normal_factor_q16[i] =
                ((uint32_t)LINE_SENSOR_ADC_MAX_VALUE << 16) / diff;
        }
    }

    sensor->white_mask = 0xFFu;
    sensor->black_mask = 0x00u;
    sensor->calibrated = 1u;
}

uint8_t LineSensor_Update(LineSensor_t *sensor)
{
    /*
     * LineSensor_Update() 是上层最常用的包装函数。
     * 它完成一次完整的“采集 -> 二值化 -> 归一化 -> 误差计算”。
     *
     * 典型调用位置：
     * - 裸机工程：放在主循环里周期调用。
     * - FreeRTOS 工程：后续可以放到灰度传感器任务里周期调用。
     *
     * 注意：
     * 这个函数是状态机式非完全阻塞接口。一次调用只推进当前采样流程；
     * 返回 0 表示 8 路还没采完，返回 1 表示本帧灰度数据已经更新完成。
     */

    /* 步骤 1：采集 8 路原始 ADC 值，结果写入 sensor->raw[]。 */
    if (!LineSensor_ReadRaw(sensor))
    {
        return 0u;
    }

    /* 步骤 2：根据黑白阈值更新 white_mask / black_mask，得到粗略二值结果。 */
    LineSensor_UpdateDigital(sensor);

    /* 步骤 3：根据黑白校准值，把每一路 raw[] 转成统一尺度的黑线强度。 */
    LineSensor_UpdateNormalized(sensor);

    /* 步骤 4：用黑线强度做加权平均，得到循迹误差 error。 */
    LineSensor_UpdateError(sensor);

    return 1u;
}

uint8_t LineSensor_ReadRaw(LineSensor_t *sensor)
{
    uint8_t index;
    uint16_t adc_value;

    if (sensor == 0)
    {
        LineSensor_ResetScan();
        return 0u;
    }

    if ((line_scan_state != LINE_SENSOR_SCAN_IDLE) &&
        (line_scan_sensor != sensor))
    {
        LineSensor_ResetScan();
        return 0u;
    }

    if (line_scan_state == LINE_SENSOR_SCAN_IDLE)
    {
        line_scan_sensor = sensor;
        line_scan_channel = 0u;
        line_adc_timeout_count = 0u;

        LineSensor_SelectChannel(line_scan_channel);
        DL_Common_delayCycles(LINE_SENSOR_MUX_SETTLE_CYCLES);
        LineSensor_StartAdcAverage();
        line_scan_state = LINE_SENSOR_SCAN_WAIT_DMA;
        return 0u;
    }

    if (!LineSensor_PollAdcAverage(&adc_value))
    {
        line_adc_timeout_count++;
        if (line_adc_timeout_count >= LINE_SENSOR_ADC_TIMEOUT_POLLS)
        {
            LineSensor_ResetScan();
        }

        return 0u;
    }

#if LINE_SENSOR_REVERSE_ORDER
    index = (uint8_t)(LINE_SENSOR_CHANNEL_COUNT - 1u - line_scan_channel);
#else
    index = line_scan_channel;
#endif

    sensor->raw[index] = adc_value;
    line_scan_channel++;
    line_adc_timeout_count = 0u;

    if (line_scan_channel >= LINE_SENSOR_CHANNEL_COUNT)
    {
        LineSensor_ResetScan();
        return 1u;
    }

    LineSensor_SelectChannel(line_scan_channel);
    DL_Common_delayCycles(LINE_SENSOR_MUX_SETTLE_CYCLES);
    LineSensor_StartAdcAverage();

    return 0u;
}

static void LineSensor_SelectChannel(uint8_t channel)
{
    /*
     * 官方例程对地址线做了取反：
     * Switch_Address_x(!(channel & bit))
     *
     * 这里保持同样逻辑，避免和官方传感器板的硬件方向不一致。
     *
     * 逻辑通道与三根地址线的关系：
     * channel bit0 -> ADDR0
     * channel bit1 -> ADDR1
     * channel bit2 -> ADDR2
     *
     * 由于官方硬件/例程使用取反逻辑：
     * bit = 0 时 GPIO 输出高电平；
     * bit = 1 时 GPIO 输出低电平。
     */
    if ((channel & 0x01u) == 0u)
    {
        /* 地址 bit0 = 0 时输出高电平，保持官方例程的取反逻辑。 */
        DL_GPIO_setPins(GPIO_GRAY_ADDR_ADDR0_PORT, GPIO_GRAY_ADDR_ADDR0_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_GRAY_ADDR_ADDR0_PORT, GPIO_GRAY_ADDR_ADDR0_PIN);
    }

    if ((channel & 0x02u) == 0u)
    {
        /* 地址 bit1 = 0 时输出高电平。 */
        DL_GPIO_setPins(GPIO_GRAY_ADDR_ADDR1_PORT, GPIO_GRAY_ADDR_ADDR1_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_GRAY_ADDR_ADDR1_PORT, GPIO_GRAY_ADDR_ADDR1_PIN);
    }

    if ((channel & 0x04u) == 0u)
    {
        /* 地址 bit2 = 0 时输出高电平。 */
        DL_GPIO_setPins(GPIO_GRAY_ADDR_ADDR2_PORT, GPIO_GRAY_ADDR_ADDR2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_GRAY_ADDR_ADDR2_PORT, GPIO_GRAY_ADDR_ADDR2_PIN);
    }
}
static void LineSensor_StartAdcAverage(void)
{
    uint8_t i;

    for (i = 0u; i < LINE_SENSOR_ADC_SAMPLE_COUNT; i++)
    {
        line_adc_dma_buf[i] = 0xFFFFu;
    }

    line_adc_average = 0u;
    line_adc_done = 0u;

    DL_TimerG_stopCounter(TIMER_ADC_GRAY_INST);
    DL_ADC12_stopConversion(ADC12_0_INST);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);

    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&line_adc_dma_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, LINE_SENSOR_ADC_SAMPLE_COUNT);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    DL_ADC12_startConversion(ADC12_0_INST);
    DL_TimerG_startCounter(TIMER_ADC_GRAY_INST);
}

static uint8_t LineSensor_PollAdcAverage(uint16_t *result)
{
    if ((result == 0) || (line_adc_done == 0u))
    {
        return 0u;
    }

    *result = line_adc_average;
    return 1u;
}

static void LineSensor_StopAdcAverage(void)
{
    DL_TimerG_stopCounter(TIMER_ADC_GRAY_INST);
    DL_ADC12_stopConversion(ADC12_0_INST);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
}

static void LineSensor_ResetScan(void)
{
    LineSensor_StopAdcAverage();
    line_scan_state = LINE_SENSOR_SCAN_IDLE;
    line_scan_sensor = 0;
    line_scan_channel = 0u;
    line_adc_timeout_count = 0u;
    line_adc_done = 0u;
}

static void LineSensor_UpdateDigital(LineSensor_t *sensor)
{
    uint8_t i;

    /*
     * 使用滞回阈值更新黑白 mask。
     *
     * 判断规则：
     * - raw > white_threshold：认为是白底。
     * - raw < black_threshold：认为是黑线。
     * - 处在两个阈值之间：保持上一状态。
     *
     * 这样可以避免黑白边界处 ADC 小幅抖动导致 mask 反复跳变。
     */
    for (i = 0u; i < LINE_SENSOR_CHANNEL_COUNT; i++)
    {
        if (sensor->raw[i] > sensor->white_threshold[i])
        {
            sensor->white_mask |= (uint8_t)(1u << i);
            sensor->black_mask &= (uint8_t)~(1u << i);
        }
        else if (sensor->raw[i] < sensor->black_threshold[i])
        {
            sensor->white_mask &= (uint8_t)~(1u << i);
            sensor->black_mask |= (uint8_t)(1u << i);
        }
    }
}

static void LineSensor_UpdateNormalized(LineSensor_t *sensor)
{
    uint8_t i;
    uint32_t value;

    /*
     * 把每一路 raw 映射到统一尺度。
     *
     * normalized 表示白度：
     * - raw 接近 black 时，normalized 接近 0。
     * - raw 接近 white 时，normalized 接近 4095。
     *
     * black_strength 表示黑线强度：
     * - black_strength = 4095 - normalized。
     * - 黑线越明显，black_strength 越大。
     *
     * 后续计算 error 时使用 black_strength 做权重，
     * 这样黑线压在哪一侧，哪一侧的权重就更大。
     */
    for (i = 0u; i < LINE_SENSOR_CHANNEL_COUNT; i++)
    {
        if ((sensor->normal_factor_q16[i] == 0u) ||
            (sensor->raw[i] <= sensor->black[i]))
        {
            value = 0u;
        }
        else
        {
            value = ((uint32_t)(sensor->raw[i] - sensor->black[i]) *
                     sensor->normal_factor_q16[i]) >> 16;
        }

        if (value > LINE_SENSOR_ADC_MAX_VALUE)
        {
            value = LINE_SENSOR_ADC_MAX_VALUE;
        }

        sensor->normalized[i] = (uint16_t)value;
        sensor->black_strength[i] = (uint16_t)(LINE_SENSOR_ADC_MAX_VALUE - value);
    }
}
 uint32_t active_sum;
static void LineSensor_UpdateError(LineSensor_t *sensor)
{
    uint8_t i;
   
    int32_t weighted_sum;

    active_sum = 0u;
    weighted_sum = 0;

    /*
     * 计算循迹误差。
     *
     * 公式：
     * error = sum(黑线强度[i] * 位置权重[i]) / sum(黑线强度[i])
     *
     * 权重定义在 line_sensor_weight[]：
     * - 左侧通道权重为负。
     * - 右侧通道权重为正。
     *
     * 所以：
     * - 黑线偏左：error < 0。
     * - 黑线居中：error 接近 0。
     * - 黑线偏右：error > 0。
     */
    for (i = 0u; i < LINE_SENSOR_CHANNEL_COUNT; i++)
    {
        active_sum += sensor->black_strength[i];
        weighted_sum += (int32_t)sensor->black_strength[i] * line_sensor_weight[i];
    }

    if (active_sum < LINE_SENSOR_MIN_ACTIVE_SUM)
    {
        sensor->line_lost = 1u;

        /*
         * 丢线处理。
         *
         * 如果 active_sum 太小，说明 8 路都没有看到明显黑线。
         * 此时不能把 error 直接置 0，否则小车会误以为黑线在正中间。
         *
         * 这里保留搜索方向：
         * - 上一次 error >= 0，说明黑线最后出现在右侧，继续给最大右偏。
         * - 上一次 error < 0，说明黑线最后出现在左侧，继续给最大左偏。
         */
        if (sensor->last_error >= 0)
        {
            sensor->error = line_sensor_weight[LINE_SENSOR_CHANNEL_COUNT - 1u];
        }
        else
        {
            sensor->error = line_sensor_weight[0];
        }

        return;
    }

    sensor->line_lost = 0u;
    sensor->error = weighted_sum / (int32_t)active_sum;
    sensor->last_error = sensor->error;
}

void LineSensor_DMA_IRQHandler(void)
{
    uint32_t sum;
    uint8_t i;

    /*
     * DMA_CH0 TC 表示当前通道 40 个 ADC 样本已经收满。
     * 先停 TIMG0，切断后续 ADC 触发源，再停止 ADC/DMA，避免 repeat 模式继续覆盖 buffer。
     *
     * 注意：
     * 这个函数在中断上下文执行，所以只做短小确定的工作：
     * 1. 停止 TIMG0 / ADC / DMA。
     * 2. 对 40 个样本求平均。
     * 3. 置位 line_adc_done，让主循环侧状态机继续推进下一通道。
     */
    DL_TimerG_stopCounter(TIMER_ADC_GRAY_INST);
    DL_ADC12_stopConversion(ADC12_0_INST);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);

    /* 步骤 1：累加本轮 DMA 收到的 40 个 ADC 样本。 */
    sum = 0u;
    for (i = 0u; i < LINE_SENSOR_ADC_SAMPLE_COUNT; i++)
    {
        sum += line_adc_dma_buf[i];
    }

    /* 步骤 2：计算平均值，降低单次 ADC 噪声对循迹误差的影响。 */
    line_adc_average = (uint16_t)(sum / LINE_SENSOR_ADC_SAMPLE_COUNT);

    /* 步骤 3：通知状态机，本轮采样已经完成。 */
    line_adc_done = 1u;
}
