#ifndef __VOFA_H__
#define __VOFA_H__

#include <stdint.h>

/*
 * VOFA 串口通信接口。
 *
 * 使用流程：
 * 1. SysConfig 配置 UART1 和 UART1 RX DMA。
 * 2. main.c 调用 VOFA_Init() 初始化发送、接收和 RX DMA。
 * 3. 周期调用 VOFA_SendFloatArray() 或 VOFA_SendVars() 发送曲线数据。
 * 4. 周期调用 VOFA_GetLine() 读取上位机发来的 ASCII 命令。
 *
 * 当前发送协议是 JustFloat：
 * 多个 float 连续小端二进制打包，最后追加 00 00 80 7F 帧尾。
 */

/* 单帧 JustFloat 最多支持的 float 通道数量。 */
#define VOFA_JUSTFLOAT_MAX_CHANNELS       (16u)

/* JustFloat 固定帧尾：00 00 80 7F。 */
#define VOFA_JUSTFLOAT_TAIL_SIZE          (4u)

/* 单帧最大长度 = 最大 float 通道数 * 4 字节 + 4 字节帧尾。 */
#define VOFA_JUSTFLOAT_MAX_FRAME_SIZE \
    ((VOFA_JUSTFLOAT_MAX_CHANNELS * 4u) + VOFA_JUSTFLOAT_TAIL_SIZE)

/* UART RX 一行命令的最大长度，命令以 '\n' 或 '\r' 结束。 */
#define VOFA_RX_LINE_SIZE                 (64u)

/* UART RX DMA 环形缓冲区大小。 */
#define VOFA_RX_DMA_SIZE                  (128u)

/* 初始化 VOFA 的 UART1 发送、RX DMA 接收和 UART1 中断。 */
void VOFA_Init(void);

/*
 * 发送任意 float 数组。
 *
 * values：第一个 float 的地址。
 * count ：本次要发送的 float 通道数量。
 *
 * 返回 1：成功写入 UART TX FIFO。
 * 返回 0：参数错误。
 *
 * 注意：
 * 当前发送是阻塞式直接写 FIFO。
 * 如果 UART 正在发送，函数会等待 FIFO 有空位后继续写。
 */
uint8_t VOFA_SendFloatArray(const float *values, uint8_t count);

/*
 * 快捷宏：直接把变量名传进来，不用手动创建数组。
 *
 * 例子：
 * VOFA_SendVars(imu_state.roll, imu_state.pitch, imu_state.yaw);
 */
#define VOFA_SendVars(...) \
    VOFA_SendFloatArray((const float[]){__VA_ARGS__}, \
        (uint8_t)(sizeof((float[]){__VA_ARGS__}) / sizeof(float)))

/*
 * 固定 IMU 发送接口，一次发送 9 个 IMU 相关通道。
 * 如果要自由选择通道，优先用 VOFA_SendFloatArray() 或 VOFA_SendVars()。
 */
uint8_t VOFA_SendIMU(float roll,
                     float pitch,
                     float yaw,
                     float ax,
                     float ay,
                     float az,
                     float gx,
                     float gy,
                     float gz);

/*
 * 当前发送不再使用 TX DMA 队列。
 * 该函数保留给旧代码兼容，固定返回 0。
 */
uint8_t VOFA_IsBusy(void);

/*
 * 当前发送不再使用 TX 环形缓冲区。
 * 该函数保留给旧代码兼容，固定返回 0。
 */
uint16_t VOFA_GetPendingBytes(void);

/*
 * 读取一行上位机发来的 ASCII 命令。
 *
 * out     ：保存命令字符串的缓冲区。
 * max_len ：out 的最大长度。
 *
 * 返回 1：读到一行新命令。
 * 返回 0：当前没有新命令。
 */
uint8_t VOFA_GetLine(char *out, uint8_t max_len);

extern volatile float vofa_rx_value;

/*
 * VOFA RX 调试变量。
 *
 * 如果上位机发命令没效果，把这些变量放进 Watch：
 * - vofa_rx_byte_count：收到的字节数。发一次命令后应该增加。
 * - vofa_rx_line_count：收到完整行的次数。发送带 \n 或 \r 后应该增加。
 * - vofa_rx_cmd_count：被 VOFA_ProcessCommand() 取走并解析的命令数。
 * - vofa_rx_kp_count：成功解析 kp= 命令的次数。
 * - vofa_rx_ki_count：成功解析 ki= 命令的次数。
 * - vofa_rx_last_cmd：最近一次解析到的命令字符串。
 */
extern volatile uint32_t vofa_rx_byte_count;
extern volatile uint32_t vofa_rx_line_count;
extern volatile uint32_t vofa_rx_cmd_count;
extern volatile uint32_t vofa_rx_kp_count;
extern volatile uint32_t vofa_rx_ki_count;
extern volatile char vofa_rx_last_cmd[VOFA_RX_LINE_SIZE];

/*
 * VOFA TX 调试变量。
 *
 * - vofa_tx_frame_count：已经写入 UART FIFO 的 JustFloat 帧数。
 * - vofa_tx_byte_count ：已经写入 UART FIFO 的总字节数。
 *
 * 这两个值只说明 MCU 确实在写 UART，不代表 PC 一定已经完整显示。
 */
extern volatile uint32_t vofa_tx_frame_count;
extern volatile uint32_t vofa_tx_byte_count;

/*
 * VOFA 发送数据真实数组。
 *
 * 如果要发送表达式，不要把表达式地址传给 VOFA；
 * 先在 vofa.c 里把表达式结果写入 vofa_senddata[]，
 * 再发送这个数组。
 *
 * 这样 vofa_senddata[0]、vofa_senddata[1] 等都有真实 RAM 地址，
 * 也方便放进 Watch 窗口观察。
 */
extern float vofa_senddata[VOFA_JUSTFLOAT_MAX_CHANNELS];

/*
 * 处理 VOFA 下发的 ASCII 命令。
 *
 * 当前支持：
 * kp=2.0   修改 Mahony Kp
 * ki=0.01  修改 Mahony Ki
 * v=123    普通数值接收测试，保存到 vofa_rx_value
 */
void VOFA_ProcessCommand(void);

/* 发送当前 IMU 常用通道：roll、pitch、yaw、补偿后的 gyro XYZ。 */
uint8_t VOFA_SendConfiguredChannels(void);

#endif
