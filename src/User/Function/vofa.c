#include "vofa.h"
#include "ti_msp_dl_config.h"
#include "icm42688.h"
/* 如果后续要在 VOFA 中重新发送 GravityKF 数据，取消下一行注释。 */
// #include "gravity_kf.h"
#include "mahony.h"
#include <stdlib.h>
#include <string.h>
#include "ts_time.h"
#include "line_sensor.h"
#include "pid.h"
#include "imu_task.h"
#include "motor_inf_task.h"
#include "car_task.h"
/*
 * ==================== VOFA 发送变量配置区 ====================
 *
 * 要修改 VOFA 曲线通道，优先只改 VOFA_UpdateSendData()。
 *
 * vofa_senddata[] 是真实 RAM 数组，每个元素都有固定地址。
 * 如果要发送表达式，先把表达式结果写入 vofa_senddata[]，
 * 再发送 vofa_senddata[]，不要把表达式地址传给 VOFA。
 *
 * 如果变量在别的 .c 文件里，在对应 .h 里 extern，或者你自己在这里 extern 后使用即可。
 */

/*
 * KF 对比发送模板，默认注释掉以减少运行开销。
 *
 * 如果 main.c 中重新启用双 Mahony + GravityKF 对比，
 * 可以取消下面这些 extern，并在 VOFA_UpdateSendData() 中打开对应通道。
 */
// extern volatile float imu_raw_roll;
// extern volatile float imu_raw_pitch;
// extern volatile float imu_raw_yaw;
// extern volatile float imu_kf_roll;
// extern volatile float imu_kf_pitch;
// extern volatile float imu_kf_yaw;

extern uint16_t send_divider_count;
float vofa_senddata[VOFA_JUSTFLOAT_MAX_CHANNELS];
uint32_t vofa_last;
float vofa_deltat;
extern volatile float line_pid_output;
extern PID_t line_pid;
/*
 * VOFA 发送分频。
 *
 * VOFA_SendConfiguredChannels() 在每帧 IMU 数据处理完成后被调用一次。
 * 如果 IMU 处理频率约 200 Hz：
 * - 1：每帧都发，VOFA 约 200 Hz。
 * - 2：每 2 帧发一次，VOFA 约 100 Hz。
 * - 4：每 4 帧发一次，VOFA 约 50 Hz。
 */
#define VOFA_SEND_FRAME_DIVIDER        (2u)
/* 当前实际发送的 float 通道数量。修改 vofa_senddata[] 通道后要同步修改这里。 */
#define VOFA_SENDDATA_CHANNEL_COUNT    (6u)
/*
 * @brief 更新 VOFA 发送数组。
 *
 * 这里可以写任意 float 表达式。
 * 注意单位要一致，例如 dps 只能减 dps，rad/s 只能减 rad/s。
 */
static void VOFA_UpdateSendData(void)
{
    /*
     * 默认发送当前 IMU 姿态角和补偿后的 gyro。
     * 如果要切换发送内容，只改这里和 VOFA_SENDDATA_CHANNEL_COUNT。
     */
		    /*
     *IMU
     */
    // vofa_senddata[0] = icm42688_data.roll;
    // vofa_senddata[1] = icm42688_data.pitch;
    // vofa_senddata[2] = icm42688_data.yaw;
    // vofa_senddata[3] = spi_dma_deltat;
    
    // vofa_senddata[3] = icm42688_data.gx_dps;
    // vofa_senddata[4] = icm42688_data.gy_dps;
    // vofa_senddata[5] = icm42688_data.gz_dps;

    /*
     *循迹
     */
    // vofa_senddata[0] = line_sensor.raw[3];
    // vofa_senddata[1] = line_sensor.raw[4];
    // vofa_senddata[2] = line_sensor.raw[2];
    // vofa_senddata[3] = line_sensor.raw[5];
    // vofa_senddata[4] = line_pid_output;
    // vofa_senddata[5] = line_pid.error;

		 /*
     *编码器
     */
			vofa_senddata[0] = motor_left_speed_fdb;
			vofa_senddata[1] = motor_right_speed_fdb;
//						vofa_senddata[2] = motor_left_speed_fdb;
//			vofa_senddata[3] = motor_right_speed_fdb;
			
//vofa_senddata[1] = motor_left_gpio_pulse_accum;
//vofa_senddata[3] = motor_left_encoder_delta;
//vofa_senddata[1] =motor_left_encoder_count;
//vofa_senddata[5] = motor_right_encoder_count;

  vofa_senddata[2] = (float)motor_left_speed_interval_us;
  vofa_senddata[3] = motor_inf_dt_s * 1000000.0f;
  vofa_senddata[4] = (float)motor_left_encoder_delta;
  vofa_senddata[5] = vofa_deltat;
//			vofa_senddata[4] = motor_left_pwm;
//			vofa_senddata[5] = motor_right_pwm;
			vofa_senddata[2] = motor_left_speed_ref;
			vofa_senddata[3] = motor_right_speed_ref;
//			vofa_senddata[6] = motor_left_encoder_count;
//			vofa_senddata[7] = motor_inf_dt_s;
		



			/*
     * KF 对比发送模板，默认不参与运行。
     *
     * 使用方法：
     * 1. 把 VOFA_SENDDATA_CHANNEL_COUNT 改成 12。
     * 2. 注释掉上面的 5 个时间戳通道，或把它们挪到 12 之后。
     * 3. 打开下面 0~11 通道。
     */
    // vofa_senddata[0] = imu_raw_roll;
    // vofa_senddata[1] = imu_raw_pitch;
    // vofa_senddata[2] = imu_raw_yaw;
    //
    // vofa_senddata[3] = imu_kf_roll;
    // vofa_senddata[4] = imu_kf_pitch;
    // vofa_senddata[5] = imu_kf_yaw;
    //
    // vofa_senddata[6] = icm42688_data.ax_g;
    // vofa_senddata[7] = icm42688_data.ay_g;
    // vofa_senddata[8] = icm42688_data.az_g;
    //
    // vofa_senddata[9] = gravity_kf.x;
    // vofa_senddata[10] = gravity_kf.y;
    // vofa_senddata[11] = gravity_kf.z;

    /*
     * mahnoy 和 bias
     *
     * IMU_Calib_Process() 已经把 icm42688_data.gx_dps / gy_dps / gz_dps
     * 更新成补偿后的角速度。
     * 如果后续不再测试时间戳，可以把 VOFA_SENDDATA_CHANNEL_COUNT 改大，
     * 然后把下面这些通道打开。
     */

//     vofa_senddata[0] = icm42688_data.roll;
//     vofa_senddata[1] = icm42688_data.pitch;
//     vofa_senddata[2] = icm42688_data.yaw;
     
    //  vofa_senddata[3] = icm42688_data.gyro_bias_x;
    //  vofa_senddata[4] = icm42688_data.gyro_bias_y;
    //  vofa_senddata[5] = icm42688_data.gyro_bias_z;
     
    //  vofa_senddata[6] = icm42688_data.gx_dps;
    //  vofa_senddata[7] = icm42688_data.gy_dps;
    //  vofa_senddata[8] = icm42688_data.gz_dps;
//vofa_senddata[3] = imu_body_accel_g.x;
//vofa_senddata[4] = imu_body_accel_g.y;
//vofa_senddata[5] = imu_body_accel_g.z;
        // vofa_senddata[3] = imu_body_accel_g.y;
        // vofa_senddata[4] = icm42688_data.ay_g;
        // vofa_senddata[5] = icm42688_data.az_g;

//     vofa_senddata[6] = icm42688_data.gy_radps;
//     vofa_senddata[7] = imu_body_gyro_radps.z;

    // vofa_senddata[9] =
    //     icm42688_data.gx_dps -
    //     (icm42688_data.gyro_bias_x / ICM42688_DEG_TO_RAD);
    // vofa_senddata[10] =
    //     icm42688_data.gy_dps -
    //     (icm42688_data.gyro_bias_y / ICM42688_DEG_TO_RAD);
    // vofa_senddata[11] =
    //     icm42688_data.gz_dps -
    //     (icm42688_data.gyro_bias_z / ICM42688_DEG_TO_RAD);

    /*
     * 原来直接写在 VOFA_SendVars() 里的表达式也可以放这里：
     *
     * vofa_senddata[9] =
     *     icm42688_data.gx_dps -
     *     (icm42688_data.gyro_bias_x / ICM42688_DEG_TO_RAD);
     * vofa_senddata[10] =
     *     icm42688_data.gy_dps -
     *     (icm42688_data.gyro_bias_y / ICM42688_DEG_TO_RAD);
     * vofa_senddata[11] =
     *     icm42688_data.gz_dps -
     *     (icm42688_data.gyro_bias_z / ICM42688_DEG_TO_RAD);
     */
}

uint8_t VOFA_SendConfiguredChannels(void)
{
    
//    if (send_divider_count < VOFA_SEND_FRAME_DIVIDER)
//    {
//        return 1u;
//    }
//    send_divider_count = 0u;
    vofa_deltat = TS_Time_GetDelta_us(&vofa_last)*0.001f;
    VOFA_UpdateSendData();
    return VOFA_SendFloatArray(vofa_senddata, VOFA_SENDDATA_CHANNEL_COUNT);
}

/* ==================== VOFA 发送变量配置区结束 ==================== */

/*
 * vofa.c 模块说明
 *
 * 这个文件实现 UART1 + DMA 的 VOFA 通信：
 * 1. 发送方向：用户调用 VOFA_SendFloatArray()，CPU 直接把一帧 JustFloat 写入 UART TX FIFO。
 * 2. 发送不再使用 TX 环形队列，也不再使用 TX DMA 排队，避免数据积压导致上位机点间隔不稳定。
 * 3. 接收方向：UART1 RX 每收到 1 字节触发 DMA，DMA 持续写入 vofa_rx_dma_buf。
 * 4. 主循环调用 VOFA_GetLine() 时扫描 RX DMA 新数据，遇到 '\n' 或 '\r' 组成一条命令。
 *
 * 发送格式使用 VOFA JustFloat：
 * 连续 float 小端二进制数据 + 固定帧尾 00 00 80 7F。
 */

/*
 * TX 发送说明：
 * 之前这里使用“软件环形队列 + UART TX DMA”。
 * 现在改成同步写 UART FIFO：
 * - 优点：不排队，当前帧要么立刻发送，要么函数等待 FIFO 有空间后继续写。
 * - 代价：发送期间 CPU 会等待 UART FIFO 空位。
 *
 * 921600 8N1 下，12 通道 JustFloat 一帧 52 字节，物理发送时间约：
 * 52 * 10 / 921600 = 0.56 ms。
 * 如果你按 1 kHz 发送，带宽仍然够用，但 CPU 会被发送占用一部分时间。
 */
volatile uint32_t vofa_tx_frame_count;
volatile uint32_t vofa_tx_byte_count;

/*
 * UART RX DMA 缓冲区。
 * DMA 持续把 UART1 RXDATA 搬到这里，CPU 只负责扫描新写入的字节。
 */
static uint8_t vofa_rx_dma_buf[VOFA_RX_DMA_SIZE];
static volatile uint16_t vofa_rx_dma_read_index;

/*
 * 接收行缓冲区。
 * CPU 从 RX DMA 缓冲区取字节，遇到 '\n' 或 '\r' 后认为一条命令结束。
 */
static volatile char vofa_rx_line[VOFA_RX_LINE_SIZE];
static volatile uint8_t vofa_rx_index;
static volatile uint8_t vofa_rx_ready;

volatile float vofa_rx_value;

/*
 * RX 调试计数。
 * 这些变量故意做成全局，方便 CCS Watch 直接观察串口接收链路卡在哪一步。
 */
volatile uint32_t vofa_rx_byte_count;
volatile uint32_t vofa_rx_line_count;
volatile uint32_t vofa_rx_cmd_count;
volatile uint32_t vofa_rx_kp_count;
volatile uint32_t vofa_rx_ki_count;
volatile char vofa_rx_last_cmd[VOFA_RX_LINE_SIZE];

/* 阻塞等待 UART TX FIFO 有空间，然后写入 1 字节。 */
static void VOFA_WriteByteBlocking(uint8_t data);

/* 连续写入 len 字节到 UART TX FIFO。 */
static void VOFA_WriteBlockBlocking(const uint8_t *data, uint16_t len);

/* 启动 UART1 RX DMA 循环接收。 */
static void VOFA_StartRxDMA(void);

/* 扫描 RX DMA 已经写入的新字节，并送入行解析器。 */
static void VOFA_ServiceRxDMA(void);

/* 处理 RX 收到的单个字节，遇到换行就形成一条命令。 */
static void VOFA_RxByteHandler(uint8_t data);
static uint8_t VOFA_StartsWith(const char *str, const char *prefix);
static const char *VOFA_SkipSpace(const char *str);

void VOFA_Init(void)
{
    /* TX 改成直接写 FIFO，不再维护 TX 环形队列和 TX DMA 状态。 */
    vofa_tx_frame_count = 0u;
    vofa_tx_byte_count = 0u;

    /* RX DMA 从缓冲区开头开始读，当前还没有完整命令行。 */
    vofa_rx_dma_read_index = 0;
    vofa_rx_index = 0;
    vofa_rx_ready = 0;

    /*
     * UART1 中断只处理 RX DMA 完成和错误事件。
     * RX 字节本身由 DMA 搬运，不再靠 RX 中断逐字节读取。
     */
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);

    VOFA_StartRxDMA();

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

uint8_t VOFA_SendIMU(float roll,
                     float pitch,
                     float yaw,
                     float ax,
                     float ay,
                     float az,
                     float gx,
                     float gy,
                     float gz)
{
    const float values[] = {
        roll, pitch, yaw,
        ax, ay, az,
        gx, gy, gz
    };

    return VOFA_SendFloatArray(values, (uint8_t)(sizeof(values) / sizeof(values[0])));
}

uint8_t VOFA_SendFloatArray(const float *values, uint8_t count)
{
    /* payload_len 是 float 数据区长度，frame_len 是数据区 + JustFloat 帧尾总长度。 */
    uint16_t payload_len;

    if ((values == 0) || (count == 0u) || (count > VOFA_JUSTFLOAT_MAX_CHANNELS))
    {
        return 0;
    }

    payload_len = (uint16_t)((uint16_t)count * sizeof(float));

    /*
     * 直接写 UART TX FIFO。
     *
     * 这里不用环形队列，也不排队：
     * 函数会等待 FIFO 有空间，把完整一帧写完再返回。
     * 这样上位机收到的帧顺序和 MCU 调用发送函数的节奏一致。
     */
    VOFA_WriteBlockBlocking((const uint8_t *)values, payload_len);
    VOFA_WriteByteBlocking(0x00u);
    VOFA_WriteByteBlocking(0x00u);
    VOFA_WriteByteBlocking(0x80u);
    VOFA_WriteByteBlocking(0x7Fu);

    vofa_tx_frame_count++;
    vofa_tx_byte_count += (uint32_t)(payload_len + VOFA_JUSTFLOAT_TAIL_SIZE);
    return 1;
}

uint8_t VOFA_IsBusy(void)
{
    return 0u;
}

uint16_t VOFA_GetPendingBytes(void)
{
    return 0u;
}

uint8_t VOFA_GetLine(char *out, uint8_t max_len)
{
    /* i 用于复制字符串，has_line 表示当前是否已经收到完整一行命令。 */
    uint8_t i;
    uint8_t has_line;
    uint32_t primask;

    if ((out == 0) || (max_len == 0u))
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    /*
     * 先扫描 RX DMA 新搬进来的字节。
     * 如果扫描到换行符，vofa_rx_ready 会被置 1。
     */
    VOFA_ServiceRxDMA();

    has_line = vofa_rx_ready;
    if (has_line)
    {
        for (i = 0; (i < (uint8_t)(max_len - 1u)) && (vofa_rx_line[i] != '\0'); i++)
        {
            out[i] = (char)vofa_rx_line[i];
        }

        out[i] = '\0';
        vofa_rx_ready = 0;
        vofa_rx_index = 0;
    }

    __set_PRIMASK(primask);

    return has_line;
}

void VOFA_ProcessCommand(void)
{
    char cmd[VOFA_RX_LINE_SIZE];
    char *end_ptr;
    const char *value_str;
    float value;
    uint8_t i;

    while (VOFA_GetLine(cmd, (uint8_t)sizeof(cmd)))
    {
        vofa_rx_cmd_count++;
        for (i = 0u; (i < (uint8_t)(VOFA_RX_LINE_SIZE - 1u)) && (cmd[i] != '\0'); i++)
        {
            vofa_rx_last_cmd[i] = cmd[i];
        }
        vofa_rx_last_cmd[i] = '\0';

        value_str = VOFA_SkipSpace(cmd);

        if (VOFA_StartsWith(value_str, "kp=") ||
            VOFA_StartsWith(value_str, "KP="))
        {
            value_str += 3;
            value = strtof(value_str, &end_ptr);
            if (end_ptr != value_str)
            {
                mahony_kp = value;
                vofa_rx_kp_count++;
            }
        }
        else if (VOFA_StartsWith(value_str, "ki=") ||
                 VOFA_StartsWith(value_str, "KI="))
        {
            value_str += 3;
            value = strtof(value_str, &end_ptr);
            if (end_ptr != value_str)
            {
                mahony_ki = value;
                vofa_rx_ki_count++;
            }
        }
        else if (VOFA_StartsWith(value_str, "v=") ||
                 VOFA_StartsWith(value_str, "V="))
        {
            value_str += 2;
            value = strtof(value_str, &end_ptr);
            if (end_ptr != value_str)
            {
                vofa_rx_value = value;
            }
        }
        else
        {
            value = strtof(value_str, &end_ptr);
            if (end_ptr != value_str)
            {
                vofa_rx_value = value;
            }
        }
    }
}

static void VOFA_WriteByteBlocking(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_1_INST))
    {
        /* 等待 UART TX FIFO 出现空位。 */
    }

    DL_UART_Main_transmitData(UART_1_INST, data);
}

static void VOFA_WriteBlockBlocking(const uint8_t *data, uint16_t len)
{
    while (len > 0u)
    {
        VOFA_WriteByteBlocking(*data);
        data++;
        len--;
    }
}

static void VOFA_StartRxDMA(void)
{
    /*
     * UART1 RX DMA 使用 CH2，CH0 预留给更高优先级的功能。
     *
     * 配置成重复单次传输：
     * UART 每收到 1 个字节触发一次 DMA；
     * DMA 把 UART1 RXDATA 搬到 vofa_rx_dma_buf；
     * 搬满 VOFA_RX_DMA_SIZE 后自动回到缓冲区开头继续接收。
     */
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);

    DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)&UART_1_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)&vofa_rx_dma_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, VOFA_RX_DMA_SIZE);
    DL_DMA_setTransferMode(DMA, DMA_CH2_CHAN_ID, DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE);

    DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
}

static void VOFA_ServiceRxDMA(void)
{
    /* DMA 当前目的地址，也就是下一次要写入 RX 缓冲区的位置。 */
    uint32_t dma_dest_addr;

    /* 把 DMA 目的地址换算成 vofa_rx_dma_buf 数组下标。 */
    uint16_t dma_write_index;

    /*
     * DMA 当前目的地址就是下一次要写入的位置。
     * 用它减去缓冲区首地址，就能得到 DMA 写指针。
     */
    dma_dest_addr = DL_DMA_getDestAddr(DMA, DMA_CH2_CHAN_ID);

    if ((dma_dest_addr < (uint32_t)&vofa_rx_dma_buf[0]) ||
        (dma_dest_addr >= (uint32_t)&vofa_rx_dma_buf[VOFA_RX_DMA_SIZE]))
    {
        /*
         * 重复模式回绕瞬间，DMA 目的地址可能已经回到初始值。
         * 如果读到异常地址，直接按 0 处理，下一次扫描会继续同步。
         */
        dma_write_index = 0;
    }
    else
    {
        dma_write_index = (uint16_t)(dma_dest_addr - (uint32_t)&vofa_rx_dma_buf[0]);
    }

    /*
     * 从上次读到的位置，一直处理到 DMA 当前写指针。
     * 这部分数据已经由 DMA 搬运到 RAM。
     */
    while (vofa_rx_dma_read_index != dma_write_index)
    {
        VOFA_RxByteHandler(vofa_rx_dma_buf[vofa_rx_dma_read_index]);

        vofa_rx_dma_read_index++;
        if (vofa_rx_dma_read_index >= VOFA_RX_DMA_SIZE)
        {
            vofa_rx_dma_read_index = 0;
        }
    }
}

static void VOFA_RxByteHandler(uint8_t data)
{
    vofa_rx_byte_count++;

    /*
     * 如果上一条命令主循环还没取走，这里先丢弃新字节。
     * 这样不会覆盖正在等待处理的完整命令。
     */
    if (vofa_rx_ready)
    {
        return;
    }

    if ((data == '\n') || (data == '\r'))
    {
        if (vofa_rx_index > 0u)
        {
            vofa_rx_line[vofa_rx_index] = '\0';
            vofa_rx_ready = 1;
            vofa_rx_line_count++;
        }

        return;
    }

    if (vofa_rx_index < (uint8_t)(VOFA_RX_LINE_SIZE - 1u))
    {
        vofa_rx_line[vofa_rx_index] = (char)data;
        vofa_rx_index++;
    }
    else
    {
        /*
         * 命令太长时直接丢弃本行，避免缓冲区溢出。
         * 下一次换行后重新接收。
         */
        vofa_rx_index = 0;
    }
}

static uint8_t VOFA_StartsWith(const char *str, const char *prefix)
{
    while (*prefix != '\0')
    {
        if (*str != *prefix)
        {
            return 0;
        }

        str++;
        prefix++;
    }

    return 1;
}

static const char *VOFA_SkipSpace(const char *str)
{
    while ((*str == ' ') || (*str == '\t'))
    {
        str++;
    }

    return str;
}

void UART_1_INST_IRQHandler(void)
{
    /* 保存 UART1 当前挂起的中断类型。 */
    DL_UART_IIDX interrupt_status;

    do
    {
        interrupt_status = DL_UART_Main_getPendingInterrupt(UART_1_INST);

        switch (interrupt_status)
        {
            case DL_UART_MAIN_IIDX_DMA_DONE_TX:
            {
                /* 当前发送不使用 TX DMA，这里只保留清中断兼容处理。 */
                break;
            }

            case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            {
                /*
                 * RX DMA 缓冲区收满一圈时会进这里。
                 * 这里只清状态并扫描一次，DMA 会继续循环接收。
                 */
                DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
                VOFA_ServiceRxDMA();

                break;
            }

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            {
                while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
                {
                    (void)DL_UART_Main_receiveData(UART_1_INST);
                }

                vofa_rx_index = 0;
                VOFA_StartRxDMA();
                break;
            }

            default:
                break;
        }
    }
    while (interrupt_status != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}
