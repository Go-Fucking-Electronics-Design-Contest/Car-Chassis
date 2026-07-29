#include "icm42688.h"
#include "line_sensor.h"
#include "ts_time.h"

static void ICM42688_CS_LOW(void);
static void ICM42688_CS_HIGH(void);

static void ICM42688_ClearRxFIFO(void);
static void ICM42688_WaitTxSpace(void);

static void icm42688_writeReg(uint8_t addr, uint8_t dat);
static void icm42688_readReg(uint8_t addr, uint8_t *dat);

static int16_t ICM42688_MakeInt16(uint8_t high, uint8_t low);

static uint8_t icm_tx_dma_buf[ICM42688_SPI_DMA_LEN];
static uint8_t icm_rx_dma_buf[ICM42688_SPI_DMA_LEN];

static volatile uint8_t icm_dma_busy = 0;
static volatile uint8_t icm_frame_ready = 0;
static volatile uint8_t icm_initialized = 0;

static icm42688_raw_t icm_raw;
volatile icm42688_data_t icm42688_data;

/*
 * ICM42688 SPI DMA 性能观测变量。
 *
 * 可以直接放进 CCS Watch：
 * - icm_dma_irq_count：RX DMA 完成中断进入次数。
 * - icm_dma_irq_cost_us：ICM42688 DMA 中断分支总耗时。
 * - icm_dma_spi_wait_us：等待 SPI busy 清零的耗时。
 * - icm_dma_spi_wait_timeout_count：等待 SPI busy 超时次数。
 */
volatile uint32_t icm_dma_irq_count;
volatile uint32_t icm_dma_irq_enter_tick_us;
volatile uint32_t icm_dma_irq_exit_tick_us;
volatile uint32_t icm_dma_irq_cost_us;
volatile uint32_t icm_dma_spi_wait_us;
volatile uint32_t icm_dma_spi_wait_timeout_count;



/* 
* 功能  片选icm42688
* 返回  无
*/
static void ICM42688_CS_LOW(void)
{
    DL_GPIO_clearPins(GPIO_CS_PORT, GPIO_CS_SPI_CS_PB5_PIN);    
}

static void ICM42688_CS_HIGH(void)
{
    DL_GPIO_setPins(GPIO_CS_PORT, GPIO_CS_SPI_CS_PB5_PIN);    
}

/* 
* 功能  等待RXFIFO清空
* 返回  无
*/
static void ICM42688_ClearRxFIFO(void)
{
    while (!DL_SPI_isRXFIFOEmpty(SPI_1_INST))
    {
        (void)DL_SPI_receiveData8(SPI_1_INST);
    }
}

/* 
* 功能  等待TXFIFO空闲
* 返回  无
*/
static void ICM42688_WaitTxSpace(void)
{
    while (DL_SPI_isTXFIFOFull(SPI_1_INST))
    {
    }
}

/* 
* 功能  阻塞式写寄存器
* 返回  无
*/
static void icm42688_writeReg(uint8_t addr, uint8_t dat)
{
    uint8_t cmd;

    /*
     * ICM42688 写命令：
     * bit7 = 0
     */
    cmd = addr & ICM42688_SPI_W;

    ICM42688_CS_LOW();

    __NOP();
    __NOP();

    ICM42688_ClearRxFIFO();

    ICM42688_WaitTxSpace();
    DL_SPI_transmitData8(SPI_1_INST, cmd);

    ICM42688_WaitTxSpace();
    DL_SPI_transmitData8(SPI_1_INST, dat);

    while (DL_SPI_isBusy(SPI_1_INST))
    {
    }

    ICM42688_ClearRxFIFO();

    ICM42688_CS_HIGH();
}

/* 
* 功能  阻塞式读寄存器
* 返回  无
*/
static void icm42688_readReg(uint8_t addr, uint8_t *dat)
{
    uint8_t cmd;
    uint8_t dummy;
    uint8_t res;

    if (dat == 0)
    {
        return;
    }

    /*
     * ICM42688 读命令：
     * bit7 = 1
     */
    cmd = addr | ICM42688_SPI_R;

    ICM42688_CS_LOW();

    __NOP();
    __NOP();

    ICM42688_ClearRxFIFO();

    /*
     * 发送读地址
     */
    ICM42688_WaitTxSpace();
    DL_SPI_transmitData8(SPI_1_INST, cmd);

    /*
     * 地址周期收到的是无效字节
     */
    while (DL_SPI_isRXFIFOEmpty(SPI_1_INST))
    {
    }

    dummy = DL_SPI_receiveData8(SPI_1_INST);
    (void)dummy;

    /*
     * 发送 dummy，读取真正数据
     */
    ICM42688_WaitTxSpace();
    DL_SPI_transmitData8(SPI_1_INST, 0x00);

    while (DL_SPI_isRXFIFOEmpty(SPI_1_INST))
    {
    }

    res = DL_SPI_receiveData8(SPI_1_INST);

    while (DL_SPI_isBusy(SPI_1_INST))
    {
    }

    ICM42688_CS_HIGH();

    *dat = res;
}

/* ================= ICM42688 初始化 ================= */

int8_t ICM42688_Init(void)
{
    uint8_t reg;
    uint8_t whoami;

    icm_initialized = 0u;
    DL_GPIO_disableInterrupt(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);

    ICM42688_CS_HIGH();

    /*
     * Bank0
     */
    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);

    /*
     * 软件复位：
     * DEVICE_CONFIG bit0 = 1
     */
    icm42688_writeReg(ICM42688_DEVICE_CONFIG, 0x01);

    /*
     * 手册要求复位后至少等待 1ms，这里给 3ms
     */
    DL_Common_delayCycles((CPUCLK_FREQ / 1000) * 3);

    /*
     * 复位后重新读 WHO_AM_I
     */
    icm42688_readReg(ICM42688_WHO_AM_I, &whoami);

    if (whoami != ICM42688_ID)
    {
        return -1;
    }

    /*
     * Bank1：配置 4 线 SPI
     * INTF_CONFIG4 bit1 = SPI_AP_4WIRE
     */
    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x01);
    icm42688_writeReg(ICM42688_INTF_CONFIG4, 0x02);

    /*
     * 回 Bank0
     */
    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);

    /*
     * Gyro:
     * ±1000 dps
     * ODR = 1kHz
     */
    reg = (uint8_t)((ICM42688_GYRO_FS_1000DPS << 5) |
                    ICM42688_GYRO_ODR_1KHZ);
    icm42688_writeReg(ICM42688_GYRO_CONFIG0, reg);

    /*
     * Accel:
     * ±8g
     * ODR = 1kHz
     */
    reg = (uint8_t)((ICM42688_ACCEL_FS_8G << 5) |
                    ICM42688_ACCEL_ODR_1KHZ);
    icm42688_writeReg(ICM42688_ACCEL_CONFIG0, reg);

    /*
     * INT1 配置：
     * Push-pull
     * Active high
     * Pulse mode
     *
     * 对应 MSPM0 PB4 配置 Rising Edge。
     */
    icm42688_writeReg(ICM42688_INT_CONFIG, 0x03);

    /*
     * INT_CONFIG1：
     * 建议清 0，避免中断异步复位相关默认行为影响 INT 输出。
     */
    icm42688_writeReg(ICM42688_INT_CONFIG1, 0x00);

    /*
     * INT_CONFIG0：
     * 配置 Data Ready 中断清除方式。
     * 这里使用读传感器数据寄存器清除。
     */
    icm42688_writeReg(ICM42688_INT_CONFIG0, 0x20);

    /*
     * INT_SOURCE0:
     * bit3 = UI_DRDY_INT1_EN
     * 把 UI Data Ready 路由到 INT1。
     */
    icm42688_readReg(ICM42688_INT_SOURCE0, &reg);
    reg |= (1u << 3);
    icm42688_writeReg(ICM42688_INT_SOURCE0, reg);

    /*
     * PWR_MGMT0:
     * TEMP_DIS = 0
     * GYRO_MODE = 3, Low Noise
     * ACCEL_MODE = 3, Low Noise
     */
    reg = (uint8_t)((ICM42688_GYRO_MODE_LN << 2) | ICM42688_ACCEL_MODE_LN);
    icm42688_writeReg(ICM42688_PWR_MGMT0, reg);

    /*
     * 开启 Gyro/Accel 后等待稳定
     */
    DL_Common_delayCycles((CPUCLK_FREQ / 1000) * 50);

    icm_initialized = 1u;
    DL_GPIO_clearInterruptStatus(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);
    DL_GPIO_enableInterrupt(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);

    return 0;
}

/* ================= 启动 TX/RX DMA 读取完整帧 ================= */

int8_t ICM42688_StartReadFrameDMA(void)
{
    if (icm_dma_busy)
    {
        return -1;
    }

    icm_dma_busy = 1;
    icm_frame_ready = 0;

    /*
     * 从 TEMP_DATA1 开始读 14 字节。
     *
     * TX:
     * [0]    = TEMP_DATA1 | 0x80
     * [1~14] = 0x00 dummy
     *
     * RX:
     * [0]    = 无效
     * [1~14] = TEMP + ACCEL + GYRO
     */
    icm_tx_dma_buf[0] = ICM42688_TEMP_DATA1 | ICM42688_SPI_R;

    for (uint8_t i = 1; i < ICM42688_SPI_DMA_LEN; i++)
    {
        icm_tx_dma_buf[i] = 0x00;
    }

    for (uint8_t i = 0; i < ICM42688_SPI_DMA_LEN; i++)
    {
        icm_rx_dma_buf[i] = 0x00;
    }

    while (DL_SPI_isBusy(SPI_1_INST))
    {
    }

    ICM42688_ClearRxFIFO();

    ICM42688_CS_LOW();

    __NOP();
    __NOP();

    /*
     * 先关闭 DMA 通道，重新配置地址和长度。
     */
    DL_DMA_disableChannel(DMA, DMA_CH3_CHAN_ID);
    DL_DMA_disableChannel(DMA, DMA_CH4_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL4);

    /*
     * RX DMA_CH3:
     * SPI RX FIFO -> icm_rx_dma_buf[]
     */
    DL_DMA_setSrcAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)&SPI_1_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)&icm_rx_dma_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, ICM42688_SPI_DMA_LEN);

    /*
     * TX DMA_CH4:
     * icm_tx_dma_buf[] -> SPI TX FIFO
     */
    DL_DMA_setSrcAddr(DMA, DMA_CH4_CHAN_ID, (uint32_t)&icm_tx_dma_buf[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH4_CHAN_ID, (uint32_t)&SPI_1_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH4_CHAN_ID, ICM42688_SPI_DMA_LEN);

    /*
     * 关键顺序：
     * 先开 RX DMA，再开 TX DMA。
     */
    DL_DMA_enableChannel(DMA, DMA_CH3_CHAN_ID);
    DL_DMA_enableChannel(DMA, DMA_CH4_CHAN_ID);

    return 0;
}

/* ================= 数据解析 ================= */

static int16_t ICM42688_MakeInt16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

void ICM42688_ParseFrame(void)
{
    uint8_t *p = &icm_rx_dma_buf[1];

    /*
     * 从 TEMP_DATA1 开始读 14 字节：
     *
     * p[0]  TEMP_DATA1
     * p[1]  TEMP_DATA0
     * p[2]  ACCEL_DATA_X1
     * p[3]  ACCEL_DATA_X0
     * p[4]  ACCEL_DATA_Y1
     * p[5]  ACCEL_DATA_Y0
     * p[6]  ACCEL_DATA_Z1
     * p[7]  ACCEL_DATA_Z0
     * p[8]  GYRO_DATA_X1
     * p[9]  GYRO_DATA_X0
     * p[10] GYRO_DATA_Y1
     * p[11] GYRO_DATA_Y0
     * p[12] GYRO_DATA_Z1
     * p[13] GYRO_DATA_Z0
     */

    icm_raw.temp = ICM42688_MakeInt16(p[0],  p[1]);

    icm_raw.ax   = ICM42688_MakeInt16(p[2],  p[3]);
    icm_raw.ay   = ICM42688_MakeInt16(p[4],  p[5]);
    icm_raw.az   = ICM42688_MakeInt16(p[6],  p[7]);

    icm_raw.gx   = ICM42688_MakeInt16(p[8],  p[9]);
    icm_raw.gy   = ICM42688_MakeInt16(p[10], p[11]);
    icm_raw.gz   = ICM42688_MakeInt16(p[12], p[13]);
}

/* ================= 对外状态接口 ================= */

uint8_t ICM42688_IsFrameReady(void)
{
    return icm_frame_ready;
}

void ICM42688_ClearFrameReady(void)
{
    icm_frame_ready = 0;
}

const icm42688_raw_t *ICM42688_GetRawData(void)
{
    return &icm_raw;
}

const volatile icm42688_data_t *ICM42688_GetData(void)
{
    return &icm42688_data;
}

static void ICM42688_ConvertRawToData(void)
{
    const float accel_scale_g = ICM42688_ACCEL_FS_G / ICM42688_RAW_HALF_SCALE;

    const float gyro_scale_dps = ICM42688_GYRO_FS_DPS / ICM42688_RAW_HALF_SCALE;

    icm42688_data.temp_c = ((float)icm_raw.temp / 132.48f) + 25.0f;

    icm42688_data.ax_g = (float)icm_raw.ax * accel_scale_g;
    icm42688_data.ay_g = (float)icm_raw.ay * accel_scale_g;
    icm42688_data.az_g = (float)icm_raw.az * accel_scale_g;

    icm42688_data.gx_dps = (float)icm_raw.gx * gyro_scale_dps;
    icm42688_data.gy_dps = (float)icm_raw.gy * gyro_scale_dps;
    icm42688_data.gz_dps = (float)icm_raw.gz * gyro_scale_dps;

    icm42688_data.gx_radps = icm42688_data.gx_dps * ICM42688_DEG_TO_RAD;
    icm42688_data.gy_radps = icm42688_data.gy_dps * ICM42688_DEG_TO_RAD;
    icm42688_data.gz_radps = icm42688_data.gz_dps * ICM42688_DEG_TO_RAD;

    icm42688_data.yaw_dps = icm42688_data.gz_dps;
    icm42688_data.yaw_radps = icm42688_data.gz_radps;
}

uint8_t ICM42688_UpdateIfReady(void)
{
    uint32_t primask;

    if (!icm_frame_ready)
    {
        return 0;
    }

    /*
     * 保持 icm_frame_ready = 1 直到解析完成。
     * 这样 GROUP1_IRQHandler 里的 (!icm_dma_busy && !icm_frame_ready)
     * 会阻止下一次 DMA 覆盖 icm_rx_dma_buf[]。
     */
    ICM42688_ParseFrame();
    ICM42688_ConvertRawToData();

//临界保护
    primask = __get_PRIMASK();
    __disable_irq();

    icm_frame_ready = 0;

    __set_PRIMASK(primask);
//临界保护

    return 1;
}



void ICM42688_GPIO_IRQHandler(void)
{
    uint32_t pending;

    pending = DL_GPIO_getEnabledInterruptStatus(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);

    if ((pending & GPIO_IMU_INT1_IMU_INT1_PA_PIN) ==
        GPIO_IMU_INT1_IMU_INT1_PA_PIN)
    {
        DL_GPIO_clearInterruptStatus(
            GPIO_IMU_INT1_PORT,
            GPIO_IMU_INT1_IMU_INT1_PA_PIN);

        /*
         * ICM42688 INT1 / Data Ready 到来。
         * 启动一次 SPI TX/RX DMA 读取。
         */
        if (icm_initialized &&
            (!icm_dma_busy) &&
            (!icm_frame_ready))
        {
            (void)ICM42688_StartReadFrameDMA();
        }
    }
}

/* ================= DMA 完成中断 ================= */

/*
 * RX DMA = DMA_CH3
 * TX DMA = DMA_CH4
 *
 * SysConfig 里只给 RX DMA_CH3 开 Enable Channel Interrupt。
 * TX DMA_CH4 不开中断。
 */

uint32_t* spi_dma_last;
void DMA_IRQHandler(void)
{
    /*
     * 工程内多个外设共用 DMA_INT_IRQn：
     *
     * - DMA_CH0：八路灰度 ADC DMA，搬完 40 个 ADC 样本后进入中断。
     * - DMA_CH3：ICM42688 SPI RX DMA，搬完一帧 IMU 数据后进入中断。
     *
     * DL_DMA_getPendingInterrupt() 会返回当前触发中断的 DMA 通道，
     * 因此这里按通道分发给各自模块处理。
     */
    switch (DL_DMA_getPendingInterrupt(DMA))
    {
        case DL_DMA_EVENT_IIDX_DMACH0:
            /*
             * 灰度传感器 ADC DMA 完成。
             * 具体的停止 ADC、求平均和置完成标志都放在 line_sensor.c，
             * 避免把灰度模块细节散落在 IMU 驱动文件里。
             */
            LineSensor_DMA_IRQHandler();
            break;

        case DL_DMA_EVENT_IIDX_DMACH3:
        {
            uint32_t timeout;
            // uint32_t irq_start_cycle;
            // uint32_t irq_end_cycle;
            // uint32_t wait_start_cycle;
            // uint32_t wait_end_cycle;

            // irq_start_cycle = TS_Time_Get_tick();

            /*
             * RX DMA 已经搬完 15 字节。
             *
             * 拉高 CS 前，必须等待 SPI 最后一位真正传输完成。
            */
            timeout = 10000;
            // wait_start_cycle = TS_Time_Get_tick();
            while (DL_SPI_isBusy(SPI_1_INST) && (timeout > 0))
            {
                timeout--;
            }
            // wait_end_cycle = TS_Time_Get_tick();

            // icm_dma_spi_wait_us = TS_Time_GetElapsed_us(
            //     wait_start_cycle, wait_end_cycle);

            if (timeout == 0u)
            {
                icm_dma_spi_wait_timeout_count++;
            }

            ICM42688_CS_HIGH();

            icm_dma_busy = 0;
            icm_frame_ready = 1;

            // irq_end_cycle = TS_Time_Get_tick();

            icm_dma_irq_count++;
            // icm_dma_irq_enter_tick_us = irq_start_cycle;
            // icm_dma_irq_exit_tick_us = irq_end_cycle;
            // icm_dma_irq_cost_us = TS_Time_GetElapsed_us(
            //     irq_start_cycle, irq_end_cycle);

          
            break;
        }

        default:
            break;
    }
}
