#ifndef __ICM42688_H__
#define __ICM42688_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ================= ICM42688 SPI 读写位 ================= */

#define ICM42688_SPI_W                  0x7F
#define ICM42688_SPI_R                  0x80

/* ================= ICM42688 Bank0 寄存器 ================= */

#define ICM42688_DEVICE_CONFIG          0x11
#define ICM42688_INT_CONFIG             0x14

#define ICM42688_TEMP_DATA1             0x1D
#define ICM42688_TEMP_DATA0             0x1E
#define ICM42688_ACCEL_DATA_X1          0x1F
#define ICM42688_ACCEL_DATA_X0          0x20
#define ICM42688_ACCEL_DATA_Y1          0x21
#define ICM42688_ACCEL_DATA_Y0          0x22
#define ICM42688_ACCEL_DATA_Z1          0x23
#define ICM42688_ACCEL_DATA_Z0          0x24
#define ICM42688_GYRO_DATA_X1           0x25
#define ICM42688_GYRO_DATA_X0           0x26
#define ICM42688_GYRO_DATA_Y1           0x27
#define ICM42688_GYRO_DATA_Y0           0x28
#define ICM42688_GYRO_DATA_Z1           0x29
#define ICM42688_GYRO_DATA_Z0           0x2A

#define ICM42688_PWR_MGMT0              0x4E
#define ICM42688_GYRO_CONFIG0           0x4F
#define ICM42688_ACCEL_CONFIG0          0x50

#define ICM42688_INT_CONFIG0            0x63
#define ICM42688_INT_CONFIG1            0x64
#define ICM42688_INT_SOURCE0            0x65

#define ICM42688_WHO_AM_I               0x75
#define ICM42688_REG_BANK_SEL           0x76

/* ================= ICM42688 Bank1 寄存器 ================= */

#define ICM42688_INTF_CONFIG4           0x7A

/* ================= ICM42688 固定值 ================= */

#define ICM42688_ID                     0x47

/* GYRO_CONFIG0 bit7:5 = FS_SEL, bit3:0 = ODR */
#define ICM42688_GYRO_FS_1000DPS        0x01
#define ICM42688_GYRO_ODR_1KHZ          0x06

/* ACCEL_CONFIG0 bit7:5 = FS_SEL, bit3:0 = ODR */
#define ICM42688_ACCEL_FS_8G            0x01
#define ICM42688_ACCEL_ODR_1KHZ         0x06

/* PWR_MGMT0 */
#define ICM42688_GYRO_MODE_LN           0x03
#define ICM42688_ACCEL_MODE_LN          0x03

/*
 * 从 TEMP_DATA1 开始读：
 * TEMP 2B + ACCEL XYZ 6B + GYRO XYZ 6B = 14B
 *
 * SPI实际传输：
 * 1字节地址 + 14字节数据 = 15B
 */
#define ICM42688_FRAME_DATA_LEN         14
#define ICM42688_SPI_DMA_LEN            (ICM42688_FRAME_DATA_LEN + 1)

#define ICM42688_ACCEL_FS_G             8.0f
#define ICM42688_GYRO_FS_DPS            1000.0f
#define ICM42688_RAW_HALF_SCALE         32768.0f
#define ICM42688_DEG_TO_RAD             0.017453292519943295f

/* ================= 数据结构 ================= */

typedef struct
{
    int16_t temp;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} icm42688_raw_t;

typedef struct
{
    float temp_c;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float gx_radps;
    float gy_radps;
    float gz_radps;

    float yaw_dps;
    float yaw_radps;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    float roll;
    float pitch;
    float yaw;
} icm42688_data_t;

/* ================= 对外接口 ================= */

int8_t ICM42688_Init(void);

uint8_t ICM42688_IsFrameReady(void);
void ICM42688_ClearFrameReady(void);
const icm42688_raw_t *ICM42688_GetRawData(void);

int8_t ICM42688_StartReadFrameDMA(void);
void ICM42688_ParseFrame(void);
void ICM42688_GPIO_IRQHandler(void);

extern volatile icm42688_data_t icm42688_data;

extern volatile uint32_t icm_dma_irq_count;
extern volatile uint32_t icm_dma_irq_enter_tick_us;
extern volatile uint32_t icm_dma_irq_exit_tick_us;
extern volatile uint32_t icm_dma_irq_cost_us;
extern volatile uint32_t icm_dma_spi_wait_us;
extern volatile uint32_t icm_dma_spi_wait_timeout_count;

const volatile icm42688_data_t *ICM42688_GetData(void);
uint8_t ICM42688_UpdateIfReady(void);

/*
 * 中断函数会在 icm42688.c 里面实现：
 * GROUP1_IRQHandler：PB4 INT1 外部中断
 * DMA_IRQHandler：RX DMA_CH3 完成中断
 */

#endif
