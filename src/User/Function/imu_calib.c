#include "imu_calib.h"

#include "flash.h"
#include "imu_logic.h"
#include "ti_msp_dl_config.h"

#include <float.h>
#include <string.h>

/* MSPM0G3507最后一个Flash sector，用于保存陀螺仪零偏。 */
#define IMU_CALIB_FLASH_ADDR                 (0x0001FC00u)
#define IMU_CALIB_FLASH_MAGIC                (0x47594231u)
#define IMU_CALIB_FLASH_VERSION              (1u)

/* ICM42688保持1 kHz；单轮阻塞式校准采集3秒，共3000点。 */
#define IMU_CALIB_SAMPLE_HZ                  (1000u)
#define IMU_CALIB_GYRO_BIAS_TIME_SEC         (3u)
#define IMU_CALIB_GYRO_BIAS_SAMPLES \
    (IMU_CALIB_SAMPLE_HZ * IMU_CALIB_GYRO_BIAS_TIME_SEC)
#define IMU_CALIB_BIAS_RATE_WINDOW_SEC       (1u)
#define IMU_CALIB_BIAS_RATE_WINDOW_SAMPLES \
    (IMU_CALIB_SAMPLE_HZ * IMU_CALIB_BIAS_RATE_WINDOW_SEC)
#define IMU_CALIB_BIAS_RATE_WINDOW_COUNT     (3u)
#define IMU_CALIB_FAULT_BLINK_CYCLES         (CPUCLK_FREQ / 10u)

#if IMU_CALIB_GYRO_BIAS_SAMPLES != \
    (IMU_CALIB_BIAS_RATE_WINDOW_SAMPLES * IMU_CALIB_BIAS_RATE_WINDOW_COUNT)
#error The bias-rate windows must cover the complete calibration interval
#endif

typedef struct
{
    float x;
    float y;
    float z;
} imu_calib_vec3_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    uint32_t reserved0;
    uint32_t checksum;
    uint32_t reserved1;
} imu_calib_flash_record_t;

#define IMU_CALIB_FLASH_WORD_COUNT \
    ((uint32_t)(sizeof(imu_calib_flash_record_t) / sizeof(uint32_t)))
#define IMU_CALIB_FLASH_CHECKSUM_INDEX       (6u)

volatile uint8_t imu_calib_running;
volatile uint8_t imu_calib_failed;
volatile uint32_t imu_calib_sample_count;
volatile uint8_t imu_calib_quality_checking;
volatile uint8_t imu_calib_retry_count;
volatile float imu_calib_bias_rate_x_radps2;
volatile float imu_calib_bias_rate_y_radps2;
volatile float imu_calib_bias_rate_z_radps2;
volatile float imu_calib_bias_rate_max_radps2;
volatile uint8_t imu_calib_bias_rate_passed;

static float imu_calib_sum_x;
static float imu_calib_sum_y;
static float imu_calib_sum_z;
static float imu_calib_accel_sum_x;
static float imu_calib_accel_sum_y;
static float imu_calib_accel_sum_z;
static float imu_calib_accel_mean_x;
static float imu_calib_accel_mean_y;
static float imu_calib_accel_mean_z;
static uint8_t imu_calib_accel_mean_valid;
static imu_calib_vec3_t imu_calib_window_sum;
static imu_calib_vec3_t
    imu_calib_window_bias[IMU_CALIB_BIAS_RATE_WINDOW_COUNT];
static uint8_t imu_calib_window_count;

static uint8_t IMU_Calib_Update(float ax, float ay, float az,
                               float gx, float gy, float gz);
static void IMU_Calib_ResetSampleStatistics(void);
static void IMU_Calib_UpdateBiasWindow(float gx, float gy, float gz);
static void IMU_Calib_ApplyBias(volatile icm42688_data_t *imu);
static uint8_t IMU_Calib_LoadBiasFromFlash(volatile icm42688_data_t *imu);
static uint8_t IMU_Calib_SaveBiasToFlash(float bias_x,
                                         float bias_y,
                                         float bias_z);
static void IMU_Calib_LED0_Set(uint8_t on);
static uint8_t IMU_Calib_ValueIsFinite(float value);
static uint32_t IMU_Calib_CalcChecksum(
    const imu_calib_flash_record_t *record);

void IMU_Calib_Init(void)
{
    IMU_Calib_LED0_Set(0u);

    imu_calib_running = 0u;
    imu_calib_failed = 0u;
    imu_calib_quality_checking = 0u;
    imu_calib_retry_count = 0u;
    IMU_Calib_ResetSampleStatistics();

    /* 读取旧记录仅用于上电校准开始前提供一个确定的Bias值。 */
    (void)IMU_Calib_LoadBiasFromFlash(&icm42688_data);
}

void IMU_Calib_Start(void)
{
    imu_calib_running = 1u;
    imu_calib_failed = 0u;
    IMU_Calib_ResetSampleStatistics();

    /* 校准和可选质量检测全部完成以前，红灯保持常亮。 */
    IMU_Calib_LED0_Set(1u);
}

uint8_t IMU_Calib_Process(volatile icm42688_data_t *imu)
{
    uint8_t calib_finished = 0u;

    if (imu == 0)
    {
        return 0u;
    }

    if (imu_calib_running)
    {
        /* 累加发生在扣除Bias以前，始终使用本帧原始角速度。 */
        calib_finished = IMU_Calib_Update(
            imu->ax_g, imu->ay_g, imu->az_g,
            imu->gx_radps, imu->gy_radps, imu->gz_radps);
    }

    IMU_Calib_ApplyBias(imu);
    return calib_finished;
}

uint8_t IMU_Calib_GetAccelMean(float *ax_g, float *ay_g, float *az_g)
{
    if ((ax_g == 0) || (ay_g == 0) || (az_g == 0) ||
        !imu_calib_accel_mean_valid)
    {
        return 0u;
    }

    *ax_g = imu_calib_accel_mean_x;
    *ay_g = imu_calib_accel_mean_y;
    *az_g = imu_calib_accel_mean_z;
    return 1u;
}

void IMU_Calib_SetBias(float bias_x, float bias_y, float bias_z)
{
    icm42688_data.gyro_bias_x = bias_x;
    icm42688_data.gyro_bias_y = bias_y;
    icm42688_data.gyro_bias_z = bias_z;
}

uint8_t IMU_Calib_SaveCurrentBiasToFlash(void)
{
    float bias_x = icm42688_data.gyro_bias_x;
    float bias_y = icm42688_data.gyro_bias_y;
    float bias_z = icm42688_data.gyro_bias_z;

    if (!IMU_Calib_ValueIsFinite(bias_x) ||
        !IMU_Calib_ValueIsFinite(bias_y) ||
        !IMU_Calib_ValueIsFinite(bias_z) ||
        !IMU_Calib_SaveBiasToFlash(bias_x, bias_y, bias_z))
    {
        imu_calib_failed = 1u;
        return 0u;
    }

    imu_calib_failed = 0u;
    IMU_Calib_LED0_Set(0u);
    return 1u;
}

void IMU_Calib_BlinkFault(void)
{
    IMU_Calib_LED0_Set(1u);
    DL_Common_delayCycles(IMU_CALIB_FAULT_BLINK_CYCLES);
    IMU_Calib_LED0_Set(0u);
    DL_Common_delayCycles(IMU_CALIB_FAULT_BLINK_CYCLES);
}

static void IMU_Calib_ResetSampleStatistics(void)
{
    uint8_t i;

    imu_calib_sample_count = 0u;
    imu_calib_sum_x = 0.0f;
    imu_calib_sum_y = 0.0f;
    imu_calib_sum_z = 0.0f;
    imu_calib_accel_sum_x = 0.0f;
    imu_calib_accel_sum_y = 0.0f;
    imu_calib_accel_sum_z = 0.0f;
    imu_calib_accel_mean_x = 0.0f;
    imu_calib_accel_mean_y = 0.0f;
    imu_calib_accel_mean_z = 0.0f;
    imu_calib_accel_mean_valid = 0u;

    imu_calib_window_sum.x = 0.0f;
    imu_calib_window_sum.y = 0.0f;
    imu_calib_window_sum.z = 0.0f;
    imu_calib_window_count = 0u;
    for (i = 0u; i < IMU_CALIB_BIAS_RATE_WINDOW_COUNT; i++)
    {
        imu_calib_window_bias[i].x = 0.0f;
        imu_calib_window_bias[i].y = 0.0f;
        imu_calib_window_bias[i].z = 0.0f;
    }

    imu_calib_bias_rate_x_radps2 = 0.0f;
    imu_calib_bias_rate_y_radps2 = 0.0f;
    imu_calib_bias_rate_z_radps2 = 0.0f;
    imu_calib_bias_rate_max_radps2 = 0.0f;
    imu_calib_bias_rate_passed = 0u;
}

static void IMU_Calib_UpdateBiasWindow(float gx, float gy, float gz)
{
    imu_calib_vec3_t *window_bias;

    imu_calib_window_sum.x += gx;
    imu_calib_window_sum.y += gy;
    imu_calib_window_sum.z += gz;

    if ((imu_calib_sample_count % IMU_CALIB_BIAS_RATE_WINDOW_SAMPLES) != 0u)
    {
        return;
    }

    if (imu_calib_window_count >= IMU_CALIB_BIAS_RATE_WINDOW_COUNT)
    {
        return;
    }

    window_bias = &imu_calib_window_bias[imu_calib_window_count];
    window_bias->x = imu_calib_window_sum.x /
                     (float)IMU_CALIB_BIAS_RATE_WINDOW_SAMPLES;
    window_bias->y = imu_calib_window_sum.y /
                     (float)IMU_CALIB_BIAS_RATE_WINDOW_SAMPLES;
    window_bias->z = imu_calib_window_sum.z /
                     (float)IMU_CALIB_BIAS_RATE_WINDOW_SAMPLES;
    imu_calib_window_count++;

    imu_calib_window_sum.x = 0.0f;
    imu_calib_window_sum.y = 0.0f;
    imu_calib_window_sum.z = 0.0f;
}

static uint8_t IMU_Calib_Update(float ax, float ay, float az,
                               float gx, float gy, float gz)
{
    float bias_x;
    float bias_y;
    float bias_z;

    imu_calib_sum_x += gx;
    imu_calib_sum_y += gy;
    imu_calib_sum_z += gz;
    imu_calib_accel_sum_x += ax;
    imu_calib_accel_sum_y += ay;
    imu_calib_accel_sum_z += az;
    imu_calib_sample_count++;
    IMU_Calib_UpdateBiasWindow(gx, gy, gz);

    if (imu_calib_sample_count < IMU_CALIB_GYRO_BIAS_SAMPLES)
    {
        return 0u;
    }

    bias_x = imu_calib_sum_x / (float)IMU_CALIB_GYRO_BIAS_SAMPLES;
    bias_y = imu_calib_sum_y / (float)IMU_CALIB_GYRO_BIAS_SAMPLES;
    bias_z = imu_calib_sum_z / (float)IMU_CALIB_GYRO_BIAS_SAMPLES;
    imu_calib_accel_mean_x =
        imu_calib_accel_sum_x / (float)IMU_CALIB_GYRO_BIAS_SAMPLES;
    imu_calib_accel_mean_y =
        imu_calib_accel_sum_y / (float)IMU_CALIB_GYRO_BIAS_SAMPLES;
    imu_calib_accel_mean_z =
        imu_calib_accel_sum_z / (float)IMU_CALIB_GYRO_BIAS_SAMPLES;

    if (imu_calib_window_count == IMU_CALIB_BIAS_RATE_WINDOW_COUNT)
    {
        imu_calib_bias_rate_x_radps2 = IMU_BiasRate_MaxAdjacent(
            imu_calib_window_bias[0].x,
            imu_calib_window_bias[1].x,
            imu_calib_window_bias[2].x,
            (float)IMU_CALIB_BIAS_RATE_WINDOW_SEC);
        imu_calib_bias_rate_y_radps2 = IMU_BiasRate_MaxAdjacent(
            imu_calib_window_bias[0].y,
            imu_calib_window_bias[1].y,
            imu_calib_window_bias[2].y,
            (float)IMU_CALIB_BIAS_RATE_WINDOW_SEC);
        imu_calib_bias_rate_z_radps2 = IMU_BiasRate_MaxAdjacent(
            imu_calib_window_bias[0].z,
            imu_calib_window_bias[1].z,
            imu_calib_window_bias[2].z,
            (float)IMU_CALIB_BIAS_RATE_WINDOW_SEC);
        imu_calib_bias_rate_max_radps2 = IMU_BiasRate_Max3(
            imu_calib_bias_rate_x_radps2,
            imu_calib_bias_rate_y_radps2,
            imu_calib_bias_rate_z_radps2);
        imu_calib_bias_rate_passed = IMU_BiasRate_IsReliable(
            imu_calib_bias_rate_max_radps2,
            IMU_CALIB_GYRO_BIAS_RATE_MAX_RADPS2);
    }

    imu_calib_accel_mean_valid = 1u;
    imu_calib_running = 0u;

    if ((imu_calib_window_count != IMU_CALIB_BIAS_RATE_WINDOW_COUNT) ||
        !IMU_Calib_ValueIsFinite(bias_x) ||
        !IMU_Calib_ValueIsFinite(bias_y) ||
        !IMU_Calib_ValueIsFinite(bias_z) ||
        !IMU_Calib_ValueIsFinite(imu_calib_bias_rate_x_radps2) ||
        !IMU_Calib_ValueIsFinite(imu_calib_bias_rate_y_radps2) ||
        !IMU_Calib_ValueIsFinite(imu_calib_bias_rate_z_radps2) ||
        !IMU_Calib_ValueIsFinite(imu_calib_bias_rate_max_radps2))
    {
        imu_calib_failed = 1u;
        return 1u;
    }

    IMU_Calib_SetBias(bias_x, bias_y, bias_z);
    imu_calib_failed = 0u;
    return 1u;
}

static void IMU_Calib_ApplyBias(volatile icm42688_data_t *imu)
{
    imu->gx_radps -= imu->gyro_bias_x;
    imu->gy_radps -= imu->gyro_bias_y;
    imu->gz_radps -= imu->gyro_bias_z;

    imu->gx_dps = imu->gx_radps / ICM42688_DEG_TO_RAD;
    imu->gy_dps = imu->gy_radps / ICM42688_DEG_TO_RAD;
    imu->gz_dps = imu->gz_radps / ICM42688_DEG_TO_RAD;
    imu->yaw_dps = imu->gz_dps;
    imu->yaw_radps = imu->gz_radps;
}

static uint8_t IMU_Calib_LoadBiasFromFlash(volatile icm42688_data_t *imu)
{
    const imu_calib_flash_record_t *record =
        (const imu_calib_flash_record_t *)IMU_CALIB_FLASH_ADDR;

    if ((record->magic != IMU_CALIB_FLASH_MAGIC) ||
        (record->version != IMU_CALIB_FLASH_VERSION) ||
        (record->checksum != IMU_Calib_CalcChecksum(record)) ||
        !IMU_Calib_ValueIsFinite(record->gyro_bias_x) ||
        !IMU_Calib_ValueIsFinite(record->gyro_bias_y) ||
        !IMU_Calib_ValueIsFinite(record->gyro_bias_z))
    {
        imu->gyro_bias_x = 0.0f;
        imu->gyro_bias_y = 0.0f;
        imu->gyro_bias_z = 0.0f;
        return 0u;
    }

    imu->gyro_bias_x = record->gyro_bias_x;
    imu->gyro_bias_y = record->gyro_bias_y;
    imu->gyro_bias_z = record->gyro_bias_z;
    return 1u;
}

static uint8_t IMU_Calib_SaveBiasToFlash(float bias_x,
                                         float bias_y,
                                         float bias_z)
{
    imu_calib_flash_record_t record;

    memset(&record, 0, sizeof(record));
    record.magic = IMU_CALIB_FLASH_MAGIC;
    record.version = IMU_CALIB_FLASH_VERSION;
    record.gyro_bias_x = bias_x;
    record.gyro_bias_y = bias_y;
    record.gyro_bias_z = bias_z;
    record.checksum = IMU_Calib_CalcChecksum(&record);

    if (!flash_erase_sector(IMU_CALIB_FLASH_ADDR))
    {
        return 0u;
    }

    return flash_program_words(
        IMU_CALIB_FLASH_ADDR,
        (const uint32_t *)&record,
        IMU_CALIB_FLASH_WORD_COUNT);
}

static void IMU_Calib_LED0_Set(uint8_t on)
{
    if (on)
    {
        /* PA0红灯低电平点亮。 */
        DL_GPIO_clearPins(GPIO_LED0_PORT, GPIO_LED0_LED0_PA0_PIN);
    }
    else
    {
        DL_GPIO_setPins(GPIO_LED0_PORT, GPIO_LED0_LED0_PA0_PIN);
    }
}

static uint8_t IMU_Calib_ValueIsFinite(float value)
{
    return ((value == value) &&
            (value >= -FLT_MAX) &&
            (value <= FLT_MAX)) ? 1u : 0u;
}

static uint32_t IMU_Calib_CalcChecksum(
    const imu_calib_flash_record_t *record)
{
    const uint32_t *words = (const uint32_t *)record;
    uint32_t checksum = 0xA5A55A5Au;
    uint32_t i;

    for (i = 0u; i < IMU_CALIB_FLASH_WORD_COUNT; i++)
    {
        if (i != IMU_CALIB_FLASH_CHECKSUM_INDEX)
        {
            checksum ^= words[i] + 0x9E3779B9u +
                        (checksum << 6) + (checksum >> 2);
        }
    }

    return checksum;
}
