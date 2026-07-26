#include "imu_task.h"

#include "icm42688.h"
#include "imu_calib.h"
#include "imu_logic.h"
#include "ti_msp_dl_config.h"
#include "ts_time.h"

/* ICM原始数据保持1 kHz；相邻两帧平均后Mahony以500 Hz运行。 */
#define IMU_MAHONY_DT_SEC                    (0.002f)

#define IMU_CALIB_MAX_ATTEMPTS               (3u)

#define IMU_QUALITY_IDLE                     (0u)
#define IMU_QUALITY_CHECKING                 (1u)
#define IMU_QUALITY_PASS                     (2u)
#define IMU_QUALITY_FAIL                     (3u)

const float imu_sensor_to_body[3][3] =
{
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

IMU_Vector3f_t imu_body_accel_g;
IMU_Vector3f_t imu_body_gyro_radps;
MahonyFilter_t imu_mahony;

volatile uint8_t imu_init_ok;
volatile uint8_t imu_init_failed;
volatile float spi_dma_deltat;

volatile uint8_t imu_quality_state;

volatile float Yaw_TotalAngle;
volatile float Yaw_AngleLast;
volatile int32_t Yaw_RoundCount;

uint16_t send_divider_count;

static uint32_t spi_dma_last;
static IMU_PairAverager_t imu_pair_averager;
static IMU_YawTracker_t imu_yaw_tracker;

static void IMU_Task_ResetFusion(void);
static uint8_t IMU_Task_InitFusionFromCalibAccel(void);
static void IMU_Task_RecordRawFrame(void);
static void IMU_Task_WaitRawFrame(void);
static uint8_t IMU_Task_UpdateAttitude(void);
static uint8_t IMU_Task_RunCalibrationAttempt(void);
static uint8_t IMU_Task_RunBootCalibration(void);
static void IMU_Task_FaultLoop(void);


void IMU_TransformSensorToBody(const IMU_Vector3f_t *sensor,
                               IMU_Vector3f_t *body)
{
    float sensor_x;
    float sensor_y;
    float sensor_z;

    if ((sensor == 0) || (body == 0))
    {
        return;
    }

    sensor_x = sensor->x;
    sensor_y = sensor->y;
    sensor_z = sensor->z;

    body->x = imu_sensor_to_body[0][0] * sensor_x +
              imu_sensor_to_body[0][1] * sensor_y +
              imu_sensor_to_body[0][2] * sensor_z;
    body->y = imu_sensor_to_body[1][0] * sensor_x +
              imu_sensor_to_body[1][1] * sensor_y +
              imu_sensor_to_body[1][2] * sensor_z;
    body->z = imu_sensor_to_body[2][0] * sensor_x +
              imu_sensor_to_body[2][1] * sensor_y +
              imu_sensor_to_body[2][2] * sensor_z;
}

void IMU_Task_Init(void)
{
    imu_init_ok = 0u;
    imu_init_failed = 0u;
    send_divider_count = 0u;
    spi_dma_deltat = 0.0f;
    spi_dma_last = TS_Time_Get_tick();

    imu_quality_state = IMU_QUALITY_IDLE;

    IMU_Calib_Init();
    IMU_Task_ResetFusion();

    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);

    if (ICM42688_Init() != 0)
    {
        imu_init_failed = 1u;
        IMU_Task_FaultLoop();
    }

    NVIC_ClearPendingIRQ(GPIO_IMU_INT1_INT_IRQN);
    NVIC_EnableIRQ(GPIO_IMU_INT1_INT_IRQN);

    if (!IMU_Task_RunBootCalibration())
    {
        imu_init_failed = 1u;
        IMU_Task_FaultLoop();
    }

    /* 启动校准帧不计入正常运行的VOFA分频和周期观测。 */
    send_divider_count = 0u;
    spi_dma_deltat = 0.0f;
    spi_dma_last = TS_Time_Get_tick();
    imu_init_ok = 1u;
}

void IMU_Task_Run(void)
{
    if (!imu_init_ok)
    {
        return;
    }

    if (!ICM42688_UpdateIfReady())
    {
        return;
    }

    IMU_Task_RecordRawFrame();
    (void)IMU_Calib_Process(&icm42688_data);
    (void)IMU_Task_UpdateAttitude();
}

static void IMU_Task_ResetFusion(void)
{
    Mahony_Init(&imu_mahony, IMU_MAHONY_DT_SEC);
    IMU_PairAverager_Reset(&imu_pair_averager);
    IMU_YawTracker_Reset(&imu_yaw_tracker);

    Yaw_TotalAngle = 0.0f;
    Yaw_AngleLast = 0.0f;
    Yaw_RoundCount = 0;

    icm42688_data.roll = 0.0f;
    icm42688_data.pitch = 0.0f;
    icm42688_data.yaw = 0.0f;
}

static uint8_t IMU_Task_InitFusionFromCalibAccel(void)
{
    IMU_Vector3f_t sensor_accel_g;

    if (!IMU_Calib_GetAccelMean(&sensor_accel_g.x,
                                &sensor_accel_g.y,
                                &sensor_accel_g.z))
    {
        return 0u;
    }

    IMU_TransformSensorToBody(&sensor_accel_g, &imu_body_accel_g);
    if (!Mahony_InitFromAccel(&imu_mahony,
                              IMU_MAHONY_DT_SEC,
                              imu_body_accel_g.x,
                              imu_body_accel_g.y,
                              imu_body_accel_g.z))
    {
        return 0u;
    }

    IMU_PairAverager_Reset(&imu_pair_averager);
    IMU_YawTracker_Reset(&imu_yaw_tracker);

    imu_body_gyro_radps.x = 0.0f;
    imu_body_gyro_radps.y = 0.0f;
    imu_body_gyro_radps.z = 0.0f;

    Yaw_TotalAngle = 0.0f;
    Yaw_AngleLast = 0.0f;
    Yaw_RoundCount = 0;

    icm42688_data.roll = imu_mahony.roll;
    icm42688_data.pitch = imu_mahony.pitch;
    icm42688_data.yaw = 0.0f;
    return 1u;
}

static void IMU_Task_RecordRawFrame(void)
{
    send_divider_count++;
    spi_dma_deltat = TS_Time_GetDelta_us(&spi_dma_last) * 0.001f;
}

static void IMU_Task_WaitRawFrame(void)
{
    while (!ICM42688_UpdateIfReady())
    {
    }

    IMU_Task_RecordRawFrame();
}

static uint8_t IMU_Task_UpdateAttitude(void)
{
    IMU_Vector3f_t sensor_accel_g;
    IMU_Vector3f_t sensor_gyro_radps;
    IMU_SixAxisSample_t input;
    IMU_SixAxisSample_t average;

    sensor_accel_g.x = icm42688_data.ax_g;
    sensor_accel_g.y = icm42688_data.ay_g;
    sensor_accel_g.z = icm42688_data.az_g;
    sensor_gyro_radps.x = icm42688_data.gx_radps;
    sensor_gyro_radps.y = icm42688_data.gy_radps;
    sensor_gyro_radps.z = icm42688_data.gz_radps;

    IMU_TransformSensorToBody(&sensor_accel_g, &imu_body_accel_g);
    IMU_TransformSensorToBody(&sensor_gyro_radps, &imu_body_gyro_radps);

    input.ax_g = imu_body_accel_g.x;
    input.ay_g = imu_body_accel_g.y;
    input.az_g = imu_body_accel_g.z;
    input.gx_radps = imu_body_gyro_radps.x;
    input.gy_radps = imu_body_gyro_radps.y;
    input.gz_radps = imu_body_gyro_radps.z;

    if (!IMU_PairAverager_Push(&imu_pair_averager, &input, &average))
    {
        return 0u;
    }

    Mahony_Update(&imu_mahony,
        average.ax_g,
        average.ay_g,
        average.az_g,
        average.gx_radps,
        average.gy_radps,
        average.gz_radps);

    icm42688_data.roll = imu_mahony.roll;
    icm42688_data.pitch = imu_mahony.pitch;
    icm42688_data.yaw = imu_mahony.yaw;

    Yaw_TotalAngle = IMU_YawTracker_Update(
        &imu_yaw_tracker, icm42688_data.yaw);
    Yaw_AngleLast = imu_yaw_tracker.last_angle;
    Yaw_RoundCount = imu_yaw_tracker.round_count;
    return 1u;
}

static uint8_t IMU_Task_RunCalibrationAttempt(void)
{
    IMU_Calib_Start();

    while (imu_calib_running)
    {
        IMU_Task_WaitRawFrame();
        (void)IMU_Calib_Process(&icm42688_data);
    }

    return (imu_calib_failed == 0u) ? 1u : 0u;
}

static uint8_t IMU_Task_RunBootCalibration(void)
{
#if IMU_QUALITY_CHECK_ENABLE
    IMU_BiasCandidate_t best = {0};
    float best_rate_x = 0.0f;
    float best_rate_y = 0.0f;
    float best_rate_z = 0.0f;
    uint8_t attempt;

    imu_calib_quality_checking = 1u;
    for (attempt = 0u; attempt < IMU_CALIB_MAX_ATTEMPTS; attempt++)
    {
        imu_calib_retry_count = attempt;
        imu_quality_state = IMU_QUALITY_CHECKING;
        if (!IMU_Task_RunCalibrationAttempt())
        {
            imu_calib_quality_checking = 0u;
            return 0u;
        }

        if (IMU_BestBias_Consider(
                &best,
                icm42688_data.gyro_bias_x,
                icm42688_data.gyro_bias_y,
                icm42688_data.gyro_bias_z,
                imu_calib_bias_rate_max_radps2))
        {
            best_rate_x = imu_calib_bias_rate_x_radps2;
            best_rate_y = imu_calib_bias_rate_y_radps2;
            best_rate_z = imu_calib_bias_rate_z_radps2;
        }

        if (imu_calib_bias_rate_passed)
        {
            imu_quality_state = IMU_QUALITY_PASS;
            break;
        }

        imu_quality_state = IMU_QUALITY_FAIL;

        if (!IMU_Quality_ShouldRetry(
                (uint8_t)(attempt + 1u),
                imu_calib_bias_rate_passed,
                IMU_CALIB_MAX_ATTEMPTS))
        {
            break;
        }
    }
    imu_calib_quality_checking = 0u;

    if (!best.valid)
    {
        return 0u;
    }

    IMU_Calib_SetBias(best.bias_x, best.bias_y, best.bias_z);
    imu_calib_bias_rate_x_radps2 = best_rate_x;
    imu_calib_bias_rate_y_radps2 = best_rate_y;
    imu_calib_bias_rate_z_radps2 = best_rate_z;
    imu_calib_bias_rate_max_radps2 = best.score;
    imu_calib_bias_rate_passed = IMU_BiasRate_IsReliable(
        best.score, IMU_CALIB_GYRO_BIAS_RATE_MAX_RADPS2);
#else
    imu_calib_retry_count = 0u;
    imu_calib_quality_checking = 0u;
    if (!IMU_Task_RunCalibrationAttempt())
    {
        return 0u;
    }
#endif

    if (!IMU_Task_InitFusionFromCalibAccel())
    {
        return 0u;
    }

    if (!IMU_Calib_SaveCurrentBiasToFlash())
    {
        return 0u;
    }

    imu_calib_quality_checking = 0u;
    return 1u;
}

static void IMU_Task_FaultLoop(void)
{
    for (;;)
    {
        IMU_Calib_BlinkFault();
    }
}
