#ifndef IMU_TEST_ENABLE
#define IMU_TEST_ENABLE (0u)
#endif

#if IMU_TEST_ENABLE

#include "imu_logic.h"
#include "mahony.h"

#include <math.h>
#include <stdio.h>

static int test_failures;

#define TEST_CHECK(expr)                                                     \
    do                                                                       \
    {                                                                        \
        if (!(expr))                                                         \
        {                                                                    \
            test_failures++;                                                 \
        }                                                                    \
    } while (0)

#define TEST_NEAR(actual, expected, tolerance)                               \
    TEST_CHECK(fabsf((actual) - (expected)) <= (tolerance))

static void Test_PairAverage(void)
{
    IMU_PairAverager_t averager;
    IMU_SixAxisSample_t first = {
        1.0f, 2.0f, 3.0f,
        4.0f, 5.0f, 6.0f
    };
    IMU_SixAxisSample_t second = {
        3.0f, 4.0f, 5.0f,
        6.0f, 7.0f, 8.0f
    };
    IMU_SixAxisSample_t output;

    IMU_PairAverager_Reset(&averager);

    TEST_CHECK(IMU_PairAverager_Push(&averager, &first, &output) == 0u);
    TEST_CHECK(IMU_PairAverager_Push(&averager, &second, &output) == 1u);
    TEST_NEAR(output.ax_g, 2.0f, 1.0e-6f);
    TEST_NEAR(output.ay_g, 3.0f, 1.0e-6f);
    TEST_NEAR(output.az_g, 4.0f, 1.0e-6f);
    TEST_NEAR(output.gx_radps, 5.0f, 1.0e-6f);
    TEST_NEAR(output.gy_radps, 6.0f, 1.0e-6f);
    TEST_NEAR(output.gz_radps, 7.0f, 1.0e-6f);

    IMU_PairAverager_Reset(&averager);
    second.ax_g = 9.0f;
    TEST_CHECK(IMU_PairAverager_Push(&averager, &second, &output) == 0u);
    TEST_CHECK(averager.sample_count == 1u);
}

static void Test_YawTotalAngle(void)
{
    IMU_YawTracker_t tracker;

    IMU_YawTracker_Reset(&tracker);
    TEST_NEAR(IMU_YawTracker_Update(&tracker, 179.0f), 179.0f, 1.0e-6f);
    TEST_NEAR(IMU_YawTracker_Update(&tracker, -179.0f), 181.0f, 1.0e-6f);
    TEST_CHECK(tracker.round_count == 1);
    TEST_NEAR(IMU_YawTracker_Update(&tracker, 178.0f), 178.0f, 1.0e-6f);
    TEST_CHECK(tracker.round_count == 0);

    IMU_YawTracker_Reset(&tracker);
    TEST_NEAR(IMU_YawTracker_Update(&tracker, -179.0f), -179.0f, 1.0e-6f);
    TEST_NEAR(IMU_YawTracker_Update(&tracker, 179.0f), -181.0f, 1.0e-6f);
    TEST_CHECK(tracker.round_count == -1);
}

static void Test_QualityRetryPolicy(void)
{
    TEST_CHECK(IMU_Quality_ShouldRetry(1u, 0u, 3u) == 1u);
    TEST_CHECK(IMU_Quality_ShouldRetry(2u, 0u, 3u) == 1u);
    TEST_CHECK(IMU_Quality_ShouldRetry(3u, 0u, 3u) == 0u);
    TEST_CHECK(IMU_Quality_ShouldRetry(1u, 1u, 3u) == 0u);
}

static void Test_BestBias(void)
{
    IMU_BiasCandidate_t best = {0};

    TEST_CHECK(IMU_BestBias_Consider(
        &best, 0.10f, 0.20f, 0.30f, 0.50f) == 1u);
    TEST_CHECK(best.valid == 1u);
    TEST_NEAR(best.bias_x, 0.10f, 1.0e-6f);
    TEST_NEAR(best.score, 0.50f, 1.0e-6f);

    TEST_CHECK(IMU_BestBias_Consider(
        &best, 1.10f, 1.20f, 1.30f, 0.80f) == 0u);
    TEST_NEAR(best.bias_x, 0.10f, 1.0e-6f);

    TEST_CHECK(IMU_BestBias_Consider(
        &best, -0.10f, -0.20f, -0.30f, 0.25f) == 1u);
    TEST_NEAR(best.bias_x, -0.10f, 1.0e-6f);
    TEST_NEAR(best.bias_y, -0.20f, 1.0e-6f);
    TEST_NEAR(best.bias_z, -0.30f, 1.0e-6f);
    TEST_NEAR(best.score, 0.25f, 1.0e-6f);
}

static void Test_BiasRateQuality(void)
{
    float rate;

    rate = IMU_BiasRate_MaxAdjacent(0.10f, 0.10f, 0.10f, 1.0f);
    TEST_NEAR(rate, 0.0f, 1.0e-6f);

    rate = IMU_BiasRate_MaxAdjacent(0.10f, 0.1002f, 0.0999f, 1.0f);
    TEST_NEAR(rate, 0.0003f, 1.0e-6f);

    /* First and last are equal: the middle-window disturbance must not cancel. */
    rate = IMU_BiasRate_MaxAdjacent(0.0f, 0.002f, 0.0f, 1.0f);
    TEST_NEAR(rate, 0.002f, 1.0e-6f);

    TEST_NEAR(IMU_BiasRate_MaxAdjacent(
                  0.0f, 0.002f, 0.0f, 0.5f),
              0.004f,
              1.0e-6f);
    TEST_CHECK(IMU_BiasRate_MaxAdjacent(
                   0.0f, 0.0f, 0.0f, 0.0f) < 0.0f);

    TEST_NEAR(IMU_BiasRate_Max3(0.0001f, 0.0004f, 0.0002f),
              0.0004f,
              1.0e-6f);
    TEST_CHECK(IMU_BiasRate_IsReliable(0.001f, 0.001f) == 1u);
    TEST_CHECK(IMU_BiasRate_IsReliable(0.0011f, 0.001f) == 0u);
    TEST_CHECK(IMU_BiasRate_IsReliable(-1.0f, 0.001f) == 0u);
}

static void Test_MahonyInitFromAccel(void)
{
    MahonyFilter_t mahony;
    const float roll_deg = -25.0f;
    const float pitch_deg = 15.0f;
    const float roll_rad = roll_deg * MAHONY_DEG_TO_RAD;
    const float pitch_rad = pitch_deg * MAHONY_DEG_TO_RAD;

    TEST_CHECK(Mahony_InitFromAccel(&mahony, 0.002f,
                                    0.0f, 0.0f, 1.0f) == 1u);
    TEST_NEAR(mahony.roll, 0.0f, 1.0e-3f);
    TEST_NEAR(mahony.pitch, 0.0f, 1.0e-3f);
    TEST_NEAR(mahony.yaw, 0.0f, 1.0e-3f);

    TEST_CHECK(Mahony_InitFromAccel(&mahony, 0.002f,
                                    0.0f, 0.5f, 0.8660254f) == 1u);
    TEST_NEAR(mahony.roll, 30.0f, 1.0e-3f);
    TEST_NEAR(mahony.pitch, 0.0f, 1.0e-3f);
    TEST_NEAR(mahony.yaw, 0.0f, 1.0e-3f);

    TEST_CHECK(Mahony_InitFromAccel(&mahony, 0.002f,
                                    -sinf(20.0f * MAHONY_DEG_TO_RAD),
                                    0.0f,
                                    cosf(20.0f * MAHONY_DEG_TO_RAD)) == 1u);
    TEST_NEAR(mahony.roll, 0.0f, 1.0e-3f);
    TEST_NEAR(mahony.pitch, 20.0f, 1.0e-3f);
    TEST_NEAR(mahony.yaw, 0.0f, 1.0e-3f);

    TEST_CHECK(Mahony_InitFromAccel(
                   &mahony,
                   0.002f,
                   -sinf(pitch_rad),
                   sinf(roll_rad) * cosf(pitch_rad),
                   cosf(roll_rad) * cosf(pitch_rad)) == 1u);
    TEST_NEAR(mahony.roll, roll_deg, 1.0e-3f);
    TEST_NEAR(mahony.pitch, pitch_deg, 1.0e-3f);
    TEST_NEAR(mahony.yaw, 0.0f, 1.0e-3f);

    TEST_CHECK(Mahony_InitFromAccel(&mahony, 0.002f,
                                    0.0f, 0.0f, 0.0f) == 0u);
    TEST_CHECK(Mahony_InitFromAccel(0, 0.002f,
                                    0.0f, 0.0f, 1.0f) == 0u);
}

int main(void)
{
    Test_PairAverage();
    Test_YawTotalAngle();
    Test_QualityRetryPolicy();
    Test_BestBias();
    Test_BiasRateQuality();
    Test_MahonyInitFromAccel();

    if (test_failures != 0)
    {
        printf("IMU tests failed: %d\n", test_failures);
        return 1;
    }

    printf("IMU tests passed\n");
    return 0;
}

#else

typedef int imu_test_disabled_translation_unit_t;

#endif
