#ifndef IMU_LOGIC_H
#define IMU_LOGIC_H

#include <stdint.h>

typedef struct
{
    float ax_g;
    float ay_g;
    float az_g;
    float gx_radps;
    float gy_radps;
    float gz_radps;
} IMU_SixAxisSample_t;

typedef struct
{
    IMU_SixAxisSample_t sum;
    uint8_t sample_count;
} IMU_PairAverager_t;

typedef struct
{
    float total_angle;
    float last_angle;
    int32_t round_count;
    uint8_t initialized;
} IMU_YawTracker_t;

typedef struct
{
    float bias_x;
    float bias_y;
    float bias_z;
    float score;
    uint8_t valid;
} IMU_BiasCandidate_t;

static inline void IMU_PairAverager_Reset(IMU_PairAverager_t *averager)
{
    if (averager == 0)
    {
        return;
    }

    averager->sum.ax_g = 0.0f;
    averager->sum.ay_g = 0.0f;
    averager->sum.az_g = 0.0f;
    averager->sum.gx_radps = 0.0f;
    averager->sum.gy_radps = 0.0f;
    averager->sum.gz_radps = 0.0f;
    averager->sample_count = 0u;
}

static inline uint8_t IMU_PairAverager_Push(
    IMU_PairAverager_t *averager,
    const IMU_SixAxisSample_t *input,
    IMU_SixAxisSample_t *average)
{
    if ((averager == 0) || (input == 0) || (average == 0))
    {
        return 0u;
    }

    if (averager->sample_count == 0u)
    {
        averager->sum = *input;
        averager->sample_count = 1u;
        return 0u;
    }

    average->ax_g = (averager->sum.ax_g + input->ax_g) * 0.5f;
    average->ay_g = (averager->sum.ay_g + input->ay_g) * 0.5f;
    average->az_g = (averager->sum.az_g + input->az_g) * 0.5f;
    average->gx_radps =
        (averager->sum.gx_radps + input->gx_radps) * 0.5f;
    average->gy_radps =
        (averager->sum.gy_radps + input->gy_radps) * 0.5f;
    average->gz_radps =
        (averager->sum.gz_radps + input->gz_radps) * 0.5f;

    IMU_PairAverager_Reset(averager);
    return 1u;
}

static inline void IMU_YawTracker_Reset(IMU_YawTracker_t *tracker)
{
    if (tracker == 0)
    {
        return;
    }

    tracker->total_angle = 0.0f;
    tracker->last_angle = 0.0f;
    tracker->round_count = 0;
    tracker->initialized = 0u;
}

static inline float IMU_YawTracker_Update(
    IMU_YawTracker_t *tracker,
    float yaw_deg)
{
    float delta;

    if (tracker == 0)
    {
        return yaw_deg;
    }

    if (!tracker->initialized)
    {
        tracker->initialized = 1u;
        tracker->last_angle = yaw_deg;
        tracker->total_angle = yaw_deg;
        return tracker->total_angle;
    }

    delta = yaw_deg - tracker->last_angle;
    if (delta > 180.0f)
    {
        tracker->round_count--;
    }
    else if (delta < -180.0f)
    {
        tracker->round_count++;
    }

    tracker->last_angle = yaw_deg;
    tracker->total_angle =
        360.0f * (float)tracker->round_count + yaw_deg;
    return tracker->total_angle;
}

static inline uint8_t IMU_BestBias_Consider(
    IMU_BiasCandidate_t *best,
    float bias_x,
    float bias_y,
    float bias_z,
    float score)
{
    if (best == 0)
    {
        return 0u;
    }

    if (best->valid && (score >= best->score))
    {
        return 0u;
    }

    best->bias_x = bias_x;
    best->bias_y = bias_y;
    best->bias_z = bias_z;
    best->score = score;
    best->valid = 1u;
    return 1u;
}

static inline uint8_t IMU_Quality_ShouldRetry(
    uint8_t completed_attempts,
    uint8_t passed,
    uint8_t max_attempts)
{
    return ((!passed) && (completed_attempts < max_attempts)) ? 1u : 0u;
}

static inline float IMU_BiasRate_MaxAdjacent(
    float bias_0,
    float bias_1,
    float bias_2,
    float interval_s)
{
    float change_01;
    float change_12;

    if (!(interval_s > 0.0f))
    {
        return -1.0f;
    }

    change_01 = bias_1 - bias_0;
    if (change_01 < 0.0f)
    {
        change_01 = -change_01;
    }

    change_12 = bias_2 - bias_1;
    if (change_12 < 0.0f)
    {
        change_12 = -change_12;
    }

    return ((change_01 > change_12) ? change_01 : change_12) /
           interval_s;
}

static inline float IMU_BiasRate_Max3(float x, float y, float z)
{
    float max_value = x;

    if (y > max_value)
    {
        max_value = y;
    }
    if (z > max_value)
    {
        max_value = z;
    }

    return max_value;
}

static inline uint8_t IMU_BiasRate_IsReliable(float max_rate, float limit)
{
    return ((max_rate >= 0.0f) &&
            (limit >= 0.0f) &&
            (max_rate <= limit)) ? 1u : 0u;
}

#endif
