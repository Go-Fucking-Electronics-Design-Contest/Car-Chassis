#include "ts_time.h"
#include "ti_msp_dl_config.h"

/*
 * TIMG12 由 SysConfig 配置，实例名为 TIMERG12_TS。
 *
 * 当前配置：
 * - 外设：TIMG12
 * - 模式：PERIODIC_UP
 * - 时钟：BUSCLK / 8 = 4 MHz
 * - 计数：1 tick = 0.25 us
 */
#define TS_TIME_INST (TIMERG12_TS_INST)

static uint32_t ts_time_timeline_last_tick_raw;

volatile uint32_t ts_time_overflow_count;
volatile uint32_t ts_time_timeline_tick_raw;
volatile float ts_time_timeline_sec;

void TS_Time_Init(void)
{
    /*
     * TIMG12 的电源、时钟和周期由 SYSCFG_DL_init() 中的
     * SYSCFG_DL_TIMERG12_TS_init() 完成。
     *
     * 这里不再手动 reset/power/config，避免和 SysConfig 管理重复。
     */
    DL_TimerG_setTimerCount(TS_TIME_INST, 0u);
    DL_TimerG_startCounter(TS_TIME_INST);

    ts_time_timeline_last_tick_raw = 0u;
    ts_time_overflow_count = 0u;
    ts_time_timeline_tick_raw = 0u;
    ts_time_timeline_sec = 0.0f;
}

uint32_t TS_Time_Get_tick(void)
{
    /*
     * TIMG12 已经在 SysConfig 中配置成向上计数，所以这里直接返回计数值。
     *
     * 如果未来换成向下计数的定时器，应该在本函数里统一转换成向上计数：
     * tick = LOAD_VALUE - current_count;
     *
     * 这样上层所有代码都可以统一使用 now - last。
     */
    return DL_TimerG_getTimerCount(TS_TIME_INST);
}

uint32_t TS_Time_GetElapsed_us(uint32_t start_tick, uint32_t end_tick)
{
    uint32_t elapsed_tick;

    elapsed_tick = (uint32_t)(end_tick - start_tick);

    return (uint32_t)(elapsed_tick / TS_TIME_TICKS_PER_US);
}

uint32_t TS_Time_GetDelta_us(uint32_t *tick_last)
{
    uint32_t tick_now;
    uint32_t delta_us;

    if (tick_last == 0)
    {
        return 0u;
    }

    tick_now = TS_Time_Get_tick();
    delta_us = TS_Time_GetElapsed_us(*tick_last, tick_now);
    *tick_last = tick_now;

    return delta_us;
}

float TS_Time_GetDelta_s(uint32_t *tick_last)
{
    return ((float)TS_Time_GetDelta_us(tick_last)) * 0.000001f;
}

uint64_t TS_Time_GetTimeline_tick(void)
{
    uint32_t tick_now;
    uint64_t timeline_tick;

    tick_now = TS_Time_Get_tick();

    /*
     * 向上计数时，正常情况 tick_now >= last。
     * 如果 tick_now < last，说明 TIMG12 从 0xFFFFFFFF 回绕到了 0。
     */
    if (tick_now < ts_time_timeline_last_tick_raw)
    {
        ts_time_overflow_count++;
    }

    timeline_tick =
        (((uint64_t)ts_time_overflow_count) << 32) +
        ((uint64_t)tick_now);

    ts_time_timeline_tick_raw = tick_now;
    ts_time_timeline_last_tick_raw = tick_now;

    return timeline_tick;
}

uint64_t TS_Time_GetTimeline_us(void)
{
    return (uint64_t)(TS_Time_GetTimeline_tick() / TS_TIME_TICKS_PER_US);
}

float TS_Time_GetTimeline_s(void)
{
    uint64_t timeline_tick;

    timeline_tick = TS_Time_GetTimeline_tick();
    ts_time_timeline_sec =
        ((float)timeline_tick) / ((float)TS_TIME_CLOCK_HZ);

    return ts_time_timeline_sec;
}
