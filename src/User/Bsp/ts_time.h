#ifndef TS_TIME_H_
#define TS_TIME_H_

#include <stdint.h>

/*
 * TIMG12 时间戳计时器。
 *
 * 配置来源：
 * - TIMG12 在 empty.syscfg 中配置，实例名为 TIMERG12_TS。
 * - TIMG12 配置为 PERIODIC_UP，也就是向上计数。
 * - 当前 TIMG12 时钟为 BUSCLK / 8 = 4 MHz。
 *
 * 计时关系：
 * - 1 tick = 0.25 us。
 * - 4 tick = 1 us。
 * - 32 位回绕周期约 1073.741824 s，也就是约 17.9 分钟。
 */
#define TS_TIME_CLOCK_HZ       (4000000u)
#define TS_TIME_TICKS_PER_US   (TS_TIME_CLOCK_HZ / 1000000u)
#define TS_TIME_TICK_MAX       (0xFFFFFFFFu)

/*
 * 调试观察变量，可以放进 CCS Watch。
 *
 * ts_time_overflow_count：
 * TIMG12 发生 32 位回绕的次数，由 TS_Time_GetTimeline_tick() 检测并累加。
 *
 * ts_time_timeline_tick_raw：
 * 最近一次读取到的 TIMG12 原始 32 位 tick。
 *
 * ts_time_timeline_sec：
 * 最近一次计算出的系统时间线，单位 s。
 */
extern volatile uint32_t ts_time_overflow_count;
extern volatile uint32_t ts_time_timeline_tick_raw;
extern volatile float ts_time_timeline_sec;

/*
 * 功能：初始化 TIMG12 时间戳计时器。
 * 参数：无。
 * 返回：无。
 *
 * 说明：
 * TIMG12 的电源、时钟、周期和计数方向由 SysConfig 生成。
 * 本函数只负责清零计数器、启动计数器和清零软件时间线。
 */
void TS_Time_Init(void);

/*
 * 功能：读取当前 TIMG12 原始 tick。
 * 参数：无。
 * 返回：当前原始 tick。
 *
 * 注意：
 * 当前 TIMG12 为 4 MHz，1 tick = 0.25 us。
 * 如果需要 us，请使用 TS_Time_GetElapsed_us() 或 TS_Time_GetDelta_us()。
 */
uint32_t TS_Time_Get_tick(void);

/*
 * 功能：计算 start_tick 到 end_tick 之间的耗时。
 * 参数：
 * - start_tick：开始原始 tick。
 * - end_tick：结束原始 tick。
 * 返回：耗时，单位 us。
 *
 * 说明：
 * uint32_t 无符号减法天然支持一次 32 位回绕。
 * 因此这个函数适合测短时间段，例如函数耗时、中断耗时、两次 IMU 更新间隔。
 */
uint32_t TS_Time_GetElapsed_us(uint32_t start_tick, uint32_t end_tick);

/*
 * 功能：计算本次调用与上次调用之间的时间差。
 * 参数：
 * - tick_last：保存上一次原始 tick 的变量地址。
 * 返回：两次调用间隔，单位 us。
 *
 * 典型用法：
 * static uint32_t imu_tick_last;
 * imu_dt_us = TS_Time_GetDelta_us(&imu_tick_last);
 */
uint32_t TS_Time_GetDelta_us(uint32_t *tick_last);

/*
 * 功能：计算本次调用与上次调用之间的时间差。
 * 参数：
 * - tick_last：保存上一次原始 tick 的变量地址。
 * 返回：两次调用间隔，单位 s。
 */
float TS_Time_GetDelta_s(uint32_t *tick_last);

/*
 * 功能：读取扩展后的 64 位时间线 tick。
 * 参数：无。
 * 返回：64 位 tick，单位仍然是 TIMG12 原始 tick。
 *
 * 说明：
 * 本函数会判断 TIMG12 是否发生 32 位回绕：
 * - 如果本次 tick 小于上次 tick，说明计数器从 0xFFFFFFFF 回到 0。
 * - 此时 ts_time_overflow_count 加 1。
 * - 最终返回值 = overflow_count * 2^32 + 当前 tick。
 *
 * 限制：
 * 必须在 17.9 分钟内至少调用一次，否则如果中间漏掉多次回绕，
 * 软件无法知道到底回绕了几次。
 */
uint64_t TS_Time_GetTimeline_tick(void);

/*
 * 功能：读取扩展后的 64 位时间线。
 * 参数：无。
 * 返回：系统运行时间，单位 us。
 */
uint64_t TS_Time_GetTimeline_us(void);

/*
 * 功能：读取扩展后的系统运行时间线。
 * 参数：无。
 * 返回：系统运行时间，单位 s。
 */
float TS_Time_GetTimeline_s(void);

#endif /* TS_TIME_H_ */
