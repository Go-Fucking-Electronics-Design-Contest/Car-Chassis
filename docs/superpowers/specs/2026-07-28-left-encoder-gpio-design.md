# 左编码器改用 GPIO 上升沿计数

## 目标

将左编码器 PA17、PA18 从 TIMG7 输入捕获改为 GPIO 上升沿中断计数。
每次速度计算时读取并清零本周期的 GPIO 脉冲累计值，然后换算为 RPM。
右编码器 TIMG8 QEI、10 ms 速度任务周期和 PID 计算方式保持不变。

本次修改只保留在工作区，不创建 Git 提交。

## 中断架构

MSPM0G3507 的 GPIOA 和 GPIOB 都映射到 IRQ 1，因此硬件入口只能是一个：

```text
GROUP1_IRQHandler
├── GPIOA -> Motor_Inf_LeftEncoderGPIO_IRQHandler()
└── GPIOB -> ICM42688_GPIO_IRQHandler()
```

`GROUP1_IRQHandler()` 只负责查询 Group 1 待处理中断来源并分发。
编码器和 IMU 的具体处理放在各自模块中，避免业务逻辑混合。

如果 GPIOA、GPIOB 同时产生中断，分发器继续读取 Group 1 待处理中断，
直到没有待处理来源，保证不会只处理其中一个端口。

## GPIO 初始化

在 `Motor_Inf_Task_Init()` 中：

1. 关闭 TIMG7 编码器捕获中断并停止 TIMG7。
2. 将 PA17、PA18 重新配置为数字输入。
3. 将 PA17、PA18 设置为只在上升沿产生 GPIO 中断。
4. 清除两只引脚已有的 GPIO 中断标志。
5. 使能两只引脚的 GPIO 中断和 GPIOA/Group 1 NVIC 中断。

SysConfig 当前生成的 TIMG7 初始化代码仍可能在 `SYSCFG_DL_init()` 中先运行，
但随后会由 `Motor_Inf_Task_Init()` 停止并把引脚切换为 GPIO，因此运行时不再使用 TIMG7 捕获。
这样不会重新生成并覆盖当前工作区中已有的 SysConfig 修改。

## 计数与方向

PA17（A 相）和 PA18（B 相）都只统计上升沿，因此每个编码器电气周期产生 2 个计数：

```text
COUNTS_PER_REV = 13 PPR * 2 * 28 = 728
```

方向规则沿用当前 AB 解码表的正方向定义：

- A 相上升沿：B=1 时记 `+1`，B=0 时记 `-1`。
- B 相上升沿：A=0 时记 `+1`，A=1 时记 `-1`。

GPIO ISR 每处理一个有效上升沿，同时更新：

- `motor_left_encoder_count`：累计位置计数，只在显式复位编码器时清零。
- `motor_left_gpio_pulse_accum`：当前测速窗口的有符号计数。
- A/B 上升沿诊断计数：用于确认两路引脚实际进入 GPIO 中断。

GPIO 硬件中断标志在对应 ISR 中处理完成后清除。

## 速度计算和清零

每次 10 ms 速度更新时，在一个很短的关中断临界区中：

1. 把 `motor_left_gpio_pulse_accum`复制到局部变量。
2. 将 `motor_left_gpio_pulse_accum` 清零。
3. 恢复进入临界区之前的中断状态。

随后使用实际 `motor_inf_dt_s` 计算：

```text
left_rpm = pulse_delta * 60 / (728 * motor_inf_dt_s)
```

临界区保证 GPIO ISR 不会在“读取”和“清零”之间写入新脉冲，避免丢计数。
累计位置计数不会随速度计算清零，因此仍可观察总计数。

## 与 IMU 中断的兼容

当前 `GROUP1_IRQHandler()` 位于 ICM42688 模块内。实现时将：

1. 把现有 PB4 数据就绪处理提取为 `ICM42688_GPIO_IRQHandler()`。
2. 在统一分发入口中，GPIOB 继续调用上述 IMU 处理函数。
3. GPIOA 调用左编码器处理函数。

IMU 的 PB4 上升沿设置、DMA 启动条件和中断标志清除顺序保持不变。

PB4 当前没有内部上下拉，而且主程序暂未调用 `IMU_Task_Init()`。因此主程序在
`SYSCFG_DL_init()` 后先关闭 PB4 的 GPIO 中断源；只有 `ICM42688_Init()`
成功完成传感器识别和配置后才重新开启。这样启用 GPIOA 编码器的共享 IRQ 1
时，不会因为未连接 IMU 导致 PB4 浮空中断或误启动 SPI DMA。

## 验证

编译通过后，在硬件上观察：

- 手动正转：GPIO A/B 诊断计数增加，窗口 delta 为正，RPM 为正。
- 手动反转：GPIO A/B 诊断计数增加，窗口 delta 为负，RPM 为负。
- 静止超过一个速度周期：窗口 delta 和 RPM 回到 0。
- 每次速度更新后窗口累计值重新从 0 开始。
- IMU 数据就绪中断和 SPI DMA 读取仍然正常。
- TIMG7 ISR 诊断计数不再增加。

10 ms 窗口下，一个计数约为：

```text
60 / (728 * 0.01) = 8.24 RPM
```

因此低速反馈会以约 8.24 RPM 为最小量化步进，这是只使用两相上升沿的预期结果。
