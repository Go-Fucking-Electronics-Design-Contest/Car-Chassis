# Left Encoder GPIO Rising-Edge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the left encoder TIMG7 capture ISR with signed GPIOA rising-edge counting on PA17/PA18, and consume/clear the window count during each 10 ms RPM update.

**Architecture:** `GROUP1_IRQHandler()` becomes a small dispatcher in `main.c`. GPIOA work is delegated to the motor module and GPIOB work remains in the ICM42688 module. The left encoder uses a cumulative position counter plus a separate signed 10 ms pulse accumulator that is snapshotted and cleared atomically.

**Tech Stack:** C11 helper test with MinGW GCC, Arm/Keil C firmware, TI MSPM0 DriverLib, Keil project build.

## Global Constraints

- PA17 is encoder phase A and PA18 is encoder phase B.
- Both phases generate GPIO interrupts on rising edges only.
- Left encoder conversion is exactly `13 PPR * 2 edges * 28 ratio = 728 counts/rev`.
- Right encoder TIMG8 QEI behavior remains unchanged.
- The speed update period and `TS_Time_GetDelta_us()` behavior remain unchanged.
- Existing dirty worktree changes must be preserved.
- Do not regenerate SysConfig files.
- Do not create a Git commit.

---

### Task 1: Prove the Rising-Edge Direction Rules

**Files:**
- Create: `tests/motor_encoder_gpio_logic_test.c`
- Create later: `src/User/Task/motor_encoder_gpio_logic.h`

**Interfaces:**
- Consumes: `Motor_Encoder_DeltaOnARise(uint8_t b_level)` and `Motor_Encoder_DeltaOnBRise(uint8_t a_level)`.
- Produces: A host-side executable test that checks all four direction cases.

- [ ] **Step 1: Write the failing test**

```c
#include <assert.h>
#include <stdint.h>

#include "motor_encoder_gpio_logic.h"

int main(void)
{
    assert(Motor_Encoder_DeltaOnARise(1u) == 1);
    assert(Motor_Encoder_DeltaOnARise(0u) == -1);
    assert(Motor_Encoder_DeltaOnBRise(0u) == 1);
    assert(Motor_Encoder_DeltaOnBRise(1u) == -1);
    return 0;
}
```

- [ ] **Step 2: Compile the test and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Isrc/User/Task tests/motor_encoder_gpio_logic_test.c -o "$env:TEMP\motor_encoder_gpio_logic_test.exe"
```

Expected: compilation fails because `motor_encoder_gpio_logic.h` does not exist.

- [ ] **Step 3: Add the minimal pure helper**

Create `src/User/Task/motor_encoder_gpio_logic.h`:

```c
#ifndef MOTOR_ENCODER_GPIO_LOGIC_H
#define MOTOR_ENCODER_GPIO_LOGIC_H

#include <stdint.h>

static inline int8_t Motor_Encoder_DeltaOnARise(uint8_t b_level)
{
    return (b_level != 0u) ? 1 : -1;
}

static inline int8_t Motor_Encoder_DeltaOnBRise(uint8_t a_level)
{
    return (a_level == 0u) ? 1 : -1;
}

#endif
```

- [ ] **Step 4: Compile and run the test to verify GREEN**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Isrc/User/Task tests/motor_encoder_gpio_logic_test.c -o "$env:TEMP\motor_encoder_gpio_logic_test.exe"
& "$env:TEMP\motor_encoder_gpio_logic_test.exe"
```

Expected: compiler and executable both return exit code 0.

---

### Task 2: Replace TIMG7 Capture with GPIOA Rising-Edge Counting

**Files:**
- Modify: `src/User/Task/motor_inf_task.c`
- Modify: `src/User/Task/motor_inf_task.h`

**Interfaces:**
- Consumes: PA17/PA18 pin and IOMUX macros currently generated for `CAPTURE_ENCODER_LEFT`.
- Produces: `void Motor_Inf_LeftEncoderGPIO_IRQHandler(void)`.
- Produces: `motor_left_gpio_pulse_accum`, `motor_diag_gpioa_isr_count`, `motor_diag_gpioa_a_rise_count`, and `motor_diag_gpioa_b_rise_count`.

- [ ] **Step 1: Change the left encoder scale**

Use separate left GPIO and right QEI scales:

```c
#define MOTOR_LEFT_GPIO_EDGE_FACTOR         (2.0f)
#define MOTOR_RIGHT_QEI_QUADRATURE_FACTOR   (4.0f)
#define MOTOR_LEFT_ENCODER_COUNTS_PER_REV   \
    (MOTOR_ENCODER_PPR * MOTOR_LEFT_GPIO_EDGE_FACTOR * MOTOR_GEAR_RATIO)
#define MOTOR_RIGHT_ENCODER_COUNTS_PER_REV  \
    (MOTOR_ENCODER_PPR * MOTOR_RIGHT_QEI_QUADRATURE_FACTOR * MOTOR_GEAR_RATIO)
```

Use the left constant only for `motor_left_speed_fdb` and the right constant
only for `motor_right_speed_fdb`.

- [ ] **Step 2: Add the GPIO window and diagnostics**

Add volatile globals:

```c
volatile int32_t motor_left_gpio_pulse_accum;
volatile uint32_t motor_diag_gpioa_isr_count;
volatile uint32_t motor_diag_gpioa_a_rise_count;
volatile uint32_t motor_diag_gpioa_b_rise_count;
volatile uint32_t motor_diag_gpioa_unhandled_count;
```

Export them from `motor_inf_task.h` together with:

```c
void Motor_Inf_LeftEncoderGPIO_IRQHandler(void);
```

- [ ] **Step 3: Reconfigure PA17/PA18 during motor initialization**

Add a focused initialization helper that:

```c
DL_TimerG_disableInterrupt(CAPTURE_ENCODER_LEFT_INST,
    DL_TIMERG_INTERRUPT_CC0_UP_EVENT |
    DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
DL_TimerG_stopCounter(CAPTURE_ENCODER_LEFT_INST);
NVIC_ClearPendingIRQ(CAPTURE_ENCODER_LEFT_INST_INT_IRQN);
NVIC_DisableIRQ(CAPTURE_ENCODER_LEFT_INST_INT_IRQN);

DL_GPIO_initDigitalInput(GPIO_CAPTURE_ENCODER_LEFT_C0_IOMUX);
DL_GPIO_initDigitalInput(GPIO_CAPTURE_ENCODER_LEFT_C1_IOMUX);
DL_GPIO_setUpperPinsPolarity(GPIOA,
    DL_GPIO_PIN_17_EDGE_RISE |
    DL_GPIO_PIN_18_EDGE_RISE);
DL_GPIO_clearInterruptStatus(GPIOA,
    GPIO_CAPTURE_ENCODER_LEFT_C0_PIN |
    GPIO_CAPTURE_ENCODER_LEFT_C1_PIN);
DL_GPIO_enableInterrupt(GPIOA,
    GPIO_CAPTURE_ENCODER_LEFT_C0_PIN |
    GPIO_CAPTURE_ENCODER_LEFT_C1_PIN);
NVIC_EnableIRQ(GPIOA_INT_IRQn);
```

Only the PA17/PA18 GPIO source flags are cleared. Do not call
`NVIC_ClearPendingIRQ(GPIOA_INT_IRQn)` because GPIOA and GPIOB share IRQ 1 and
clearing the NVIC pending bit could discard an already-pending IMU request.

Call the helper from `Motor_Inf_Task_Init()` after encoder reset. Delete the
TIMG7 NVIC enable sequence and remove the TIMG7 capture state
synchronization/ISR code.

- [ ] **Step 4: Snapshot and clear the window atomically**

During each 10 ms update, replace the left cumulative-count subtraction with:

```c
uint32_t primask;

primask = __get_PRIMASK();
__disable_irq();
motor_left_encoder_delta = motor_left_gpio_pulse_accum;
motor_left_gpio_pulse_accum = 0;
__set_PRIMASK(primask);
```

Keep the right QEI update and cumulative right encoder subtraction unchanged.

- [ ] **Step 5: Implement the GPIOA module handler**

Read the enabled PA17/PA18 status, process each asserted rising edge with the tested helper, update both the cumulative count and the 10 ms window, then clear exactly the pending pin flags:

```c
pending = DL_GPIO_getEnabledInterruptStatus(GPIOA, pins);

if ((pending & GPIO_CAPTURE_ENCODER_LEFT_C0_PIN) != 0u) {
    delta = Motor_Encoder_DeltaOnARise(Motor_Inf_ReadLeftEncoderB());
    motor_left_encoder_count += delta;
    motor_left_gpio_pulse_accum += delta;
}

if ((pending & GPIO_CAPTURE_ENCODER_LEFT_C1_PIN) != 0u) {
    delta = Motor_Encoder_DeltaOnBRise(Motor_Inf_ReadLeftEncoderA());
    motor_left_encoder_count += delta;
    motor_left_gpio_pulse_accum += delta;
}

DL_GPIO_clearInterruptStatus(GPIOA, pending & pins);
```

Reset all new globals in `Motor_Inf_ResetEncoder()`.

---

### Task 3: Separate the Shared Group 1 Dispatch from Module Logic

**Files:**
- Modify: `src/User/Core/main.c`
- Modify: `src/User/Bsp/icm42688.c`
- Modify: `src/User/Bsp/icm42688.h`

**Interfaces:**
- Consumes: `Motor_Inf_LeftEncoderGPIO_IRQHandler()` from Task 2.
- Produces: `void ICM42688_GPIO_IRQHandler(void)`.
- Produces: the single hardware entry `void GROUP1_IRQHandler(void)`.

- [ ] **Step 1: Extract the GPIOB IMU handler**

Rename the existing `GROUP1_IRQHandler()` body in `icm42688.c` to:

```c
void ICM42688_GPIO_IRQHandler(void)
```

The extracted function reads and clears PB4 status and retains the existing DMA start condition. It no longer calls `DL_Interrupt_getPendingGroup()`.

Declare the function in `icm42688.h`.

Because GPIOB PB4 is configured without a pull resistor and `IMU_Task_Init()`
is currently disabled in `main.c`, disable the PB4 interrupt source directly
after `SYSCFG_DL_init()`. `ICM42688_Init()` re-enables the PB4 source only
after WHO_AM_I and sensor configuration succeed. The module handler also
checks an internal initialized flag before starting DMA.

- [ ] **Step 2: Add the shared dispatcher to main.c**

Include `icm42688.h`, then add:

```c
void GROUP1_IRQHandler(void)
{
    uint32_t iidx;

    for (;;) {
        iidx = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

        switch (iidx) {
            case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
                Motor_Inf_LeftEncoderGPIO_IRQHandler();
                break;

            case GPIO_IMU_INT1_INT_IIDX:
                ICM42688_GPIO_IRQHandler();
                break;

            default:
                return;
        }
    }
}
```

This loop drains simultaneous GPIOA and GPIOB sources without defining duplicate interrupt vectors.

---

### Task 4: Build and Review

**Files:**
- Review: all files modified by Tasks 1-3

**Interfaces:**
- Consumes: host test and firmware sources.
- Produces: compiler evidence plus an exact user-facing diff.

- [ ] **Step 1: Re-run the host test**

Run the Task 1 GCC compile and executable commands.

Expected: exit code 0.

- [ ] **Step 2: Build the Keil project**

Locate `UV4.exe`, run a hidden command-line build of `i_dont_know.uvprojx`, and save the build log inside the workspace.

Expected: zero compiler errors. Existing warnings, if any, must be reported rather than hidden.

- [ ] **Step 3: Review generated diff**

Run:

```powershell
git diff --check
git diff -- src/User/Core/main.c src/User/Bsp/icm42688.c src/User/Bsp/icm42688.h src/User/Task/motor_inf_task.c src/User/Task/motor_inf_task.h src/User/Task/motor_encoder_gpio_logic.h tests/motor_encoder_gpio_logic_test.c
```

Confirm:

- No unrelated user changes were removed.
- TIMG7 capture ISR is no longer active.
- GPIO hardware flags and the 10 ms software accumulator are both cleared in the correct places.
- Right TIMG8 QEI code is unchanged.
- No Git commit exists for this work.

- [ ] **Step 4: Give hardware watch instructions**

Report these variables:

```text
motor_diag_gpioa_isr_count
motor_diag_gpioa_a_rise_count
motor_diag_gpioa_b_rise_count
motor_left_gpio_pulse_accum
motor_left_encoder_delta
motor_left_encoder_count
motor_left_speed_fdb
```

Expected behavior:

- Forward rotation: signed delta and RPM have the configured positive sign.
- Reverse rotation: signed delta and RPM have the opposite sign.
- Stopped for one update window: delta and RPM become zero.
- Window accumulator repeatedly returns to zero after speed updates.
