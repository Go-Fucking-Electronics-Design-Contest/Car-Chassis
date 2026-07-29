# Left Encoder X4 20 ms Speed Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change only the left wheel to GPIO quadrature x4 counting with a fixed 20 ms M-method speed window.

**Architecture:** PA17 and PA18 trigger on rising and falling edges. A pure AB transition decoder produces signed counts, while `Motor_Inf_Task_Run()` atomically snapshots and clears the left window counter every 20 ms. The existing 10 ms task/PID cadence and the right TIMG8 QEI path remain unchanged.

**Tech Stack:** C11, TI MSPM0 DriverLib, ARMClang for Cortex-M0+, host GCC tests.

## Global Constraints

- Left encoder: 13 PPR, 28:1 gearbox, quadrature x4, therefore 1456 counts/output-shaft revolution.
- Left measurement window: 20000 us; use actual accumulated microseconds for conversion.
- Do not modify the right TIMG8 QEI measurement path.
- Do not delete `motor_speed_window.h` while it may be open in Keil; it can remain unused.
- Do not stage or commit Git changes.

---

### Task 1: Quadrature x4 transition decoder

**Files:**
- Modify: `tests/motor_encoder_gpio_logic_test.c`
- Modify: `src/User/Task/motor_encoder_gpio_logic.h`

**Interfaces:**
- Produces: `int8_t Motor_Encoder_DecodeTransition(uint8_t previous_ab, uint8_t current_ab)`

- [ ] **Step 1: Write the failing test**

Test the forward sequence `00, 01, 11, 10, 00` as four `+1` transitions, the reverse sequence as four `-1` transitions, and same/invalid two-bit transitions as zero.

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Isrc\User\Task tests\motor_encoder_gpio_logic_test.c -o tests\motor_encoder_gpio_logic_test.exe
```

Expected: compilation fails because `Motor_Encoder_DecodeTransition` is not defined.

- [ ] **Step 3: Implement the minimal decoder**

Use a 16-entry lookup indexed by `(previous_ab << 2) | current_ab`:

```c
static const int8_t delta_table[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};
```

- [ ] **Step 4: Run the test and require exit code 0**

### Task 2: Fixed 20 ms interval helper

**Files:**
- Create: `tests/motor_speed_interval_test.c`
- Create: `src/User/Task/motor_speed_interval.h`

**Interfaces:**
- Produces: `uint8_t Motor_SpeedInterval_Accumulate(uint32_t *, uint32_t, uint32_t, uint32_t *)`

- [ ] **Step 1: Write the failing test**

Verify one 10 ms update is not ready, the second reaches 20 ms and resets the time accumulator, and a delayed 22 ms interval returns 22000 us.

- [ ] **Step 2: Run test to verify it fails**

Expected: missing-header compilation failure.

- [ ] **Step 3: Implement minimal accumulated-time threshold logic**

Return the actual completed interval and clear the time accumulator only when it reaches 20000 us.

- [ ] **Step 4: Run the test and require exit code 0**

### Task 3: Integrate x4 GPIO counting and 20 ms M-method speed

**Files:**
- Modify: `src/User/Task/motor_inf_task.c`
- Modify: `src/User/Task/motor_inf_task.h`

**Interfaces:**
- Consumes: `Motor_Encoder_DecodeTransition`, `Motor_SpeedInterval_Accumulate`
- Produces diagnostics for current/previous AB state, transition delta, invalid transitions, last completed left interval, last completed window count, count/s, and RPM.

- [ ] **Step 1: Configure PA17/PA18 for `DL_GPIO_PIN_17_EDGE_RISE_FALL | DL_GPIO_PIN_18_EDGE_RISE_FALL`**

- [ ] **Step 2: Read AB once per GPIO interrupt and decode the transition**

Only valid one-bit transitions change `motor_left_encoder_count` and `motor_left_gpio_pulse_accum`.

- [ ] **Step 3: Set left counts per revolution to `13 * 28 * 4 = 1456`**

- [ ] **Step 4: Replace the left sliding calculation with an independent 20 ms snapshot**

Inside a brief interrupt-disabled region, copy `motor_left_gpio_pulse_accum` to `motor_left_encoder_delta` and clear only the window accumulator. Convert using the actual elapsed interval.

- [ ] **Step 5: Keep right QEI and the 10 ms PID calls unchanged**

- [ ] **Step 6: Run all host tests and ARMClang compile**

Require all test executables and the Cortex-M0+ compilation of `motor_inf_task.c` to exit with code 0.

- [ ] **Step 7: Inspect scoped diff and clean temporary executables/objects**

