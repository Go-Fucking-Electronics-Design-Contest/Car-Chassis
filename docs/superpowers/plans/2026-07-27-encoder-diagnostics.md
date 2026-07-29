# Encoder Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Instrument the failed TIMG7/software-quadrature path and the working TIMG8/QEI path so one wheel rotation identifies the exact stage that loses counts.

**Architecture:** Keep the existing logical names unchanged: logical left is the failed physical-right TIMG7 path, and logical right is the working physical-left TIMG8 path. Add volatile observation counters at the ISR, decoder, and speed-sampling boundaries without altering control outputs or encoder arithmetic.

**Tech Stack:** C11-compatible embedded C, TI MSPM0 DriverLib, Keil MDK ARM compiler, MSPM0G3507 hardware.

## Global Constraints

- Do not create a Git commit.
- Do not swap logical left/right control variables during diagnosis.
- Do not change timer, motor output, encoder count, or speed-estimation behavior.
- Validate on real encoder inputs with Keil Watch because host tests cannot reproduce TIMG7 capture hardware.

---

### Task 1: Expose encoder diagnostic state

**Files:**
- Modify: `src/User/Task/motor_inf_task.h`
- Modify: `src/User/Task/motor_inf_task.c`

**Interfaces:**
- Consumes: TIMG7 pending interrupt index, sampled A/B levels, decoded delta, TIMG8 timer count and delta.
- Produces: global `volatile` variables prefixed with `motor_diag_`.

- [ ] **Step 1: Establish the failing hardware observation**

Run the current firmware, rotate the physical-right wheel, and confirm
`motor_left_encoder_count == 0` while the physical-left wheel changes
`motor_right_encoder_count`. This is the RED hardware observation supplied
by the user.

- [ ] **Step 2: Declare diagnostic variables**

Add public `extern volatile` declarations for TIMG7 ISR totals, per-channel
totals, unhandled IIDX totals, last IIDX, last AB states/delta, transition
class totals, and TIMG8 raw-count comparison data.

- [ ] **Step 3: Record TIMG7 ISR and decoder observations**

Read the pending interrupt index exactly once into a local variable. Record
it before the switch, increment the corresponding CC0/CC1 counter, then
record `prev`, `curr`, and `delta` inside the existing decoder. Classify
`prev == curr` as same-state, `delta != 0` as valid, and every other
transition as invalid.

- [ ] **Step 4: Record the working TIMG8 reference**

At the existing TIMG8 sample point, copy the raw 16-bit count and signed
delta into diagnostic variables without changing the original calculation.

- [ ] **Step 5: Reset all diagnostic variables**

Initialize all `motor_diag_` variables in `Motor_Inf_ResetEncoder()` so each
debug session begins with unambiguous values.

- [ ] **Step 6: Build**

Run the existing Keil/ARM build. Expected result: zero compile and link
errors, and every `motor_diag_` symbol is visible in the map/debug image.

- [ ] **Step 7: Inspect the diff**

Run `git diff -- src/User/Task/motor_inf_task.c src/User/Task/motor_inf_task.h`.
Expected result: diagnostic-only changes, no Git commit, and no change to
motor output or speed arithmetic.

### Task 2: Hardware classification

**Files:**
- No source changes until the collected values identify the failing stage.

**Interfaces:**
- Consumes: all `motor_diag_` values from Task 1.
- Produces: one evidence-based root-cause classification.

- [ ] **Step 1: Rotate only the physical-right wheel**

Observe TIMG7 ISR total, CC0 count, CC1 count, last IIDX, unhandled count,
last AB states/delta, and transition-class totals in Keil Watch.

- [ ] **Step 2: Rotate only the physical-left wheel**

Observe the TIMG8 raw count/delta as the working reference and confirm that
the physical-left wheel maps to logical-right variables.

- [ ] **Step 3: Classify**

- ISR total zero: pin mux, timer start, NVIC, or electrical route problem.
- ISR total grows but unhandled grows: IIDX/event selection mismatch.
- Only CC0 or CC1 grows: missing capture input/channel or wiring problem.
- Both channels grow but invalid transitions dominate: AB phase/order,
  simultaneous sampling, or missed-edge problem.
- Valid transitions grow but logical-left count remains zero: accumulator
  corruption or an unexpected reset path.

- [ ] **Step 4: Design the minimal fix**

Apply only the fix justified by the observed classification, then separately
decide whether to rename/remap logical left/right variables to physical wheel
semantics.

### Task 3: Repair TIMG7 software quadrature state

**Files:**
- Modify: `src/User/Task/motor_inf_task.c`

**Interfaces:**
- Consumes: initial PA17/PA18 GPIO levels and TIMG7 CC0/CC1 capture events.
- Produces: valid AB transitions in `motor_left_encoder_count` without GPIO
  reads inside the capture ISR.

- [ ] **Step 1: Synchronize the initial AB state**

Stop TIMG7 in a critical section, temporarily select GPIO input mode for
PA17/PA18, read both levels, restore TIMG7_CCP0/CCP1 pin functions, clear
capture status, and restart the timer.

- [ ] **Step 2: Reconstruct each captured edge**

For `CC0_UP`, XOR the current software AB state with `0x02`. For `CC1_UP`,
XOR it with `0x01`. Pass the reconstructed levels through the existing
quadrature lookup decoder.

- [ ] **Step 3: Verify**

Compile `motor_inf_task.c` with the project's ArmClang 6.16 options. On the
board, rotate the physical-right wheel and require both CC counters and valid
transition count to grow, same-state count to remain unchanged, and logical
left count/speed to become nonzero.
