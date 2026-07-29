# Task 4 Report: motor.c — Q2 Lap State Machine + Point-A Detection

## Status: ✅ Complete

## Changes Made

**File: `ucdriver/motor/motor.c`**

### 1. Added extern declarations (lines 15-20)
```c
extern volatile int question;
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
extern volatile uint32_t sys_tick_ms;
```
These variables are defined in `key.c` (lap_state_t, lap_start_ms, lap_total_ms, question) and `mpu_port.c` (sys_tick_ms).

### 2. Modified `motor_pid_INST_IRQHandler` (lines 183-208)
- **Removed `break`** after `motor_PID(2)` — now execution falls through to the `if(enter == 0)` stop block, even when Q2 completes and sets `enter = 0`.
- **Added Q2 lap state machine** inside `if(enter == 1)`:
  - `LAP_IDLE` or `LAP_COMPLETE` → `LAP_RUNNING`, capture `lap_start_ms`
  - `LAP_RUNNING` → after 2s delay, detect point A via gray sensors
- **Point A detection**: `huidu_value[1]` (L1) && `huidu_value[2]` (M) && `huidu_value[3]` (R1) == 1, AND `huidu_value[0]` (L2) && `huidu_value[4]` (R2) == 0
- On detection: `lap_state = LAP_COMPLETE`, `lap_total_ms` recorded, `enter = 0` to stop motors
- The outer `if(enter == 0)` block then sets both motors to 0 duty

## Verification Checklist
- [x] Original `break` removed — enter==0 block handles motor stop post-competition
- [x] `huidu_value` indices match declaration: `[0]=L2, [1]=L1, [2]=M, [3]=R1, [4]=R2`
- [x] `sys_tick_ms` extern resolved (declared in `mpu_port.c`, extern in `main.c`)
- [x] 2-second delay before A detection prevents false trigger at start
- [x] State machine: IDLE → RUNNING → COMPLETE, properly cycles

## Commit
`ebf2e70` — `motor.c: 添加 Q2 跑圈状态机 + A点检测逻辑`

## Concerns
- A点检测完全依赖灰度传感器的灵敏度标定，实际场地光照条件需测试验证
- 起步2s延迟硬编码，若实际A-B直线更长需调整
