# 第二问设计文档：基本循线行驶（一圈自动停止）

## 概述

实现赛题第二问（基本循线行驶，16分）：题号=2 时，按下发车键后小车沿黑线顺时针行驶一圈，回到 A 点自动停止，计时显示在 OLED 上。

## 检测方式

使用 A 点启停线特征：L1、M、R1 三路灰度传感器同时检测到黑线（=1），L2、R2 丢线（=0）。

公式：`IS_AT_POINT_A()` = `huidu_value[1] && huidu_value[2] && huidu_value[3] && !huidu_value[0] && !huidu_value[4]`

起步后延迟 2s 再开始检测，避免出发位置误判。

## 状态机

```
LAP_IDLE → (enter=1, Q2) → LAP_RUNNING → (A点检测到) → LAP_COMPLETE
  ↑                                                          │
  └──────── (enter=1, 重新发车) ──────────────────────────────┘
```

- **IDLE**: 待命状态，OLED 显示 "Press ENTER"
- **RUNNING**: 行驶中，OLED 显示实时计时 "T: XX.XXs"
- **COMPLETE**: 完成状态，OLED 显示 "F: XX.XXs!"，电机停止

## 需要修改的文件

| 文件 | 修改内容 |
|---|---|
| `ucdriver/key/key.h` | 新增 `lap_state_t` 枚举、`lap_state`/`lap_start_ms`/`lap_total_ms` 声明 |
| `ucdriver/key/key.c` | 定义上述变量并初始化为 0 |
| `ucdriver/huidu/huidu.h` | 导出 `huidu_value[5]` 数组 |
| `ucdriver/motor/motor.c` | 在 `motor_pid_INST_IRQHandler` 30ms ISR 中添加状态机 + A点检测 |
| `main.c` | 修改 `oled_proc()` 显示 Q2 计时信息 |

## 执行流程

### 初始化
- `lap_state = LAP_IDLE`, `lap_start_ms = 0`, `lap_total_ms = 0`

### 按键切换至 question=2
- OLED 正常显示 "question: 2", 第三行显示 "Press ENTER"

### 按下 enter (PB7 中断 → enter=1)
- 30ms ISR 检测到 `enter==1 && question==2`
- `lap_state = LAP_IDLE` 或 `LAP_COMPLETE` → 转为 `LAP_RUNNING`
- 记录 `lap_start_ms = sys_tick_ms`

### 行驶中 (30ms ISR 每周期)
- 正常执行 `adjust_motor()` → `calculate_speed()` → `motor_PID()`
- Q2 检测：若 `lap_state==RUNNING && 已过2s`
  - 检查 `IS_AT_POINT_A()` → 满足则停车

### 回到 A 点
- 检测到 A 点条件 →
  - `lap_state = LAP_COMPLETE`
  - `lap_total_ms = sys_tick_ms - lap_start_ms`
  - `enter = 0`（触发电机停止）

### 再次按下 enter
- `enter=1` → ISR 检测 `lap_state==COMPLETE` → 重置为 `RUNNING`，重新计时

## OLED 显示布局

12-size 字体，每行 12 像素高：

- **Line 0 (y=0)**: `Status: RUN/STOP`
- **Line 1 (y=12)**: `question: 2`
- **Line 2 (y=24)**: 
  - IDLE: `Press ENTER`
  - RUNNING: `T: 12.34s`
  - COMPLETE: `F: 12.34s!`

## 对其他题号的影响

无影响。仅当 `question == 2` 时进入 lap 状态机逻辑，其他题号保持原有 `enter` 控制电机启停行为不变。
