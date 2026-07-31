# Q3 摆球控制（步进电机 + 视觉）设计文档

日期：2026-07-31
状态：已确认

## 1. 背景与目标

赛题 H 题第 3 问（静态摆球控制）：小车静止时，控制钢球从中心点 O 向 +5cm 处运行，到达后折返至 -5cm 处并稳定，运行时间 ≤5s，±5cm 处最大误差绝对值 ≤1cm。

本次实现：题号 Q3 + enter=1 时，通过步进电机（X42S，EMM 固件）控制水管（摆杆）摆动，使钢球稳定在水管中间（视觉数据中点）。

## 2. 硬件与通信

| 项目 | 配置 |
|---|---|
| 步进电机 | ZDT_X42S 第二代闭环步进电机（EMM 固件），1.8° 步进角，16 细分（1 脉冲 = 0.1125°，一圈 3200 脉冲） |
| 控制串口 | UART3（PA26=TX, PA25=RX），115200 8N1，已配置于 syscfg |
| 视觉串口 | UART0（PA28=TX, PA31=RX），115200 8N1，已配置于 syscfg |
| 视觉协议 | 3 字节帧 `0xaa data 0x55`，data = 小球距水管左端距离（mm，0~250），约 20Hz |
| 机械 | 电机轴经摇臂连接摆杆，无精确线性映射；电机角度限位 -24° ~ +80°（相对零点） |

角度限位换算（16 细分）：-24° = -213 脉冲，+80° = +711 脉冲。

## 3. 架构与数据流

```
UART0 RX (视觉 20Hz) ──帧解析(0xaa/data/0x55)──> ball.c 位置缓存(0~250mm)
                                                      │
30ms 定时器 ISR (现有 motor_pid) ──> ball_proc()：增量PID → 目标角度(脉冲)
                                                      │
                                          stepper.c 位置模式命令 ──UART3 TX──> X42S
                                          UART3 RX (02/12/E2/EE/9F应答) <────────┘
```

模块划分（方案 A，已确认）：

- `ucdriver/stepper/`：步进电机协议层（EMM 帧打包、命令应答解析、回零状态机）
- `ucdriver/ball/`：Q3 业务层（视觉帧解析、丢帧检测、增量 PID、运行状态机、OLED 数据）
- `main.c` / `motor.c` / `uart.c`：接线层改动（见 §7）

## 4. stepper 协议层（ucdriver/stepper/）

EMM 协议（校验固定 `0x6B`，手册 4.1 / 5.3.12 / 5.4.2 / 5.4.4 节）：

| 命令 | 帧 | 说明 |
|---|---|---|
| 触发回零 | `01 9A 00 00 6B` | 单圈就近回零（模式 00，同步 00），返回 02 / 12 |
| 读回零状态 | `01 3B 6B` | 返回标志：bit2=正在回零(0x04)，bit3=失败(0x08)，`(flag & 0x0C) == 0` 即完成 |
| 位置模式 | `01 FD 方向 速度(2B) 加速度 脉冲(4B) 运动模式00 同步00 6B` | 速度 0000-0BB8(0-3000RPM)，加速度 00-FF 档位，脉冲 4B 无符号 |

设计决策：用标准 `FD` 位置模式而非快速位置模式 `F1/FC`（后者需固件 V2.0.0+；FD 兼容所有版本，13 字节 @115200 传输约 1.1ms，30ms 周期下无性能问题）。

API：

```c
typedef enum { STEER_IDLE, STEER_HOMING, STEER_DONE, STEER_FAIL } stepper_state_t;

void    stepper_init(void);                 // 使能 UART3 接收中断
void    stepper_home_start(void);           // 发送 9A 回零命令
void    stepper_home_proc(void);            // 周期轮询 3B 回零状态（≥100ms 间隔），超时 5s 判失败
void    stepper_move(int32_t pulses);       // 有符号脉冲位置运动（内部拆分方向字节）
stepper_state_t stepper_get_state(void);
```

- 回零流程：HOMING 期间轮询 `3B`，标志 `&0x0C`：0x04 继续等待，0x08 失败（重试 1 次后 FAIL），0x00 完成 → DONE。
- `stepper_move()`：脉冲 ≥0 → 方向 00(CW) + 原值；脉冲 <0 → 方向 01(CCW) + 绝对值。
- 运动参数常量（可调）：`STEER_SPEED_RPM`、`STEER_ACC`。
- UART3 应答解析（ISR 中）：02=命令正确，12=已在零点/限位触发，E2=参数错误，EE=格式错误，9F=动作完成（回零完成主动上报）。应答记录到状态变量供上层读取。

## 5. ball 控制层（ucdriver/ball/）

### 5.1 视觉帧解析（UART0 ISR 改为状态机）

- 状态机：期望 `0xaa → data → 0x55`；首个字节非 0xaa 丢弃并重新等待。
- 校验：`data ≤ 250`，非法帧丢弃。
- 成功：更新 `ball_position_mm`（volatile，0~250mm）、`frame_cnt++`、`last_frame_ms = sys_tick_ms`。

### 5.2 丢帧检测

- 30ms 周期内检查：`sys_tick_ms - last_frame_ms > 500ms` → 置丢帧标志（OLED 提示），PID 停止输出增量（保持当前角度），避免无数据乱动。
- 恢复：收到新帧即清除标志。

### 5.3 增量式 PID（周期 30ms）

```
e  = 125.0f - ball_position_mm        // 中位 = 125mm
ΔU = Kp*(e - e1) + Ki*e + Kd*(e - 2*e1 + e2)
angle_pulse += ΔU
angle_pulse = clamp(angle_pulse, -213, +711)
stepper_move((int32_t)angle_pulse)
```

- 参数常量（初始量级，实测标定）：`BALL_KP = 5.0f`（脉冲/mm）、`BALL_KI = 0.1f`、`BALL_KD = 30.0f`。
- 进入 RUN 时清零 PID 历史（e1/e2）。
- 限幅常量与 stepper 层角度限位一致：`-213 ~ +711` 脉冲。

### 5.4 运行状态机

```
上电 ──> HOMING（回零轮询，OLED 显示）──> READY（零点 = 0 = 中位）
Q3 且 enter=1 ──> RUN（PID 控制）
enter=0 ──> 目标角度回 0（固定速度位置运动）──> READY
切走 Q3 ──> IDLE（电机停，PID 状态清零）
```

- 回零在上电后自动执行，与 enter 无关（用户要求"上电执行一次回零"）。
- 回零未完成时进入 Q3 也等回零完成才允许 RUN。
- 回零 FAIL：OLED 显示错误，不启动 RUN，等待重上电。

### 5.5 OLED 显示（Q3 复用第 3/4 行）

- 第 3 行：小球位置 mm（`P: 123mm`）+ 丢帧标记（无数据时显示 `LOST`）。
- 第 4 行：摆球状态（`HOMING` / `READY` / `RUN` / `FAIL`）+ 帧计数。

## 6. 关键常量汇总（可调）

| 常量 | 值 | 含义 |
|---|---|---|
| STEER_PULSES_PER_DEG | 8.889 (1/0.1125) | 度→脉冲换算 |
| STEER_MIN_PULSES / MAX | -213 / +711 | 角度限位 -24°/+80° |
| STEER_SPEED_RPM | 500（初始） | 位置模式速度 |
| STEER_ACC | 100（初始） | 加速度档位 0-255 |
| STEER_HOME_TIMEOUT_MS | 5000 | 回零超时 |
| BALL_KP / KI / KD | 5.0 / 0.1 / 30.0 | 增量 PID 参数（标定） |
| BALL_CENTER_MM | 125 | 水管中点 |
| BALL_FRAME_TIMEOUT_MS | 500 | 视觉丢帧判定 |

## 7. 接线层改动

- `main.c`：初始化流程加 `ball_init()`（内部含 stepper_init + 回零启动）；使能 `UART_3_INST_INT_IRQN`；`oled_proc()` 增加 Q3 分支显示。
- `motor.c`（30ms ISR）：`question == 3` 时跳过灰度循迹与电机 PID，两路占空比置 0（静态摆球，车不动），调用 `ball_proc()`；保留按键消抖。非 Q3 时 `ball_proc()` 不执行。
- `uart.c`：UART0 ISR 由回显改为视觉帧状态机解析（`ball_uart_rx()` 回调）。
- `stepper.c`：定义 `UART_3_INST_IRQHandler` 解析电机应答。

## 8. 构建集成（无需改 syscfg）

按 AGENTS.md 流程：

1. 新增 `ucdriver/stepper/stepper.{c,h}`、`ucdriver/ball/ball.{c,h}`
2. `Debug/sources.mk`：SUBDIRS 加两个目录
3. 各目录 `subdir_vars.mk`：注册 .c 文件
4. `Debug/makefile`：ORDERED_OBJS 末尾加 `.o` 目标

## 9. 测试验证

- CCS 内编译通过（本环境无法构建，只能语法/结构自审后由用户在 CCS 构建）。
- 串口调试验证（UART3 上位机观察应答）：
  - 上电 → 观察回零完成（OLED HOMING→READY）
  - 发 `0xaa 0x7d 0x55`（125mm）→ 电机不动或小幅修正
  - 发 `0xaa 0x5f 0x55`（95mm）→ 摆杆倾斜补偿，小球回中
  - 停发数据 >500ms → OLED 显示 LOST，电机保持
- 实车标定：调 BALL_KP/KI/KD 直到小球 2s 内稳定在中点 ±1cm。

## 10. 非目标（YAGNI）

- 不做 Q6（任意位置稳定）的状态，但模块划分已为其预留（改 ball 层目标值即可）。
- 不做视觉数据滑动平均（帧率 20Hz + PID 本身可滤，先实车验证，抖动明显再补）。
- 不做多电机 CAN 同步、心跳保护、限位开关接线（机械无开关，用单圈回零）。
