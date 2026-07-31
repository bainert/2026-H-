# Task 2 报告：ball 控制层（视觉解析 + 丢帧检测 + 增量 PID + 状态机）

## 状态
DONE_WITH_CONCERNS（逻辑与 brief 完全一致，无编译条件）

## 创建的文件清单
- `ucdriver/ball/ball.h`（新建，31 行）：接口声明 + 3 个全局标志 extern
- `ucdriver/ball/ball.c`（新建，140 行）：帧解析状态机 / 丢帧检测 / 增量 PID / 状态机 / 回零初始化

## 自审结果（brief Step 3 检查点）
1. **`ball_uart_rx` 状态机覆盖 0xAA/0x55 重入**：state0 等 0xAA 帧头；state1 无条件存 data 进 state2（与 brief 逐字一致）；state2 遇 0x55 提交完整帧（更新 `ball_position_mm`/`last_frame_ms`/清丢帧），遇 0xAA 视为丢 data 字节按新帧头回到 state1，其他字节重置 state0。通过。
2. **`question/enter` 来自 key.h extern**：ball.c include key.h，`extern volatile int enter/question` 已在 key.h:6-7 声明。通过。
3. **`sys_tick_ms`**：ball.c 中 `extern volatile uint32_t sys_tick_ms` 声明；符号实际定义在 `ucdriver/mpu6050/mpu_port.c:9`，`main.c:66` SysTick_Handler 递增。链接无冲突。通过（见 concern 1）。
4. **PID 历史仅在 RUN 分支更新**：`pid_e1/pid_e2` 只在 `enter==1 && !frame_lost` 分支更新；丢帧保持分支与 enter=0 分支不动/清零，非 Q3 时清零，防积分。通过。
5. **限幅常量与规格一致**：CENTER=125.0f、MIN=-213 / MAX=711 脉冲（-24°~+80° @0.1125°/脉冲）、超时 500ms、KP=5.0 / KI=0.1 / KD=30.0、DU_MAX=100。与 brief 逐字一致。通过。
6. **丢帧时 `last_sent_pulse` 不变 → 不重复发命令**：丢帧分支不改 `target_pulse`，末尾 `target_pulse != last_sent_pulse` 不触发，不重复下发 `stepper_move`。通过。

## 额外核对
- 代码逻辑与 brief 逐字一致（含 `frame_data = byte` 在 state1 无条件赋值等细节），中文注释按语义还原。
- 未触碰工作区其他改动，仅提交 ball 两文件。

## Concerns
1. 任务描述称 `sys_tick_ms` 在 main.c 定义，实际定义在 `mpu_port.c`（main.c 仅 extern 声明+递增）。既有事实，不影响链接，但如有意移除 MPU6050 编译单元需注意。
2. 本任务无编译条件（CCS 外无法构建，AGENTS.md 已说明），正确性仅经静态自审。
3. `ball_proc` 需外部接入 `motor_pid_INST_IRQHandler`（30ms 周期）与 UART0 ISR 回调 `ball_uart_rx`，属后续任务范围。

## Commit
- `ae429fa` feat: 新增 ball 摆球控制层（视觉解析/丢帧检测/增量PID/状态机）
