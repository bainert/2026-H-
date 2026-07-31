# Task 3 接线层报告（uart.c / motor.c / main.c）

## 修改文件清单

| 文件 | 修改位置 | 说明 |
|---|---|---|
| `ucdriver/uart/uart.c` | 头部 + 整个 `UART_0_INST_IRQHandler` | UART0 ISR 由回显改为视觉帧解析 |
| `ucdriver/motor/motor.c` | 头部 + `motor_pid_INST_IRQHandler` 中 `case DL_TIMER_IIDX_LOAD:` 分支整段 | Q3 停小车不循迹 + 无条件调用 `ball_proc()` |
| `main.c` | include 区 / extern 区 / `main()` / `oled_proc()` | ball.h include、extern 声明、`ball_init()`、OLED 第三/四行 Q3 分支 |

## 每处修改的 diff 摘要

### ucdriver/uart/uart.c
- 头部：`#include "uart.h"` 之后新增 `#include "ball.h"`
- ISR 替换：
  - 函数注释改为「串口接收中断处理函数（视觉数据）」
  - `case DL_UART_IIDX_RX:` 内 `uart_send_char(UART_0_INST, data);`（回显）→ `ball_uart_rx(data);`
  - 其余结构（switch/getPendingInterrupt/default）保持不变

### ucdriver/motor/motor.c
- 头部：`#include "key.h"` 之后新增 `#include "ball.h"`
- `case DL_TIMER_IIDX_LOAD:` 分支（原 176-217 行）整段替换为 brief 提供代码：
  - `if(enter == 1)` 内新增 `if(question == 3)` 分支：`motor_set_duty(1, 0); motor_set_duty(2, 0);`（Q3 停车不循迹）
  - `else` 分支：原灰度循迹 + 里程累计 + 速度计算 + PID + Q2 跑圈状态机逻辑整体保留（缩进 +4 空格）
  - `if(enter == 0)` 停车逻辑保留不变
  - 分支末尾新增无条件 `ball_proc();`（30ms 周期，非 Q3 时 ball.c 内部自动待命）

### main.c
- include 区：`#include "motor.h"` 之后新增 `#include "ball.h"`
- extern 区：`extern volatile uint32_t sys_tick_ms;` 之后新增 `ball_position_mm / ball_frame_lost / ball_running` 三个 extern 声明
- `main()`：`NVIC_EnableIRQ(dc_motor_GPIOA_INT_IRQN);` 之后、`motor_init(1);` 之前新增 `ball_init();`（含步进电机回零，阻塞约 1s；此时 OLED 已初始化，显示正常）
- `oled_proc()` 第三行（里程计）替换为 Q3/else 双分支：Q3 显示 `P:xxxmm` 或 `LOST`，否则保持 `D:x.xm`
- `oled_proc()` 第四行（Q2 计时）替换为三分支：Q2 计时不变 / Q3 显示 `BAL:RUN|STOP` / 其他清空

## 自审结果（brief Step 4 检查点）

- [x] `uart.c` 引用 `ball_uart_rx` 需 `ball.h`（Task 2 已提供，`ucdriver/ball/ball.h:18` 声明存在）
- [x] `motor.c` 引用 `ball_proc` 需 `ball.h`（`ucdriver/ball/ball.h:16` 声明存在）
- [x] Q3 分支保留了 `enter==0` 时的停车逻辑（`if(enter == 0)` 未改动）
- [x] `ball_proc()` 无条件调用（非 Q3 时 ball.c 内部 `return` 待命，开销可忽略）
- [x] `main.c` 的 `ball_init()` 放在中断使能（UART/按键/编码器）之后、`motor_init` 之前；回零 1s 阻塞期间 OLED 已初始化，显示正常
- [x] 修改前后上下文核对：三处修改的原文均与 brief 描述的行号/上下文一致；非修改区逻辑零改动
- [x] 注释按中文风格还原（brief 为 GBK 乱码，按含义还原）

## Commit
- hash: 见提交输出
- 信息: `feat: Q3 接线层 — UART0 视觉解析/电机Q3停转/OLED显示小球位置`
