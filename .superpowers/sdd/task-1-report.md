# Task 1 报告：stepper 步进电机协议层（EMM 帧打包 + UART3 应答）

## 状态
DONE

## 创建的文件清单
- `ucdriver/stepper/stepper.h` — 接口声明：`stepper_ack`（extern volatile uint8_t）、`stepper_init()`、`stepper_home_start()`、`stepper_move(int32_t pulses)`
- `ucdriver/stepper/stepper.c` — 实现：EMM 帧打包（回零 9A / 位置模式 FD）、`stepper_send_frame()` 经 UART3 发送、`UART_3_INST_IRQHandler` 解析应答字节

## 自审结果（对照 brief Step 3）
1. **`stepper_send_frame` 使用 `UART_3_INST`**：确认 `Debug/ti_msp_dl_config.h:169` 定义 `UART_3_INST = UART3`。✅
2. **`uart_send_char` 声明于 uart.h**：`ucdriver/uart/uart.h:7` 声明 `void uart_send_char(UART_Regs *uart, const uint8_t data)`，签名与调用一致；stepper.c 已 include "uart.h"。✅
3. **帧长度 13 字节**：Addr(1) + Code(1) + 方向(1) + 速度(2) + 加速度(1) + 脉冲(4) + 运动模式(1) + 同步(1) + 校验(1) = 13，`frame[13]` 数组与 sizeof 一致。✅
4. **FD 帧字段与手册 5.3.12 一致**：帧头 01(addr) FD(位置模式) dir 速度2B 加速度 脉冲4B 运动模式01(绝对) 同步00 6B(CRC)，字节序为大端。✅
5. **ISR 名称**：`UART_3_INST_IRQHandler` 经宏展开为 `UART3_IRQHandler`（ti_msp_dl_config.h:171），与启动文件向量表匹配。✅
6. **头文件依赖**：stepper.h 包含 `<stdint.h>` 供 int32_t/uint8_t；stepper.c 先包含 stepper.h 再包含 ti_msp_dl_config.h / uart.h，与 uart.c 风格一致。✅
7. **volatile 一致**：头文件 `extern volatile uint8_t stepper_ack`，.c 定义 `volatile uint8_t stepper_ack = 0`。✅

## 编码说明
brief 文件本身中文因 GBK/UTF-8 编码错乱显示为乱码，已按原意还原为正确中文注释；C 代码部分逐字照抄。

## 提交
- commit hash: 见下方

## 未编译说明
本任务无编译条件（工程依赖 CCS 硬编码绝对路径），正确性由上述自审保证。
