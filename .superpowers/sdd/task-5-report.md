# Task 5 Report: main.c — OLED 显示计时信息

## 状态: ✅ 完成

## 修改文件
- `main.c`

## 修改内容

### 1. 添加 lap 变量 extern 声明 (main.c:56-59)
在 `extern volatile uint32_t sys_tick_ms;` 之后添加了 Q2 跑圈状态变量的外部声明：
- `extern volatile lap_state_t lap_state;`
- `extern volatile uint32_t lap_start_ms;`
- `extern volatile uint32_t lap_total_ms;`

### 2. 修改 oled_proc() 函数 (main.c:80-100)
在第二行（题号）显示之后、`OLED_Refresh()` 之前添加了 Q2 计时显示逻辑：
- **LAP_IDLE**: 显示 "Press ENTER" 提示用户按键启动
- **LAP_RUNNING**: 实时显示 "T: xx.xxs"（已用时间，通过 `sys_tick_ms - lap_start_ms` 计算）
- **LAP_COMPLETE**: 显示 "F: xx.xxs!"（完成总时间）

## 接口
- 消费: `lap_state`, `lap_start_ms`, `lap_total_ms` (来自 key.c), `question`, `sys_tick_ms`
- OLED 坐标: 第三行 (y=24)
