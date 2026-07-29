# Task 3 Report: huidu.h — 导出 huidu_value 数组

## Status: ✅ 完成

## 变更
- **`ucdriver/huidu/huidu.h`**: 在 `extern volatile uint8_t huidu_straight_count;` 后添加 `extern uint8_t huidu_value[5];`

## 说明
使 `huidu_value[5]` 数组对 `motor.c` 和 `main.c` 可见，以便在 Question 2 的 A 点检测逻辑中直接读取各灰度传感器状态。

## Commits
- `b06a239` — Task 3: huidu.h — 导出 huidu_value 数组供 motor.c 和 main.c 使用
