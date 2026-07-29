# Task 2 Report: key.c — 定义 lap 状态变量

## 变更摘要

在 `ucdriver/key/key.c` 的静态变量定义区末尾（`key_debounce_cnt` 之后）添加了三个全局变量定义：

```c
volatile lap_state_t lap_state = LAP_IDLE;
volatile uint32_t lap_start_ms = 0;
volatile uint32_t lap_total_ms = 0;
```

这些变量与 `key.h`（Task 1）中的 `extern` 声明配对，供 Question 2 的跑圈计时逻辑使用。

## 已修改文件

- `ucdriver/key/key.c` — 添加 5 行（+ 3 定义 + 1 注释 + 1 空行）

## 提交

```
997656e Task 2: key.c — 添加跑圈状态变量定义 (lap_state, lap_start_ms, lap_total_ms)
```

## 验证

- `git diff HEAD~1` — 确认仅在指定位置添加了 5 行
- `gcc -fsyntax-only` — N/A（嵌入式交叉编译，CCS 构建）
- 与 `key.h` 的类型/命名一致
