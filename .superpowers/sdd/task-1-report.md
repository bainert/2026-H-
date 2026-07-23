# Task 1 Report: 重写 adjust_motor() 为比例控制

## Status: DONE

## Commits

- `3af7101` — refactor(huidu): 重写 adjust_motor() 为比例控制

## Summary

将 `adjust_motor()` 从基于 5 种传感器模式的离散 if-else 阶梯改为比例控制。使用 5 路灰度传感器加权偏差（权重 -2/-1/0/+1/+2），经归一化后通过 `target_speed = base_speed ± Kp * error` 连续调节左右轮速度。添加了 `huidu_base_speed`（基准速度）、`huidu_Kp`（比例系数）、`last_error`（丢线保持）三个静态变量，删除了废弃的 `target_speed_5[]` 数组。

## Concerns

1. **无法在本会话中进行编译验证** — 无 CCS 工具链。代码经人工审查符合 C11 语法。
2. **`huidu_value[]` 的类型是 `uint8_t` 而非 `bool`** — `sum` 累加时使用 `uint8_t`，与 `huidu_value[i]` 返回的 0/1 值兼容，无风险。
3. **`target_speed_1` / `target_speed_2` 是 `float`** — 最终输出到电机 PID 环节处理，支持浮点。

## Report File

`.superpowers/sdd/task-1-report.md`
