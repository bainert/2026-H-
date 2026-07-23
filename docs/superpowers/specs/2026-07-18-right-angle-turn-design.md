# 直角转弯 — adjust_motor() 丢线急转逻辑

## 问题

当前 `adjust_motor()` 在 5 路灰度传感器全丢线（`sum == 0`）时，仅使用 `last_error` 维持上一次有效偏差进行比例控制。该方式在**线位于两个传感器间隙**导致的短暂（1~2 周期）丢线时尚可工作，但在**直角转弯**场景下，驱动力不足，车无法拐过 90° 弯。

## 设计

### 丢线计数区分两种场景

新增 `static uint8_t lost_count`，在 `adjust_motor()` 的 `sum == 0` 分支递增，在 `sum > 0` 时清零。

- **短暂丢线（lost_count < 3）**：仍使用 `last_error` 维持比例控制，与原逻辑一致
- **持续丢线（lost_count >= 3，≥90ms）**：判定为直角转弯，进入急转模式

### 急转模式

根据 `last_error` 符号确定转向方向：

| last_error | 意义 | 左轮 (motor 1) | 右轮 (motor 2) |
|---|---|---|---|
| > 0 | 之前偏右，继续右转 | 500 mm/s | 0 |
| < 0 | 之前偏左，继续左转 | 0 | 500 mm/s |
| == 0 | 无法判断方向 | huidu_base_speed | huidu_base_speed |

M 传感器重新检测到线后（`sum > 0`），下一周期自动回到正常比例循迹。

### 影响范围

只修改 `ucdriver/huidu/huidu.c` 中的 `adjust_motor()` 实现。接口 `void adjust_motor()` 不变。

### 参数

| 参数 | 值 | 说明 |
|---|---|---|
| lost_count 阈值 | 3 | 对应 ~90ms，可调 |
| 转弯速度 | 500 mm/s | 旋转轮目标速度 |
