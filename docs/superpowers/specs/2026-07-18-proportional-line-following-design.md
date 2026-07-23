# 比例循迹 — adjust_motor() 平滑化改造

## 问题

`adjust_motor()` 使用离散 if-else 阶梯，传感器状态到电机速度的映射存在跳变，转弯时产生明显卡顿。

## 解决方案

用 5 路灰度传感器的加权位置连续计算偏差 `error`，通过比例系数 `Kp` 连续调节两侧轮速。

### 传感器加权

| 传感器 | 权重 |
|--------|------|
| L2     | -2   |
| L1     | -1   |
| M      | 0    |
| R1     | +1   |
| R2     | +2   |

```
error = Σ(weight_i * value_i) / Σ(value_i)
```

范围 `[-2, +2]`。分母为 0 时表示丢线。

### 速度控制公式

```c
target_speed_1 = base_speed + Kp * error;   // motor 1 = 左轮
target_speed_2 = base_speed - Kp * error;   // motor 2 = 右轮
```

- `error < 0`：线偏左 → 左轮减速、右轮加速 → 左转
- `error > 0`：线偏右 → 左轮加速、右轮减速 → 右转
- `error = 0`：居中 → 直行

### 边界处理

| 情况 | 行为 |
|------|------|
| 正常循迹（至少一个传感器为 1） | 计算 error，比例调速 |
| 丢线（全部为 0） | 保持最近一次有效 error，继续前进 |
| 全白（全部为 1） | error = 0，直行 |

### 新增参数

- `huidu_base_speed`：可调基准速度
- `huidu_Kp`：比例系数，初始推荐 `base_speed * 0.2`

### 改动范围

只修改 `ucdriver/huidu/huidu.c` 中 `adjust_motor()` 的实现。接口 `void adjust_motor()` 不变。

## 文件

- `ucdriver/huidu/huidu.c` — 实现修改

## 测试

编译通过，实际跑车观察转弯是否平滑。
