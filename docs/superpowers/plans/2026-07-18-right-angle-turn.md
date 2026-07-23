# 直角转弯丢线急转 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Modify `adjust_motor()` 的 `sum == 0` 分支，加入丢线计数区分短暂丢线与直角转弯，持续丢线时进入急转模式（一侧停转、另一侧 500 mm/s）。

**Architecture:** 只修改 `ucdriver/huidu/huidu.c` 中 `adjust_motor()` 内部逻辑，接口不变。增加一个 `static uint8_t lost_count` 计数器，30ms 周期自然驱动的退出条件。

**Tech Stack:** 裸机 C11, MSPM0G3507, tiarmclang

## Global Constraints

- 只修改 `ucdriver/huidu/huidu.c`，不修改其他文件
- 接口 `void adjust_motor()` 保持不变
- `lost_count` 阈值 3（~90ms），转弯速度 500 mm/s

---

### Task 1: 修改 adjust_motor() 丢线分支

**Files:**
- Modify: `ucdriver/huidu/huidu.c:27-69`

**Interfaces:**
- Consumes: `extern float target_speed_1`, `extern float target_speed_2`, `static float last_error`
- Produces: `adjust_motor()` — 增加 `static uint8_t lost_count`，修改 `sum == 0` 分支逻辑

- [ ] **Step 1: 修改 adjust_motor()**

在 `adjust_motor()` 中添加 `static uint8_t lost_count = 0;`，修改 `sum == 0` 分支如下：

```c
    if(sum > 0 && sum < 5)
    {
        // 正常循迹: 至少一个传感器检测到线，但不是全黑
        lost_count = 0;
        error /= (float)sum;
        last_error = error;
    }
    else if(sum == 0)
    {
        // 丢线
        lost_count++;
        if(lost_count >= 3)
        {
            // 连续丢线 >= 90ms → 直角转弯，进入急转模式
            if(last_error > 0.0f)
            {
                target_speed_1 = 500.0f;
                target_speed_2 = 0.0f;
            }
            else if(last_error < 0.0f)
            {
                target_speed_1 = 0.0f;
                target_speed_2 = 500.0f;
            }
            else
            {
                target_speed_1 = huidu_base_speed;
                target_speed_2 = huidu_base_speed;
            }
            return;  // 跳过比例计算和钳位
        }
        else
        {
            // 短暂丢线（传感器间隙），保持上次有效偏差
            error = last_error;
        }
    }
    else  // sum == 5, 全黑
    {
        lost_count = 0;
        error = 0.0f;
    }
```

`sum == 5` 分支也要添加 `lost_count = 0;`。

- [ ] **Step 2: 检查编译**

检查 `huidu.c` 中头文件包含和变量声明是否完整（`static uint8_t lost_count` 在函数内，不需要额外头文件）。

- [ ] **Step 3: 提交**

```bash
git add ucdriver/huidu/huidu.c
git commit -m "feat: adjust_motor() 直角转弯丢线急转逻辑"
```
