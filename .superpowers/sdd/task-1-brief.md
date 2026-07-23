### Task 1: 重写 adjust_motor() 为比例控制

**Files:**
- Modify: `ucdriver/huidu/huidu.c:24-79`

**Interfaces:**
- Consumes: `target_speed_1` (motor.c), `target_speed_2` (motor.c), `huidu_get_value()` (本文件)
- Produces: `adjust_motor()` — 设置 `target_speed_1` 和 `target_speed_2`

- [ ] **Step 1: 在 huidu.c 添加比例参数和 last_error 变量**

```c
// 比例控制参数
static float huidu_base_speed = 300.0f;  // 可调基准速度 mm/s
static float huidu_Kp = 60.0f;           // 比例系数
static float last_error = 0.0f;          // 上次有效偏差，丢线时保持
```

- [ ] **Step 2: 重写 adjust_motor() 函数**

将整个 `adjust_motor()` 函数替换为比例控制版本：

```c
void adjust_motor()
{
    uint8_t i;
    float error = 0.0f;
    uint8_t sum = 0;

    // 权重数组: L2=-2, L1=-1, M=0, R1=+1, R2=+2
    const int8_t weights[5] = {-2, -1, 0, 1, 2};

    huidu_get_value();

    // 计算加权偏差
    for(i = 0; i < 5; i++)
    {
        error += (float)(huidu_value[i] * weights[i]);
        sum += huidu_value[i];
    }

    if(sum > 0 && sum < 5)
    {
        // 正常循迹: 至少一个传感器检测到线，但不是全白
        error /= (float)sum;  // error 范围 [-2, +2]
        last_error = error;   // 记录有效偏差
    }
    else if(sum == 0)
    {
        // 丢线: 保持上次有效偏差
        error = last_error;
    }
    else  // sum == 5, 全白
    {
        // 全白: 直行
        error = 0.0f;
    }

    // 比例控制: 左轮减速 = 右轮加速
    target_speed_1 = huidu_base_speed + huidu_Kp * error;   // motor 1 = 左轮
    target_speed_2 = huidu_base_speed - huidu_Kp * error;   // motor 2 = 右轮

    // 钳位到有效范围
    if(target_speed_1 < 0.0f) target_speed_1 = 0.0f;
    if(target_speed_2 < 0.0f) target_speed_2 = 0.0f;
}
```

- [ ] **Step 3: 清理不再使用的变量**

删除 `huidu.c:22` 的 `float target_speed_5[] = {125, 175, 200, 400, 500};`

保留 `extern float target_speed_1; extern float target_speed_2;`（huidu.c:19-20）

- [ ] **Step 4: 编译验证**

在 CCS 中执行构建，确认无编译错误。

- [ ] **Step 5: 实车调试**

下载到开发板，走循迹路线观察转弯是否平滑。如有需要调整 `huidu_Kp` 参数（偏大则转向过冲震荡，偏小则转弯不足）。
