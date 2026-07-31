# Q3 静态摆球运动序列实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Q3 + enter=1 时，小球自动完成 中心O(125mm) → +5cm(175mm) → 折返 → -5cm(75mm) 保持 的运动序列。

**Architecture:** 在 `ball.c` 现有位置式 PID 闭环骨架上，把固定目标 `BALL_CENTER_MM` 换成由状态机更新的动态目标 `ball_target_mm`，新增"误差≤8mm 持续 300ms"到达判定与目标切换防冲击处理。状态机跑在 10ms 周期 `ball_proc()` 内。

**Tech Stack:** 裸机 C (C11)，tiarmclang，MSPM0G3507，MSPM0 SDK 2.10 DriverLib。

## Global Constraints

- 项目无法脱离 CCS 构建（`Debug/makefile` 含硬编码绝对路径），**不新增文件**，无需改 `Debug/sources.mk` / `Debug/makefile`
- 坐标系：中心=125mm，+5cm=175mm，-5cm=75mm（`ball_position_mm` 为 0~250 的视觉测量值）
- 视觉帧：`0xAA data 0x55`，`ball_uart_rx()` 已解析，本计划不改动
- 注释使用中文，按项目现有风格（块注释解释"为什么"）
- 非 Q3 / enter=0 时行为必须与现状一致（待命回中）
- PID 原参数 KP=1.1 / KI=0.01 / KD=10.5 不变，只通过 `BALL_MOVE_KP_GAIN` 行程倍率调快

---

### Task 1: ball.c 目标点状态机 + 到达判定 + 防冲击

**Files:**
- Modify: `ucdriver/ball/ball.c`（宏区、全局变量区、`ball_proc()`）
- Modify: `ucdriver/ball/ball.h`（新增 `ball_phase` 声明）

**Interfaces:**
- Consumes: `question`、`enter`（key.c 已有）、`sys_tick_ms`（main.c 已有）、`stepper_move()`（stepper.c 已有）
- Produces: `volatile uint8_t ball_phase`（0=待命/中心 1=前往+5 2=折返-5 3=完成保持），Task 2 的 OLED 显示依赖它

- [ ] **Step 1: 修改 `ball.h`，新增 `ball_phase` 声明**

在 `ball.h` 的 `ball_running` 声明后新增一行：

```c
//摆球运动阶段：0-待命(中心) 1-前往+5cm 2-折返-5cm 3-完成保持
extern volatile uint8_t ball_phase;
```

- [ ] **Step 2: 修改 `ball.c` 宏区，替换固定中心目标为动态目标点组**

将 `ball.c:7-9` 的宏：

```c
//水管中点（mm）
#define BALL_CENTER_MM      125.0f
```

替换为：

```c
//目标点（mm）：中心O / ±5cm（视觉坐标系 0~250，中心 125）
#define BALL_TARGET_CENTER_MM  125.0f
#define BALL_TARGET_PLUS5_MM   175.0f
#define BALL_TARGET_MINUS5_MM  75.0f
//到达判定：|位置-目标|≤8mm 且持续 30×10ms=300ms 视为到位
//（8mm 留 2mm 裕量给 ±1cm 赛题要求，300ms 抗视觉抖动误判）
#define BALL_ARRIVE_ERR_MM     8.0f
#define BALL_ARRIVE_CNT_MAX    30
//行程段比例增益倍率：实测跑得慢（>4s）时调大，如 1.5/2.0；回中过冲则调小
//（只乘在 KP 上，不污染原参数，稳态精度由 KI 积分保证）
#define BALL_MOVE_KP_GAIN      1.0f
```

- [ ] **Step 3: 修改 `ball.c` 全局变量区，新增状态机变量**

在 `ball.c:34` 的 `ball_running` 声明后新增：

```c
//摆球运动阶段：0-待命(中心) 1-前往+5cm 2-折返-5cm 3-完成保持
volatile uint8_t ball_phase = 0;
//当前目标点（mm，由状态机更新，替代原固定 BALL_CENTER_MM）
static float ball_target_mm = BALL_TARGET_CENTER_MM;
//到达判定计数器（10ms 周期累加，>=30 判到达）
static uint8_t arrive_cnt = 0;
//目标切换防冲击标志：置位后 PID 第一拍微分项清零
static uint8_t target_changed = 0;
```

- [ ] **Step 4: 重写 `ball_proc()` 为状态机版本**

将 `ball.c:101-167` 整个 `ball_proc()` 函数体替换为：

```c
//10ms 周期调用（motor_pid 定时器 ISR 中）
void ball_proc(void)
{
    //非 Q3：待命，清理 PID 状态，电机不动
    if(question != 3)
    {
        pid_e1 = 0.0f;
        ball_integral = 0.0f;
        ball_running = 0;
        ball_phase = 0;
        ball_target_mm = BALL_TARGET_CENTER_MM;
        arrive_cnt = 0;
        target_changed = 0;
        return;
    }

    //丢帧检测：超过500ms无新帧
    if(sys_tick_ms - last_frame_ms > BALL_FRAME_TIMEOUT_MS)
    {
        ball_frame_lost = 1;
    }

    if(enter == 1 && ball_frame_lost == 0)
    {
        //目标点状态机：中心O → +5cm → 折返 → -5cm 保持
        switch(ball_phase)
        {
        case 0:
            //S0→S1：enter=1 启动，目标改为 +5cm（175mm）
            ball_target_mm = BALL_TARGET_PLUS5_MM;
            ball_phase = 1;
            arrive_cnt = 0;
            target_changed = 1;   //防冲击：第一拍微分清零
            break;
        case 1:
            //S1→S2：到达 +5cm（|误差|≤8mm 持续 300ms）后折返 -5cm
            if(ball_position_mm >= BALL_TARGET_PLUS5_MM - BALL_ARRIVE_ERR_MM
            && ball_position_mm <= BALL_TARGET_PLUS5_MM + BALL_ARRIVE_ERR_MM)
            {
                if(++arrive_cnt >= BALL_ARRIVE_CNT_MAX)
                {
                    ball_target_mm = BALL_TARGET_MINUS5_MM;
                    ball_phase = 2;
                    arrive_cnt = 0;
                    target_changed = 1;
                }
            }
            else
            {
                //一次出窗就清零，必须连续 300ms 才算到达
                arrive_cnt = 0;
            }
            break;
        case 2:
            //S2→S3：到达 -5cm 后进入保持阶段
            if(ball_position_mm >= BALL_TARGET_MINUS5_MM - BALL_ARRIVE_ERR_MM
            && ball_position_mm <= BALL_TARGET_MINUS5_MM + BALL_ARRIVE_ERR_MM)
            {
                if(++arrive_cnt >= BALL_ARRIVE_CNT_MAX)
                {
                    ball_phase = 3;
                    arrive_cnt = 0;
                }
            }
            else
            {
                arrive_cnt = 0;
            }
            break;
        case 3:
        default:
            //S3：保持 -5cm 不动（闭环继续，直到 enter=0）
            break;
        }

        //RUN：位置式 PID，输出直接为目标角度（脉冲），
        //球一偏离目标点立即得到大角度 → 加速度瞬间建立，响应快
        float e = ball_target_mm - (float)ball_position_mm;
        //误差死区：|e|<=1mm 视为到位（视觉分辨率 1mm，更小无法分辨）
        //【关键】死区若加在增量 du 上：稳态时 KP/KD 项恒为 0，只剩 KI*e=0.1*e，
        //任何 <180mm 的稳态误差产生的 du 都 < 18 脉冲被丢弃 → 永不回中，KP 调多大都无效
        if(e > -BALL_ERR_DEADZONE_MM && e < BALL_ERR_DEADZONE_MM) e = 0.0f;

        //【防冲击】目标刚切换的第一拍：微分记忆对齐当前误差，
        //KD*(e-e1)=0，避免 e 突变 50mm 时微分项输出 ~525 脉冲被打满限幅
        if(target_changed)
        {
            target_changed = 0;
            pid_e1 = e;
        }

        //积分项（限幅防 windup：球必须能停在偏离中心的位置，积分负责补足
        //"摆杆保持固定角度"所需的持续输出——增量式 PID 做不到这点）
        ball_integral += e;
        if(ball_integral >  BALL_INTEGRAL_MAX) ball_integral =  BALL_INTEGRAL_MAX;
        if(ball_integral < -BALL_INTEGRAL_MAX) ball_integral = -BALL_INTEGRAL_MAX;

        //位置式输出：target_pulse = KP*e + KI*Σe + KD*de/dt
        //（de/dt 用相邻两帧误差差分近似，一阶差分即足够；KP 可乘行程倍率）
        target_pulse = (int32_t)((BALL_KP * BALL_MOVE_KP_GAIN) * e
                               + BALL_KI * ball_integral
                               + BALL_KD * (e - pid_e1));
        pid_e1 = e;

        //角度限幅 -24° ~ +80°（即输出限幅，位置式无需额外的单周期限幅）
        if(target_pulse > BALL_MAX_PULSE) target_pulse = BALL_MAX_PULSE;
        if(target_pulse < BALL_MIN_PULSE) target_pulse = BALL_MIN_PULSE;

        ball_running = 1;
    }
    else if(enter == 1)
    {
        //丢帧中：保持当前角度，不更新目标（到达计数也不走，见 case 分支条件）
        ball_running = 1;
    }
    else
    {
        //enter=0：回中位（零点），清 PID 状态，状态机复位待命
        target_pulse = 0;
        pid_e1 = 0.0f;
        ball_integral = 0.0f;
        ball_running = 0;
        ball_phase = 0;
        ball_target_mm = BALL_TARGET_CENTER_MM;
        arrive_cnt = 0;
        target_changed = 0;
    }

    //目标变化才下发，避免重复发相同命令
    if(target_pulse != last_sent_pulse)
    {
        last_sent_pulse = target_pulse;
        stepper_move(target_pulse);
    }
}
```

注意：`BALL_ERR_DEADZONE_MM`、`BALL_INTEGRAL_MAX`、`BALL_MIN_PULSE`、`BALL_MAX_PULSE`、`BALL_KP`、`BALL_KI`、`BALL_KD`、`BALL_FRAME_TIMEOUT_MS`、`pid_e1`、`ball_integral`、`target_pulse`、`last_sent_pulse`、`last_frame_ms` 均为现有定义，本任务不修改。

- [ ] **Step 5: 检查 `BALL_CENTER_MM` 无残留引用**

`BALL_CENTER_MM` 已被 `BALL_TARGET_*` 宏取代。用搜索确认无残留：

```bash
grep -rn "BALL_CENTER_MM" ucdriver/ main.c
```

Expected: 无输出（或仅注释提及）。若 `ball.c` 内仍有使用，替换为 `BALL_TARGET_CENTER_MM`。

- [ ] **Step 6: CCS 编译验证**

在 CCS 中 Build 工程（无法命令行构建）。Expected: 0 error / 0 warning（新代码无未使用变量：`ball_phase` 本任务已使用于状态机赋值，虽无读取方，但 `volatile` 全局变量不触发 warning）。

- [ ] **Step 7: Commit**

```bash
git add ucdriver/ball/ball.c ucdriver/ball/ball.h
git commit -m "feat: Q3 摆球目标点状态机（中心→+5cm→-5cm 保持）"
```

---

### Task 2: main.c OLED 显示摆球阶段

**Files:**
- Modify: `main.c`（`oled_proc()` Q3 分支）

**Interfaces:**
- Consumes: `volatile uint8_t ball_phase`（Task 1 定义于 ball.c，ball.h 已声明）
- Produces: 无

- [ ] **Step 1: 在 main.c 声明区新增 `ball_phase` extern**

在 `main.c:63` 的 `ball_running` extern 声明后新增：

```c
// Q3 摆球运动阶段（0-待命 1-去+5cm 2-去-5cm 3-保持）
extern volatile uint8_t ball_phase;
```

- [ ] **Step 2: 修改 `oled_proc()` Q3 第四行，追加阶段字符**

将 `main.c:127-134` 的 Q3 分支：

```c
    else if(question == 3)
    {
        OLED_ShowString(0, 36, (u8 *)"BAL:", 12);
        if(ball_running)
            OLED_ShowString(36, 36, (u8 *)"RUN ", 12);
        else
            OLED_ShowString(36, 36, (u8 *)"STOP", 12);
    }
```

替换为：

```c
    else if(question == 3)
    {
        OLED_ShowString(0, 36, (u8 *)"BAL:", 12);
        if(ball_running)
            OLED_ShowString(36, 36, (u8 *)"RUN ", 12);
        else
            OLED_ShowString(36, 36, (u8 *)"STOP", 12);
        //阶段字符：C-待命中心 +-去+5cm --去-5cm H-保持
        //（x=60：紧接 "RUN "/"STOP" 之后，12 号字体每字符 6px，RUN 占用 36~60）
        switch(ball_phase)
        {
        case 1: OLED_ShowString(60, 36, (u8 *)"+", 12); break;
        case 2: OLED_ShowString(60, 36, (u8 *)"-", 12); break;
        case 3: OLED_ShowString(60, 36, (u8 *)"H", 12); break;
        default: OLED_ShowString(60, 36, (u8 *)"C", 12); break;
        }
    }
```

- [ ] **Step 3: CCS 编译验证**

在 CCS 中 Build 工程。Expected: 0 error / 0 warning。

- [ ] **Step 4: Commit**

```bash
git add main.c
git commit -m "feat: OLED 显示 Q3 摆球运动阶段"
```

---

### Task 3: 实车验证（手动，计划外协作任务）

**Files:** 无

**前置条件:** 摄像头对准摆杆、视觉板发送 `0xAA pos 0x55` 帧、步进电机回零完成（`ball_init()` 阻塞 1s）。

- [ ] **Step 1: 验证待命与回中**

上电后：OLED 显示 Q3 阶段 `C`，`BAL:STOP`。手动滚动小球到任意位置，`enter` 保持 0，球不应被驱动。

- [ ] **Step 2: 验证完整序列**

按键切到 question=3，按 enter=1：
1. 球从中心出发前往 +5cm（OLED 阶段 `+`），到达后折返（阶段变 `-`）
2. 到达 -5cm 后稳定（阶段变 `H`），误差 ≤1cm（目视刻度线/上位机）
3. 全程（按 enter 到稳定）计时 ≤5s
4. 按 enter=0：球回中心（阶段回 `C`），电机停

- [ ] **Step 3: 超时排查（若 >5s）**

将 `BALL_MOVE_KP_GAIN` 从 1.0 调至 1.5 重新编译实测；若出现过冲振荡则改回 1.0 并加大 `BALL_KD`（当前 10.5）至 12~14。每次只改一个参数。

- [ ] **Step 4: 记录调参结果**

在 `docs/superpowers/plans/2026-07-31-q3-ball-motion-seq.md` 末尾追加实测记录（最终参数、总耗时、稳态误差），供后续 Q4/Q5 参考。

---

## Self-Review

- **Spec 覆盖**：状态机 4 态 ✓（Task1 Step4）；到达判定 8mm/300ms ✓；防冲击 pid_e1 对齐 ✓；`BALL_MOVE_KP_GAIN` 倍率宏 ✓（默认 1.0）；丢帧保持角度/enter=0 回中重置 ✓；`ball_phase` OLED 显示 ✓（Task2）；不改构建系统 ✓（无新文件）。
- **占位符扫描**：所有代码步骤均含完整代码，无 TBD/TODO。
- **类型一致性**：`ball_phase` 在 Task1 定义为 `volatile uint8_t`，Task2 的 extern 声明一致；状态机阶段值 0/1/2/3 在两处使用一致（OLED case 1/2/3 对应 +/−/H，default 为 C）。
