# 第二问：基本循线行驶（一圈自动停止）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 题号=2 按下发车键后，小车沿黑线顺时针行驶一圈，回到 A 点自动停止，OLED 显示行驶总时间。

**架构:** 在现有 30ms PID 定时器 ISR 中添加 lap 状态机 + A 点检测；在 main.c oled_proc() 中显示计时信息。

**技术栈:** MSPM0G3507 裸机 C11, tiarmclang, DriverLib, SSD1306 OLED

## 全局约束

- 遵循现有代码风格（注释方式、命名习惯、文件结构）
- 仅修改已有文件，不新增文件
- A 点检测条件: `huidu_value[1] && huidu_value[2] && huidu_value[3] && !huidu_value[0] && !huidu_value[4]`
- 起步后 2s 才启用 A 点检测，避免出发位置误判
- 不影响其他题号（1,3,4,5）的原有行为

---

### Task 1: key.h — 新增 lap 状态变量声明

**Files:**
- Modify: `ucdriver/key/key.h`

**接口:**
- Produces: `lap_state_t` 枚举类型, `lap_state`, `lap_start_ms`, `lap_total_ms` extern 声明

- [ ] **Step 1: 在 key.h 中添加枚举和 extern 声明**

修改 `ucdriver/key/key.h`，在 `#include` 下方、`extern volatile uint8_t key_debounce_cnt;` 后面添加：

```c
typedef enum {
    LAP_IDLE = 0,      // 待命
    LAP_RUNNING,       // 行驶中
    LAP_COMPLETE       // 完成一圈回到A点
} lap_state_t;

extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
```

---

### Task 2: key.c — 定义 lap 状态变量

**Files:**
- Modify: `ucdriver/key/key.c`

**接口:**
- Consumes: `lap_state_t` 枚举（来自 Task 1）
- Produces: `lap_state`, `lap_start_ms`, `lap_total_ms` 变量定义

- [ ] **Step 1: 在 key.c 末尾 static 变量定义区之后添加**

在 `volatile uint8_t key_debounce_cnt = 0;` 后面添加：

```c
// 第二问：跑圈状态
volatile lap_state_t lap_state = LAP_IDLE;
volatile uint32_t lap_start_ms = 0;
volatile uint32_t lap_total_ms = 0;
```

---

### Task 3: huidu.h — 导出 huidu_value 数组

**Files:**
- Modify: `ucdriver/huidu/huidu.h`

**接口:**
- Produces: `huidu_value[5]` extern 声明（供 motor.c 和 main.c 读取传感器状态）

- [ ] **Step 1: 添加 extern 声明**

在 `extern volatile uint8_t huidu_straight_count;` 后面添加：

```c
extern uint8_t huidu_value[5];
```

---

### Task 4: motor.c — 30ms ISR 添加 A 点检测逻辑

**Files:**
- Modify: `ucdriver/motor/motor.c`

**接口:**
- Consumes: `lap_state`, `lap_start_ms`, `lap_total_ms`（来自 Task 2）, `huidu_value[5]`（来自 Task 3）, `question`, `enter`, `sys_tick_ms`
- Produces: 修改 `enter` 标志以停止电机

- [ ] **Step 1: 在 motor.c 的 extern 声明区添加 lap 相关变量**

在 `extern volatile uint8_t key_debounce_cnt;` 后面添加：

```c
/* 第二问 lap 状态机 */
extern volatile int question;
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
extern volatile uint32_t sys_tick_ms;
```

- [ ] **Step 2: 修改 motor_pid_INST_IRQHandler**

将原 ISR 函数体替换为：

```c
//30ms定时器中断服务函数
void motor_pid_INST_IRQHandler(void)
{
    //按键消抖计数器递减（60ms消抖窗口）
    if(key_debounce_cnt) key_debounce_cnt--;
   switch(DL_Timer_getPendingInterrupt(motor_pid_INST))
   {
    case DL_TIMER_IIDX_LOAD:
        //发车
        if(enter == 1)
        {
            //灰度控制
            adjust_motor();
            //计算电机速度
            calculate_speed(1);
            calculate_speed(2);
            //pid控制
            motor_PID(1);
            motor_PID(2);

            /* 第二问: 跑圈计时 + 自动停止 */
            if(question == 2)
            {
                if(lap_state == LAP_IDLE || lap_state == LAP_COMPLETE)
                {
                    // 从待命或完成状态 -> 开始跑圈
                    lap_state = LAP_RUNNING;
                    lap_start_ms = sys_tick_ms;
                }
                else if(lap_state == LAP_RUNNING)
                {
                    // 起步后延迟2s开始检测A点，避免出发位置误判
                    if(sys_tick_ms - lap_start_ms > 2000)
                    {
                        /* A点特征: L1+M+R1同时检测到黑线, L2+R2丢线 */
                        if(huidu_value[1] && huidu_value[2] && huidu_value[3]
                            && !huidu_value[0] && !huidu_value[4])
                        {
                            lap_state = LAP_COMPLETE;
                            lap_total_ms = sys_tick_ms - lap_start_ms;
                            enter = 0;  // 停止电机
                        }
                    }
                }
            }
        }
        //未发车时（包括Q2完成后的状态），电机停止
        if(enter == 0)
        {
            motor_set_duty(1, 0);
            motor_set_duty(2, 0);
        }

    default:
        break;
   }
}
```

---

### Task 5: main.c — OLED 显示计时信息

**Files:**
- Modify: `main.c`

**接口:**
- Consumes: `lap_state`, `lap_start_ms`, `lap_total_ms`（来自 Task 2）, `question`, `enter`, `sys_tick_ms`

- [ ] **Step 1: 添加 lap 变量 extern 声明**

在 `extern volatile uint32_t sys_tick_ms;` 后面添加：

```c
// 第二问: 跑圈状态
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
```

- [ ] **Step 2: 修改 oled_proc() 函数**

在原有显示之后、`OLED_Refresh()` 之前添加 Q2 计时显示：

```c
//oled显示
void oled_proc(void)
{
    //第一行：发车状态
    OLED_ShowString(0, 0, (u8 *)"Status:", 12);
    if(enter)
        OLED_ShowString(42, 0, (u8 *)"RUN ", 12);
    else
        OLED_ShowString(42, 0, (u8 *)"STOP", 12);

    //第二行：题号
    OLED_ShowString(0,12,(u8 *)"question: ", 12);
    OLED_ShowNum(54, 12, question, 2, 12);

    /* 第二问: 跑圈计时显示 */
    if(question == 2)
    {
        if(lap_state == LAP_RUNNING)
        {
            // 实时显示 elapsed time (秒)
            uint32_t elapsed = sys_tick_ms - lap_start_ms;
            OLED_ShowString(0, 24, (u8 *)"T:", 12);
            OLED_ShowFloat(12, 24, (float)elapsed / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 24, (u8 *)"s", 12);
        }
        else if(lap_state == LAP_COMPLETE)
        {
            // 显示完成总时间
            OLED_ShowString(0, 24, (u8 *)"F:", 12);
            OLED_ShowFloat(12, 24, (float)lap_total_ms / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 24, (u8 *)"s!", 12);
        }
        else  // LAP_IDLE
        {
            OLED_ShowString(0, 24, (u8 *)"Press ENTER", 12);
        }
    }

    //刷新显示
    OLED_Refresh();
}
```

---

## 验证方法

由于无单元测试框架，验证方式为代码审查：

1. **语法检查**: 确保所有变量名、枚举值拼写一致
2. **逻辑跟踪**: 模拟执行 Q2 流程确保状态机转换正确
3. **回归确认**: 非 Q2 场景下原有逻辑不受影响
4. **构建检查**: CCS 中编译，确认无 warning/error
