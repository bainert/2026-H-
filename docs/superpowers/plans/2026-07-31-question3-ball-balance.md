# Q3 摆球控制（步进电机 + 视觉）实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Q3 题号下通过 UART3 控制 X42S 闭环步进电机驱动摆杆，使视觉（UART0 `0xaa data 0x55`）报告的小球位置稳定在 125mm（水管中点）。

**Architecture:** 分层双模块 — `ucdriver/stepper/`（EMM 协议层：帧打包、回零、位置模式）与 `ucdriver/ball/`（Q3 业务层：视觉帧解析、丢帧检测、增量 PID、状态机）。ball_proc() 在现有 30ms 电机定时器 ISR 中调用；Q3 时小车电机停转、走摆球控制。

**Tech Stack:** MSPM0G3507 裸机 C11，MSPM0 SDK 2.10.00.04 DriverLib，tiarmclang v4.0.4.LTS，CCS 20.5.0（SysConfig 已配好 UART0/UART3，无需改 syscfg）。

## Global Constraints

- 注释使用中文，风格与现有 `ucdriver/` 代码一致（不写 Doxygen，用 `//` 短注释）
- 帧校验码固定 `0x6B`；电机地址 `0x01`；波特率 115200
- 角度限位：-24° ~ +80° 换算为脉冲 [-213, +711]（16 细分，0.1125°/脉冲）
- 视觉帧：`0xAA data 0x55`，data 为小球距左端 mm，不校验取值范围
- 回零：上电发 `01 9A 00 00 6B` 后阻塞 `delay_ms(1000)`，无状态轮询
- 位置模式用 FD 命令，运动模式取 01（绝对位置，相对坐标零点）
- 本环境无法构建（CCS 硬编码绝对路径），任务"验证"= 代码自审 + 用户在 CCS 构建
- 构建文件改动遵循 AGENTS.md：sources.mk SUBDIRS → 各目录 subdir_vars.mk/subdir_rules.mk → makefile ORDERED_OBJS

---

### Task 1: stepper 协议层（EMM 帧打包 + UART3 应答）

**Files:**
- Create: `ucdriver/stepper/stepper.h`
- Create: `ucdriver/stepper/stepper.c`

**Interfaces:**
- Consumes: `uart.h` 的 `uart_send_char(UART_Regs*, uint8_t)`、`UART_3_INST`/`UART_3_INST_INT_IRQN`（ti_msp_dl_config.h）、`DL_UART_getPendingInterrupt`/`DL_UART_receiveDataBlocking`/`DL_UART_IIDX_RX`
- Produces: `void stepper_init(void)`、`void stepper_home_start(void)`、`void stepper_move(int32_t pulses)`、`volatile uint8_t stepper_ack`（最近应答字节，供调试）

- [ ] **Step 1: 创建 `ucdriver/stepper/stepper.h`**

```c
#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <stdint.h>

// 最近一次电机应答字节（02=命令正确 12=已在零点 E2=参数错 EE=格式错 9F=动作完成）
extern volatile uint8_t stepper_ack;

// 初始化：使能 UART3 接收中断
void stepper_init(void);
// 触发单圈就近回零（9A 命令）
void stepper_home_start(void);
// 位置模式绝对运动：pulses 为相对零点的有符号脉冲（正=CW 负=CCW）
void stepper_move(int32_t pulses);

#endif
```

- [ ] **Step 2: 创建 `ucdriver/stepper/stepper.c`**

```c
#include "stepper.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

//电机地址（出厂默认1，不可为0广播）
#define STEPPER_ADDR        0x01
//校验码（出厂默认0x6B）
#define STEPPER_CRC         0x6B

//位置模式运动参数（可调）
#define STEPPER_SPEED_RPM   500     //速度 0-3000 RPM
#define STEPPER_ACC         100     //加速度档位 0-255，越大加速越快

//最近一次电机应答字节
volatile uint8_t stepper_ack = 0;

//发送一帧命令
static void stepper_send_frame(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        uart_send_char(UART_3_INST, frame[i]);
    }
}

//初始化：使能 UART3 接收中断
void stepper_init(void)
{
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

//触发单圈就近回零：01 9A 00 00 6B（模式00单圈就近，同步00立即）
void stepper_home_start(void)
{
    static const uint8_t frame[] = {0x01, 0x9A, 0x00, 0x00, 0x6B};
    stepper_send_frame(frame, sizeof(frame));
}

//位置模式绝对运动
//帧: 01 FD 方向 速度(2B) 加速度 脉冲(4B) 运动模式01(绝对) 同步00 6B
void stepper_move(int32_t pulses)
{
    uint8_t frame[13];
    uint8_t dir;
    uint32_t abs_pulses;

    //正负脉冲拆分：正=CW(00) 负=CCW(01)，脉冲数取绝对值
    if(pulses >= 0)
    {
        dir = 0x00;
        abs_pulses = (uint32_t)pulses;
    }
    else
    {
        dir = 0x01;
        abs_pulses = (uint32_t)(-pulses);
    }

    frame[0] = STEPPER_ADDR;
    frame[1] = 0xFD;                                  //位置模式功能码
    frame[2] = dir;
    frame[3] = (STEPPER_SPEED_RPM >> 8) & 0xFF;       //速度高字节
    frame[4] = STEPPER_SPEED_RPM & 0xFF;              //速度低字节
    frame[5] = STEPPER_ACC;                           //加速度档位
    frame[6]  = (abs_pulses >> 24) & 0xFF;            //脉冲数 4B 大端
    frame[7]  = (abs_pulses >> 16) & 0xFF;
    frame[8]  = (abs_pulses >> 8) & 0xFF;
    frame[9]  = abs_pulses & 0xFF;
    frame[10] = 0x01;                                 //运动模式：相对坐标零点绝对位置
    frame[11] = 0x00;                                 //同步标志：立即执行
    frame[12] = STEPPER_CRC;

    stepper_send_frame(frame, sizeof(frame));
}

//UART3 接收中断：解析电机应答（只取应答字节，供上层/调试使用）
void UART_3_INST_IRQHandler(void)
{
    switch(DL_UART_getPendingInterrupt(UART_3_INST))
    {
    case DL_UART_IIDX_RX:
        stepper_ack = DL_UART_receiveDataBlocking(UART_3_INST);
        break;
    default:
        break;
    }
}
```

- [ ] **Step 3: 自审**

检查：`stepper_send_frame` 使用 `UART_3_INST`（ti_msp_dl_config.h 已定义，见 `Debug/ti_msp_dl_config.h:169`）；`uart_send_char` 声明于 uart.h；帧长度 13 字节 = Addr+Code+方向+速度2+加速度+脉冲4+运动模式+同步+校验；FD 帧字段与手册 5.3.12 一致。

- [ ] **Step 4: Commit**

```bash
git add ucdriver/stepper/stepper.h ucdriver/stepper/stepper.c
git commit -m "feat: 新增 stepper 步进电机协议层（EMM 回零/位置模式/UART3应答）"
```

---

### Task 2: ball 控制层（视觉解析 + 增量 PID + 状态机）

**Files:**
- Create: `ucdriver/ball/ball.h`
- Create: `ucdriver/ball/ball.c`

**Interfaces:**
- Consumes: Task 1 的 `stepper_init/stepper_home_start/stepper_move`、`key.h` 的 `enter/question`、`delay.h` 的 `delay_ms`、`sys_tick_ms`（main.c 中定义并递增）、`UART_0_INST`（ti_msp_dl_config.h）
- Produces: `void ball_init(void)`、`void ball_proc(void)`、`void ball_uart_rx(uint8_t byte)`、`volatile uint8_t ball_position_mm`、`volatile uint8_t ball_frame_lost`、`volatile uint8_t ball_running`

- [ ] **Step 1: 创建 `ucdriver/ball/ball.h`**

```c
#ifndef __BALL_H__
#define __BALL_H__

#include <stdint.h>

//小球位置（mm，0~250），由 UART0 视觉帧更新
extern volatile uint8_t ball_position_mm;
//视觉丢帧标志：0-正常 1-超过500ms无新帧
extern volatile uint8_t ball_frame_lost;
//摆球控制运行标志：Q3 且 enter=1 时为 1
extern volatile uint8_t ball_running;

//初始化：使能 UART3 + 触发回零 + 阻塞延时1s
void ball_init(void);
//30ms 周期调用（在 motor_pid 定时器 ISR 中）
void ball_proc(void);
//视觉串口字节回调（UART0 ISR 中逐字节调用）
void ball_uart_rx(uint8_t byte);

#endif
```

- [ ] **Step 2: 创建 `ucdriver/ball/ball.c`**

```c
#include "ball.h"
#include "ti_msp_dl_config.h"
#include "stepper.h"
#include "key.h"
#include "delay.h"

//水管中点（mm）
#define BALL_CENTER_MM      125.0f
//电机角度限位（脉冲）：-24° ~ +80° @16细分 0.1125°/脉冲
#define BALL_MIN_PULSE      (-213)
#define BALL_MAX_PULSE      (711)
//视觉丢帧判定时间（ms）
#define BALL_FRAME_TIMEOUT_MS  500
//增量 PID 参数（单位：脉冲/mm，实车标定）
#define BALL_KP             5.0f
#define BALL_KI             0.1f
#define BALL_KD             30.0f
//单周期输出限幅（脉冲，防猛转）
#define BALL_DU_MAX         100

//小球位置（mm）
volatile uint8_t ball_position_mm = 0;
//丢帧标志
volatile uint8_t ball_frame_lost = 0;
//运行标志
volatile uint8_t ball_running = 0;

//帧解析状态机：0-等帧头 1-已收data 2-等帧尾
static uint8_t frame_state = 0;
static uint8_t frame_data = 0;

//最近一帧时间戳（sys_tick_ms）
static volatile uint32_t last_frame_ms = 0;

//PID 历史误差
static float pid_e1 = 0.0f;
static float pid_e2 = 0.0f;
//当前目标脉冲（绝对位置，相对零点）
static int32_t target_pulse = 0;
//上一次下发的脉冲（目标变化才下发，去重）
static int32_t last_sent_pulse = 0;
//上一周期题号（检测切题号）
static int last_question = 0;

extern volatile uint32_t sys_tick_ms;

//视觉帧解析：0xAA data 0x55 三字节帧
void ball_uart_rx(uint8_t byte)
{
    if(frame_state == 0)
    {
        //等帧头
        if(byte == 0xAA) frame_state = 1;
    }
    else if(frame_state == 1)
    {
        //收 data
        frame_data = byte;
        frame_state = 2;
    }
    else
    {
        if(byte == 0x55)
        {
            //完整帧：更新小球位置与时间戳
            ball_position_mm = frame_data;
            last_frame_ms = sys_tick_ms;
            ball_frame_lost = 0;
        }
        else if(byte == 0xAA)
        {
            //可能丢失 data 字节，按新帧头重新解析
            frame_state = 1;
        }
        else
        {
            frame_state = 0;
        }
    }
}

//初始化：使能 UART3 + 触发回零 + 阻塞延时1s等待回零完成
void ball_init(void)
{
    stepper_init();
    stepper_home_start();
    delay_ms(1000);
    //回零后电机位于零点（中位），同步目标与已下发值
    target_pulse = 0;
    last_sent_pulse = 0;
}

//30ms 周期调用（motor_pid 定时器 ISR 中）
void ball_proc(void)
{
    //非 Q3：待命，清理 PID 状态，电机不动作
    if(question != 3)
    {
        pid_e1 = 0.0f;
        pid_e2 = 0.0f;
        ball_running = 0;
        last_question = question;
        return;
    }
    last_question = question;

    //丢帧检测：超过500ms无新帧
    if(sys_tick_ms - last_frame_ms > BALL_FRAME_TIMEOUT_MS)
    {
        ball_frame_lost = 1;
    }

    if(enter == 1 && ball_frame_lost == 0)
    {
        //RUN：增量式 PID，误差 = 中点 - 当前位置
        float e = BALL_CENTER_MM - (float)ball_position_mm;
        float du = BALL_KP * (e - pid_e1)
                 + BALL_KI * e
                 + BALL_KD * (e - 2.0f * pid_e1 + pid_e2);
        pid_e2 = pid_e1;
        pid_e1 = e;

        //单周期输出限幅，防止猛转
        if(du >  BALL_DU_MAX) du =  BALL_DU_MAX;
        if(du < -BALL_DU_MAX) du = -BALL_DU_MAX;

        target_pulse += (int32_t)du;

        //角度限幅 -24° ~ +80°
        if(target_pulse > BALL_MAX_PULSE) target_pulse = BALL_MAX_PULSE;
        if(target_pulse < BALL_MIN_PULSE) target_pulse = BALL_MIN_PULSE;

        ball_running = 1;
    }
    else if(enter == 1)
    {
        //丢帧中：保持当前角度，不更新目标
        ball_running = 1;
    }
    else
    {
        //enter=0：回中位（零点），清 PID 状态
        target_pulse = 0;
        pid_e1 = 0.0f;
        pid_e2 = 0.0f;
        ball_running = 0;
    }

    //目标变化才下发，避免重复发相同命令
    if(target_pulse != last_sent_pulse)
    {
        last_sent_pulse = target_pulse;
        stepper_move(target_pulse);
    }
}
```

- [ ] **Step 3: 自审**

检查：`ball_uart_rx` 状态机覆盖 0xAA/0x55 重入情形；`question/enter` 来自 key.h extern；`sys_tick_ms` 在 main.c 定义；PID 历史只在 RUN 分支更新（丢帧/待命时不动，防止积发）；限幅常量与规格一致；丢帧时 `last_sent_pulse` 不变 → 不重复发命令。

- [ ] **Step 4: Commit**

```bash
git add ucdriver/ball/ball.h ucdriver/ball/ball.c
git commit -m "feat: 新增 ball 摆球控制层（视觉解析/丢帧检测/增量PID/状态机）"
```

---

### Task 3: 接线层（uart.c / motor.c / main.c）

**Files:**
- Modify: `ucdriver/uart/uart.c:20-36`（UART0 ISR 改视觉解析）
- Modify: `ucdriver/motor/motor.c:176-217`（Q3 屏蔽循迹 + 调 ball_proc）
- Modify: `main.c:33-38`（include）、`main.c:120-153`（初始化）、`main.c:70-117`（oled_proc Q3 分支）

**Interfaces:**
- Consumes: Task 2 的 `ball_uart_rx/ball_proc/ball_position_mm/ball_frame_lost/ball_running`、`UART_0_INST`（ti_msp_dl_config.h）
- Produces: 无新接口

- [ ] **Step 1: 修改 `ucdriver/uart/uart.c` — UART0 ISR 由回显改为视觉帧解析**

`uart.c` 开头加 `#include "ball.h"`（在 `#include "uart.h"` 之后）：

```c
#include "uart.h"
#include "ball.h"
#include "ti_msp_dl_config.h"
```

将 `UART_0_INST_IRQHandler` 整个替换为：

```c
//串口接收中断处理函数（视觉数据）
void UART_0_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
    case DL_UART_IIDX_RX:
    {
        //将接收字节送入视觉帧解析
        uint8_t data = DL_UART_receiveDataBlocking(UART_0_INST);
        ball_uart_rx(data);
        break;
    }

    default:
        break;
    }
}
```

- [ ] **Step 2: 修改 `ucdriver/motor/motor.c` — Q3 屏蔽小车驱动 + 调用 ball_proc**

`motor.c` 开头加 `#include "ball.h"`（在 `#include "key.h"` 之后）：

```c
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "huidu.h"
#include "key.h"
#include "ball.h"
```

将 `motor_pid_INST_IRQHandler` 中 `case DL_TIMER_IIDX_LOAD:` 分支替换为：

```c
    case DL_TIMER_IIDX_LOAD:
        //发车
        if(enter == 1)
        {
            if(question == 3)
            {
                //Q3 静态摆球：小车电机停，不循迹，交给摆球控制
                motor_set_duty(1, 0);
                motor_set_duty(2, 0);
            }
            else
            {
                //灰度控制
                adjust_motor();
                //里程累计（使用编码器计数值，精度更高）
                odometer += ((float)counter_1_A + (float)counter_2_A) * 0.5f
                            / motor_xianshu * PI * motor_wheel_d;
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
                        // 从待命或完成状态 -> 开始跑圈，里程清零
                        odometer = 0.0f;
                        lap_state = LAP_RUNNING;
                        lap_start_ms = sys_tick_ms;
                    }
                    else if(lap_state == LAP_RUNNING)
                    {
                        /* 里程计到达5.20m时停车 */
                        if(odometer >= 5000.0f)
                        {
                            lap_state = LAP_COMPLETE;
                            lap_total_ms = sys_tick_ms - lap_start_ms;
                            enter = 0;  // 停止电机
                        }
                    }
                }
                else
                {
                    // 非Q2题号时重置圈状态，防止切换题号回来时状态混乱
                    lap_state = LAP_IDLE;
                }
            }
        }
        //未发车时（包括Q2完成后的状态），电机停止
        if(enter == 0)
        {
            motor_set_duty(1, 0);
            motor_set_duty(2, 0);
        }

        //摆球控制 30ms 周期调用（Q3 时生效，非 Q3 自动待命）
        ball_proc();

    default:
        break;
```

- [ ] **Step 3: 修改 `main.c` — include + 初始化 + OLED Q3 显示**

`main.c` 的 include 区（`#include "motor.h"` 后）加：

```c
#include "ball.h"
```

extern 区（`#include "key.h"` 附近，`extern volatile uint32_t sys_tick_ms;` 后面）加：

```c
// Q3 摆球控制
extern volatile uint8_t ball_position_mm;
extern volatile uint8_t ball_frame_lost;
extern volatile uint8_t ball_running;
```

`main()` 中 `OLED_Clear();` 之后加（放在按键/编码器中断使能之后、motor_init 之前）：

```c
    //Q3 摆球控制初始化（含步进电机回零，阻塞约1s）
    ball_init();
```

`oled_proc()` 中第三行（里程计）与第四行（Q2 计时）替换为：

```c
    //第三行：Q3 显示小球位置，否则里程计
    if(question == 3)
    {
        OLED_ShowString(0, 24, (u8 *)"P:", 12);
        if(ball_frame_lost)
            OLED_ShowString(12, 24, (u8 *)"LOST", 12);
        else
            OLED_ShowNum(12, 24, (u32)ball_position_mm, 3, 12);
        OLED_ShowString(36, 24, (u8 *)"mm", 12);
    }
    else
    {
        OLED_ShowString(0, 24, (u8 *)"D:", 12);
        OLED_ShowFloat(12, 24, odometer / 1000.0f, 1, 2, 12);
        OLED_ShowString(34, 24, (u8 *)"m", 12);
    }

    //第四行：Q2 计时 / Q3 摆球状态 / 其他清空
    if(question == 2)
    {
        if(lap_state == LAP_RUNNING)
        {
            uint32_t elapsed = sys_tick_ms - lap_start_ms;
            OLED_ShowString(0, 36, (u8 *)"T:", 12);
            OLED_ShowFloat(12, 36, (float)elapsed / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 36, (u8 *)"s", 12);
        }
        else if(lap_state == LAP_COMPLETE)
        {
            OLED_ShowString(0, 36, (u8 *)"F:", 12);
            OLED_ShowFloat(12, 36, (float)lap_total_ms / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 36, (u8 *)"s!   ", 12);
        }
        else  // LAP_IDLE
        {
            OLED_ShowString(0, 36, (u8 *)"Press ENTER", 12);
        }
    }
    else if(question == 3)
    {
        OLED_ShowString(0, 36, (u8 *)"BAL:", 12);
        if(ball_running)
            OLED_ShowString(36, 36, (u8 *)"RUN ", 12);
        else
            OLED_ShowString(36, 36, (u8 *)"STOP", 12);
    }
    else
    {
        // 非Q2/Q3时清除第四行
        OLED_ShowString(0, 36, (u8 *)"            ", 12);
    }
```

- [ ] **Step 4: 自审**

检查：`uart.c` 引用 `ball_uart_rx` 需 `ball.h`（Task 2）；`motor.c` 引用 `ball_proc` 需 `ball.h`；Q3 分支保留了 `enter==0` 时的停车逻辑；`ball_proc()` 无条件调用（非 Q3 内部 return，开销可忽略）；`main.c` 的 `ball_init()` 放在中断使能之后（回零的 1s 阻塞期间 OLED 已初始化，显示正常）。

- [ ] **Step 5: Commit**

```bash
git add ucdriver/uart/uart.c ucdriver/motor/motor.c main.c
git commit -m "feat: Q3 接线层 — UART0 视觉解析/电机Q3停转/OLED显示小球位置"
```

---

### Task 4: 构建集成（sources.mk / subdir 文件 / makefile）

**Files:**
- Modify: `Debug/sources.mk:120-129`（SUBDIRS）
- Create: `Debug/ucdriver/ball/subdir_vars.mk`
- Create: `Debug/ucdriver/ball/subdir_rules.mk`
- Create: `Debug/ucdriver/stepper/subdir_vars.mk`
- Create: `Debug/ucdriver/stepper/subdir_rules.mk`
- Modify: `Debug/makefile:12-24`（ORDERED_OBJS）、`Debug/makefile:37-51`（-include）、`Debug/makefile:162-163`（clean）

**Interfaces:**
- Consumes: Task 1/2 的源文件
- Produces: 可在 CCS 中构建的工程

- [ ] **Step 1: `Debug/sources.mk` — SUBDIRS 追加两目录**

在 `ucdriver/uart \` 行后追加：

```makefile
ucdriver/ball \
ucdriver/stepper \
```

（注意保留每行结尾的 `\`，最后一行 `ucdriver/uart \` 原有行尾续行符不变）

- [ ] **Step 2: 创建 `Debug/ucdriver/ball/subdir_vars.mk`**

```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ucdriver/ball/ball.c 

C_DEPS += \
./ucdriver/ball/ball.d 

OBJS += \
./ucdriver/ball/ball.o 

OBJS__QUOTED += \
"ucdriver\ball\ball.o" 

C_DEPS__QUOTED += \
"ucdriver\ball\ball.d" 

C_SRCS__QUOTED += \
"../ucdriver/ball/ball.c" 
```

- [ ] **Step 3: 创建 `Debug/ucdriver/ball/subdir_rules.mk`**

（目录名 ball，编译命令 -I 列表追加 ball/stepper 两个路径，其余照抄现有 `Debug/ucdriver/uart/subdir_rules.mk` 格式）

```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ucdriver/ball/%.o: ../ucdriver/ball/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Administrator/workspace_ccstheia/car_demo" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/Debug" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/oled" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/uart" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/delay" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/key" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/huidu" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/motor" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/ball" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/stepper" -gdwarf-3 -Wall -MMD -MP -MF"ucdriver/ball/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '
```

- [ ] **Step 4: 创建 `Debug/ucdriver/stepper/subdir_vars.mk`**

```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ucdriver/stepper/stepper.c 

C_DEPS += \
./ucdriver/stepper/stepper.d 

OBJS += \
./ucdriver/stepper/stepper.o 

OBJS__QUOTED += \
"ucdriver\stepper\stepper.o" 

C_DEPS__QUOTED += \
"ucdriver\stepper\stepper.d" 

C_SRCS__QUOTED += \
"../ucdriver/stepper/stepper.c" 
```

- [ ] **Step 5: 创建 `Debug/ucdriver/stepper/subdir_rules.mk`**

（目录名 stepper，-I 列表同样追加 ball/stepper 两个路径）

```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ucdriver/stepper/%.o: ../ucdriver/stepper/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Administrator/workspace_ccstheia/car_demo" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/Debug" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/oled" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/uart" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/delay" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/key" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/huidu" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/motor" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/ball" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/stepper" -gdwarf-3 -Wall -MMD -MP -MF"ucdriver/stepper/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '
```

- [ ] **Step 6: 修改 `Debug/makefile`**

`ORDERED_OBJS` 块（`"./ucdriver/uart/uart.o" \` 行后）追加：

```makefile
"./ucdriver/ball/ball.o" \
"./ucdriver/stepper/stepper.o" \
```

`-include` 的 subdir_vars.mk 块（`ucdriver/uart/subdir_vars.mk` 行后）追加：

```makefile
-include ucdriver/ball/subdir_vars.mk
-include ucdriver/stepper/subdir_vars.mk
```

`-include` 的 subdir_rules.mk 块（`ucdriver/uart/subdir_rules.mk` 行后）追加：

```makefile
-include ucdriver/ball/subdir_rules.mk
-include ucdriver/stepper/subdir_rules.mk
```

clean 目标的两行 `-$(RM) "..." "ucdriver\uart\uart.o" ...`（162 行）和 `... "ucdriver\uart\uart.d" ...`（163 行）末尾分别追加：

```makefile
"ucdriver\ball\ball.o" "ucdriver\stepper\stepper.o"
"ucdriver\ball\ball.d" "ucdriver\stepper\stepper.d"
```

- [ ] **Step 7: 自审**

检查：`Debug/ucdriver/ball/`、`Debug/ucdriver/stepper/` 目录存在（新建 subdir_vars.mk/rules.mk 前先 `mkdir`）；sources.mk 续行符完整；makefile 三处（ORDERED_OBJS / 2 个 -include 块 / clean）都加了对应条目；-I 列表包含两个新路径。

- [ ] **Step 8: Commit**

```bash
git add Debug/sources.mk Debug/makefile Debug/ucdriver/ball Debug/ucdriver/stepper
git commit -m "build: 注册 ball/stepper 模块到 CCS 构建系统"
```

---

### Task 5: 构建验证（用户在 CCS 执行）

**Files:**
- 无代码改动

- [ ] **Step 1: 用户在 CCS 中构建工程**

在 CCS 20.5.0 打开工程 → Project → Build（或 Ctrl+B）。预期：0 error，0 warning（若有 warning 按 `Debug/` 输出修正后重提）。

- [ ] **Step 2: 烧录后串口调试验证（供用户操作）**

1. 上电：OLED 正常显示，电机上电执行回零（约 1s 内电机小幅转动回零点）
2. 切到题号 3（PB6），按 enter（PB7）：OLED 第 3 行显示 `P:` 与视觉数值，第 4 行 `BAL: RUN`
3. UART3 接上位机（115200）：观察电机位置模式命令 `01 FD ... 6B` 与应答 `01 FD 02 6B`
4. 视觉发 `0xAA 0x7D 0x55`（125mm 中点）：摆杆静止或小幅修正，小球回中
5. 视觉发 `0xAA 0x5F 0x55`（95mm）：摆杆倾斜补偿，小球向中滚动
6. 停止视觉发送 >500ms：OLED 显示 `LOST`，摆杆保持当前角度
7. enter=0：摆杆回中位（零点）

- [ ] **Step 3: 实车标定**

若小球振荡：调小 `BALL_KP` / `BALL_KD`；若回中慢/稳态误差大：调大 `BALL_KI`。参数在 `ucdriver/ball/ball.c` 顶部宏，改后重编译烧录。
