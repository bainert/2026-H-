# 🚗 车载平衡滚球运动控制系统

> **2026 年全国大学生电子设计竞赛赛区赛（TI 杯）暨模拟电子系统设计专题赛选拔赛 · H 题**
> **省赛二等奖**开源作品

<p align="center">
  <img alt="MCU" src="https://img.shields.io/badge/MCU-MSPM0G3507-0A84FF?style=flat-square">
  <img alt="Core" src="https://img.shields.io/badge/Core-Cortex--M0%2B%2080MHz-0A84FF?style=flat-square">
  <img alt="IDE" src="https://img.shields.io/badge/IDE-CCS%2020.5-FF9F0A?style=flat-square">
  <img alt="Language" src="https://img.shields.io/badge/language-C11-34C759?style=flat-square">
  <img alt="Award" src="https://img.shields.io/badge/award-省赛二等奖-FFD60A?style=flat-square">
  <img alt="License" src="https://img.shields.io/badge/license-竞赛学习用途-8E8E93?style=flat-square">
</p>

一辆载有「平衡滚球运动控制装置」的循线小车：沿黑色环形赛道行驶，车上的摆杆由**闭环步进电机**调节倾角，使钢球在凹槽中保持稳定；**上位机视觉**通过串口下发钢球位置，主控完成闭环控制。整套系统完全自主运行，无人干预。

---

## 📑 目录

- [🏆 项目简介](#项目简介)
- [🎯 赛题与评分（8 项）](#赛题与评分8-项)
- [✨ 核心亮点](#核心亮点)
- [🔧 硬件平台](#硬件平台)
- [🧩 软件架构与模块](#软件架构与模块)
- [📌 外设与引脚分配](#外设与引脚分配)
- [⏱️ 时钟树](#时钟树)
- [🎛️ 控制策略](#控制策略)
- [📂 目录结构](#目录结构)
- [🛠️ 构建与烧录](#构建与烧录)
- [📷 效果展示](#效果展示)
- [🐞 已知问题与 TODO](#已知问题与-todo)
- [📜 许可证](#许可证)

---

## 🏆 项目简介

小车需载有平衡滚球运动控制装置（摆杆 + 控制机构 + 钢球），沿黑色环形路线行驶，行驶过程中保持钢球稳定。核心评分项如下。

## 🎯 赛题与评分（8 项）

| 项目 | 分值 | 要点 |
| --- | --- | --- |
| 图传监测 | 6 | 发送端固定于小车，接收端在场外，实时显示并录制钢球滚动 |
| 基本循线 | 16 | A 点启动，顺时针一圈回到 A，≤20s，停车偏差 ≤2cm |
| 静态摆球 | 13 | 静止时球由 O 到 +5cm，折返 -5cm 并稳定，≤5s，误差 ≤1cm |
| 带球到 B | 20 | A→B 带球行驶，AB 段 ≤8s，全程球在中心附近，误差 ≤1cm |
| 带球整圈 | 20 | A 出发顺时针整圈，≤30s，全程球在中心附近，误差 ≤1cm |
| 任意位置稳定 | 20 | 球初始置于任意指定位置，整圈行驶并稳定在该位置，≤30s，误差 ≤1cm |
| 创新 | 5 | 其他附加功能 |
| 设计报告 | 20 | 方案论证、理论分析、电路与程序流程、测试、规范性 |

赛道为田径场形（AB、CD 直线各 1.5m，BC、DA 半圆弧半径 0.5m），中心线周长约 6.14m；A 点设 5cm 启停线与 30cm 细虚线基准。

## ✨ 核心亮点

- **双闭环协同**：电机增量式 PI 速度闭环（10ms 周期）+ 摆球位置式 PID 闭环，行驶与控球解耦又联动。
- **视觉引导的摆球控制**：上位机摄像头识别钢球位置，经串口以约 20Hz 下发坐标，主控只做控制与执行，符合赛题「槽内不增摩擦、仅用摄像头检测」要求。
- **弯道前馈补偿**：顺时针入弯时球受惯性向车头偏，叠加负向摆杆前馈并在出弯前平滑撤除，显著提升 Q4/Q5 带球行驶稳定性。
- **分段 PID 整定**：AB / 弯道 / CD / DA 各段实测一组摆球参数，按里程自动切换，参数随赛道几何自适应。
- **状态机驱动题型**：Q3 静态摆球、Q4/Q5 带球行驶由清晰的状态机（`ball.c` 中 Q3/Q4/Q5）管理，题型切换与停车逻辑可靠。
- **完整人机交互**：OLED 实时显示发车状态 / 题号 / 球位置 / 里程 / 计时；双按键完成题号切换与发车 / 停车。

---

## 🔧 硬件平台

- **主控**：TI MSPM0G3507（Cortex-M0+, 80MHz），tiarmclang v4.0.4.LTS，MSPM0 SDK 2.10.00.04 DriverLib
- **IDE**：TI Code Composer Studio 20.5.0（Theia），SysConfig 配置外设
- **直流减速电机 ×2**：TB6612 驱动，编码器测速（增量式 PI 速度闭环）
- **闭环步进电机（摆杆控制）**：ZDT_X42S 第二代闭环步进，UART 位置模式绝对运动，16 细分，约 0.1125°/脉冲；机械限位约 −24° ~ +70°
- **灰度循迹传感器 ×5**：红外光电（L1/L2/M/R1/R2）
- **OLED**：SSD1306，I2C，≤2 英寸，显示发车状态/题号/位置/里程/计时
- **视觉**：上位机摄像头识别钢球位置，通过串口下发坐标（小车仅用摄像头做位置检测，槽内不加摩擦）
- **按键 ×2**：题号切换（next）、发车/停车（enter）
- **图传**：摄像头固定于摆杆上方，接收端场外录制

---

## 🧩 软件架构与模块

```
main.c
  ├─ SYSCFG_DL_init()         SysConfig 生成的板级初始化（时钟/引脚/外设）
  ├─ OLED_Init()              显示初始化
  ├─ NVIC 中断使能            UART0(视觉) / GPIOB(按键) / GPIOA(编码器) / UART3(步进)
  ├─ ball_init()              步进回零 + 摆球控制初始化（阻塞约 1s）
  ├─ motor_init(1/2)          电机与编码器初始化
  └─ while(1) { target_led_proc(); oled_proc(); }

10ms 定时中断 (TIMA0) ──> motor_pid_INST_IRQHandler()
  ├─ calculate_speed()        编码器计数 → 实时速度 (mm/s)
  ├─ motor_PID()              增量式 PI 速度闭环
  ├─ adjust_motor()          灰度循迹比例控制（设置 target_speed）
  └─ ball_proc()              摆球位置式 PID（输出步进目标脉冲）

UART0 接收中断 ──> ball_uart_rx()   视觉帧解析（位置帧/目标帧）
UART3 接收中断 ──> stepper 应答解析
GPIO 中断       ──> 按键(题号/发车) + 编码器计数
SysTick 1ms     ──> sys_tick_ms++（计时基准）
```

### 模块清单（`ucdriver/`）

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| `ball` | ball.c / ball.h | 摆球控制核心：视觉帧解析、位置式 PID、Q3/Q4/Q5 状态机、弯道前馈 |
| `stepper` | stepper.c / stepper.h | ZDT_X42S 闭环步进驱动：使能 UART3、回零、位置模式绝对运动 |
| `motor` | motor.c / motor.h | TB6612 直流电机：PWM 输出、方向、增量式 PI 速度闭环、编码器测速 |
| `huidu` | huidu.c / huidu.h | 五路灰度循迹：比例控制（权重 −2/−1/0/+1/+2）、丢线检测 |
| `key` | key.c / key.h | 双按键中断、题号切换、发车/停车、编码器计数、消抖 |
| `oled` | oled.c / oled.h | SSD1306 显示驱动（GRAM 缓冲 + 刷新） |
| `uart` | uart.c / uart.h | UART0（视觉，115200）/ UART3（步进）收发 |
| `delay` | delay.c / delay.h | `delay_ms()` 基于 CPU 周期 |
| `mpu6050` | inv_mpu*.c / mpu_port.c | 六轴驱动（已编译，`main.c` 暂未使用） |

---

## 📌 外设与引脚分配

| 外设 | 引脚 | 功能 |
| --- | --- | --- |
| OLED (I2C1) | PB2=SCL, PB3=SDA | 显示，0x3C，100kHz |
| UART0 | PA28=TX, PA31=RX | 视觉位置帧，115200 8N1 |
| UART3 | （步进电机） | ZDT_X42S 控制，地址 0x01，校验 0x6B |
| 按键 next | PB6 | 题号 +1（1~5 循环），下降沿中断 |
| 按键 enter | PB7 | 发车/停车取反，下降沿中断 |
| 编码器 AA（左轮） | PA21=AA, PA22=AB | 上升沿计数 |
| 编码器 BA（右轮） | PB19=BA, PB20=BB | 上升沿计数 |
| 电机1 PWM | PA27 | TIMG7 CCP1，10kHz |
| 电机2 PWM | PA13 | TIMG0 CCP1，10kHz |
| 电机方向 | AIN1=PA9, AIN2=PA8, BIN1=PB18, BIN2=PA7, STBY=PB24 | TB6612 |
| 灰度 L2/L1/M/R1/R2 | PA17 / PB8 / PB9 / PA24 / PA2 | 循迹输入 |
| MPU6050 (I2C0) | PA0=SDA, PA1=SCL | 六轴（未使用） |
| LED | LED_LED0_PIN | 第五题目标接收确认闪烁 |
| ADC12 | PA16 | 已配置（未使用） |

> 外设配置的唯一来源是 `empty.syscfg`，修改后须用 SysConfig CLI 重新生成 `Debug/ti_msp_dl_config.{h,c}`，**不要手动编辑生成文件**。

---

## ⏱️ 时钟树

- 40MHz 内部晶振 → PLL(×5) → UDIV(/2) → **80MHz CPUCLK**
- **TIMG0**（电机2 PWM）：BUSCLK=40MHz，period=4000 → **10kHz**
- **TIMG7**（电机1 PWM）：BUSCLK=80MHz，prescale=1，period=4000 → **10kHz**
- **TIMA0**（PID 定时）：BUSCLK=80MHz，prescale=99，LOAD=7999 → **10ms**
- **SysTick**：80MHz / 80000 = **1ms** → `sys_tick_ms`

---

## 🎛️ 控制策略

### 1. 循线（灰度比例控制）

五路加权 `−2,−1,0,+1,+2`，比例系数 `Kp=40`；丢线 ≥300ms 标记 `was_lost`，恢复时 `huidu_straight_count++`，用于分段里程判定。

### 2. 电机速度闭环（增量式 PI）

`Kp=0.6, Ki=0.63`（Ki 已按 10ms 周期重定标）。速度公式：`counter / 260 × π × 47 / 0.01`（mm/s，编码器 260 线/转，轮径 47mm）。

### 3. 摆球控制（位置式 PID + 状态机）

- **坐标系**：视觉坐标 0~250mm，中心 O = 125mm；`BALL_VISION_REVERSE_AXIS=1` 做坐标翻转统一方向。
- **PID 输出** = 目标摆杆角度（脉冲）：`KP·e + KI·Σe + KD·de/dt`，含死区（1mm）、积分限幅（±300 防 windup）、微分低通、斜率限幅（防摆杆瞬间反向）。
- **Q3 静态摆球**：状态机 `中心O → +5cm(175) → 折返 −5cm(75) → 保持`，到达判定 `|e|≤8mm 持续 300ms`。
- **Q4/Q5 带球行驶**：目标恒为中心 O（或 Q5 的指定位置），按里程分段切换 PID 参数（AB / 弯道 / CD / DA 各一组已实测参数），并叠加**弯道前馈**（顺时针入弯球向车头偏，叠加负向摆杆前馈，出弯前平滑撤除）。
- **视觉通信协议**：
  - 位置帧：`0xAA <position> 0x55`（约 20Hz）
  - 目标帧（仅 Q5 停车阶段）：`0xBB <target> 0x55`
  - 丢帧 >500ms 置 `ball_frame_lost`，控制保持当前角度。

### 4. 题号与操作

- `next` 键：题号 1↔5 循环；进入 Q5 时强制停车并等待本轮新目标。
- `enter` 键：发车/停车取反；Q5 未收到有效目标时禁止发车（LED 闪烁 3 次确认目标已收到）。
- OLED 显示：状态 / 题号 / 球位置(mm) 或 里程(m) / Q2 计时 或 摆球状态。

---

## 📂 目录结构

```
car_demo/
├── main.c                 主循环、中断、OLED 显示
├── empty.syscfg           SysConfig 外设配置（唯一来源）
├── Debug/                 CCS 构建产物（makefile/sources.mk/编译数据库）
├── ucdriver/
│   ├── ball/              摆球控制（PID + 状态机）
│   ├── stepper/           闭环步进电机驱动
│   ├── motor/             直流电机 + 编码器 + PI
│   ├── huidu/             灰度循迹
│   ├── key/               按键 + 编码器中断
│   ├── oled/              SSD1306 显示
│   ├── uart/              UART 收发
│   ├── delay/             延时
│   └── mpu6050/           六轴驱动（未启用）
├── docs/                  需求/方案设计文档（plans、specs）
├── README.md
└── ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.docx
```

---

## 🛠️ 构建与烧录

1. 用 **TI Code Composer Studio 20.5.0** 打开本工程（依赖 `F:/ti/ccs2050/...` 中的 SDK 绝对路径，**无法脱离 CCS 构建**）。
2. 若修改 `empty.syscfg`，先通过 SysConfig CLI 重新生成 `Debug/ti_msp_dl_config.{h,c}`。
3. 新增 `.c` 源文件：在 `Debug/sources.mk` 的 `SUBDIRS` 加目录 → 该目录 `subdir_vars.mk` 注册文件 → `Debug/makefile` 的 `ORDERED_OBJS` 末尾加 `.o`。
4. 编译下载，`while(1)` 中实时显示；上电自动步进回零（约 1s）。

---

## 📷 效果展示

> 以下为建议补充的素材位，仓库当前未包含图片，请自行拍摄后放入 `docs/imgs/` 并替换占位说明：

- **整车外观**：小车 + 摆杆 + 摄像头安装姿态
- **摆杆特写**：PPR 水管凹槽、刻度线（间距 0.1cm）、铰链/控制机构
- **图传画面**：接收端显示的钢球在凹槽中滚动的实时画面
- **循迹运行**：沿黑色环形赛道顺时针行驶的现场
- **OLED 显示**：发车状态 / 题号 / 球位置 / 计时界面

<!-- 占位示意（替换为真实截图即可） -->
<p align="center">
  <img alt="整车/摆杆/图传效果占位" src="https://placehold.co/720x320/0A84FF/FFFFFF?text=小车+摆杆+图传+效果截图">
</p>

---

## 🐞 已知问题与 TODO

- `OLED_GRAM[144][8]` 偏大，应为 `[128][8]`（浪费约 128B）。
- `OLED_ClearPoint()` 实现晦涩，可简化为 `OLED_GRAM[x][i] &= ~n`。
- ADC12（PA16, VREF 2.5V）已配置但代码未使用。
- MPU6050 驱动已编译但 `main.c` 未初始化/使用（当前用摄像头做位置检测，符合赛题要求）。
- `SysTick 1ms` 全局 `sys_tick_ms` 目前仅 `SysTick_Handler` 递增，作计时基准。
- Q4 全圈分段参数仍在实测调优中（AB 段参数已锁定，禁止修改）。

---

## 📜 许可证

本工程为电赛参赛作品，代码仅供学习与竞赛使用。DriverLib 相关头文件版权归 Texas Instruments 所有。
