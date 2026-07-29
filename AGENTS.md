# AGENTS.md — MSPM0G3507 智能车项目 (裸机)

## 项目性质

裸机嵌入式 C（C11），MSPM0G3507 (Cortex-M0+, 80MHz)，tiarmclang v4.0.4.LTS，MSPM0 SDK 2.10.00.04 DriverLib。
IDE: TI Code Composer Studio 20.5.0 (Theia)。**无法脱离 CCS 构建** — `Debug/makefile` 及 `.vscode/c_cpp_properties.json` 含硬编码绝对路径（`F:/ti/ccs2050/...`）。

## 构建

- **SysConfig** `empty.syscfg` 是外设配置唯一来源。修改后必须运行 CLI 重新生成 `Debug/ti_msp_dl_config.{h,c}`。不要编辑生成的文件。
- **添加 .c 源文件**：在 `Debug/sources.mk` 的 `SUBDIRS` 中加入对应目录，再在该目录的 `subdir_vars.mk` 中注册具体文件。最后在 `Debug/makefile` 的 `ORDERED_OBJS` 末尾添加 `.o` 目标。
- `.clangd` 重定向到 `Debug/.clangd/compile_commands.json`（编译数据库关闭了诊断，不适用于 IDE 外的静态分析）。

## 时钟树

40MHz 内部晶振 → PLL (×5) → UDIV (/2) → 80MHz CPUCLK。
- TIMG0 (PWMAB, 电机2 PWM): BUSCLK=40MHz, prescale=0, period=4000 → **10kHz**
- TIMG7 (PWMA, 电机1 PWM): BUSCLK=80MHz, prescale=1 (`clockPrescale=2` 在 syscfg 中用 1-indexed), period=4000 → **10kHz**
- TIMA0 (PID 30ms 定时器): BUSCLK=80MHz, prescale=99, 800kHz, LOAD=23999 → 30ms
- SysTick: 80MHz / 80000 = 1ms → `sys_tick_ms++` 在 `SysTick_Handler`

## 外设 & 驱动模块

| 模块 | 路径 | 关键点 |
|---|---|---|
| **OLED** (SSD1306) | `ucdriver/oled/` | 硬件 I2C1 (PB2=SCL, PB3=SDA), 0x3C, 100kHz。先操作 `OLED_GRAM[128][8]`（实际声明 `[144][8]` 偏大），再调用 `OLED_Refresh()` 刷屏。依赖 `delay_ms()` |
| **UART** | `ucdriver/uart/` | UART0 (PA28=TX, PA31=RX), 115200 8N1。RX 中断 `UART_0_INST_IRQHandler` → 回显 |
| **Key** (×2) | `ucdriver/key/` | **下降沿**中断。PB6=next （`question` 递增 1~5 循环），PB7=enter（`enter` 取反）。消抖: `key_debounce_cnt` 在 ISR 中置 10，30ms PID 定时器 ISR 递减 |
| **AA 编码器** (电机1, 左轮) | `ucdriver/key/` | PA21 (AA 上升沿), PA22 (AB)。`counter_1_A` 在 `GROUP1_IRQHandler` 递增，30ms PID 定时器读取并清零 |
| **BA 编码器** (电机2, 右轮) | `ucdriver/key/` | PB19 (BA 上升沿), PB20 (BB)。`counter_2_A` 在 `GROUP1_IRQHandler` 递增 |
| **Delay** | `ucdriver/delay/` | `delay_ms(ms)` = `delay_cycles(ms * CPUCLK_FREQ / 1000)` |
| **直流电机** (TB6612) | `ucdriver/motor/` | STBY=PB24, AIN1=PA9, AIN2=PA8, BIN1=PB18, BIN2=PA7。PWM: TIMG7 CCP1=PA27(电机1), TIMG0 CCP1=PA13(电机2)。10ms 定时器 TIMA0 → `motor_pid_INST_IRQHandler` 调用 `calculate_speed()` + `motor_PID()`。增量式 PI (Kp=0.6, Ki=1.9)。API: `motor_init(id)`, `motor_set_duty(id, 0-4000)`, `motor_set_direction(id, 0/1)` |
| **灰度循迹** (×5) | `ucdriver/huidu/` | L2=PA17, L1=PB8, M=PB9, R1=PA24, R2=PA2。`adjust_motor()` 比例控制（权重 -2,-1,0,+1,+2，Kp=40），丢线 ≥300ms 标记 `was_lost`，恢复时 `huidu_straight_count` 自增 |
| **MPU6050** (六轴) | `ucdriver/mpu6050/` | I2C0 (PA0=SDA, PA1=SCL)。DMP 驱动库 (`inv_mpu_dmp_motion_driver`)。目前已编译但 `main.c` 未引用 |

## 关键常量 (`motor.h`)

- `PI = 3.14`（非 `M_PI`，不要替换）
- 编码器线数 260 线/转，车轮直径 47mm
- 速度公式: `counter / 260 * PI * 47 / 0.03` (mm/s)

## 模块间数据流

```
main.c  -> 调用各模块 Init，while(1) 中调用 oled_proc()
key.c  定义: enter, question, counter_1_A, counter_2_A, key_debounce_cnt
motor.c 使用: counter_1_A, counter_2_A, enter, key_debounce_cnt
motor.c 定义: speed_1, speed_2, target_speed_1, target_speed_2
huidu.c 使用: target_speed_1, target_speed_2（通过 adjust_motor() 设置）
main.c  使用: enter, question, speed_1, speed_2, huidu_straight_count
```

## main.c 执行流

`SYSCFG_DL_init()` → OLED/UART 初始化 → 使能中断 (UART, GPIOB, GPIOA) → `motor_init(1)` / `motor_init(2)` → `motor_set_direction()` → `while(1) { oled_proc() }`

- `oled_proc()` 显示发车状态、题号、数据

## 代码约定

- 我是一个参加全国大学生电子设计大赛的学生，我现在做的是比赛中的某个项目，你是一个资深的嵌入式开发大佬，你要合理的规划我的项目结构，合理的修改代码，实现我的功能需求，同时要保持代码的可读性
- 注释和回答使用中文
- 使用库函数必须参考 MSPM0 SDK DriverLib 官方头文件
- 按我的习惯写注释，注释可以详细一点
- 不要修改这段话，不要修改代码约定！！！！

## 已知问题

- **OLED_GRAM 尺寸偏大**: `OLED_GRAM[144][8]` 应改为 `[128][8]`（多余 16 列, ~128B 浪费）
- **OLED_ClearPoint() 实现晦涩**: `oled.c:109-118` 的 `~|~` 三段式等效于 `OLED_GRAM[x][i] &= ~n`
- **ADC12**: 已在 syscfg 配置（PA16, VREF 2.5V），但代码中未使用
- **MPU6050**: 驱动已编译 (`ucdriver/mpu6050/`)，但 `main.c` 未初始化或使用
- **SysTick 1ms 全局 tick**: `sys_tick_ms` 在 `main.c` 声明为 extern，但当前仅 `SysTick_Handler` 递增，无引用方

## 赛题
赛题名称：车载平衡滚球运动控制系统（H题）
赛事背景：2026年全国大学生电子设计竞赛赛区赛（TI杯）暨模拟电子系统设计专题赛选拔赛赛题。

一、任务
设计并制作一辆载有平衡滚球运动控制装置的循线小车。小车可沿黑色环形路线行驶，行驶过程中需使钢球在滚球运动控制装置内保持相对稳定。装置由小车、带凹槽的摆杆、摆杆控制机构和钢球组成，摆杆左端通过铰链或合页固定于小车平板，高度不低于5cm，右端连接控制机构，可调节摆杆角度，使钢球稳定在指定位置。

二、设计要求（共8项，分数累加）
图传监测装置（6分）
安装摆球位置监测图传系统，含发送（固定于小车）和接收模块（置于线路外）。需实时显示钢球在凹槽中的滚动画面，并完整记录测试视频，支持回放。

基本循线行驶（16分）
小车从A点启动，沿黑线顺时针行驶一圈并停回A点，总时间≤20s，停车偏差≤2cm。

静态摆球控制（13分）
小车静止时，控制钢球从中心点O向+5cm处运行，到达后折返至-5cm处并稳定。运行时间≤5s，±5cm处的最大误差绝对值≤1cm。

带球行驶至B点（20分）
小车从A点出发，钢球置于中心点O，沿黑线顺时针行驶至B点，AB段用时≤8s，过程中钢球须稳定在中心点附近，误差≤1cm。

带球行驶完整一圈（20分）
从A点出发，钢球置于中心点O，沿黑线顺时针行驶一圈并经过A点，总时间≤30s，全程钢球稳定在中心点附近，误差≤1cm。

带球行驶且任意位置稳定（20分）
从A点出发，钢球初始置于摆杆任意指定位置，沿黑线顺时针行驶一圈并经过A点，总时间≤30s，全程钢球稳定在该指定位置附近，误差≤1cm。

其他创新或附加功能（5分）

设计报告（20分）
包括方案论证、理论分析、电路与程序流程、测试方案与结果、报告结构与规范性等。

三、技术说明
场地：白色广告布喷绘，黑色环形轨迹线宽1.8±0.2cm，设有A、B、C、D四点，AB和CD段为1.5m直线，BC和DA段为半径0.5m的半圆弧。A点设有垂直的启停线及停车基准虚线。

车身：尺寸≤35cm×25cm，轮式驱动，车载电池供电，需指定测试基准点。

行驶规则：不得人为干涉或遥控，小车投影不得脱离轨迹线，否则失败。停车偏差为基准点与启停线的绝对距离。

循迹模块：仅限红外光电模块，数量不限。必须有启动按键和显示屏（≤2英寸），计时同步启动。

图传要求：摄像头固定于摆杆上方，画面覆盖整个摆杆，能清晰显示钢球位置；接收端连接存储设备，赛后封存。

摆杆结构：由25cm长、4分PPR水管（外径2cm，壁厚0.34cm）改造，凹槽内壁光滑，钢球直径约1cm。槽内不得增加摩擦，位置检测仅用摄像头。凹槽边沿贴有刻度线（间距0.1cm，不允许贴在槽内），摆杆不超出车身。

### 赛道布局
一、场地材质与制作方式
材质：采用白色广告布喷绘制作而成。

颜色基调：背景为纯白色，路线轨迹为黑色。

携带规定：测试时允许参赛队伍自带符合规格的场地。

二、主路线（环形黑色轨迹）核心几何参数
轨迹线宽：1.8 cm，允许误差 ±0.2 cm（即范围在 1.6 cm 至 2.0 cm 之间）。

整体形状：类田径跑道形（由两条平行的直线段和两个半圆弧衔接而成的闭环）。

三、四个关键点位（A、B、C、D）及分段尺寸
地图上明确设定了 A、B、C、D 四个点，将环形赛道划分为四段：

AB段（直线段）：

长度 1.5 米。

从 A 点出发，沿顺时针方向到达 B 点。

BC段（圆弧段）：

半径为 0.5 米 的半圆弧。

连接 B 点和 C 点，方向为顺时针弯曲。

CD段（直线段）：

长度 1.5 米。

从 C 点到达 D 点，与 AB 段平行。

DA段（圆弧段）：

半径为 0.5 米 的半圆弧。

连接 D 点回到 A 点，与 BC 段对称，构成完整闭环。

总周长推算：两条直线段（1.5m × 2 = 3m）加上两个半圆弧（合起来为一个完整圆，周长 = 2 × π × 0.5m ≈ 3.14m），赛道中心线总长约 6.14 米。

四、A点（起点/终点）的特殊停车标记（极其关键）
在 A 点位置，除了主线黑线外，还额外绘制了用于精准停车的辅助标记：

启停线（垂直于主路线）：

这是一条与环形黑线垂直相交的黑色短线段。

长度：5 cm。

线宽：与主路线一致，为 1.8 ± 0.2 cm。

位置：居中横跨在环形黑线上，作为小车跑完一圈后停止的参照横线。

停车基准虚线（沿主路线中心绘制）：

在启停线的中心轴线上，沿着行车方向绘制了一条极细的虚线。

线宽（粗细）：仅 0.1 cm（非常细，用于精确定位）。

长度：30 cm。

作用：这条虚线贯穿启停线，裁判依据小车车体上的指定测试点相对于这条虚线（或启停线）的投影距离来判断停车偏差（≤2cm）。

五、整体幅面尺寸建议
虽然题中没有严格限定整张广告布的外框尺寸，但为了容纳：

两个直线段长度 1.5m；

两个半径为 0.5m 的半圆弧（直径方向跨度为 1m）；

加上小车运行以及启停线（30cm 虚线延伸）；

建议场地幅面至少为：长约 2.5 米（1.5m直线 + 1m直径），宽约 2.0 米（1.5m直线间距 + 两侧余量），以确保小车在弯道处不会跑出边界。

六、行驶方向与轨迹判定
行驶方向：全程要求顺时针行驶（即 A→B→C→D→A）。

压线规则：行驶过程中，小车的投影必须完全压在黑色轨迹线上，若投影完全脱离黑线，即判定为本次测试失败。


