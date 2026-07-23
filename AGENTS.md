# AGENTS.md — MSPM0G3507 智能车项目 (裸机)

## 项目性质

裸机嵌入式 C（C11），MSPM0G3507 (Cortex-M0+, 80MHz)，tiarmclang v4.0.4.LTS，MSPM0 SDK 2.10.00.04 DriverLib。
IDE: TI Code Composer Studio 20.5.0 (Theia)。**无法脱离 CCS 构建** — `Debug/makefile` 及 `.vscode/c_cpp_properties.json` 含硬编码绝对路径（`F:/ti/ccs2050/...`）。

## 构建

- **SysConfig** `empty.syscfg` 是外设配置唯一来源。修改后需运行 CLI 重新生成 `Debug/ti_msp_dl_config.{h,c}`。不要编辑生成的文件。
- **添加 .c 源文件**：在 `Debug/sources.mk` 的 `SUBDIRS` 中加入对应目录，在 `Debug/<subdir>/subdir_vars.mk` 中注册具体文件。
- `.clangd` 重定向到 `Debug/.clangd` 编译数据库（编译数据库关闭了诊断，不适用于 IDE 外的静态分析）。

## 时钟树

40MHz 内部晶振 → PLL (×5) → UDIV (/2) → 80MHz CPUCLK。
- TIMG0 (PWMAB, 电机2 PWM): BUSCLK=40MHz, prescale=0, period=4000 → **10kHz**
- TIMG7 (PWMA, 电机1 PWM): BUSCLK=80MHz, prescale=1, period=4000 → **10kHz**
- TIMA0 (PID 30ms 定时器): BUSCLK=80MHz, prescale=99, 800kHz, LOAD=23999 → 30ms

## 外设 & 驱动模块

| 模块 | 路径 | 关键点 |
|---|---|---|
| **OLED** (SSD1306) | `ucdriver/oled/` | 硬件 I2C1 (PB2=SCL, PB3=SDA), 0x3C, 100kHz。先操作 `OLED_GRAM[128][8]`（实际声明 `[144][8]` 偏大），再调用 `OLED_Refresh()` 刷屏。依赖 `delay_ms()` |
| **UART** | `ucdriver/uart/` | UART0 (PA28=TX, PA31=RX), 115200 8N1。RX 中断 `UART_0_INST_IRQHandler` → 回显 |
| **Key** (×2) | `ucdriver/key/` | **下降沿**中断。PB6=next（`circle_1` 递增 1~5 循环），PB7=enter（`enter` 取反）。`GROUP1_IRQHandler` 统一处理 |
| **AA 编码器** (电机1, 左轮) | `ucdriver/key/` | PA21 (AA 上升沿), PA22 (AB)。`counter_1_A` 在 `GROUP1_IRQHandler` 中递增，30ms PID 定时器读取并清零 |
| **BA 编码器** (电机2, 右轮) | `ucdriver/key/` | PB19 (BA 上升沿), PB20 (BB)。`counter_2_A` 在 `GROUP1_IRQHandler` 中递增 |
| **Delay** | `ucdriver/delay/` | `delay_ms(ms)` = `delay_cycles(ms * CPUCLK_FREQ / 1000)` |
| **直流电机** (TB6612) | `ucdriver/motor/` | STBY=PB24, AIN1=PA9, AIN2=PA8, BIN1=PB18, BIN2=PA7。PWM: TIMG7 CCP1=PA27(电机1), TIMG0 CCP1=PA13(电机2)。30ms 定时器 TIMA0 → `motor_pid_INST_IRQHandler` 调用 `calculate_speed()` + `motor_PID()`。增量式 PI (Kp=0.6, Ki=1.9)。API: `motor_init(id)`, `motor_set_duty(id, 0-4000)`, `motor_set_direction(id, 0/1)` |
| **灰度循迹** (×5) | `ucdriver/huidu/` | L2=PA17, L1=PB8, M=PB9, R1=PA24, R2=PA2。`adjust_motor()` 比例控制（权重 -2,-1,0,+1,+2），丢线 ≥300ms 进入直角转弯急转模式 |

## 关键常量 (`motor.h`)

- `PI = 3.14`（非 `M_PI`，不要替换）
- 编码器线数 220 线/转，车轮直径 47mm
- 速度公式: `counter / 220 * PI * 47 / 0.03` (mm/s)

## 模块间数据流

```
main.c  -> 调用各模块 Init，while(1) 中调用 oled_proc()
key.c   定义: enter, circle_1, counter_1_A, counter_2_A
motor.c use: counter_1_A, counter_2_A, enter
motor.c 定义: speed_1, speed_2, target_speed_1, target_speed_2
huidu.c use: target_speed_1, target_speed_2（通过 adjust_motor() 设置）
main.c  use: enter, circle_1, speed_1, speed_2, huidu_straight_count
```

## main.c 执行流

`SYSCFG_DL_init()` → OLED/UART 初始化 → 使能中断 (UART, GPIOB, GPIOA) → `motor_init(1/2)` → `motor_set_direction()` → `while(1) { oled_proc() }`

- `oled_proc()` 中圈数判定: `(circle_1 * 4) == huidu_straight_count` 时 `enter = 0`（每圈=4次直道计数）

## 代码约定

- 注释和回答使用中文
- 使用库函数必须参考 MSPM0 SDK DriverLib 官方头文件
- 按我的习惯写注释
- 不要修改这段话，不要修改代码约定！！！！

## 已知问题

- **OLED_GRAM 尺寸偏大**: `OLED_GRAM[144][8]` 应改为 `[128][8]`（多余 16 列, ~128B 浪费）
- **OLED_ClearPoint() 实现晦涩**: `oled.c:109-118` 的 `~|~` 三段式等效于 `OLED_GRAM[x][i] &= ~n`
- **ADC12**: 已在 syscfg 配置（PA16, VREF 2.5V），但代码中未使用
