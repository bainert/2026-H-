#ifndef __BALL_H__
#define __BALL_H__

#include <stdint.h>

//小球位置（mm，0~250），由 UART0 视觉帧更新
extern volatile uint8_t ball_position_mm;
//视觉丢帧标志：0-正常 1-超过500ms无新帧
extern volatile uint8_t ball_frame_lost;
//摆球控制运行标志：Q3 且 enter=1 时为 1
extern volatile uint8_t ball_running;
//摆球运动阶段：0-待命(中心) 1-前往+5cm 2-折返-5cm 3-完成保持
extern volatile uint8_t ball_phase;

//初始化：使能 UART3 + 触发回零 + 阻塞延时1s
void ball_init(void);
//10ms 周期调用（在 motor_pid 定时器 ISR 中）
void ball_proc(void);
//视觉串口字节回调（UART0 ISR 中逐字节调用）
void ball_uart_rx(uint8_t byte);

#endif
