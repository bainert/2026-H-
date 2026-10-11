#ifndef __BALL_H__
#define __BALL_H__

#include <stdint.h>

//小球位置（mm，0~250），由 UART0 视觉帧更新
extern volatile uint8_t ball_position_mm;
//视觉丢帧标志：0-正常 1-超过500ms无新帧
extern volatile uint8_t ball_frame_lost;
//摆球控制运行标志：Q3/Q4/Q5 且 enter=1 时为 1
extern volatile uint8_t ball_running;
//摆球运动阶段：0-待命/中心 1-前往+5cm 2-折返-5cm 3-保持（Q5指定位置）
extern volatile uint8_t ball_phase;
//第五题指定位置（mm，0~250），由 0xBB data 0x55 目标帧更新
extern volatile uint8_t ball_target_position_mm;
//第五题目标接收标志：收到有效目标后置1，未置1时禁止按 Enter 发车
extern volatile uint8_t ball_target_received;
//目标更新序号：每接收一个新的有效目标递增，供主循环触发 LED 闪烁确认
extern volatile uint8_t ball_target_update_seq;

//初始化：使能 UART3 + 触发回零 + 阻塞延时1s
void ball_init(void);
//10ms 周期调用（在 motor_pid 定时器 ISR 中）
void ball_proc(void);
//视觉串口字节回调（UART0 ISR 中逐字节调用）
void ball_uart_rx(uint8_t byte);
//清除第五题目标，下一次发车前必须重新接收 0xBB 目标帧
void ball_q5_reset_target(void);

#endif
