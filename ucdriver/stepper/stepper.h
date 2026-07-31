#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <stdint.h>

//最近一次电机应答字节（02=命令正确 12=已在零点 E2=参数错 EE=格式错 9F=动作完成）
extern volatile uint8_t stepper_ack;

//初始化：使能 UART3 接收中断
void stepper_init(void);
//触发单圈就近回零（9A 命令）
void stepper_home_start(void);
//位置模式绝对运动：pulses 为相对零点的有符号脉冲（正=CW 负=CCW）
void stepper_move(int32_t pulses);

#endif
