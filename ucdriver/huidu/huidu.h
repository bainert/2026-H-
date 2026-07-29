#ifndef HUIDU_H
#define HUIDU_H


#include "ti_msp_dl_config.h"
#include "motor.h"

// 接线
// 灰度模块
// VCC       5V(根据说明书确定具体是多少电压，不能接错了)
// GND       GND
// L2       GPIOA.17
// L1       GPIOB.8
// M        GPIOB.9
// R1       GPIOA.24
// R2       GPIOA.2

extern volatile uint8_t huidu_straight_count;
extern uint8_t huidu_value[5];

void huidu_get_value();
void adjust_motor();

#endif
