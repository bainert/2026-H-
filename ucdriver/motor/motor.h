#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"

#define PI 3.14
//编码器线数
#define motor_xianshu 260
//轮胎直径 mm
#define motor_wheel_d 47

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

extern volatile float speed_1;
extern volatile float speed_2;
extern volatile float target_speed_1;
extern volatile float target_speed_2;
extern volatile float odometer;

#endif

//G3507   TB6612
//PB24 - STBY
//PA8 - AIN1
//PA9 - AIN2
//PA12 - PWMMA
//3V3 - VCC

//TB6612  MC301
//AO1 - M+
//AO2 - M-
