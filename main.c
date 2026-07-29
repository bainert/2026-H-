/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "uart.h"
#include "delay.h"
#include "key.h"
#include "motor.h"

//是否发车 0-未发车 1-发车
extern volatile int enter;

//题号
extern volatile int question;

//直角弯计数变量
extern volatile uint8_t huidu_straight_count;

//实时速度
extern volatile float speed_1;
extern volatile float speed_2;

// 外部声明 mpu_port.c 里的时间戳变量
extern volatile uint32_t sys_tick_ms;

// 第二问: 跑圈状态
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;

// SysTick 中断服务函数 (每 1ms 触发一次)
void SysTick_Handler(void) {
    sys_tick_ms++;
}

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
            uint32_t elapsed = sys_tick_ms - lap_start_ms;
            OLED_ShowString(0, 24, (u8 *)"T:", 12);
            OLED_ShowFloat(12, 24, (float)elapsed / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 24, (u8 *)"s", 12);
        }
        else if(lap_state == LAP_COMPLETE)
        {
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

int main(void)
{
    SYSCFG_DL_init();

    //初始化OLED
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    
    //开启串口接收中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    //开启按键中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    //开启AA编码器中断
    NVIC_EnableIRQ(dc_motor_GPIOA_INT_IRQN);
    
    //初始化电机
    motor_init(1);
    motor_init(2);

    //设置目标速度
    // target_speed_1 = 200.0f;
    // target_speed_2 = 500.0f;

     //设置电机方向
    motor_set_direction(1, 0);
    motor_set_direction(2, 0);  
    while (1) {
        //oled常亮
        DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
        oled_proc();
        
    }
}
