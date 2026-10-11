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
#include "ball.h"

#define TARGET_LED_HALF_PERIOD_MS  150U
#define TARGET_LED_PHASE_COUNT       6U

//是否发车 0-未发车 1-发车
extern volatile int enter;

//题号
extern volatile int question;

//直角弯计数变量
extern volatile uint8_t huidu_straight_count;

//实时速度
extern volatile float speed_1;
extern volatile float speed_2;

//里程计
extern volatile float odometer;

// 外部声明 mpu_port.c 里的时间戳变量
extern volatile uint32_t sys_tick_ms;

// Q3/Q4/Q5 摆球控制
extern volatile uint8_t ball_position_mm;
extern volatile uint8_t ball_frame_lost;
extern volatile uint8_t ball_running;
extern volatile uint8_t ball_target_position_mm;
extern volatile uint8_t ball_target_received;
extern volatile uint8_t ball_target_update_seq;

// Q3 摆球运动阶段（0-待命 1-去+5cm 2-去-5cm 3-保持）
extern volatile uint8_t ball_phase;

// 第二问: 跑圈状态
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;

// SysTick 中断服务函数 (每 1ms 触发一次)
void SysTick_Handler(void) {
    sys_tick_ms++;
}

//收到新的第五题指定位置后，LED 非阻塞闪烁 3 次作为通信确认。
//闪烁期间中断和主循环仍正常运行，不使用 delay_ms() 阻塞发车控制。
static void target_led_proc(void)
{
    static uint8_t last_update_seq = 0;
    static uint8_t blinking = 0;
    static uint32_t blink_start_ms = 0;
    uint8_t update_seq = ball_target_update_seq;

    if(update_seq != last_update_seq)
    {
        last_update_seq = update_seq;
        blink_start_ms = sys_tick_ms;
        blinking = 1;
    }

    if(blinking)
    {
        uint32_t elapsed_ms = sys_tick_ms - blink_start_ms;
        uint32_t phase = elapsed_ms / TARGET_LED_HALF_PERIOD_MS;

        if(phase >= TARGET_LED_PHASE_COUNT)
        {
            blinking = 0;
            DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        }
        else if((phase & 1U) == 0U)
        {
            DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
        }
        else
        {
            DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        }
    }
    else
    {
        DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
    }
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

    //第三行：先清空动态显示区域，防止 LOST、数值和单位切换时残留字符
    OLED_ShowString(0, 24, (u8 *)"            ", 12);

    //第三行：Q3/Q4/Q5 显示小球位置，否则显示里程
    if(question == 3 || question == 4 || question == 5)
    {
        OLED_ShowString(0, 24, (u8 *)"P:", 12);
        if(ball_frame_lost)
        {
            OLED_ShowString(12, 24, (u8 *)"LOST", 12);
        }
        else
        {
            OLED_ShowNum(12, 24, (u32)ball_position_mm, 3, 12);
            OLED_ShowString(30, 24, (u8 *)"mm", 12);
        }

        if(question == 5)
        {
            OLED_ShowString(54, 24, (u8 *)"T:", 12);
            if(ball_target_received)
                OLED_ShowNum(66, 24, (u32)ball_target_position_mm, 3, 12);
            else
                OLED_ShowString(66, 24, (u8 *)"---", 12);
        }
    }
    else
    {
        OLED_ShowString(0, 24, (u8 *)"D:", 12);
        OLED_ShowFloat(12, 24, odometer / 1000.0f, 1, 2, 12);
        OLED_ShowString(36, 24, (u8 *)"m", 12);
    }

    //第四行：先清空动态显示区域，防止长文本切换为短文本后出现残影
    OLED_ShowString(0, 36, (u8 *)"            ", 12);

    //第四行：Q2 计时 / Q3、Q4、Q5 摆球状态 / 其他题号保持空白
    if(question == 2)
    {
        if(lap_state == LAP_RUNNING)
        {
            uint32_t elapsed = sys_tick_ms - lap_start_ms;
            OLED_ShowString(0, 36, (u8 *)"T:", 12);
            OLED_ShowFloat(12, 36, (float)elapsed / 1000.0f, 2, 2, 12);
            OLED_ShowString(42, 36, (u8 *)"s", 12);
        }
        else if(lap_state == LAP_COMPLETE)
        {
            OLED_ShowString(0, 36, (u8 *)"F:", 12);
            OLED_ShowFloat(12, 36, (float)lap_total_ms / 1000.0f, 2, 2, 12);
            OLED_ShowString(42, 36, (u8 *)"s!", 12);
        }
        else  // LAP_IDLE
        {
            OLED_ShowString(0, 36, (u8 *)"Press ENTER", 12);
        }
    }
    else if(question == 5 && ball_target_received == 0)
    {
        OLED_ShowString(0, 36, (u8 *)"WAIT TARGET", 12);
    }
    else if(question == 3 || question == 4 || question == 5)
    {
        OLED_ShowString(0, 36, (u8 *)"BAL:", 12);
        if(ball_running)
            OLED_ShowString(36, 36, (u8 *)"RUN ", 12);
        else
            OLED_ShowString(36, 36, (u8 *)"STOP", 12);
        //阶段字符：C-待命中心 +-去+5cm --去-5cm H-保持
        //（x=60：紧接 "RUN "/"STOP" 之后，12 号字体每字符 6px，RUN 占用 36~60）
        switch(ball_phase)
        {
        case 1: OLED_ShowString(60, 36, (u8 *)"+", 12); break;
        case 2: OLED_ShowString(60, 36, (u8 *)"-", 12); break;
        case 3: OLED_ShowString(60, 36, (u8 *)"H", 12); break;
        default: OLED_ShowString(60, 36, (u8 *)"C", 12); break;
        }
    }
    else
    {
        // 第四行已经在分支判断前清空
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
    
    //Q3 摆球控制初始化（含步进电机回零，阻塞约1s）
    ball_init();

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
        //第五题目标接收指示灯
        target_led_proc();
        oled_proc();
        
    }
}
