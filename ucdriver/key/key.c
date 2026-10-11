#include "key.h"
#include "ball.h"
#include "ti_msp_dl_config.h"

//是否发车 0-未发车 1-发车
volatile int enter = 0;
//题号
volatile int question = 0;

//AA编码器计数变量
volatile uint32_t counter_1_A = 0;
volatile uint32_t counter_2_A = 0;

//按键消抖计数器：key ISR置30，10ms PID定时器ISR递减，30×10ms=300ms消抖
volatile uint8_t key_debounce_cnt = 0;

// 第二问：跑圈状态
volatile lap_state_t lap_state = LAP_IDLE;
volatile uint32_t lap_start_ms = 0;
volatile uint32_t lap_total_ms = 0;

uint8_t get_key_state(uint32_t key_state)
{
    uint32_t state = DL_GPIO_readPins(key_PORT, key_state);

    if(state == key_state) return 1;
    else return 0;
} 

void GROUP1_IRQHandler(void)
{
    uint32_t key_int = DL_GPIO_getPendingInterrupt(key_PORT);

    //按键消抖处理（不干扰编码器中断）
    if(key_int == key_next_IIDX || key_int == key_enter_IIDX)
    {
        //消抖窗口内（30×10ms=300ms），跳过所有弹跳边沿
        if(key_debounce_cnt)
            return;

        //锁定：置30，由10ms PID定时器ISR递减至0
        key_debounce_cnt = 30;

        if(key_int == key_next_IIDX)
        {
            question += 1;
            if(question > 5) question = 1;

            //进入第五题时强制保持停车，并等待本轮新的指定位置。
            if(question == 5)
            {
                enter = 0;
                ball_q5_reset_target();
            }
        }
        else if(key_int == key_enter_IIDX)
        {
            if(question == 5)
            {
                if(enter)
                {
                    //手动停车后清除旧目标，防止下一次误用上一次的位置发车。
                    enter = 0;
                    ball_q5_reset_target();
                }
                else if(ball_target_received)
                {
                    //第五题只有收到 0xBB data 0x55 完整目标帧后才允许发车。
                    enter = 1;
                }
            }
            else
            {
                enter = !enter;
            }
        }
    }
    else if(key_int == dc_motor_BA_IIDX)
    {
        //BA编码器中断 计数值加1
        counter_2_A ++;
    }

    switch(DL_GPIO_getPendingInterrupt(dc_motor_AA_PORT))
    {
        case dc_motor_AA_IIDX:
            //AA编码器中断 计数值加1
            counter_1_A ++;
            break;    
        default:
            break;
    }
}
