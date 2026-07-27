#include "key.h"
#include "ti_msp_dl_config.h"

//是否发车 0-未发车 1-发车
volatile int enter = 0;
//题号
volatile int question = 0;

//AA编码器计数变量
volatile uint32_t counter_1_A = 0;
volatile uint32_t counter_2_A = 0;

//按键消抖计数器：key ISR置2，30ms PID定时器ISR递减，2×30ms=60ms消抖
volatile uint8_t key_debounce_cnt = 0;

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
        //消抖窗口内（300ms），跳过所有弹跳边沿
        if(key_debounce_cnt)
            return;

        //锁定：置2，由30ms PID定时器ISR递减至0
        key_debounce_cnt = 10;

        if(key_int == key_next_IIDX)
        {
            question += 1;
            if(question > 5) question = 1;
        }
        else if(key_int == key_enter_IIDX)
        {
            enter = !enter;
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